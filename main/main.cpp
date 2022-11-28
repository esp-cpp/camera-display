#include "sdkconfig.h"

#include <algorithm>
#include <chrono>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "esp_heap_caps.h"
#include "hal/spi_types.h"
#include "driver/gpio.h"
#include "driver/i2c.h"
#include "driver/spi_master.h"
#include "nvs_flash.h"

// #include "jpeg_decoder.h"
#include "jpegdec.h"

#include "format.hpp"
#include "task.hpp"
#include "task_monitor.hpp"
#include "tcp_socket.hpp"
#include "udp_socket.hpp"
#include "wifi_sta.hpp"
#include "st7789.hpp"

#include "lcd.hpp"

using namespace std::chrono_literals;

// function for drawing the minimum compressible units
int drawMCUs(JPEGDRAW *pDraw) {
  int iCount = pDraw->iWidth * pDraw->iHeight;
  auto xs = pDraw->x;
  auto ys = pDraw->y;
  auto xe = pDraw->x + pDraw->iWidth - 1;
  auto ye = pDraw->y + pDraw->iHeight - 1;

  static size_t frame_buffer_index = 0;
  uint8_t* out_img_buf = (uint8_t*)(frame_buffer_index ? get_vram1() : get_vram0());
  frame_buffer_index = frame_buffer_index ? 0 : 1;
  memcpy(out_img_buf, pDraw->pPixels, iCount * 2);

  lcd_send_lines(xs, ys, xe, ye, out_img_buf, 0);
  // returning true (1) tells JPEGDEC to continue decoding. Returning false
  // (0) would quit decoding immediately.
  return 1;
}

struct Image {
  uint8_t *data{nullptr};
  uint32_t num_bytes{0};
  size_t offset{0};
  int bytes_remaining{0};
};

size_t get_image_length(const uint8_t* header) {
  return
    header[4] << 24 |
    header[5] << 16 |
    header[6] << 8  |
    header[7];
}

void init_image(const uint8_t* data, size_t data_len, Image* image) {
  // 4 start bytes, 4 bytes of image length in header before start of image data
  static size_t data_offset = 8;
  size_t jpeg_len = get_image_length(data);
  size_t img_bytes_received = std::min(data_len - data_offset, jpeg_len);
  image->num_bytes = jpeg_len;
  image->data = (uint8_t*)heap_caps_malloc(image->num_bytes, MALLOC_CAP_8BIT | MALLOC_CAP_SPIRAM);
  // add the bytes to our new array
  memcpy(image->data, &data[data_offset], img_bytes_received);
  image->offset = img_bytes_received;
  image->bytes_remaining = jpeg_len - img_bytes_received;
}

void update_image(const uint8_t* data, size_t data_len, Image* image) {
  memcpy(&image->data[image->offset], data, data_len);
  image->bytes_remaining -= data_len;
  image->offset += data_len;
  // fmt::print("Updated image offset = {}, remaining = {}/{}\n",
  //            image->offset, image->bytes_remaining, image->num_bytes);
}

int find_header(std::basic_string_view<uint8_t> data) {
  static std::vector<uint8_t> header{0xAA, 0xBB, 0xCC, 0xDD};
  auto res = std::search(std::begin(data), std::end(data), std::begin(header), std::end(header));
  bool has_header = res != std::end(data);
  if (!has_header) {
    return -1;
  }
  size_t header_offset = res - std::begin(data);
  return header_offset;
}

int handle_image_data(std::basic_string_view<uint8_t> data, QueueHandle_t image_queue) {
  static bool has_seen_header = false;
  static Image image0, image1;
  static size_t image_index = 0;
  Image* image = image_index ? &image1 : &image0;
  // will need to put multiple packets together into a single image based on
  // header + length. only copy / allocate if there is space in the queue,
  // otherwise just discard this image.
  if (!has_seen_header) {
    auto header_offset = find_header(data);
    if (header_offset < 0) return -1;
    size_t length = data.size() - header_offset;
    init_image(&data[header_offset], length, image);
    // fmt::print("Got header at offset {}/{}, remaining = {}/{}\n",
    //            header_offset, data.size(), image->bytes_remaining, image->num_bytes);
    // update state
    has_seen_header = true;
  } else {
    // we've seen the header, so the beginning of this packet must be the
    // continuation of the image.
    size_t num_bytes = std::min(image->bytes_remaining, (int)data.size());
    // fmt::print("continuation: offset = {}, data length = {}/{}, remaining = {}/{}\n",
    //            image->offset, num_bytes, data.size(), image->bytes_remaining, image->num_bytes);
    update_image(data.data(), num_bytes, image);
  }

  // return bytes remaining
  if (image->bytes_remaining > 0) {
    return image->bytes_remaining;
  }

  // bytes_remaining is <= 0, so send the image
  auto num_spots = uxQueueSpacesAvailable(image_queue);
  if (num_spots > 0) {
    xQueueSend(image_queue, image, portMAX_DELAY);
    image_index = image_index ? 0 : 1;
    image = image_index ? &image1 : &image0;
  } else {
    free(image->data);
    image->data = nullptr;
  }
  has_seen_header = false;
  return -1;
}

extern "C" void app_main(void) {
  esp_err_t err;
  espp::Logger logger({.tag = "Camera Display", .level = espp::Logger::Verbosity::INFO});
  logger.info("Bootup");

  // initialize NVS, needed for WiFi
  esp_err_t ret = nvs_flash_init();
  if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
    logger.warn("Erasing NVS flash...");
    ESP_ERROR_CHECK(nvs_flash_erase());
    ret = nvs_flash_init();
  }
  ESP_ERROR_CHECK(ret);

  // initialize WiFi
  logger.info("Initializing WiFi");
  espp::WifiSta wifi_sta({
      .ssid = CONFIG_ESP_WIFI_SSID,
        .password = CONFIG_ESP_WIFI_PASSWORD,
        .num_connect_retries = CONFIG_ESP_MAXIMUM_RETRY,
        .on_connected = nullptr,
        .on_disconnected = nullptr,
        .on_got_ip = [&logger](ip_event_got_ip_t* eventdata) {
          logger.info("got IP: {}.{}.{}.{}", IP2STR(&eventdata->ip_info.ip));
        }
        });

  // wait for network
  while (!wifi_sta.is_connected()) {
    logger.info("waiting for wifi connection...");
    std::this_thread::sleep_for(1s);
  }

  // multicast our receiver info over UDP
  // create threads
  auto client_task_fn = [](auto&, auto&) {
    static espp::UdpSocket client_socket({});
    static std::string multicast_group = "239.1.1.1";
    static size_t multicast_port = 5000;
    static std::string payload = "hello world";
    static auto send_config = espp::UdpSocket::SendConfig{
      .ip_address = multicast_group,
      .port = multicast_port,
      .is_multicast_endpoint = true,
    };
    // NOTE: now this call blocks until the response is received
    client_socket.send(payload, send_config);
    std::this_thread::sleep_for(1s);
  };
  auto client_task = espp::Task::make_unique({
      .name = "Client Task",
      .callback = client_task_fn,
      .stack_size_bytes = 3*1024
    });
  client_task->start();

  // initialize the lcd for the image display
  lcd_init();

  // create the parsing and display task
  logger.info("Creating display task");
  std::atomic<int> num_frames_displayed{0};
  QueueHandle_t receive_queue = xQueueCreate(2, sizeof(Image));

  std::atomic<float> elapsed{0};
  auto display_task_fn = [&receive_queue, &num_frames_displayed, &elapsed](auto& m, auto& cv) {
    // the original (max) image size is 1600x1200, but the S3 BOX has a resolution of 320x240
    // wait on the queue until we have an image ready to display
    static Image image;
    static JPEGDEC jpeg;
    static espp::Logger logger({.tag = "Decoder", .level = espp::Logger::Verbosity::INFO});

    if (xQueueReceive(receive_queue, &image, portMAX_DELAY) == pdPASS) {
      logger.debug("Got image, length = {}", image.num_bytes);
      static auto start = std::chrono::high_resolution_clock::now();
      if (jpeg.openRAM(image.data, image.num_bytes, drawMCUs)) {
        logger.debug("Image size: {} x {}, orientation: {}, bpp: {}", jpeg.getWidth(),
                    jpeg.getHeight(), jpeg.getOrientation(), jpeg.getBpp());
        jpeg.setPixelType(RGB565_BIG_ENDIAN);
        if (!jpeg.decode(0,0,0)) {
          logger.error("Error decoding");
        }
      } else {
        logger.error("error opening jpeg image");
      }

      auto end = std::chrono::high_resolution_clock::now();
      elapsed = std::chrono::duration<float>(end-start).count();
      num_frames_displayed += 1;

      // now free the memory we allocated when receiving the jpeg buffer
      free(image.data);
    }
  };
  // Start the display task
  logger.info("Starting display task");
  auto display_task = espp::Task::make_unique({
      .name = "Display Task",
      .callback = display_task_fn,
      .stack_size_bytes = 10 * 1024,
    });
  display_task->start();

  // make the tcp_server
  logger.info("Starting server task");
  std::atomic<int> num_frames_received{0};
  espp::TcpSocket server_socket({.log_level=espp::Logger::Verbosity::WARN});
  static constexpr size_t port = 8888;
  // bind
  if (!server_socket.bind(port)){
    return;
  }
  // listen
  static constexpr size_t max_pending_connections = 1;
  if (!server_socket.listen(max_pending_connections)) {
    return;
  }
  auto server_task = espp::Task::make_unique({
    .name = "TcpServer Task",
    .callback = [&server_socket, &receive_queue, &num_frames_received](auto& m, auto& cv) {
      static espp::Logger logger({.tag = "Receiver", .level = espp::Logger::Verbosity::INFO});
      // ensure our socket is already closed
      server_socket.close_accepted_socket();
      // accept
      if (!server_socket.accept()) {
        return;
      }
      // receive
      auto client_socket = server_socket.get_accepted_socket();
      static constexpr size_t max_buffer_size = 1024;
      static uint8_t *data = (uint8_t*)heap_caps_malloc(max_buffer_size, MALLOC_CAP_8BIT | MALLOC_CAP_SPIRAM);
      size_t receive_buffer_size = max_buffer_size;
      while (true) {
        memset(data, 0, max_buffer_size);
        logger.debug("Trying to receive {} B", receive_buffer_size);
        auto num_bytes = server_socket.receive(client_socket, receive_buffer_size, data);
        if (num_bytes < 0) {
          // couldn't receive, let's see if we can break to try to accept again
          break;
        }
        int bytes_remaining = handle_image_data(std::basic_string_view<uint8_t>(data, num_bytes), receive_queue);
        if (bytes_remaining > 0) {
          receive_buffer_size = std::min(bytes_remaining, (int)max_buffer_size);
        } else {
          num_frames_received += 1;
          receive_buffer_size = max_buffer_size;
        }
      }
    },
    .stack_size_bytes = 10 * 1024,
  });
  server_task->start();

  auto start = std::chrono::high_resolution_clock::now();
  while (true) {
    auto end = std::chrono::high_resolution_clock::now();
    float current_time = std::chrono::duration<float>(end-start).count();
    fmt::print("[TM] {}\n", espp::TaskMonitor::get_latest_info());
    fmt::print("[{:.3f}] Received {} frames\n", current_time, num_frames_received);
    float disp_elapsed = elapsed;
    if (disp_elapsed > 1) {
      fmt::print("[{:.3f}] Framerate: {} FPS\n", current_time, num_frames_displayed / disp_elapsed);
    }
    std::this_thread::sleep_for(1s);
  }
}
