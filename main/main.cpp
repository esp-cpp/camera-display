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
  image->offset = 0;
  if (image->data) free(image->data);
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

void handle_image_data(const std::vector<uint8_t> &data, QueueHandle_t image_queue) {
  static bool has_seen_header = false;
  static Image image0, image1;
  static size_t image_index = 0;
  Image* image = image_index ? &image1 : &image0;
  // will need to put multiple packets together into a single image based on
  // header + length. only copy / allocate if there is space in the queue,
  // otherwise just discard this image.
  if (!has_seen_header) {
    auto header_offset = find_header(std::basic_string_view<uint8_t>(data.data(), data.size()));
    if (header_offset < 0) return;
    size_t length = data.size() - header_offset;
    init_image(&data[header_offset], length, image);
    // update state
    has_seen_header = true;
  } else {
    // we've seen the header, so the beginning of this packet must be the
    // continuation of the image.
    size_t num_bytes = std::min(image->bytes_remaining, (int)data.size());
    update_image(data.data(), num_bytes, image);
    fmt::print("continuation: offset = {}, data length = {}/{}, remaining = {}\n",
               image->offset, num_bytes, data.size(), image->bytes_remaining);
    if (image->bytes_remaining > 0) {
      return;
    }

    // bytes_remaining is <= 0, so send the image
    auto num_spots = uxQueueSpacesAvailable(image_queue);
    if (num_spots > 0) {
      xQueueSend(image_queue, image, 10 / portTICK_PERIOD_MS);
      // num_frames_received += 1;
      image_index = image_index ? 0 : 1;
      image = image_index ? &image1 : &image0;
    }

    has_seen_header = false;
    // we didn't use the whole packet finishing the image, so start a new one
    size_t remaining = data.size() - num_bytes;
    if (remaining) {
      fmt::print("Still have another image, looking again!\n");
      auto new_data = std::basic_string_view<uint8_t>(&data[num_bytes], remaining);
      auto header_offset = find_header(new_data);
      if (header_offset < 0) return;
      size_t new_length = new_data.size() - header_offset;
      init_image(&new_data[header_offset], new_length, image);
      has_seen_header = true;
      fmt::print("\tfound it!!\n");
    }
  }
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
  auto display_task_fn = [&receive_queue, &num_frames_displayed, &elapsed, &logger](auto& m, auto& cv) {
    // the original (max) image size is 1600x1200, but the S3 BOX has a resolution of 320x240
    // wait on the queue until we have an image ready to display
    static Image image;
    static JPEGDEC jpeg;

    if (xQueueReceive(receive_queue, &image, portMAX_DELAY) == pdPASS) {
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

  // make the tcp_server
  size_t port = 8888;
  std::atomic<int> num_frames_received{0};
  espp::TcpSocket server_socket({.log_level=espp::Logger::Verbosity::WARN});
  auto server_task_config = espp::Task::Config{
    .name = "TcpServer",
    .callback = nullptr, // the callback is provided in the ReceiveConfig struct
    .stack_size_bytes = 5 * 1024,
  };
  auto server_config = espp::TcpSocket::ReceiveConfig{
    .port = port,
    .buffer_size = 1024,
    .on_receive_callback = [&receive_queue](auto& data, auto& source) -> auto {
      handle_image_data(data, receive_queue);
      // don't respond to client
      return std::nullopt;
    }
  };
  server_socket.start_receiving(server_task_config, server_config);

  // Start the display task
  logger.info("Starting display task");
  auto display_task = espp::Task::make_unique({
      .name = "Display Task",
      .callback = display_task_fn,
      .stack_size_bytes = 5 * 1024,
    });
  display_task->start();
  auto start = std::chrono::high_resolution_clock::now();
  while (true) {
    fmt::print("[TM] {}\n", espp::TaskMonitor::get_latest_info());
    fmt::print("Received {} frames\n", num_frames_received);
    float disp_elapsed = elapsed;
    if (disp_elapsed > 1) {
      fmt::print("Framerate: {} FPS\n", num_frames_displayed / disp_elapsed);
    }
    std::this_thread::sleep_for(1s);
  }
}
