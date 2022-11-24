#include "sdkconfig.h"

#include <chrono>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "esp_heap_caps.h"
#include "driver/i2c.h"
#include "nvs_flash.h"

#include "format.hpp"
#include "task.hpp"
#include "tcp_socket.hpp"
#include "wifi_sta.hpp"

using namespace std::chrono_literals;

struct Image {
  uint8_t *data;
  uint32_t num_bytes;
};

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
  // TODO: create the lvgl display & display driver
  // TODO: create the gui for displaying the iamge
  // TODO: create the display task
  logger.info("Creating display task");
  std::atomic<int> num_frames_displayed{0};
  QueueHandle_t receive_queue = xQueueCreate(10, sizeof(Image));
  auto display_task_fn = [&receive_queue, &num_frames_displayed, &logger](auto& m, auto& cv) {
    static Image image;
    // wait on the queue until we have an image ready to display
    if (xQueueReceive(receive_queue, &image, portMAX_DELAY) == pdPASS) {
      auto end = std::chrono::high_resolution_clock::now();
      // TODO: parse the jpeg data into a byte array
      // TODO: update the lvgl display
      // now free the memory we allocated
      free(image.data);
    }
  };
  // make the tcp_server
  std::string ip_address = "192.168.1.23";
  size_t port = 8888;
  std::atomic<int> num_frames_received{0};
  espp::TcpSocket server_socket({.log_level=espp::Logger::Verbosity::WARN});
  auto server_task_config = espp::Task::Config{
    .name = "TcpServer",
    .callback = nullptr,
    .stack_size_bytes = 6 * 1024
  };
  auto server_config = espp::TcpSocket::ReceiveConfig{
    .port = port,
    .buffer_size = 1024,
    .on_receive_callback = [&receive_queue, &num_frames_received](auto& data, auto& source) -> auto {
      fmt::print("Server received: {}\n"
                 "    from source: {}:{}\n",
                 data, source.address, source.port);
      // TODO: get data / length and put it into the queue as a received image
      // only copy / allocate if there is space in the queue, otherwise just
      // discard this image.
      auto num_spots = uxQueueSpacesAvailable(receive_queue);
      if (num_spots > 0) {
        // copy the image data into a buffer and pass that buffer to the sending
        // task notify that image is ready.
        size_t jpeg_len =
          data[4] << 24 |
          data[5] << 16 |
          data[6] << 8  |
          data[7];
        Image image;
        image.num_bytes = jpeg_len;
        image.data = (uint8_t*)heap_caps_malloc(image.num_bytes, MALLOC_CAP_SPIRAM);
        if (image.data != nullptr) {
          num_frames_received += 1;
          // data
          memcpy(&image.data[8], &data[8], jpeg_len);
          if (xQueueSend(receive_queue, &image, portMAX_DELAY) != pdPASS) {
            // couldn't receive the image, so we should free the memory here
            free(image.data);
          }
        }
      }
      // don't respond to server
      return std::nullopt;
    }
  };
  server_socket.start_receiving(server_task_config, server_config);

  logger.info("Starting display task");
  // TODO: lvgl display task
  auto start = std::chrono::high_resolution_clock::now();
  while (true) {
    std::this_thread::sleep_for(1s);
  }
}
