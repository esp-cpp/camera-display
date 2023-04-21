#include <algorithm>
#include <chrono>
#include <functional>
#include <iterator>
#include <thread>

#if CONFIG_ESP32_WIFI_NVS_ENABLED
#include "nvs_flash.h"
#endif

#include "logger.hpp"
#include "task.hpp"
#include "tcp_socket.hpp"
#include "udp_socket.hpp"
#include "wifi_sta.hpp"

#include "jpeg_frame.hpp"
#include "rtsp_client.hpp"

using namespace std::chrono_literals;
using namespace std::placeholders;

extern "C" void app_main(void) {
  espp::Logger logger({.tag = "RtspClient example", .level = espp::Logger::Verbosity::INFO});
  logger.info("Starting RtspClient example");

#if CONFIG_ESP32_WIFI_NVS_ENABLED
  // Initialize NVS
  esp_err_t ret = nvs_flash_init();
  if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
    ESP_ERROR_CHECK(nvs_flash_erase());
    ret = nvs_flash_init();
  }
  ESP_ERROR_CHECK(ret);
#endif

  // create a wifi station here so that LwIP will be init for this example
  espp::WifiSta wifi_sta({
      .ssid = CONFIG_ESP_WIFI_SSID,
        .password = CONFIG_ESP_WIFI_PASSWORD,
        .num_connect_retries = CONFIG_ESP_MAXIMUM_RETRY,
        .on_connected = nullptr,
        .on_disconnected = nullptr,
        .on_got_ip = [](ip_event_got_ip_t* eventdata) {
          fmt::print("got IP: {}.{}.{}.{}\n", IP2STR(&eventdata->ip_info.ip));
        }
        });

  // wait until wifi is connected
  while (!wifi_sta.is_connected()) {
    std::this_thread::sleep_for(1s);
  }

  std::error_code ec;
  espp::RtspClient rtsp_client({
      .server_address = "192.168.86.181",
        .rtsp_port = 8554,
        .path = "/mjpeg/1",
        .on_jpeg_frame = [](std::unique_ptr<espp::JpegFrame> jpeg_frame) {
          auto jpeg_data = jpeg_frame->get_data();
          auto jpeg_size = jpeg_data.size();
          fmt::print("Got JPEG frame: {} bytes\n", jpeg_size);
        },
        .log_level = espp::Logger::Verbosity::INFO,
        });

  rtsp_client.connect(ec);
  if (ec) {
    logger.error("Failed to connect to RTSP server: {}", ec.message());
    return;
  }

  rtsp_client.describe(ec);
  if (ec) {
    logger.error("Failed to describe stream: {}", ec.message());
    return;
  }

  rtsp_client.setup(ec);
  if (ec) {
    logger.error("Failed to setup stream: {}", ec.message());
    return;
  }

  rtsp_client.play(ec);
  if (ec) {
    logger.error("Failed to play stream: {}", ec.message());
    return;
  }

  // sleep for 5 seconds
  std::this_thread::sleep_for(5s);

  // NOTE: the current server doesn't properly respond to the teardown request, so we'll just
  //       ignore the error here
  rtsp_client.teardown(ec);

  // NOTE: the current server doesn't properly respond to the teardown request, so we'll just
  //       ignore the error here
  rtsp_client.disconnect(ec);

  logger.info("RtspClient example finished");

  // wait forever
  while (true) {
    std::this_thread::sleep_for(1s);
  }
}
