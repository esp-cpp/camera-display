#include "sdkconfig.h"

#include <algorithm>
#include <chrono>
#include <deque>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "mdns.h"

#include "JPEGDEC.h"

#include "format.hpp"
#include "nvs.hpp"
#include "rtsp_client.hpp"
#include "task.hpp"
#include "task_monitor.hpp"
#include "tcp_socket.hpp"
#include "udp_socket.hpp"
#include "wifi_sta.hpp"
#include "wifi_sta_menu.hpp"

#if CONFIG_HARDWARE_BOX
#include "esp-box.hpp"
using hal = espp::EspBox;
#elif CONFIG_HARDWARE_TDECK
#include "t-deck.hpp"
using hal = espp::TDeck;
#else
#error "No hardware defined"
#endif

using namespace std::chrono_literals;

static espp::Logger logger({.tag = "Camera Display", .level = espp::Logger::Verbosity::INFO});

static std::atomic<int> num_frames_received{0};
static std::atomic<int> num_frames_displayed{0};
static std::atomic<float> elapsed{0};
static std::chrono::high_resolution_clock::time_point connected_time;

std::unique_ptr<espp::Task> start_rtsp_task;
static std::shared_ptr<espp::RtspClient> rtsp_client;

static std::mutex jpeg_mutex;
static std::condition_variable jpeg_cv;
static constexpr size_t MAX_JPEG_FRAMES = 2;
static std::deque<std::unique_ptr<espp::JpegFrame>> jpeg_frames;

bool start_rtsp_client(std::mutex &m, std::condition_variable &cv, bool &task_notified);
int drawMCUs(JPEGDRAW *pDraw);
bool display_task_fn(std::mutex &m, std::condition_variable &cv);
void mdns_print_results(mdns_result_t *results);
bool find_mdns_service(const char *service_name, const char *proto, std::string &host, int &port,
                       int timeout_ms = 3000);

extern "C" void app_main(void) {
  logger.info("Bootup");

  // initialize the hardware
  auto &hw = hal::get();
  if (!hw.initialize_lcd()) {
    logger.error("Could not initialize LCD");
    return;
  }
  static constexpr size_t pixel_buffer_size = hw.lcd_width() * 50;
  if (!hw.initialize_display(pixel_buffer_size)) {
    logger.error("Could not initialize display");
    return;
  }
  if (!hw.initialize_touch()) {
    logger.error("Could not initialize touch");
    return;
  }

  // create the parsing and display task
  logger.info("Starting display task");
  auto display_task = espp::Task::make_unique({.callback = display_task_fn,
                                               .task_config = {
                                                   .name = "Display Task",
                                                   .stack_size_bytes = 5 * 1024,
                                               }});
  display_task->start();

  std::error_code ec;

#if CONFIG_ESP32_WIFI_NVS_ENABLED
  // initialize NVS, needed for WiFi
  espp::Nvs nvs;
  nvs.init(ec);
#endif

  // initialize WiFi
  logger.info("Initializing WiFi");
  espp::WifiSta wifi_sta(
      {.ssid = CONFIG_ESP_WIFI_SSID,
       .password = CONFIG_ESP_WIFI_PASSWORD,
       .num_connect_retries = CONFIG_ESP_MAXIMUM_RETRY,
       .on_connected = []() { logger.info("Connected to WiFi, waiting for IP address"); },
       .on_disconnected =
           []() {
             logger.info("Disconnected from WiFi, stopping task");
             // ensure the rtsp start function is not running
             start_rtsp_task.reset();
             logger.info("Stopping RTSP Client");
             // stop and delete the RTSP client
             rtsp_client.reset();
           },
       .on_got_ip =
           [](ip_event_got_ip_t *eventdata) {
             logger.info("got IP: {}.{}.{}.{}", IP2STR(&eventdata->ip_info.ip));
             // since the starting has to wait for rtsp client to connect to
             // server, we run it in a separate thread
             start_rtsp_task =
                 espp::Task::make_unique(espp::Task::Config{.callback = start_rtsp_client,
                                                            .task_config = {
                                                                .name = "RTSP Client Task",
                                                                .stack_size_bytes = 8 * 1024,
                                                                .priority = 5,
                                                            }});
             start_rtsp_task->start();
           }});

  espp::WifiStaMenu sta_menu(wifi_sta);
  auto root_menu = sta_menu.get();
  root_menu->Insert(
      "memory",
      [](std::ostream &out) {
        out << "Minimum free memory: " << heap_caps_get_minimum_free_size(MALLOC_CAP_DEFAULT)
            << std::endl;
      },
      "Display minimum free memory.");
  root_menu->Insert(
      "stats",
      [](std::ostream &out) {
        auto now = std::chrono::high_resolution_clock::now();
        float disp_elapsed = std::chrono::duration<float>(now - connected_time).count();
        out << fmt::format("Received {} frames, Framerate: {} FPS\n", num_frames_received,
                           num_frames_displayed / disp_elapsed);
      },
      "Display RTSP client statistics.");

  cli::Cli cli(std::move(root_menu));
  cli::SetColor();
  cli.ExitAction([](auto &out) { out << "Goodbye and thanks for all the fish.\n"; });

  espp::Cli input(cli);
  input.SetInputHistorySize(10);
  input.Start();
}

bool start_rtsp_client(std::mutex &m, std::condition_variable &cv, bool &task_notified) {
  // initialize mDNS
  logger.info("Initializing mDNS");
  auto err = mdns_init();
  if (err != ESP_OK) {
    logger.error("Could not initialize mDNS: {}", err);
    return true;
  }

  uint8_t mac[6];
  esp_read_mac(mac, ESP_MAC_WIFI_STA);
  std::string hostname = fmt::format("camera-display-{:x}{:x}{:x}", mac[3], mac[4], mac[5]);
  err = mdns_hostname_set(hostname.c_str());
  if (err != ESP_OK) {
    logger.error("Could not set mDNS hostname: {}", err);
    return true;
  }
  logger.info("mDNS hostname set to '{}'", hostname);
  err = mdns_instance_name_set("Camera Display");
  if (err != ESP_OK) {
    logger.error("Could not set mDNS instance name: {}", err);
    return true;
  }
  std::string mdns_service_address;
  int mdns_service_port;
  bool found_mdns_server{false};
  while (!found_mdns_server) {
    logger.info("Searching for RTSP server...");
    found_mdns_server =
        find_mdns_service("_rtsp", "_tcp", mdns_service_address, mdns_service_port, 3000);
    if (task_notified) {
      logger.info("Stopping RTSP client task");
      return true;
    }
  }
  logger.info("Found RTSP server: {}:{}", mdns_service_address, mdns_service_port);

  // make the rtsp client
  logger.info("Starting RTSP client");
  rtsp_client = std::make_shared<espp::RtspClient>(espp::RtspClient::Config{
      .server_address = mdns_service_address,
      .rtsp_port = mdns_service_port,
      .path = "/mjpeg/1",
      .on_jpeg_frame =
          [](std::unique_ptr<espp::JpegFrame> jpeg_frame) {
            {
              std::lock_guard<std::mutex> lock(jpeg_mutex);
              if (jpeg_frames.size() >= MAX_JPEG_FRAMES) {
                jpeg_frames.pop_front();
              }
              jpeg_frames.push_back(std::move(jpeg_frame));
            }
            jpeg_cv.notify_all();
            num_frames_received += 1;
          },
      .log_level = espp::Logger::Verbosity::ERROR,
  });

  std::error_code ec;
  do {
    // clear the error code
    ec.clear();
    rtsp_client->connect(ec);
    if (ec) {
      logger.error("Error connecting to server: {}", ec.message());
      logger.info("Retrying in 1s...");
      std::unique_lock<std::mutex> lk(m);
      auto stop_requested = cv.wait_for(lk, 1s, [&task_notified] { return task_notified; });
      if (stop_requested) {
        logger.info("Stopping RTSP client task");
        return true; // exit the task if stop was requested
      }
    }
  } while (ec);

  rtsp_client->describe(ec);
  if (ec) {
    logger.error("Error describing server: {}", ec.message());
  }
  if (task_notified) {
    logger.info("Stopping RTSP client task");
    return true; // exit the task if stop was requested
  }

  rtsp_client->setup(ec);
  if (ec) {
    logger.error("Error setting up server: {}", ec.message());
  }
  if (task_notified) {
    logger.info("Stopping RTSP client task");
    return true; // exit the task if stop was requested
  }

  rtsp_client->play(ec);
  if (ec) {
    logger.error("Error playing server: {}", ec.message());
  }

  connected_time = std::chrono::high_resolution_clock::now();

  return true; // we're done with our work, no need to run again, so stop the task
}

// function for drawing the minimum compressible units
// cppcheck-suppress constParameterCallback
int drawMCUs(JPEGDRAW *pDraw) {
  int iCount = pDraw->iWidth * pDraw->iHeight;
  auto xs = pDraw->x;
  auto ys = pDraw->y;
  auto xe = pDraw->x + pDraw->iWidth - 1;
  auto ye = pDraw->y + pDraw->iHeight - 1;

  static size_t frame_buffer_index = 0;
  uint8_t *out_img_buf = (uint8_t *)(frame_buffer_index ? hal::get().vram1() : hal::get().vram0());
  frame_buffer_index = frame_buffer_index ? 0 : 1;
  memcpy(out_img_buf, pDraw->pPixels, iCount * 2);

  hal::get().write_lcd_lines(xs, ys, xe, ye, out_img_buf, 0);
  // returning true (1) tells JPEGDEC to continue decoding. Returning false
  // (0) would quit decoding immediately.
  return 1;
}

bool display_task_fn(std::mutex &m, std::condition_variable &cv) {
  // the original (max) image size is 1600x1200, but the S3 BOX has a resolution of 320x240
  // wait on the queue until we have an image ready to display
  static JPEGDEC jpeg;
  static espp::Logger logger({.tag = "Decoder", .level = espp::Logger::Verbosity::WARN});

  std::unique_ptr<espp::JpegFrame> image;
  {
    // wait for a frame to be available
    std::unique_lock<std::mutex> lock(jpeg_mutex);
    jpeg_cv.wait(lock, [] { return !jpeg_frames.empty(); });
    image = std::move(jpeg_frames.front());
    jpeg_frames.pop_front();
  }
  static auto start = std::chrono::high_resolution_clock::now();
  auto image_data = image->get_data();
  logger.info("Decoding image of size {} B, shape = {} x {}", image_data.size(), image->get_width(),
              image->get_height());
  if (jpeg.openRAM((uint8_t *)(image_data.data()), image_data.size(), drawMCUs)) {
    logger.debug("Image size: {} x {}, orientation: {}, bpp: {}", jpeg.getWidth(), jpeg.getHeight(),
                 jpeg.getOrientation(), jpeg.getBpp());
    jpeg.setPixelType(RGB565_BIG_ENDIAN);
    if (!jpeg.decode(0, 0, 0)) {
      logger.error("Error decoding");
    }
  } else {
    logger.error("error opening jpeg image");
  }
  auto end = std::chrono::high_resolution_clock::now();
  elapsed = std::chrono::duration<float>(end - start).count();
  num_frames_displayed += 1;
  // signal that we do not want to stop the task
  return false;
}

static const char *ip_protocol_str[] = {"V4", "V6", "MAX"};
void mdns_print_results(mdns_result_t *results) {
  mdns_result_t *r = results;
  mdns_ip_addr_t *a = NULL;
  int i = 1, t;
  while (r) {
    if (r->esp_netif) {
      printf("%d: Interface: %s, Type: %s, TTL: %lu\n", i++, esp_netif_get_ifkey(r->esp_netif),
             ip_protocol_str[r->ip_protocol], r->ttl);
    }
    if (r->instance_name) {
      printf("  PTR : %s.%s.%s\n", r->instance_name, r->service_type, r->proto);
    }
    if (r->hostname) {
      printf("  SRV : %s.local:%u\n", r->hostname, r->port);
    }
    if (r->txt_count) {
      printf("  TXT : [%zu] ", r->txt_count);
      for (t = 0; t < r->txt_count; t++) {
        printf("%s=%s(%d); ", r->txt[t].key, r->txt[t].value ? r->txt[t].value : "NULL",
               r->txt_value_len[t]);
      }
      printf("\n");
    }
    a = r->addr;
    while (a) {
      if (a->addr.type == ESP_IPADDR_TYPE_V6) {
        // cppcheck-suppress unknownMacro
        printf("  AAAA: " IPV6STR "\n", IPV62STR(a->addr.u_addr.ip6));
      } else {
        printf("  A   : " IPSTR "\n", IP2STR(&(a->addr.u_addr.ip4)));
      }
      a = a->next;
    }
    r = r->next;
  }
}

bool find_mdns_service(const char *service_name, const char *proto, std::string &host, int &port,
                       int timeout_ms) {
  fmt::print("Query PTR: {}.{}.local\n", service_name, proto);

  mdns_result_t *results = NULL;
  int max_results = 20;
  esp_err_t err = mdns_query_ptr(service_name, proto, timeout_ms, max_results, &results);
  if (err) {
    fmt::print("Query Failed\n");
    return false;
  }
  if (!results) {
    fmt::print("No results found!\n");
    return false;
  }

  mdns_print_results(results);
  // now set the host ip address string and port number from the results
  mdns_result_t *r = results;
  if (r->addr) {
    if (r->addr->addr.type == ESP_IPADDR_TYPE_V6) {
      host = fmt::format(IPV6STR, IPV62STR(r->addr->addr.u_addr.ip6));
    } else {
      host = fmt::format("{}.{}.{}.{}", IP2STR(&(r->addr->addr.u_addr.ip4)));
    }
  }
  if (r->port) {
    port = r->port;
  }
  mdns_query_results_free(results);
  return true;
}
