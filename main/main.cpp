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
#elif CONFIG_HARDWARE_BYTE90
#include "byte90.hpp"
using hal = espp::Byte90;
#elif CONFIG_HARDWARE_WS_S3_TOUCH
#include "ws-s3-touch.hpp"
using hal = espp::WsS3Touch;
#else
#error "No hardware defined"
#endif

using namespace std::chrono_literals;
using DisplayDriver = hal::DisplayDriver;

static espp::Logger logger({.tag = "Camera Display", .level = espp::Logger::Verbosity::INFO});

// frame buffers for decoding into
static uint8_t *fb0 = nullptr;
static uint8_t *fb1 = nullptr;
// DRAM for actual vram (used by SPI to send to LCD)
static uint8_t *vram0 = nullptr;
static uint8_t *vram1 = nullptr;
static std::atomic<int> num_frames_received{0};
static std::atomic<int> num_frames_displayed{0};
static std::atomic<float> elapsed{0};
static std::chrono::high_resolution_clock::time_point connected_time;

// video
static std::unique_ptr<espp::Task> video_task_{nullptr};
static QueueHandle_t video_queue_{nullptr};
static bool initialize_video();
static bool video_task_callback(std::mutex &m, std::condition_variable &cv, bool &task_notified);
static void clear_screen();
static void push_frame(const void *frame);

// rtsp
std::unique_ptr<espp::Task> start_rtsp_task;
static std::shared_ptr<espp::RtspClient> rtsp_client;

static constexpr int num_rows_in_vram = 50;
static constexpr size_t vram_size = hal::lcd_width() * num_rows_in_vram * sizeof(hal::Pixel);
static constexpr size_t fb_size = hal::lcd_width() * hal::lcd_height() * sizeof(hal::Pixel);
static std::mutex jpeg_mutex;
static std::condition_variable jpeg_cv;
static constexpr size_t MAX_JPEG_FRAMES = 3;
static std::deque<std::shared_ptr<espp::JpegFrame>> jpeg_frames;

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

  // allocate some frame buffers for jpeg decoding, which should be screen-size
  // and in PSRAM
  fb0 = (uint8_t *)heap_caps_malloc(fb_size, MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
  fb1 = (uint8_t *)heap_caps_malloc(fb_size, MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
  if (!fb0 || !fb1) {
    logger.error("Could not allocate frame buffers for LCD");
    if (fb0) {
      heap_caps_free(fb0);
    }
    if (fb1) {
      heap_caps_free(fb1);
    }
    return;
  }

  // allocate some DMA-capable VRAM for jpeg decoding / display operations
  vram0 = (uint8_t *)heap_caps_malloc(vram_size, MALLOC_CAP_DMA | MALLOC_CAP_8BIT);
  vram1 = (uint8_t *)heap_caps_malloc(vram_size, MALLOC_CAP_DMA | MALLOC_CAP_8BIT);
  if (!vram0 || !vram1) {
    logger.error("Could not allocate VRAM for LCD");
    if (vram0) {
      heap_caps_free(vram0);
    }
    if (vram1) {
      heap_caps_free(vram1);
    }
    return;
  }

  logger.info("Allocated frame buffers: fb0 = {} B, fb1 = {} B", fb_size, fb_size);
  logger.info("Allocated VRAM: vram0 = {} B, vram1 = {} B", vram_size, vram_size);

  // initialize the video task
  if (!initialize_video()) {
    logger.error("Could not initialize video task");
    return;
  }

  // clear the screen
  logger.info("Clearing screen");
  clear_screen();

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
       // .phy_rate = WIFI_PHY_RATE_MCS5_SGI,
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
             // free mdns resources
             mdns_free();
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
      "log_level", {"log level <debug, info, warn, error, none>"},
      [](std::ostream &out, const std::string &level_str) {
        espp::Logger::Verbosity level;
        if (level_str == "debug") {
          level = espp::Logger::Verbosity::DEBUG;
        } else if (level_str == "info") {
          level = espp::Logger::Verbosity::INFO;
        } else if (level_str == "warn") {
          level = espp::Logger::Verbosity::WARN;
        } else if (level_str == "error") {
          level = espp::Logger::Verbosity::ERROR;
        } else if (level_str == "none") {
          level = espp::Logger::Verbosity::NONE;
        } else {
          out << "Unknown log level: " << level_str << std::endl;
          return;
        }
        logger.set_verbosity(level);
        out << "Log level set to: " << level_str << std::endl;
      },
      "Set the log level for the application. Options: debug, info, warn, error, none.");
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
        out << fmt::format("Received {} frames, displayed {} frames, Framerate: {} FPS\n",
                           num_frames_received, num_frames_displayed,
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

  // free mDNS resources
  mdns_free();

  // make the rtsp client
  logger.info("Starting RTSP client");
  rtsp_client = std::make_shared<espp::RtspClient>(espp::RtspClient::Config{
      .server_address = mdns_service_address,
      .rtsp_port = mdns_service_port,
      .path = "/mjpeg/1",
      .on_jpeg_frame =
          [](std::shared_ptr<espp::JpegFrame> jpeg_frame) {
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

static size_t frame_buffer_index = 0;
// function for drawing the minimum compressible units
// cppcheck-suppress constParameterCallback
int drawMCUs(JPEGDRAW *pDraw) {
  int iCount = pDraw->iWidth * pDraw->iHeight;
  auto xs = pDraw->x;
  auto ys = pDraw->y;
  auto xe = pDraw->x + pDraw->iWidth - 1;
  auto ye = pDraw->y + pDraw->iHeight - 1;

  uint16_t *dst = (uint16_t *)(frame_buffer_index ? fb1 : fb0);
  uint16_t *src = (uint16_t *)(pDraw->pPixels);
  // copy the pixels from the JPEG draw structure to the framebuffer at the
  // appropriate position
  for (int row = 0; row < pDraw->iHeight; row++) {
    // copy a whole row at a time
    memcpy(&dst[(ys + row) * hal::lcd_width() + xs], &src[row * pDraw->iWidth],
           pDraw->iWidth * sizeof(uint16_t));
  }

  // returning true (1) tells JPEGDEC to continue decoding. Returning false
  // (0) would quit decoding immediately.
  return 1;
}

bool display_task_fn(std::mutex &m, std::condition_variable &cv) {
  // the original (max) image size is 1600x1200, but the S3 BOX has a resolution of 320x240
  // wait on the queue until we have an image ready to display
  static JPEGDEC jpeg;
  static espp::Logger logger({.tag = "Decoder", .level = espp::Logger::Verbosity::WARN});

  std::shared_ptr<espp::JpegFrame> image;
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
  // update to the current frame buffer index
  frame_buffer_index = frame_buffer_index ^ 0x01;
  if (jpeg.openRAM((uint8_t *)(image_data.data()), image_data.size(), drawMCUs)) {
    logger.debug("Image size: {} x {}, orientation: {}, bpp: {}", jpeg.getWidth(), jpeg.getHeight(),
                 jpeg.getOrientation(), jpeg.getBpp());
    jpeg.setPixelType(RGB565_BIG_ENDIAN);
    // decode the JPEG image
    if (!jpeg.decode(0, 0, JPEG_USES_DMA)) {
      logger.debug("Error decoding");
    } else {
      num_frames_displayed += 1;
      // push the frame for rendering
      push_frame(frame_buffer_index ? fb1 : fb0);
    }
  } else {
    logger.error("error opening jpeg image");
  }
  auto end = std::chrono::high_resolution_clock::now();
  elapsed = std::chrono::duration<float>(end - start).count();
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
      } else if (a->addr.type == ESP_IPADDR_TYPE_V4) {
        printf("  A   : " IPSTR "\n", IP2STR(&(a->addr.u_addr.ip4)));
      }
      a = a->next;
    }
    r = r->next;
  }
}

bool find_mdns_service(const char *service_name, const char *proto, std::string &host, int &port,
                       int timeout_ms) {
  logger.debug("Query PTR: {}.{}.local", service_name, proto);

  mdns_result_t *results = NULL;
  int max_results = 20;
  esp_err_t err = mdns_query_ptr(service_name, proto, timeout_ms, max_results, &results);
  if (err) {
    logger.error("Query Failed");
    return false;
  }
  if (!results) {
    logger.info("No results found!");
    return false;
  }

  bool found_service = false;
  mdns_print_results(results);
  // now set the host ip address string and port number from the results
  mdns_result_t *r = results;
  while (r) {
    port = 0;     // reset port to 0 for each result
    host.clear(); // reset host string for each result
    mdns_ip_addr_t *addr = r->addr;
    while (addr) {
      if (addr->addr.type == ESP_IPADDR_TYPE_V6) {
        host = fmt::format("{:02x}:{:02x}:{:02x}:{:02x}:{:02x}:{:02x}",
                           IPV62STR(addr->addr.u_addr.ip6));
        break;
      } else if (addr->addr.type == ESP_IPADDR_TYPE_V4) {
        host = fmt::format("{}.{}.{}.{}", IP2STR(&(addr->addr.u_addr.ip4)));
        break;
      }
      addr = addr->next;
    }
    if (r->port) {
      port = r->port;
    }
    // if we found a valid host and port, we can break out of the loop
    if (port > 0 && !host.empty()) {
      found_service = true;
      break;
    }
    // we got here, so we didn't find a viable service in this result, so go to
    // the next one
    r = r->next;
  }
  mdns_query_results_free(results);
  return found_service;
}

/// Video related functions:

bool initialize_video() {
  if (video_queue_ || video_task_) {
    return true;
  }

  video_queue_ = xQueueCreate(1, sizeof(uint16_t *));
  using namespace std::placeholders;
  video_task_ = espp::Task::make_unique({
      .callback = std::bind(video_task_callback, _1, _2, _3),
      .task_config =
          {.name = "video task", .stack_size_bytes = 4 * 1024, .priority = 20, .core_id = 1},
  });
  video_task_->start();
  return true;
}

void clear_screen() {
  static int buffer = 0;
  xQueueSend(video_queue_, &buffer, portMAX_DELAY);
}

void IRAM_ATTR push_frame(const void *frame) { xQueueSend(video_queue_, &frame, portMAX_DELAY); }

bool video_task_callback(std::mutex &m, std::condition_variable &cv, bool &task_notified) {
  const void *_frame_ptr;
  if (xQueueReceive(video_queue_, &_frame_ptr, portMAX_DELAY) != pdTRUE) {
    return false;
  }
  static constexpr int num_lines_to_write = num_rows_in_vram;
  using Pixel = hal::Pixel;

  static auto &hw = hal::get();

  auto lcd_height = hw.lcd_height();
  auto lcd_width = hw.lcd_width();
  int x_offset = 0;
  int y_offset = 0;
  DisplayDriver::get_offset(x_offset, y_offset);

  static uint16_t vram_index = 0; // has to be static so that it persists between calls

  // special case: if _frame_ptr is null, then we simply fill the screen with 0
  if (_frame_ptr == nullptr) {
    for (int y = 0; y < lcd_height; y += num_lines_to_write) {
      Pixel *_buf = (Pixel *)((uint32_t)vram0 * (vram_index ^ 0x01) + (uint32_t)vram1 * vram_index);
      int num_lines = std::min<int>(num_lines_to_write, lcd_height - y);
      // memset the buffer to 0
      memset(_buf, 0, lcd_width * num_lines * sizeof(Pixel));
      hw.write_lcd_lines(x_offset, y + y_offset, x_offset + lcd_width - 1,
                         y + y_offset + num_lines - 1, (uint8_t *)&_buf[0], 0);
      vram_index = vram_index ^ 0x01;
    }

    // now return
    return false;
  }

  for (int y = 0; y < lcd_height; y += num_lines_to_write) {
    uint16_t *_buf =
        (uint16_t *)((uint32_t)vram0 * (vram_index ^ 0x01) + (uint32_t)vram1 * vram_index);
    int num_lines = std::min<int>(num_lines_to_write, lcd_height - y);
    const uint16_t *_frame = (const uint16_t *)_frame_ptr;
    for (int i = 0; i < num_lines; i++) {
      // write two pixels (32 bits) at a time because it's faster
      for (int j = 0; j < lcd_width; j += 2) {
        uint32_t *src = (uint32_t *)&_frame[(y + i) * lcd_width + j];
        uint32_t *dst = (uint32_t *)&_buf[i * lcd_width + j];
        dst[0] = src[0]; // copy two pixels (32 bits) at a time
      }
    }
    hw.write_lcd_lines(x_offset, y + y_offset, x_offset + lcd_width - 1,
                       y + y_offset + num_lines - 1, (uint8_t *)&_buf[0], 0);
    vram_index = vram_index ^ 0x01;
  }

  return false;
}
