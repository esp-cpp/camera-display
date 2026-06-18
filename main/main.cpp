#include "sdkconfig.h"

#include <algorithm>
#include <atomic>
#include <chrono>
#include <deque>
#include <vector>

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
static std::atomic<int> num_audio_frames_received{0};
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
static std::atomic<bool> rtsp_rediscovery_requested{false};
static std::atomic<int> active_audio_track_id{-1};
static std::atomic<int> active_audio_channels{0};
static std::atomic<int> active_audio_sample_rate{0};
static std::atomic<int> active_audio_output_sample_rate{0};
static std::atomic<float> desired_audio_volume_percent{100.0f};

static constexpr int num_rows_in_vram = 16;
static constexpr size_t vram_size = hal::lcd_width() * num_rows_in_vram * sizeof(hal::Pixel);
static constexpr size_t fb_size = hal::lcd_width() * hal::lcd_height() * sizeof(hal::Pixel);
static constexpr size_t min_rtp_port = 10000;
static constexpr size_t max_rtp_port = 20000;
static std::mutex jpeg_mutex;
static std::condition_variable jpeg_cv;
static constexpr size_t MAX_JPEG_FRAMES = 3;
static std::deque<std::shared_ptr<espp::JpegFrame>> jpeg_frames;
static void clear_jpeg_frames();

#if CONFIG_HARDWARE_BOX || CONFIG_HARDWARE_TDECK
static constexpr bool bsp_supports_pcm_audio_output = true;
#else
static constexpr bool bsp_supports_pcm_audio_output = false;
#endif

#if CONFIG_HARDWARE_BOX || CONFIG_HARDWARE_TDECK || CONFIG_HARDWARE_WS_S3_TOUCH
static constexpr bool bsp_supports_touchscreen = true;
#else
static constexpr bool bsp_supports_touchscreen = false;
#endif

static void set_audio_output_volume(float volume_percent) {
  auto clamped_volume_percent = std::clamp(volume_percent, 0.0f, 100.0f);
  desired_audio_volume_percent = clamped_volume_percent;
  if constexpr (bsp_supports_pcm_audio_output) {
    hal::get().volume(clamped_volume_percent);
  }
}

static float get_audio_volume_for_touch_x(uint16_t touch_x) {
  constexpr float max_touch_x =
      hal::lcd_width() > 1 ? static_cast<float>(hal::lcd_width() - 1) : 1.0f;
  auto clamped_touch_x = std::clamp(static_cast<float>(touch_x), 0.0f, max_touch_x);
  return (clamped_touch_x / max_touch_x) * 100.0f;
}

static std::pair<size_t, size_t> get_next_rtp_port_pair() {
  static std::atomic<size_t> next_rtp_slot{0};
  constexpr size_t num_rtp_slots = ((max_rtp_port - min_rtp_port) / 2) + 1;
  size_t rtp_port = min_rtp_port + 2 * (next_rtp_slot.fetch_add(1) % num_rtp_slots);
  return {rtp_port, rtp_port + 1};
}

bool start_rtsp_client(std::mutex &m, std::condition_variable &cv, bool &task_notified);
int drawMCUs(JPEGDRAW *pDraw);
bool display_task_fn(std::mutex &m, std::condition_variable &cv);
void mdns_print_results(mdns_result_t *results);
bool find_mdns_service(const char *service_name, const char *proto, std::string &host, int &port,
                       int timeout_ms = 3000);
void reset_audio_stream_state();
void configure_audio_playback(const espp::RtspClient &client);
void handle_rtsp_frame(int track_id, std::vector<uint8_t> &&data);
void clear_audio_output_buffer();

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
    heap_caps_free(fb0);
    heap_caps_free(fb1);
    return;
  }

  logger.info("Allocated frame buffers: fb0 = {} B, fb1 = {} B", fb_size, fb_size);
  logger.info("Allocated VRAM: vram0 = {} B, vram1 = {} B ({} rows each)", vram_size, vram_size,
              num_rows_in_vram);
  logger.info("DMA heap after VRAM alloc: free={} B, largest_block={} B",
              heap_caps_get_free_size(MALLOC_CAP_DMA),
              heap_caps_get_largest_free_block(MALLOC_CAP_DMA));

  // initialize the video task
  if (!initialize_video()) {
    logger.error("Could not initialize video task");
    heap_caps_free(fb0);
    heap_caps_free(fb1);
    heap_caps_free(vram0);
    heap_caps_free(vram1);
    return;
  }

  // clear the screen
  logger.info("Clearing screen");
  clear_screen();

  if constexpr (bsp_supports_touchscreen && bsp_supports_pcm_audio_output) {
    auto touch_callback = [](const auto &touch) {
      auto &hw = hal::get();
      auto touchpad_data = hw.touchpad_convert(touch);
      if (touchpad_data.num_touch_points == 0) {
        return;
      }
      set_audio_output_volume(get_audio_volume_for_touch_x(touchpad_data.x));
    };
    if (!hw.initialize_touch(touch_callback)) {
      logger.warn("Could not initialize touch input for audio volume control");
    }
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
       // .phy_rate = WIFI_PHY_RATE_MCS5_SGI,
       .num_connect_retries = CONFIG_ESP_MAXIMUM_RETRY,
       .on_connected = []() { logger.info("Connected to WiFi, waiting for IP address"); },
       .on_disconnected =
           []() {
             logger.info("Disconnected from WiFi, stopping task");
             // ensure the rtsp start function is not running
             start_rtsp_task.reset();
             rtsp_rediscovery_requested = false;
             reset_audio_stream_state();
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
        if (auto audio_track_id = active_audio_track_id.load(); audio_track_id >= 0) {
          auto input_rate = active_audio_sample_rate.load();
          auto output_rate = active_audio_output_sample_rate.load();
          out << fmt::format("Audio track {}: {} frames at {} Hz", audio_track_id,
                             num_audio_frames_received.load(), input_rate);
          if (output_rate > 0 && output_rate != input_rate) {
            out << fmt::format(" -> {} Hz", output_rate);
          }
          out << fmt::format(" ({} channel{})\n", active_audio_channels.load(),
                             active_audio_channels.load() == 1 ? "" : "s");
        }
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
  reset_audio_stream_state();
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
    mdns_free();
    return true;
  }
  logger.info("mDNS hostname set to '{}'", hostname);
  err = mdns_instance_name_set("Camera Display");
  if (err != ESP_OK) {
    logger.error("Could not set mDNS instance name: {}", err);
    mdns_free();
    return true;
  }
  auto wait_for_stop = [&](auto duration) {
    std::unique_lock<std::mutex> lk(m);
    auto stop_requested = cv.wait_for(lk, duration, [&task_notified] { return task_notified; });
    task_notified = false;
    return stop_requested;
  };

  while (!task_notified) {
    std::string mdns_service_address;
    int mdns_service_port = 0;
    bool found_mdns_server = false;
    while (!found_mdns_server && !task_notified) {
      logger.info("Searching for RTSP server...");
      found_mdns_server =
          find_mdns_service("_rtsp", "_tcp", mdns_service_address, mdns_service_port, 3000);
    }
    if (task_notified) {
      break;
    }

    logger.info("Found RTSP server: {}:{}", mdns_service_address, mdns_service_port);
    rtsp_rediscovery_requested = false;

    logger.info("Starting RTSP client");
    auto client = std::make_shared<espp::RtspClient>(espp::RtspClient::Config{
        .server_address = mdns_service_address,
        .rtsp_port = mdns_service_port,
        .path = "/mjpeg/1",
        .on_frame = handle_rtsp_frame,
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
              if (num_frames_received.fetch_add(1) == 0) {
                logger.info("Received first RTSP JPEG frame");
              }
            },
        .on_connection_lost =
            []() {
              logger.warn("RTSP server disappeared, returning to discovery");
              rtsp_rediscovery_requested = true;
            },
        .log_level = espp::Logger::Verbosity::ERROR,
    });
    rtsp_client = client;

    std::error_code ec;
    for (int connect_attempt = 1; connect_attempt <= 3; connect_attempt++) {
      client->connect(ec);
      if (!ec) {
        break;
      }
      logger.error("Error connecting to server (attempt {}/3): {}", connect_attempt, ec.message());
      if (connect_attempt == 3) {
        break;
      }
      if (wait_for_stop(1s)) {
        task_notified = true;
        break;
      }
      ec.clear();
    }
    if (!ec) {
      client->describe(ec);
      if (!ec) {
        configure_audio_playback(*client);
      }
    }
    if (!ec) {
      auto [rtp_port, rtcp_port] = get_next_rtp_port_pair();
      logger.info("Using RTP/RTCP client ports {}-{}", rtp_port, rtcp_port);
      client->setup(rtp_port, rtcp_port, 5s, ec);
    }
    if (!ec) {
      client->play(ec);
    }

    if (ec) {
      logger.error("RTSP startup failed: {}", ec.message());
    } else {
      // reset the per-session frame counters so the stats command reports the
      // framerate for the current session rather than across all reconnects
      num_frames_received = 0;
      num_frames_displayed = 0;
      connected_time = std::chrono::high_resolution_clock::now();
      logger.info("RTSP playback started");
      bool stop_requested = false;
      while (!task_notified && !rtsp_rediscovery_requested.load()) {
        if (wait_for_stop(500ms)) {
          stop_requested = true;
          break;
        }
      }
      if (stop_requested) {
        break;
      }
    }

    std::error_code disconnect_ec;
    client->disconnect(disconnect_ec);
    rtsp_client.reset();
    clear_jpeg_frames();
    clear_screen();
    reset_audio_stream_state();

    if (task_notified) {
      break;
    }
    if (rtsp_rediscovery_requested.exchange(false)) {
      logger.info("Re-entering RTSP discovery");
      continue;
    }

    logger.info("Retrying RTSP discovery in 1s...");
    if (wait_for_stop(1s)) {
      break;
    }
  }

  rtsp_client.reset();
  clear_jpeg_frames();
  reset_audio_stream_state();
  mdns_free();
  logger.info("Stopping RTSP client task");
  return true;
}

void reset_audio_stream_state() {
  active_audio_track_id = -1;
  active_audio_channels = 0;
  active_audio_sample_rate = 0;
  active_audio_output_sample_rate = 0;
  num_audio_frames_received = 0;
  clear_audio_output_buffer();
}

#if CONFIG_HARDWARE_BOX || CONFIG_HARDWARE_TDECK
void clear_audio_output_buffer() {}

static uint32_t get_audio_output_sample_rate(uint32_t input_sample_rate_hz) {
#if CONFIG_HARDWARE_BOX
  return std::max<uint32_t>(input_sample_rate_hz, 48000);
#else
  return input_sample_rate_hz;
#endif
}

static bool ensure_audio_output_ready(uint32_t sample_rate_hz) {
  auto &hw = hal::get();
  if (!hw.initialize_sound(sample_rate_hz)) {
    logger.error("Could not initialize BSP audio output");
    return false;
  }
  if (hw.audio_sample_rate() != sample_rate_hz) {
    hw.audio_sample_rate(sample_rate_hz);
  }
  set_audio_output_volume(desired_audio_volume_percent.load());
  clear_audio_output_buffer();
  return true;
}

static void play_pcm_audio_frame(const uint8_t *data, size_t num_bytes, int channels,
                                 uint32_t input_sample_rate_hz, uint32_t output_sample_rate_hz) {
  constexpr size_t stereo_sample_frame_size = sizeof(int16_t) * 2;
  auto input_frame_size = sizeof(int16_t) * channels;
  if (input_frame_size == 0) {
    return;
  }
  auto aligned_num_bytes = num_bytes - (num_bytes % input_frame_size);
  if (aligned_num_bytes == 0) {
    return;
  }

  if (channels < 1 || channels > 2) {
    static int warned_channels = 0;
    if (warned_channels != channels) {
      logger.warn("Ignoring unsupported RTSP audio with {} channels", channels);
      warned_channels = channels;
    }
    return;
  }

  if (input_sample_rate_hz == 0 || output_sample_rate_hz == 0) {
    static bool warned_sample_rate = false;
    if (!warned_sample_rate) {
      logger.warn("Ignoring RTSP audio with invalid sample rate conversion {} -> {} Hz",
                  input_sample_rate_hz, output_sample_rate_hz);
      warned_sample_rate = true;
    }
    return;
  }

  auto *src = reinterpret_cast<const int16_t *>(data);
  auto input_frames = aligned_num_bytes / input_frame_size;
  if (input_frames == 0) {
    return;
  }

  static thread_local std::vector<uint8_t> stereo_frame;
  auto make_stereo_frame = [&](size_t output_frames) -> int16_t * {
    stereo_frame.resize(output_frames * stereo_sample_frame_size);
    return reinterpret_cast<int16_t *>(stereo_frame.data());
  };

  if (input_sample_rate_hz == output_sample_rate_hz) {
    if (channels == 2) {
      auto stereo_aligned_num_bytes =
          aligned_num_bytes - (aligned_num_bytes % stereo_sample_frame_size);
      if (stereo_aligned_num_bytes > 0) {
        hal::get().play_audio(data, stereo_aligned_num_bytes);
      }
      return;
    }

    auto *dst = make_stereo_frame(input_frames);
    for (size_t i = 0; i < input_frames; ++i) {
      dst[2 * i] = src[i];
      dst[2 * i + 1] = src[i];
    }
    hal::get().play_audio(stereo_frame.data(), stereo_frame.size());
    return;
  }

  size_t output_frames =
      std::max<size_t>(1, (static_cast<uint64_t>(input_frames) * output_sample_rate_hz +
                           (input_sample_rate_hz / 2)) /
                              input_sample_rate_hz);
  auto *dst = make_stereo_frame(output_frames);

  for (size_t out_index = 0; out_index < output_frames; ++out_index) {
    uint64_t scaled_position = static_cast<uint64_t>(out_index) * input_sample_rate_hz;
    size_t base_index = std::min<size_t>(scaled_position / output_sample_rate_hz, input_frames - 1);
    size_t next_index = std::min(base_index + 1, input_frames - 1);
    uint32_t fraction = scaled_position % output_sample_rate_hz;

    auto interpolate_channel = [&](int channel) -> int16_t {
      int src_channel = std::min(channel, channels - 1);
      int32_t sample0 = src[base_index * channels + src_channel];
      int32_t sample1 = src[next_index * channels + src_channel];
      int64_t delta = static_cast<int64_t>(sample1) - sample0;
      int64_t interpolated =
          static_cast<int64_t>(sample0) +
          ((delta * fraction + (output_sample_rate_hz / 2)) / output_sample_rate_hz);
      return static_cast<int16_t>(std::clamp<int64_t>(interpolated, INT16_MIN, INT16_MAX));
    };

    dst[2 * out_index] = interpolate_channel(0);
    dst[2 * out_index + 1] = interpolate_channel(channels > 1 ? 1 : 0);
  }

  hal::get().play_audio(stereo_frame.data(), stereo_frame.size());
}

static void play_pcm_audio_frame(const uint8_t *data, size_t num_bytes, int channels) {
  auto input_sample_rate_hz = static_cast<uint32_t>(std::max(active_audio_sample_rate.load(), 0));
  auto output_sample_rate_hz =
      static_cast<uint32_t>(std::max(active_audio_output_sample_rate.load(), 0));
  play_pcm_audio_frame(data, num_bytes, channels, input_sample_rate_hz, output_sample_rate_hz);
}

static void configure_audio_output(uint32_t input_sample_rate_hz) {
  auto output_sample_rate_hz = get_audio_output_sample_rate(input_sample_rate_hz);
  if (!ensure_audio_output_ready(output_sample_rate_hz)) {
    return;
  }
  active_audio_output_sample_rate = static_cast<int>(output_sample_rate_hz);
  logger.info("Configured audio output at {} Hz for {} Hz input", output_sample_rate_hz,
              input_sample_rate_hz);
}
#else
void clear_audio_output_buffer() {}
#endif

void configure_audio_playback(const espp::RtspClient &client) {
  reset_audio_stream_state();

  const auto &tracks = client.tracks();
  auto audio_track_it = std::find_if(tracks.begin(), tracks.end(),
                                     [](const auto &track) { return track.media_type == "audio"; });
  if (audio_track_it == tracks.end()) {
    logger.info("RTSP session has no audio track");
    return;
  }

  if (audio_track_it->encoding_name != "L16") {
    logger.warn("RTSP audio track {} uses unsupported encoding '{}'", audio_track_it->track_id,
                audio_track_it->encoding_name);
    return;
  }
  if (audio_track_it->clock_rate <= 0) {
    logger.warn("RTSP audio track {} did not advertise a valid sample rate",
                audio_track_it->track_id);
    return;
  }

  auto channels = std::max(audio_track_it->channels, 1);

  if constexpr (bsp_supports_pcm_audio_output) {
#if CONFIG_HARDWARE_BOX || CONFIG_HARDWARE_TDECK
    configure_audio_output(audio_track_it->clock_rate);
    if (active_audio_output_sample_rate.load() <= 0) {
      return;
    }
#endif
    active_audio_track_id = audio_track_it->track_id;
    active_audio_channels = channels;
    active_audio_sample_rate = audio_track_it->clock_rate;
    auto output_rate = active_audio_output_sample_rate.load();
    logger.info("Configured RTSP audio playback: track {}, {} Hz{}, {} channel{}",
                audio_track_it->track_id, audio_track_it->clock_rate,
                output_rate > 0 && output_rate != audio_track_it->clock_rate
                    ? fmt::format(" -> {} Hz", output_rate)
                    : "",
                channels, channels == 1 ? "" : "s");
  } else {
    logger.info(
        "RTSP session includes audio track {}, but the selected BSP has no PCM audio output",
        audio_track_it->track_id);
  }
}

void handle_rtsp_frame(int track_id, std::vector<uint8_t> &&data) {
  auto audio_track_id = active_audio_track_id.load();
  if (audio_track_id < 0 || track_id != audio_track_id || data.empty()) {
    return;
  }

#if CONFIG_HARDWARE_BOX || CONFIG_HARDWARE_TDECK
  play_pcm_audio_frame(data.data(), data.size(), std::max(active_audio_channels.load(), 1));
#endif
  if (num_audio_frames_received.fetch_add(1) == 0) {
    logger.info("Received first RTSP audio frame");
  }
}

static size_t frame_buffer_index = 0;
// function for drawing the minimum compressible units
// cppcheck-suppress constParameterCallback
int drawMCUs(JPEGDRAW *pDraw) {
  // int iCount = pDraw->iWidth * pDraw->iHeight;
  auto xs = pDraw->x;
  auto ys = pDraw->y;
  // auto xe = pDraw->x + pDraw->iWidth - 1;
  // auto ye = pDraw->y + pDraw->iHeight - 1;

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
      logger.warn("Error decoding JPEG frame");
    } else {
      if (num_frames_displayed.fetch_add(1) == 0) {
        logger.info("Displayed first RTSP frame");
      }
      // push the frame for rendering
      push_frame(frame_buffer_index ? fb1 : fb0);
    }
  } else {
    logger.error("error opening jpeg image");
  }
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
        host = fmt::format("{:04x}:{:04x}:{:04x}:{:04x}:{:04x}:{:04x}:{:04x}:{:04x}",
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

  video_queue_ = xQueueCreate(1, sizeof(const void *));
  using namespace std::placeholders;
  video_task_ = espp::Task::make_unique({
      .callback = std::bind(video_task_callback, _1, _2, _3),
      .task_config =
          {.name = "video task", .stack_size_bytes = 4 * 1024, .priority = 20, .core_id = 1},
  });
  video_task_->start();
  return true;
}

void clear_jpeg_frames() {
  std::lock_guard<std::mutex> lock(jpeg_mutex);
  jpeg_frames.clear();
}

void clear_screen() {
  // a null frame pointer signals the video task to blank the screen
  const void *null_frame = nullptr;
  xQueueSend(video_queue_, &null_frame, portMAX_DELAY);
}

static void IRAM_ATTR push_frame(const void *frame) {
  xQueueSend(video_queue_, &frame, portMAX_DELAY);
}

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
  static auto &display_driver = hw.display_driver();
  display_driver->get_offset(x_offset, y_offset);

  static uint16_t vram_index = 0; // has to be static so that it persists between calls

  // special case: if _frame_ptr is null, then we simply fill the screen with 0
  if (_frame_ptr == nullptr) {
    for (int y = 0; y < lcd_height; y += num_lines_to_write) {
      Pixel *_buf = (Pixel *)(vram_index ? vram1 : vram0);
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
    uint16_t *_buf = (uint16_t *)(vram_index ? vram1 : vram0);
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
