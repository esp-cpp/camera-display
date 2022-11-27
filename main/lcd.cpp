#include "lcd.hpp"

#include "hal/spi_types.h"
#include "driver/spi_master.h"

#include "display.hpp"
#include "st7789.hpp"

static spi_device_handle_t spi;

static constexpr size_t NUM_ROWS_IN_FRAME_BUFFER = 50;
static constexpr size_t display_width = 320;
static constexpr size_t display_height = 240;
static constexpr size_t pixel_buffer_size = display_width*NUM_ROWS_IN_FRAME_BUFFER;
std::shared_ptr<espp::Display> display;

static const int spi_queue_size = 7;
static spi_transaction_t ts_[spi_queue_size];
static size_t ts_index = 0;
static size_t num_queued_trans = 0;

static constexpr size_t frame_buffer_size = (((320) * 2) * 240);
static uint8_t *frame_buffer0;
static uint8_t *frame_buffer1;

// the user flag for the callbacks does two things:
// 1. Provides the GPIO level for the data/command pin, and
// 2. Sets some bits for other signaling (such as LVGL FLUSH)
static constexpr int FLUSH_BIT = (1 << (int)espp::display_drivers::Flags::FLUSH_BIT);
static constexpr int DC_LEVEL_BIT = (1 << (int)espp::display_drivers::Flags::DC_LEVEL_BIT);
static constexpr int DC_PIN_NUM = 4;

// This function is called (in irq context!) just before a transmission starts.
// It will set the D/C line to the value indicated in the user field
// (DC_LEVEL_BIT).
static void IRAM_ATTR lcd_spi_pre_transfer_callback(spi_transaction_t *t)
{
    uint32_t user_flags = (uint32_t)(t->user);
    bool dc_level = user_flags & DC_LEVEL_BIT;
    gpio_set_level((gpio_num_t)DC_PIN_NUM, dc_level);
}

// This function is called (in irq context!) just after a transmission ends. It
// will indicate to lvgl that the next flush is ready to be done if the
// FLUSH_BIT is set.
static void IRAM_ATTR lcd_spi_post_transfer_callback(spi_transaction_t *t)
{
    uint16_t user_flags = (uint32_t)(t->user);
    bool should_flush = user_flags & FLUSH_BIT;
    if (should_flush) {
        lv_disp_t * disp = _lv_refr_get_disp_refreshing();
        lv_disp_flush_ready(disp->driver);
    }
}

extern "C" void IRAM_ATTR lcd_write(const uint8_t *data, size_t length, uint32_t user_data) {
    if (length == 0) {
        return;
    }
    esp_err_t ret;
    static spi_transaction_t t;
    memset(&t, 0, sizeof(t));
    t.length = length * 8;
    t.tx_buffer = data;
    t.user = (void*)user_data;
    ret=spi_device_polling_transmit(spi, &t);
}

static void lcd_wait_lines() {
    spi_transaction_t *rtrans;
    esp_err_t ret;
    // Wait for all transactions to be done and get back the results.
    while (num_queued_trans) {
        // fmt::print("Waiting for {} lines\n", num_queued_trans);
        ret=spi_device_get_trans_result(spi, &rtrans, portMAX_DELAY);
        if (ret != ESP_OK) {
            fmt::print("Could not get trans result: {} '{}'\n", ret, esp_err_to_name(ret));
        }
        num_queued_trans--;
        //We could inspect rtrans now if we received any info back. The LCD is treated as write-only, though.
    }
}

void IRAM_ATTR lcd_send_lines(int xs, int ys, int xe, int ye, const uint8_t *data, uint32_t user_data) {
    // if we haven't waited by now, wait here...
    lcd_wait_lines();
    esp_err_t ret;
    //Transaction descriptors. Declared static so they're not allocated on the stack; we need this memory even when this
    //function is finished because the SPI driver needs access to it even while we're already calculating the next line.
    static spi_transaction_t trans[6];
    //In theory, it's better to initialize trans and data only once and hang on to the initialized
    //variables. We allocate them on the stack, so we need to re-init them each call.
    for (int i=0; i<6; i++) {
        memset(&trans[i], 0, sizeof(spi_transaction_t));
        if ((i&1)==0) {
            //Even transfers are commands
            trans[i].length=8;
            trans[i].user=(void*)0;
        } else {
            //Odd transfers are data
            trans[i].length=8*4;
            trans[i].user=(void*)DC_LEVEL_BIT;
        }
        trans[i].flags=SPI_TRANS_USE_TXDATA;
    }
    size_t length = (xe-xs+1)*(ye-ys+1)*2;
    trans[0].tx_data[0]=(uint8_t)espp::St7789::Command::caset;
    trans[1].tx_data[0]=(xs)>> 8;
    trans[1].tx_data[1]=(xs)&0xff;
    trans[1].tx_data[2]=(xe)>>8;
    trans[1].tx_data[3]=(xe)&0xff;
    trans[2].tx_data[0]=(uint8_t)espp::St7789::Command::raset;
    trans[3].tx_data[0]=(ys)>>8;
    trans[3].tx_data[1]=(ys)&0xff;
    trans[3].tx_data[2]=(ye)>>8;
    trans[3].tx_data[3]=(ye)&0xff;
    trans[4].tx_data[0]=(uint8_t)espp::St7789::Command::ramwr;
    trans[5].tx_buffer=data;
    trans[5].length=length*8;
    // undo SPI_TRANS_USE_TXDATA flag
    trans[5].flags=0;
    // we need to keep the dc bit set, but also add our flags
    trans[5].user = (void*)(DC_LEVEL_BIT | user_data);
    //Queue all transactions.
    for (int i=0; i<6; i++) {
        ret=spi_device_queue_trans(spi, &trans[i], portMAX_DELAY);
        if (ret != ESP_OK) {
            fmt::print("Couldn't queue trans: {} '{}'\n", ret, esp_err_to_name(ret));
        } else {
            num_queued_trans++;
        }
    }
    //When we are here, the SPI driver is busy (in the background) getting the
    //transactions sent. That happens mostly using DMA, so the CPU doesn't have
    //much to do here. We're not going to wait for the transaction to finish
    //because we may as well spend the time calculating the next line. When that
    //is done, we can call send_line_finish, which will wait for the transfers
    //to be done and check their status.
}

uint16_t* get_vram0() {
    return display->vram0();
}

uint16_t* get_vram1() {
    return display->vram1();
}

uint8_t* get_frame_buffer0() {
    return frame_buffer0;
}

uint8_t* get_frame_buffer1() {
    return frame_buffer1;
}

static spi_bus_config_t buscfg = {
    .mosi_io_num=GPIO_NUM_6,
    .miso_io_num=-1,
    .sclk_io_num=GPIO_NUM_7,
    .quadwp_io_num=-1,
    .quadhd_io_num=-1,
    .max_transfer_sz=display_width * display_height * 2 + 8
};
static spi_device_interface_config_t devcfg = {
    .mode=0,
    .clock_speed_hz=60*1000*1000,
    .input_delay_ns=0,
    .spics_io_num=GPIO_NUM_5,
    .queue_size=spi_queue_size,
    .pre_cb=lcd_spi_pre_transfer_callback,
    .post_cb=lcd_spi_post_transfer_callback,
};

void lcd_init() {
  static bool initialized = false;
  if (initialized) {
    return;
  }
  esp_err_t ret;
  //Initialize the SPI bus
  ret=spi_bus_initialize(SPI2_HOST, &buscfg, SPI_DMA_CH_AUTO);
  ESP_ERROR_CHECK(ret);
  //Attach the LCD to the SPI bus
  ret=spi_bus_add_device(SPI2_HOST, &devcfg, &spi);
  ESP_ERROR_CHECK(ret);
  // initialize the controller
  espp::St7789::initialize(espp::display_drivers::Config{
      .lcd_write = lcd_write,
      .lcd_send_lines = lcd_send_lines,
      .reset_pin = (gpio_num_t)48,
      .data_command_pin = (gpio_num_t)DC_PIN_NUM,
      .backlight_pin = (gpio_num_t)45,
      .backlight_on_value = true,
      .invert_colors = true,
      .mirror_x = true,
      .mirror_y = true,
    });
  // initialize the display / lvgl
  using namespace std::chrono_literals;
  display = std::make_shared<espp::Display>(espp::Display::AllocatingConfig{
      .width = display_width,
      .height = display_height,
      .pixel_buffer_size = pixel_buffer_size,
      .flush_callback = espp::St7789::flush,
      .update_period = 5ms,
      .double_buffered = true,
      .allocation_flags = MALLOC_CAP_8BIT | MALLOC_CAP_DMA,
      .rotation = espp::Display::Rotation::LANDSCAPE,
      .software_rotation_enabled = true,
    });
  // frame_buffer0 = (uint8_t*)heap_caps_malloc(frame_buffer_size, MALLOC_CAP_8BIT | MALLOC_CAP_SPIRAM);
  // frame_buffer1 = (uint8_t*)heap_caps_malloc(frame_buffer_size, MALLOC_CAP_8BIT | MALLOC_CAP_SPIRAM);
  initialized = true;
}
