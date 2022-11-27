#pragma once

#include "display.hpp"

void lcd_init();
uint16_t* get_vram0();
uint16_t* get_vram1();
uint8_t* get_frame_buffer0();
uint8_t* get_frame_buffer1();
void lcd_send_lines(int xs, int ys, int xe, int ye, const uint8_t *data, uint32_t user_data);
// std::shared_ptr<espp::Display> get_display();
