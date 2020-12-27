/*
 * lcd.h
 *
 *  Created on: 15 paź 2020
 *      Author: adam
 */

#ifndef INC_LCD_H_
#define INC_LCD_H_

#include <stdint.h>
#include "stm32f1xx.h"
#include <string.h>

#define LCD_WIDTH				84
#define LCD_HEIGHT				48

void lcd_setup(void);
void lcd_cmd(uint8_t cmd);

void lcd_clear(void);
void lcd_draw_bitmap(const uint8_t* data);
void lcd_draw_text(int row, int col, const char* text);
void lcd_draw_pixel(int x, int y);
void lcd_draw_line(int x1, int y1, int x2, int y2);
void lcd_deinit(void);

void lcd_copy(void);

#endif /* INC_LCD_H_ */
