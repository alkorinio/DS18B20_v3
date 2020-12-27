/*
 * lcd.c
 *
 *  Created on: 15 paź 2020
 *      Author: adam
 */

#include <string.h>
#include "lcd.h"
#include "font.h"
#include "spi.h"

#define PCD8544_FUNCTION_SET		0x20
#define PCD8544_DISP_CONTROL		0x08
#define PCD8544_DISP_NORMAL			0x0c
#define PCD8544_SET_Y				0x40
#define PCD8544_SET_X				0x80
#define PCD8544_H_TC				0x04
#define PCD8544_H_BIAS				0x10
#define PCD8544_H_VOP				0x80

#define LCD_BUFFER_SIZE			(84 * 48 / 8)

static uint8_t lcd_buffer[LCD_BUFFER_SIZE];

void lcd_cmd(uint8_t cmd)
{
	HAL_GPIO_WritePin(GPIOC, LCD_CE_CS_Pin | LCD_DC_Pin, GPIO_PIN_RESET);
	HAL_SPI_Transmit(&hspi2, &cmd, 1, HAL_MAX_DELAY);
	HAL_GPIO_WritePin(GPIOC, LCD_CE_CS_Pin | LCD_DC_Pin, GPIO_PIN_SET);
}

void lcd_setup(void)
{
	HAL_GPIO_WritePin(GPIOC, LCD_RST_Pin, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(GPIOC, LCD_RST_Pin, GPIO_PIN_SET);

	lcd_cmd(PCD8544_FUNCTION_SET | 1);
	lcd_cmd(PCD8544_H_BIAS | 4);
	lcd_cmd(PCD8544_H_VOP | 0x3f);
	lcd_cmd(PCD8544_FUNCTION_SET);
	lcd_cmd(PCD8544_DISP_NORMAL);
}

void lcd_deinit(void)
{
	HAL_GPIO_WritePin(GPIOC, LCD_RST_Pin, GPIO_PIN_RESET);
}

void lcd_clear(void)
{
	memset(lcd_buffer, 0, LCD_BUFFER_SIZE);
}

void lcd_draw_bitmap(const uint8_t* data)
{
	memcpy(lcd_buffer, data, LCD_BUFFER_SIZE);
}

void lcd_draw_text(int row, int col, const char* text)
{
	int i;
	uint8_t* pbuf = &lcd_buffer[row * 84 + col];
	while ((*text) && (pbuf < &lcd_buffer[LCD_BUFFER_SIZE - 6])) {
		int ch = *text++;
		const uint8_t* font = &font_ASCII[ch - ' '][0];
		for (i = 0; i < 5; i++) {
			*pbuf++ = *font++;
		}
		*pbuf++ = 0;
	}
}

inline void lcd_draw_pixel(int x, int y)
{
	lcd_buffer[ x + (y >> 3) * LCD_WIDTH] |= 1 << (y & 7);
}

void lcd_draw_line(int x1, int y1, int x2, int y2)
{
	int dx, dy, sx, sy;
	if (x2 >= x1) {
		dx = x2 - x1;
		sx = 1;
	} else {
		dx = x1 - x2;
		sx = -1;
	}
	if (y2 >= y1) {
		dy = y1 - y2;
		sy = 1;
	} else {
		dy = y2 - y1;
		sy = -1;
	}

	int dx2 = dx << 1;
	int dy2 = dy << 1;
    int err = dx2 + dy2;
    while (1) {
        lcd_draw_pixel(x1, y1);
        if (err >= dy) {
            if (x1 == x2) break;
            err += dy2;
            x1 += sx;
        }
        if (err <= dx) {
            if (y1 == y2) break;
            err += dx2;
            y1 += sy;
        }
    }
}

void lcd_copy(void)
{
	HAL_GPIO_WritePin(GPIOC, LCD_DC_Pin, GPIO_PIN_SET);
	HAL_GPIO_WritePin(GPIOC, LCD_CE_CS_Pin, GPIO_PIN_RESET);
	HAL_SPI_Transmit(&hspi2, lcd_buffer, LCD_BUFFER_SIZE, HAL_MAX_DELAY);
	HAL_GPIO_WritePin(GPIOC, LCD_CE_CS_Pin, GPIO_PIN_SET);
}
