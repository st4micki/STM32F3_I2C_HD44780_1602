#include "stm32f3xx_hal.h"
//#include "hd44780_lcd_i2c.c"
#ifndef HD44780_LCD_I2C_H
#define HD44780_LCD_I2C_H
#define MODULE_ADDRESS 	0x4E
#define ALIGN_RIGHT		1
#define ALIGN_MIDDLE	2
#define MAX_COLUMN		15

typedef struct LCD_HandleTypeDef {
	I2C_HandleTypeDef *hi2c;
	TIM_HandleTypeDef *htim;
	uint8_t i2c_address;
	uint8_t current_col;
	uint8_t current_row;
	uint8_t num_of_rows;
}LCD_HandleTypeDef;

void LCD_init(LCD_HandleTypeDef* lcd);
void LCD_putchar(LCD_HandleTypeDef* lcd, char data);
void LCD_printf(LCD_HandleTypeDef* lcd, char *data);
void LCD_clear(LCD_HandleTypeDef* lcd);
void LCD_set_position(LCD_HandleTypeDef* lcd, uint8_t col, uint8_t row);
void LCD_reset_position(LCD_HandleTypeDef* lcd);
void LCD_printf_align(LCD_HandleTypeDef* lcd, char *data, uint8_t alignment);
#endif
