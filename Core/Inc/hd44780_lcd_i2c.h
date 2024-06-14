#include "stm32f3xx_hal.h"
//#include "hd44780_lcd_i2c.c"

#define MODULE_ADDRESS 			0x4E

void LCD_init(I2C_HandleTypeDef* hi2c, uint8_t num_of_lines);
void LCD_putchar(I2C_HandleTypeDef* hi2c, char data);
void LCD_printf(I2C_HandleTypeDef* hi2c, char *data);
void LCD_clear(I2C_HandleTypeDef* hi2c);
void LCD_set_position(I2C_HandleTypeDef* hi2c, uint8_t col, uint8_t row);
