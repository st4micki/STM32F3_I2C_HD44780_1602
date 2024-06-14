#include "stm32f3xx_hal.h"
//#include "hd44780_lcd_i2c.c"

void LCD_init(I2C_HandleTypeDef* hi2c, uint8_t num_of_lines);
void LCD_putchar(I2C_HandleTypeDef* hi2c, char data);
void LCD_printf(I2C_HandleTypeDef* hi2c, char *data);
