/*
 * hd44780_lcd_i2c.c
 *
 *  Created on: Jun 14, 2024
 *      Author: PC
 */
#include "hd44780_lcd_i2c.h"
#include "stm32f3xx_hal.h"

#define TWO_LINES_ENABLE 		(0x01 << 3)
#define E_PIN_MASK 				0x04
#define RS_PIN_MASK				0x01
#define FUNCTION_SET_4_BIT_MODE 0x20
#define CLEAR_DISPLAY 			0x01
#define STARTUP 				0x30
#define DISPLAY_OFF				0x08
#define INCREMENT_NO_SHIFT		0x06
#define BACKLIGHT_ON			0x08
#define SET_POSITION_MASK		0x80



uint8_t _LCD_get_young_bits(uint8_t data){
	data <<= 4;
	data &= 0xF0;
	return data;

}


uint8_t _LCD_get_old_bits(uint8_t data){
	return data & 0xF0;
}

void _LCD_send_command(I2C_HandleTypeDef* hi2c, uint8_t command){
	uint8_t send[4];
	send[0] = _LCD_get_old_bits(command) | E_PIN_MASK | BACKLIGHT_ON; // older half of the command byte with E pin set to high
	send[1] = _LCD_get_old_bits(command) | BACKLIGHT_ON; // older half of the command byte with E pin set to low
	send[2] = _LCD_get_young_bits(command) | E_PIN_MASK | BACKLIGHT_ON; // younger half of the command byte with E pin set to high
	send[3] = _LCD_get_young_bits(command) | BACKLIGHT_ON; // younger half of the command byte with E pin set to low
	uint16_t send_size = sizeof(send);
	HAL_I2C_Master_Transmit(hi2c, MODULE_ADDRESS, send, send_size, 100);

}

void _LCD_startup(I2C_HandleTypeDef* hi2c){
	uint8_t send[2] = {
			STARTUP | E_PIN_MASK | BACKLIGHT_ON,
			STARTUP | BACKLIGHT_ON
	};
	HAL_Delay(20);
	HAL_I2C_Master_Transmit(hi2c, MODULE_ADDRESS, send, 2, 100);
	HAL_Delay(5);
	HAL_I2C_Master_Transmit(hi2c, MODULE_ADDRESS, send, 2, 100);
	HAL_Delay(1);
	HAL_I2C_Master_Transmit(hi2c, MODULE_ADDRESS, send, 2, 100);


}

void _LCD_set_4_bits(I2C_HandleTypeDef* hi2c, uint8_t num_of_lines){
	uint8_t data[2] = {
			FUNCTION_SET_4_BIT_MODE | E_PIN_MASK | BACKLIGHT_ON,
			FUNCTION_SET_4_BIT_MODE | BACKLIGHT_ON
	};
	HAL_I2C_Master_Transmit(hi2c, MODULE_ADDRESS , data, 2, 100);
	HAL_Delay(1);
	if(num_of_lines == 2)
		_LCD_send_command(hi2c, FUNCTION_SET_4_BIT_MODE | TWO_LINES_ENABLE);
}

void LCD_init(I2C_HandleTypeDef* hi2c, uint8_t num_of_lines){
	_LCD_startup(hi2c);
	HAL_Delay(1);
	_LCD_set_4_bits(hi2c, num_of_lines);
	HAL_Delay(1);
	_LCD_send_command(hi2c, DISPLAY_OFF);
	HAL_Delay(1);
	_LCD_send_command(hi2c, CLEAR_DISPLAY);
	HAL_Delay(1);
	_LCD_send_command(hi2c, INCREMENT_NO_SHIFT);
	HAL_Delay(1);
	_LCD_send_command(hi2c, 0x0C);
	HAL_Delay(1);

}

void LCD_putchar(I2C_HandleTypeDef* hi2c, char data){
	uint8_t send[4];
	send[0] = _LCD_get_old_bits((uint8_t)data) | E_PIN_MASK | RS_PIN_MASK | BACKLIGHT_ON; // older half of the command byte with E pin set to high
	send[1] = _LCD_get_old_bits((uint8_t)data) | RS_PIN_MASK | BACKLIGHT_ON; // older half of the command byte with E pin set to low
	send[2] = _LCD_get_young_bits((uint8_t)data) | E_PIN_MASK | RS_PIN_MASK | BACKLIGHT_ON; // younger half of the command byte with E pin set to high
	send[3] = _LCD_get_young_bits((uint8_t)data) | RS_PIN_MASK | BACKLIGHT_ON; // younger half of the command byte with E pin set to low
	int16_t send_size = sizeof(send);
	HAL_I2C_Master_Transmit(hi2c, MODULE_ADDRESS, send, send_size, 100);

}

void LCD_printf(I2C_HandleTypeDef* hi2c, char *data){
	for(char* i = data; *i != '\0'; i++){
		LCD_putchar(hi2c, *i);
	}
}

void LCD_clear(I2C_HandleTypeDef* hi2c){
	_LCD_send_command(hi2c, CLEAR_DISPLAY);
}

void LCD_set_position(I2C_HandleTypeDef* hi2c, uint8_t col, uint8_t row){
	if((col <= 0x27 && col >= 0) && (row <= 1 && row >=0)){
		uint8_t address = col + row * 0x40;
		address |= SET_POSITION_MASK;
		_LCD_send_command(hi2c, address);

	}



}

