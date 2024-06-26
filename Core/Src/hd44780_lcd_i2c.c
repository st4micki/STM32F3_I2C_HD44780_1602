/*
 * hd44780_lcd_i2c.c
 *
 *  Created on: Jun 14, 2024
 *      Author: PC
 */
#include "hd44780_lcd_i2c.h"
#include "stm32f3xx_hal.h"
#include "stm32f3xx_hal_rcc.h"
#include <math.h>

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
#define CLOCK_NOT_DEFINED		0x00
#define US_BETWEEN_COMMANDS		380




typedef enum {FALSE, TRUE} bool;


bool LCD_delay_us_first_call = TRUE;

void _LCD_delay_us_init(TIM_HandleTypeDef *htim){
	//this function initializes user defined timer
	//calling this function during LCD_Init() lets _LCD_Delay_us() generate accurate delays
	uint32_t apb_freq = HAL_RCC_GetHCLKFreq(); //fetch HCLK frequency. It appears that delays work the same no matter what APB bus the timer is connected to.
	uint16_t new_prescaler = (uint16_t)(apb_freq/1000000); //timer prescaler is set so every increment of CNT take 1 microsecond
	htim->Instance->PSC = new_prescaler - 1;
	htim->Instance->ARR = 0xFFFF - 1;
	htim->Instance->CNT = 0; //just to be sure that timer starts counting from 0,  CNT is reseted here
	HAL_TIM_Base_Start(htim);
	while(htim->Instance->CNT < US_BETWEEN_COMMANDS){} //while is blocking until right ammonunt of time elapses
	htim->Instance->CNT = 0;

}

void _LCD_delay_us(uint32_t us, TIM_HandleTypeDef *htim){
	//this a blocking delay funcions that generates delays measured in microseconds
	uint32_t apb_freq = HAL_RCC_GetHCLKFreq();
	uint16_t new_prescaler = (uint16_t)(apb_freq/1000000); //timer prescaler is set so every increment of CNT take 1 microsecond
	htim->Instance->PSC = new_prescaler - 1;
	htim->Instance->ARR = 0xFFFF - 1;
	htim->Instance->CNT = 0;
	while(htim->Instance->CNT < us){}

}


uint8_t _LCD_get_young_bits(uint8_t data){
	//shifts data's bits to the right by 4 places and masks 4 most significant bits
	data <<= 4;
	data &= 0xF0;
	return data;

}


uint8_t _LCD_get_old_bits(uint8_t data){
	//masks 4 most significant bits, clears 4 less significant bits
	return data & 0xF0;
}

void _LCD_send_command(LCD_HandleTypeDef* lcd,  uint8_t command){
	//sends 8-bit command to the display. Command is split into two 4 bit commands.
	//every 4-bit command has to be sent with E_pin enabled and later disabled
	I2C_HandleTypeDef* hi2c = lcd->hi2c;
	uint8_t address = lcd->i2c_address;
	uint8_t send[4] = {
		_LCD_get_old_bits(command) | E_PIN_MASK | BACKLIGHT_ON, // older half of the command byte with E pin set to high
		_LCD_get_old_bits(command) | BACKLIGHT_ON, // older half of the command byte with E pin set to low
		_LCD_get_young_bits(command) | E_PIN_MASK | BACKLIGHT_ON, // younger half of the command byte with E pin set to high
		_LCD_get_young_bits(command) | BACKLIGHT_ON}; // younger half of the command byte with E pin set to low
	uint16_t send_size = sizeof(send);
	HAL_I2C_Master_Transmit(hi2c, address, send, send_size, 100); //data is transmitted through I2C interface using HAL library function
}

void _LCD_startup(LCD_HandleTypeDef* lcd){
	/*Initializing display by instruction. According to HD44780's data sheet "If the power supply conditions for correctly operating the internal reset circuit are not met, initialization by
	instructions becomes necessary" this is done just in case*/
	TIM_HandleTypeDef* htim = lcd->htim;
	I2C_HandleTypeDef* hi2c = lcd->hi2c;
	uint8_t send[2] = {
			STARTUP | E_PIN_MASK | BACKLIGHT_ON,
			STARTUP | BACKLIGHT_ON
	};
	_LCD_delay_us(16000, htim);
	HAL_I2C_Master_Transmit(hi2c, MODULE_ADDRESS, send, 2, 100);
	_LCD_delay_us(4100, htim);
	HAL_I2C_Master_Transmit(hi2c, MODULE_ADDRESS, send, 2, 100);
	_LCD_delay_us(110, htim);
	HAL_I2C_Master_Transmit(hi2c, MODULE_ADDRESS, send, 2, 100);


}

void _LCD_set_4_bits(LCD_HandleTypeDef* lcd){
	//sets a mode that lets the display take 4-bit commands by sending 8-bit command first
	I2C_HandleTypeDef* hi2c = lcd->hi2c;
	uint8_t num_of_lines = lcd->num_of_rows;
	uint8_t data[2] = {
			FUNCTION_SET_4_BIT_MODE | E_PIN_MASK | BACKLIGHT_ON,
			FUNCTION_SET_4_BIT_MODE | BACKLIGHT_ON};
	HAL_I2C_Master_Transmit(hi2c, MODULE_ADDRESS , data, 2, 100);
	HAL_Delay(1);
	if(num_of_lines == 2)
		// it is possible to 2 lines of text on the display command is sent with special mask
		_LCD_send_command(lcd, FUNCTION_SET_4_BIT_MODE | TWO_LINES_ENABLE);
}

void LCD_init(LCD_HandleTypeDef* lcd){
	//initializes lcd using LCD_HandleTypeDef
	TIM_HandleTypeDef* htim = lcd->htim;
	_LCD_delay_us_init(htim);
	_LCD_startup(lcd);
	_LCD_delay_us(US_BETWEEN_COMMANDS, htim);
	_LCD_set_4_bits(lcd);
	_LCD_delay_us(US_BETWEEN_COMMANDS, htim);
	_LCD_send_command(lcd, DISPLAY_OFF);
	_LCD_delay_us(US_BETWEEN_COMMANDS, htim);
	_LCD_send_command(lcd, CLEAR_DISPLAY);
	_LCD_delay_us(US_BETWEEN_COMMANDS, htim);
	_LCD_send_command(lcd, INCREMENT_NO_SHIFT);
	_LCD_delay_us(US_BETWEEN_COMMANDS, htim);
	_LCD_send_command(lcd, 0x0C);
	_LCD_delay_us(US_BETWEEN_COMMANDS, htim);

}

void LCD_putchar(LCD_HandleTypeDef* lcd, char data){
	//prints a single character in cursors position
	I2C_HandleTypeDef* hi2c = lcd->hi2c;
	uint8_t address = lcd->i2c_address;
	uint8_t send[4] = {
			_LCD_get_old_bits((uint8_t)data) | E_PIN_MASK | RS_PIN_MASK | BACKLIGHT_ON, // older half of the command byte with E pin set to high
			_LCD_get_old_bits((uint8_t)data) | RS_PIN_MASK | BACKLIGHT_ON, // older half of the command byte with E pin set to low
			_LCD_get_young_bits((uint8_t)data) | E_PIN_MASK | RS_PIN_MASK | BACKLIGHT_ON, // younger half of the command byte with E pin set to high
			_LCD_get_young_bits((uint8_t)data) | RS_PIN_MASK | BACKLIGHT_ON}; // younger half of the command byte with E pin set to low
	int16_t send_size = sizeof(send);
	HAL_I2C_Master_Transmit(hi2c, address, send, send_size, 100);
}

void LCD_printf(LCD_HandleTypeDef* lcd, char *data){
	//prints a string starting at cursors position
	//IMPLEMENT \n as NEWLINE
	//IMPLEMENT \t as two spaces
	for(char* i = data; *i != '\0'; i++)
		LCD_putchar(lcd, *i);

}

void LCD_set_position(LCD_HandleTypeDef* lcd, uint8_t col, uint8_t row){
	//rows and columns are 0 indexed
	//function omits positions outside of displays memory
	if((col <= 0x27 && col >= 0) && (row <= 1 && row >=0)){
		uint8_t address = col + row * 0x40;
		address |= SET_POSITION_MASK;
		lcd->current_col = col;
		lcd->current_row = row;
		_LCD_send_command(lcd, address);

	}

}

void LCD_reset_position(LCD_HandleTypeDef* lcd){
	//sets cursors position to (0,0)
	LCD_set_position(lcd, 0, 0);
}


void LCD_clear(LCD_HandleTypeDef* lcd){
	//clears display and sets position to (0,0)
	TIM_HandleTypeDef* htim = lcd->htim;
	_LCD_send_command(lcd, CLEAR_DISPLAY);
	_LCD_delay_us(US_BETWEEN_COMMANDS, htim);
	_LCD_send_command(lcd, INCREMENT_NO_SHIFT);
	_LCD_delay_us(US_BETWEEN_COMMANDS, htim);
	_LCD_send_command(lcd, 0x0C);
	_LCD_delay_us(US_BETWEEN_COMMANDS, htim);
	LCD_set_position(lcd, lcd->current_col, lcd->current_row);
}

void LCD_printf_align(LCD_HandleTypeDef* lcd, char *data, uint8_t alignment){
	//prints given string with left or middle alignment of the text
	//function resets cursor position to (0,0) after execution
	uint16_t len = 0;
	for(char* i = data; *i != '\0'; i++, len++);
	switch(alignment){
		case ALIGN_RIGHT:
			LCD_set_position(lcd, MAX_COLUMN - len + 1, lcd->current_row);
			break;
		case ALIGN_MIDDLE:
			LCD_set_position(lcd, (uint8_t)(floor(MAX_COLUMN + 1)/2) - (uint8_t)(len / 2), lcd->current_row);
			break;
	}
	for(char* i = data; *i != '\0'; i++)
		LCD_putchar(lcd, *i);
	LCD_reset_position(lcd);

}

