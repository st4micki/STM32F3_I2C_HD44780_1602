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

#ifndef TIM1
	#define TIM1 CLOCK_NOT_DEFINED
#endif
#ifndef TIM15
	#define TIM15 CLOCK_NOT_DEFINED
#endif
#ifndef TIM16
	#define TIM16 CLOCK_NOT_DEFINED
#endif
#ifndef TIM17
	#define TIM17 CLOCK_NOT_DEFINED
#endif
#ifndef TIM2
	#define TIM17 CLOCK_NOT_DEFINED
#endif
#ifndef TIM3
	#define TIM3 CLOCK_NOT_DEFINED
#endif
#ifndef TIM4
	#define TIM4 CLOCK_NOT_DEFINED
#endif
#ifndef TIM6
	#define TIM6 CLOCK_NOT_DEFINED
#endif

typedef struct{
	uint8_t current_row;
	uint8_t current_col;
} LCD_current_pos;

LCD_current_pos lcd_pos;


void LCD_delay_us(uint32_t us, TIM_HandleTypeDef *htim){
	uint32_t apb_freq = 0;
	if(htim->Instance == TIM6 || htim->Instance == TIM4 || htim->Instance == TIM3 || htim->Instance == TIM2)
		apb_freq = HAL_RCC_GetPCLK2Freq();
	else if(htim->Instance == TIM17 || htim->Instance == TIM16 ||htim->Instance == TIM15 || htim->Instance == TIM1)
		apb_freq = HAL_RCC_GetPCLK2Freq();
	else
		apb_freq = HAL_RCC_GetHCLKFreq();
	uint16_t new_prescaler = (uint16_t)(apb_freq/1000000);
	htim->Instance->PSC = new_prescaler - 1;
	htim->Instance->ARR = 0xFFFF - 1;
	HAL_TIM_Base_Start(htim);
	__HAL_TIM_SET_COUNTER(htim, 0);
	uint32_t tick_start = HAL_GetTick();
	while(htim->Instance->CNT < us){}
//	HAL_TIM_Base_Stop(htim);
	uint32_t tick_diff = HAL_GetTick() - tick_start;
	HAL_Delay(1);
}


uint8_t _LCD_get_young_bits(uint8_t data){
	data <<= 4;
	data &= 0xF0;
	return data;

}


uint8_t _LCD_get_old_bits(uint8_t data){
	return data & 0xF0;
}

void _LCD_send_command(I2C_HandleTypeDef* hi2c, uint8_t command){
	uint8_t send[4] = {
		_LCD_get_old_bits(command) | E_PIN_MASK | BACKLIGHT_ON, // older half of the command byte with E pin set to high
		_LCD_get_old_bits(command) | BACKLIGHT_ON, // older half of the command byte with E pin set to low
		_LCD_get_young_bits(command) | E_PIN_MASK | BACKLIGHT_ON, // younger half of the command byte with E pin set to high
		_LCD_get_young_bits(command) | BACKLIGHT_ON}; // younger half of the command byte with E pin set to low
	uint16_t send_size = sizeof(send);
	HAL_I2C_Master_Transmit(hi2c, MODULE_ADDRESS, send, send_size, 100);
}

void _LCD_startup(I2C_HandleTypeDef* hi2c){
	//startup needs to use standard HAL_Delay(ms) in places where it needs to  due to displays hardware constrains
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
			FUNCTION_SET_4_BIT_MODE | BACKLIGHT_ON};
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
	uint8_t send[4] = {
			_LCD_get_old_bits((uint8_t)data) | E_PIN_MASK | RS_PIN_MASK | BACKLIGHT_ON, // older half of the command byte with E pin set to high
			_LCD_get_old_bits((uint8_t)data) | RS_PIN_MASK | BACKLIGHT_ON, // older half of the command byte with E pin set to low
			_LCD_get_young_bits((uint8_t)data) | E_PIN_MASK | RS_PIN_MASK | BACKLIGHT_ON, // younger half of the command byte with E pin set to high
			_LCD_get_young_bits((uint8_t)data) | RS_PIN_MASK | BACKLIGHT_ON}; // younger half of the command byte with E pin set to low
	int16_t send_size = sizeof(send);
	HAL_I2C_Master_Transmit(hi2c, MODULE_ADDRESS, send, send_size, 100);
}

void LCD_printf(I2C_HandleTypeDef* hi2c, char *data){
	//IMPLEMENT \n as NEWLINE
	//IMPLEMENT \t as two spaces
	for(char* i = data; *i != '\0'; i++)
		LCD_putchar(hi2c, *i);

}

void LCD_set_position(I2C_HandleTypeDef* hi2c, uint8_t col, uint8_t row){
	//rows and columns are 0 indexed
	//function omits positions outside of displays memory
	if((col <= 0x27 && col >= 0) && (row <= 1 && row >=0)){
		uint8_t address = col + row * 0x40;
		address |= SET_POSITION_MASK;
		lcd_pos.current_col = col;
		lcd_pos.current_row = row;
		_LCD_send_command(hi2c, address);

	}

}

void LCD_reset_position(I2C_HandleTypeDef *hi2c){
	LCD_set_position(hi2c, 0, 0);
}


void LCD_clear(I2C_HandleTypeDef* hi2c){
	//clears display and sets position to (0,0)
	_LCD_send_command(hi2c, CLEAR_DISPLAY);
	HAL_Delay(1);
	_LCD_send_command(hi2c, INCREMENT_NO_SHIFT);
	HAL_Delay(1);
	_LCD_send_command(hi2c, 0x0C);
	HAL_Delay(1);
	LCD_set_position(hi2c, lcd_pos.current_col, lcd_pos.current_row);
}

void LCD_printf_align(I2C_HandleTypeDef *hi2c, char *data, uint8_t alignment){
	//prints given string with left or middle alignment of the text
	//function resets cursor position to (0,0) after execution
	uint16_t len = 0;
	for(char* i = data; *i != '\0'; i++, len++);
	switch(alignment){
		case ALIGN_RIGHT:
			LCD_set_position(hi2c, MAX_COLUMN - len + 1, lcd_pos.current_row);
			break;
		case ALIGN_MIDDLE:
			LCD_set_position(hi2c, (uint8_t)(floor(MAX_COLUMN + 1)/2) - (uint8_t)(len / 2), lcd_pos.current_row);
			break;
	}
	for(char* i = data; *i != '\0'; i++)
		LCD_putchar(hi2c, *i);
	LCD_reset_position(hi2c);

}

