/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2024 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdio.h>
#include <string.h>
#include"hd44780_lcd_i2c.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
//#define TWO_LINES_ENABLE 		(0x01 << 3)
//#define MODULE_ADDRESS 			0x4E
//#define E_PIN_MASK 				0x04
//#define RS_PIN_MASK				0x01
//#define FUNCTION_SET_4_BIT_MODE 0x20
//#define CLEAR_DISPLAY 			0x01
//#define STARTUP 				0x30
//#define DISPLAY_OFF				0x08
//#define INCREMENT_NO_SHIFT		0x06
//#define BACKLIGHT_ON			0x08
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
I2C_HandleTypeDef hi2c1;
I2C_HandleTypeDef hi2c2;
I2C_HandleTypeDef hi2c3;

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_I2C1_Init(void);
static void MX_I2C3_Init(void);
static void MX_I2C2_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

//uint8_t LCD_get_young_bits(uint8_t data){
//	data <<= 4;
//	data &= 0xF0;
//	return data;
//
//}
//uint8_t LCD_get_old_bits(uint8_t data){
//	return data & 0xF0;
//}




//void LCD_send_command(I2C_HandleTypeDef* hi2c, uint8_t command){
//	uint8_t send[4];
//	send[0] = LCD_get_old_bits(command) | E_PIN_MASK | BACKLIGHT_ON; // older half of the command byte with E pin set to high
//	send[1] = LCD_get_old_bits(command) | BACKLIGHT_ON; // older half of the command byte with E pin set to low
//	send[2] = LCD_get_young_bits(command) | E_PIN_MASK | BACKLIGHT_ON; // younger half of the command byte with E pin set to high
//	send[3] = LCD_get_young_bits(command) | BACKLIGHT_ON; // younger half of the command byte with E pin set to low
//	uint16_t send_size = sizeof(send);
//	HAL_I2C_Master_Transmit(hi2c, MODULE_ADDRESS, send, send_size, 100);
//
//}

//void LCD_startup(I2C_HandleTypeDef* hi2c){
//	uint8_t send[2] = {
//			STARTUP | E_PIN_MASK | BACKLIGHT_ON,
//			STARTUP | BACKLIGHT_ON
//	};
//	HAL_Delay(20);
//	HAL_I2C_Master_Transmit(hi2c, MODULE_ADDRESS, send, 2, 100);
//	HAL_Delay(5);
//	HAL_I2C_Master_Transmit(hi2c, MODULE_ADDRESS, send, 2, 100);
//	HAL_Delay(1);
//	HAL_I2C_Master_Transmit(hi2c, MODULE_ADDRESS, send, 2, 100);
//
//
//}


//void LCD_set_4_bits(I2C_HandleTypeDef* hi2c, uint8_t num_of_lines){
//	uint8_t data[2] = {
//			FUNCTION_SET_4_BIT_MODE | E_PIN_MASK | BACKLIGHT_ON,
//			FUNCTION_SET_4_BIT_MODE | BACKLIGHT_ON
//	};
//	HAL_I2C_Master_Transmit(hi2c, MODULE_ADDRESS , data, 2, 100);
//	HAL_Delay(1);
//	if(num_of_lines == 2)
//		LCD_send_command(hi2c, FUNCTION_SET_4_BIT_MODE | TWO_LINES_ENABLE);
//}

//void LCD_init(I2C_HandleTypeDef* hi2c, uint8_t num_of_lines){
//	LCD_startup(hi2c);
//	HAL_Delay(1);
//	LCD_set_4_bits(hi2c, num_of_lines);
//	HAL_Delay(1);
//	LCD_send_command(hi2c, DISPLAY_OFF);
//	HAL_Delay(1);
//	LCD_send_command(hi2c, CLEAR_DISPLAY);
//	HAL_Delay(1);
//	LCD_send_command(hi2c, INCREMENT_NO_SHIFT);
//	HAL_Delay(1);
//	LCD_send_command(hi2c, 0x0C);
//	HAL_Delay(1);
//
//}

//void LCD_putchar(I2C_HandleTypeDef* hi2c, char data){
//	uint8_t send[4];
//	send[0] = LCD_get_old_bits((uint8_t)data) | E_PIN_MASK | RS_PIN_MASK | BACKLIGHT_ON; // older half of the command byte with E pin set to high
//	send[1] = LCD_get_old_bits((uint8_t)data) | RS_PIN_MASK | BACKLIGHT_ON; // older half of the command byte with E pin set to low
//	send[2] = LCD_get_young_bits((uint8_t)data) | E_PIN_MASK | RS_PIN_MASK | BACKLIGHT_ON; // younger half of the command byte with E pin set to high
//	send[3] = LCD_get_young_bits((uint8_t)data) | RS_PIN_MASK | BACKLIGHT_ON; // younger half of the command byte with E pin set to low
//	int16_t send_size = sizeof(send);
//	HAL_I2C_Master_Transmit(hi2c, MODULE_ADDRESS, send, send_size, 100);
//
//}

//void LCD_printf(I2C_HandleTypeDef* hi2c, char *data){
//	for(char* i = data; *i != '\0'; i++){
//		LCD_putchar(hi2c, *i);
//	}
//}




void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin){

	  char data = '!';
	  LCD_putchar(&hi2c3, data);

}

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_I2C1_Init();
  MX_I2C3_Init();
  MX_I2C2_Init();
  /* USER CODE BEGIN 2 */
  LCD_init(&hi2c3, 2);
  int year = 2024;
//  char send[] = "hello world";
  char buf[20] = {0};
//  sprintf(buf, "Hello World%d", year);
//  LCD_printf(&hi2c3, buf);
  char data = '!';
  LCD_set_position(&hi2c3, 3, 1);
  LCD_putchar(&hi2c3, data);
  LCD_set_position(&hi2c3, 0, 0);
  LCD_putchar(&hi2c3, data);
  LCD_set_position(&hi2c3, 15, 1);
  LCD_putchar(&hi2c3, data);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI|RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_I2C1|RCC_PERIPHCLK_I2C2
                              |RCC_PERIPHCLK_I2C3;
  PeriphClkInit.I2c1ClockSelection = RCC_I2C1CLKSOURCE_HSI;
  PeriphClkInit.I2c2ClockSelection = RCC_I2C2CLKSOURCE_HSI;
  PeriphClkInit.I2c3ClockSelection = RCC_I2C3CLKSOURCE_HSI;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief I2C1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C1_Init(void)
{

  /* USER CODE BEGIN I2C1_Init 0 */

  /* USER CODE END I2C1_Init 0 */

  /* USER CODE BEGIN I2C1_Init 1 */

  /* USER CODE END I2C1_Init 1 */
  hi2c1.Instance = I2C1;
  hi2c1.Init.Timing = 0x2000090E;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.OwnAddress2Masks = I2C_OA2_NOMASK;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Analogue filter
  */
  if (HAL_I2CEx_ConfigAnalogFilter(&hi2c1, I2C_ANALOGFILTER_ENABLE) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Digital filter
  */
  if (HAL_I2CEx_ConfigDigitalFilter(&hi2c1, 0) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C1_Init 2 */

  /* USER CODE END I2C1_Init 2 */

}

/**
  * @brief I2C2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C2_Init(void)
{

  /* USER CODE BEGIN I2C2_Init 0 */

  /* USER CODE END I2C2_Init 0 */

  /* USER CODE BEGIN I2C2_Init 1 */

  /* USER CODE END I2C2_Init 1 */
  hi2c2.Instance = I2C2;
  hi2c2.Init.Timing = 0x2000090E;
  hi2c2.Init.OwnAddress1 = 0;
  hi2c2.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c2.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c2.Init.OwnAddress2 = 0;
  hi2c2.Init.OwnAddress2Masks = I2C_OA2_NOMASK;
  hi2c2.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c2.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c2) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Analogue filter
  */
  if (HAL_I2CEx_ConfigAnalogFilter(&hi2c2, I2C_ANALOGFILTER_ENABLE) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Digital filter
  */
  if (HAL_I2CEx_ConfigDigitalFilter(&hi2c2, 0) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C2_Init 2 */

  /* USER CODE END I2C2_Init 2 */

}

/**
  * @brief I2C3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C3_Init(void)
{

  /* USER CODE BEGIN I2C3_Init 0 */

  /* USER CODE END I2C3_Init 0 */

  /* USER CODE BEGIN I2C3_Init 1 */

  /* USER CODE END I2C3_Init 1 */
  hi2c3.Instance = I2C3;
  hi2c3.Init.Timing = 0x2000090E;
  hi2c3.Init.OwnAddress1 = 0;
  hi2c3.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c3.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c3.Init.OwnAddress2 = 0;
  hi2c3.Init.OwnAddress2Masks = I2C_OA2_NOMASK;
  hi2c3.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c3.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c3) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Analogue filter
  */
  if (HAL_I2CEx_ConfigAnalogFilter(&hi2c3, I2C_ANALOGFILTER_ENABLE) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Digital filter
  */
  if (HAL_I2CEx_ConfigDigitalFilter(&hi2c3, 0) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C3_Init 2 */

  /* USER CODE END I2C3_Init 2 */

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
/* USER CODE BEGIN MX_GPIO_Init_1 */
/* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOF_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin : PC13 */
  GPIO_InitStruct.Pin = GPIO_PIN_13;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI15_10_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
