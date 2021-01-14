/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2020 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
  *                        opensource.org/licenses/BSD-3-Clause
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "adc.h"
#include "dma.h"
#include "spi.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "dma_printf.h"
#include "as5147.h"
#include "drv8305.h"
#include "bldc_calib.h"
#include "bldc_foc.h"

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
//UART Receive
uint8_t uart_buf[6] = "0000\n\r";
uint8_t uart_flag = 0;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim) {
	if (htim->Instance == TIM6) {
		static int32_t t = 0;

		BLDCVqConstControl(0, 5.0f);

		/*
		//sensored 120 deg conduction
		static float pwm_duty = 0.25;
		static uint16_t angle_data;
		static float elec_angle_deg;

		angle_data = (AS5147Read(AS5147_ANGLECOM) & 0x3FFF);		//mask lower 14bit
		elec_angle_deg = fmodf(((float)angle_data + ((float)0x3FFF / 12) - 361), ((float)0x3FFF / 12)) * ((float)(360 * 12) / 0x3FFF);

		if (HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_7) == GPIO_PIN_SET) {
			if     ((90.0f  <= elec_angle_deg) && (elec_angle_deg < 150.0f)) {BLDC120DegConduction(BLDC_UtoV, pwm_duty);}
			else if((150.0f <= elec_angle_deg) && (elec_angle_deg < 210.0f)) {BLDC120DegConduction(BLDC_UtoW, pwm_duty);}
			else if((210.0f <= elec_angle_deg) && (elec_angle_deg < 270.0f)) {BLDC120DegConduction(BLDC_VtoW, pwm_duty);}
			else if((270.0f <= elec_angle_deg) && (elec_angle_deg < 330.0f)) {BLDC120DegConduction(BLDC_VtoU, pwm_duty);}
			else if((330.0f <= elec_angle_deg) || (elec_angle_deg <  30.0f)) {BLDC120DegConduction(BLDC_WtoU, pwm_duty);}
			else if(( 30.0f <= elec_angle_deg) && (elec_angle_deg <  90.0f)) {BLDC120DegConduction(BLDC_WtoV, pwm_duty);}
		} else {
			__HAL_TIM_SET_COMPARE(&htim8, TIM_CHANNEL_1, 0);
			__HAL_TIM_SET_COMPARE(&htim8, TIM_CHANNEL_2, 0);
			__HAL_TIM_SET_COMPARE(&htim8, TIM_CHANNEL_3, 0);
			HAL_GPIO_WritePin(GPIOA, GPIO_PIN_7, GPIO_PIN_RESET);
			HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0, GPIO_PIN_RESET);
			HAL_GPIO_WritePin(GPIOB, GPIO_PIN_1, GPIO_PIN_RESET);
		}
		*/

		if (t >= 5000) {
			HAL_GPIO_TogglePin(GPIOA, GPIO_PIN_6);
			t = 0;
		} else {
			t++;
		}
	}
}


//printf settings
#ifdef __GNUC__
#define PUTCHAR_PROTOTYPE int __io_putchar(int ch)
#else
#define PUTCHAR_PROTOTYPE int fputc(int ch, FILE *f)
#endif /* __GNUC__ */
void __io_putchar(uint8_t ch) {
	dma_printf_putc(ch);
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
  MX_DMA_Init();
  MX_USART1_UART_Init();
  MX_SPI2_Init();
  MX_TIM8_Init();
  MX_SPI3_Init();
  MX_TIM6_Init();
  MX_ADC1_Init();
  MX_TIM2_Init();
  /* USER CODE BEGIN 2 */
  //wait after power on
  HAL_Delay(10);

  //printf settings
  dma_printf_init(&huart1);
  setbuf(stdin, NULL);
  setbuf(stdout, NULL);
  setbuf(stderr, NULL);
  printf("Hello BLDCDriver_v1!\n");

  //dummy spi com(to avoid error at the first com)
  AS5147Read(AS5147_ANGLECOM);
  DRV8305Read(DRV8305_WARNING_WATCHDOG_RESET);

  //current sense
  BLDCStartCurrentSense();

  //Enable BLDC and initialize gate-driver
  BLDCEnable();		//must be run before DRV8305Init()
  DRV8305Init();

  while (HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_7) == GPIO_PIN_RESET);
//  BLDCCalibZeroPos();

  //start timer interrupt
  HAL_TIM_Base_Start_IT(&htim6);

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
	  if (idx == 6000) {
		  for (int32_t i = 0; i < 1000; i++) {
			  printf("%f %f %f %f %f %f\n", g_curt[i][0], g_curt[i][1], g_curt[i][2], g_curt[i][3], g_curt[i][4], g_curt[i][5]);
			  HAL_Delay(5);
		  }
		  idx = 6001;
	  }
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

  /** Configure the main internal regulator output voltage 
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);
  /** Initializes the CPU, AHB and APB busses clocks 
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 8;
  RCC_OscInitStruct.PLL.PLLN = 168;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 4;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB busses clocks 
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */
//UART Callback for printf(...)
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *UartHandle) {
	uart_flag=1;
	HAL_UART_Receive_IT(&huart1, uart_buf, 4);
}
void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart) {
    uart_com_send_it(huart);
    dma_printf_send_it(huart);
}

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */

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
     tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
