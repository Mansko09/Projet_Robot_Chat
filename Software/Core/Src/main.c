/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2025 STMicroelectronics.
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
#include "cmsis_os.h"
#include "dma.h"
#include "i2c.h"
#include "lptim.h"
#include "usart.h"
#include "memorymap.h"
#include "tim.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "ADXL.h"
#include "i2c.h"
#include <stdio.h>
#include <stdlib.h>
#include "TOF_Control.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */


/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

SemaphoreHandle_t sem_TOF;
SemaphoreHandle_t sem_ADXL;



ADXL_InitTypeDef adxl ={
		.IntMode = INT_ACTIVEHIGH,
		.LPMode = LPMODE_NORMAL,
		.Rate = BWRATE_100,
		.Range = RANGE_4G,
		.Resolution = RESOLUTION_10BIT,
		.Justify = JUSTIFY_SIGNED,
		.AutoSleep = AUTOSLEEPOFF,
		.LinkMode = LINKMODEOFF
};



int __io_putchar(int chr){
	HAL_UART_Transmit(&hlpuart1, (uint8_t*)&chr, 1, HAL_MAX_DELAY);
	return chr;
}


void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
	//printf("glapitouADXL\r\n");
	if (GPIO_Pin == GPIO_PIN_0 || GPIO_Pin == GPIO_PIN_1){
			ADXL_IntProto();
			int16_t acc[3] = {0,0,0};
			ADXL_getAccel(acc ,OUTPUT_SIGNED);
			printf("x : %d, y : %d, z : %d\r\n",(int)acc[0],(int)acc[1],(int)acc[2]);
			ADXL_disableSingleTap();
	  		BaseType_t higher_priority_task_woken = pdFALSE;
	  		xSemaphoreGiveFromISR(sem_ADXL, &higher_priority_task_woken);
	  		portYIELD_FROM_ISR(higher_priority_task_woken);
	  	}
}


void taskAccelDetection(void * unused){
	ADXL_Init(&adxl);
	sem_ADXL = xSemaphoreCreateBinary();
	uint8_t mesured_axes = X_axes | Y_axes;
	uint8_t duration_choc = 0x1B;
	uint8_t threshold_choc = 0x21;
	ADXL_SetOffset(2,0,-63);
	ADXL_Measure(ON);
	for (;;){
		ADXL_enableSingleTap(INT2, mesured_axes,duration_choc, threshold_choc);
		xSemaphoreTake(sem_ADXL,portMAX_DELAY);
		printf("Ouille\r\n");
		vTaskDelay(500);
	}
}



void taskTOFDetection(void * unused){
	//printf("AHHAHAHAHuuuuuuuuu\r\n");
	TOF_Init();
	sem_TOF = xSemaphoreCreateBinary();
	if(HAL_TIM_Base_Start_IT(&htim17) != HAL_OK){
		printf("ca marche pas\r\n");
	}
	for(;;){
		//printf("glapitou\r\n");
		__HAL_TIM_GetCounter(&htim17);
		xSemaphoreTake(sem_TOF,portMAX_DELAY);
		//printf("glapitou1\r\n");
		data_read_TOF(VL53L0X_DEFAULT_ADDRESS,CHANNEL_0);
		vTaskDelay(100);
		data_read_TOF(VL53L0X_DEFAULT_ADDRESS,CHANNEL_1);
		vTaskDelay(100);
		data_read_TOF(VL53L0X_DEFAULT_ADDRESS,CHANNEL_2);
		vTaskDelay(100);
		data_read_TOF(VL53L0X_DEFAULT_ADDRESS,CHANNEL_3);
		vTaskDelay(100);
	}
}
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
void PeriphCommonClock_Config(void);
void MX_FREERTOS_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

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

/* Configure the peripherals common clocks */
  PeriphCommonClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_I2C1_Init();
  MX_I2C3_Init();
  MX_MEMORYMAP_Init();
  MX_LPUART1_UART_Init();
  MX_TIM17_Init();
  MX_LPTIM1_Init();
  MX_TIM1_Init();
  MX_TIM2_Init();
  /* USER CODE BEGIN 2 */
  if(xTaskCreate(taskTOFDetection,"Detection",512,NULL,2,NULL) != pdPASS){
	  printf("Error creating task detection\r\n");
	  Error_Handler();
  }

  if(xTaskCreate(taskAccelDetection,"Detection Choc",512,NULL,1,NULL) != pdPASS){
  	  printf("Error creating task detection choc\r\n");
  	  Error_Handler();
  }

//  printf("\r\n================== TEST CONTROL MOTORs  ===============\r\n");
//  	HAL_TIM_Base_Start(&htim1);
//  	HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
//  	HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_2);
//  	HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_3);
//  	HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_4);
//
//
//  	__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, 0);   // M1 FWD
//  	__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, 0);     // M1 REV OFF
//  	__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_3, 0);     // M1 REV OFF
//  	__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_4, 0);     // M1 REV OFF
//
//  	uint32_t arr = htim1.Init.Period;
//  	arr *= 0.80f;   // 20% duty cycle SAFE
//
//  	/* ------------------ M1 FWD ------------------ */
//  	//	__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, arr);   // M1 FWD
//  	//	__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, 0);     // M1 REV OFF
//  	//	printf("M1 FWD 80%%\r\n");
//  	//	HAL_Delay(3000);
//
//  	//	/* ------- STANDBY sécuritaire avant inversion ------- */
//  	//	__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, 0);
//  	//	__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, 0);
//  	//	printf("M1 STANDBY\r\n");
//  	//	HAL_Delay(1000);
//  	//
//  	//	/* ------------------ M1 REV ------------------ */
//  	//	__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, arr);   // M1 REV
//  	//	__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, 0);     // M1 FWD OFF
//  	//	printf("M1 REV 20%%\r\n");
//  	//	HAL_Delay(3000);
//
//  	//	/* ------------------ M2 FWD ------------------ */
//  	//	__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_3, arr);   // M2 FWD
//  	//	__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_4, 0);     // M2 REV OFF
//  	//	printf("M2 FWD 80%%\r\n");
//  	//	HAL_Delay(3000);
//
//  	//	/* ------- STANDBY sécuritaire avant inversion ------- */
//  	//	__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_3, 0);
//  	//	__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_4, 0);
//  	//	printf("M2 STANDBY\r\n");
//  	//	HAL_Delay(1000);
//  	//
//  	//	/* ------------------ M2 REV ------------------ */
//  	//	__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_4, arr);   // M2 REV
//  	//	__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_3, 0);     // M2 FWD OFF
//  	//	printf("M2 REV 20%%\r\n");
//  	//	HAL_Delay(3000);

  vTaskStartScheduler();
  /* USER CODE END 2 */

  /* Call init function for freertos objects (in freertos.c) */
  MX_FREERTOS_Init();

  /* Start scheduler */
  osKernelStart();

  /* We should never get here as control is now taken by the scheduler */
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

  /** Configure the main internal regulator output voltage
  */
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI|RCC_OSCILLATORTYPE_MSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.MSIState = RCC_MSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.MSICalibrationValue = RCC_MSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.MSIClockRange = RCC_MSIRANGE_6;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure the SYSCLKSource, HCLK, PCLK1 and PCLK2 clocks dividers
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK4|RCC_CLOCKTYPE_HCLK2
                              |RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_MSI;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.AHBCLK2Divider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.AHBCLK4Divider = RCC_SYSCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief Peripherals Common Clock Configuration
  * @retval None
  */
void PeriphCommonClock_Config(void)
{
  RCC_PeriphCLKInitTypeDef PeriphClkInitStruct = {0};

  /** Initializes the peripherals clock
  */
  PeriphClkInitStruct.PeriphClockSelection = RCC_PERIPHCLK_SMPS;
  PeriphClkInitStruct.SmpsClockSelection = RCC_SMPSCLKSOURCE_HSI;
  PeriphClkInitStruct.SmpsDivSelection = RCC_SMPSCLKDIV_RANGE1;

  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN Smps */

  /* USER CODE END Smps */
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
  * @brief  Period elapsed callback in non blocking mode
  * @note   This function is called  when TIM16 interrupt took place, inside
  * HAL_TIM_IRQHandler(). It makes a direct call to HAL_IncTick() to increment
  * a global variable "uwTick" used as application time base.
  * @param  htim : TIM handle
  * @retval None
  */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  /* USER CODE BEGIN Callback 0 */

  /* USER CODE END Callback 0 */
  if (htim->Instance == TIM16) {
    HAL_IncTick();
  }
  /* USER CODE BEGIN Callback 1 */
  if (htim->Instance == TIM17){
	  	//printf("glapitousem\r\n");
  		BaseType_t higher_priority_task_woken = pdFALSE;

  		xSemaphoreGiveFromISR(sem_TOF, &higher_priority_task_woken);

  		portYIELD_FROM_ISR(higher_priority_task_woken);
  	}
  /* USER CODE END Callback 1 */
}

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
