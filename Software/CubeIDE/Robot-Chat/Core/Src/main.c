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
#include "i2c.h"
#include "lptim.h"
#include "usart.h"
#include "tim.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "../drivers/motors.h"
#include <string.h>
#include <stdlib.h>
#include <stdio.h>
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
h_Motor_t motors;
char rx_line[64];
int rx_index = 0;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
void PeriphCommonClock_Config(void);
void MX_FREERTOS_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
int __io_putchar(int ch)
{
	HAL_UART_Transmit(&hlpuart1, (uint8_t*)&ch, 1, HAL_MAX_DELAY);
	return ch;
}
int split(char *str, char **argv)
{
	int argc = 0;

	while (*str != '\0')
	{
		while (*str == ' ' || *str == '\n' || *str == '\r' || *str == '\t')
			*str++ = '\0';

		if (*str)
		{
			argv[argc++] = str;
			while (*str && *str != ' ' && *str != '\n' && *str != '\r' && *str != '\t')
				str++;
		}
	}
	return argc;
}
void parse_cmd(int argc, char **argv, h_Motor_t *motors)
{
	if (argc == 0) return;

	// --- Commande STOP ---
	if (strcmp(argv[0], "STOP") == 0)
	{
		Motor_Stop(motors);
		char msg[] = "Motors stopped\r\n";
		HAL_UART_Transmit(&huart1, (uint8_t*)msg, strlen(msg), 100);
		return;
	}

	// --- Commande M1 xx ---
	if (argc >= 2 && strcmp(argv[0], "M1") == 0)
	{
		int val = atoi(argv[1]);
		motors->mode_mot1 = (val >= 0) ? FORWARD_MODE : REVERSE_MODE;
		Motor_SetMode(motors);
		Motor_SetSpeed_percent(motors, val, motors->target_speed2);

		char msg[64];
		sprintf(msg, "M1 set to %d\r\n", val);
		HAL_UART_Transmit(&huart1, (uint8_t*)msg, strlen(msg), 100);
		return;
	}

	// --- Commande M2 xx ---
	if (argc >= 2 && strcmp(argv[0], "M2") == 0)
	{
		int val = atoi(argv[1]);
		motors->mode_mot2 = (val >= 0) ? FORWARD_MODE : REVERSE_MODE;
		Motor_SetMode(motors);
		Motor_SetSpeed_percent(motors, motors->target_speed1, val);

		char msg[64];
		sprintf(msg, "M2 set to %d\r\n", val);
		HAL_UART_Transmit(&huart1, (uint8_t*)msg, strlen(msg), 100);
		return;
	}

	// --- Unknown command ---
	char msg[] = "Unknown command\r\n";
	HAL_UART_Transmit(&huart1, (uint8_t*)msg, strlen(msg), 100);
}

/* Helper pour régler un duty en pourcentage (0..100) */
static void set_pwm_percent(TIM_HandleTypeDef *htim, uint32_t channel, float percent)
{
	if (percent < 0.0f) percent = 0.0f;
	if (percent > 100.0f) percent = 100.0f;
	uint32_t arr = htim->Init.Period;
	uint32_t ccr = (uint32_t)((percent/100.0f) * (float)arr);
	__HAL_TIM_SET_COMPARE(htim, channel, ccr);
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

  /* Configure the peripherals common clocks */
  PeriphCommonClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_TIM1_Init();
  MX_TIM2_Init();
  MX_I2C1_Init();
  MX_I2C3_Init();
  MX_LPTIM1_Init();
  MX_LPUART1_UART_Init();
  MX_USART1_UART_Init();
  /* USER CODE BEGIN 2 */
	printf("\r\n================== TEST CONTROL MOTORs  ===============\r\n");
	/* Init motors */
	//	Motor_Init(&motors, &htim1);
	//
	//	motors.m1_forward_channel = TIM_CHANNEL_1;
	//	motors.m1_reverse_channel = TIM_CHANNEL_2;

	//motors.m2_forward_channel = TIM_CHANNEL_3;
	//motors.m2_reverse_channel = TIM_CHANNEL_4;
	//
	//	const char *msg = "Robot Motor Test Ready\r\n";
	//	HAL_UART_Transmit(&huart1, (uint8_t*)msg, strlen(msg), 100);
	//	uint8_t c;
	//	uint32_t last_update = HAL_GetTick();

	printf("=== PWM Test Ready ===\r\n");

	/* Start PWM channels */
	HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
	HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_2);
	HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_3);
	HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_4);

	/* Force everything OFF at startup */
	__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, 0);
	__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, 0);
	__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_3, 0);
	__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_4, 0);

	printf("PWM ZERO — connect oscilloscope now.\r\n");
	HAL_Delay(2000);

	/* Test 1 : M1 FWD -> CH1 = 20 % */
	uint32_t arr = htim1.Init.Period;
	arr *=  .8f;
//	__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, arr);
//	__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, 0);
//
//	printf("M1 FWD 20%% on CH1 %lu\r\n", arr);
//	HAL_Delay(5000);
//	HAL_receive(1);

//	/* Test 2 : standby */
//	__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, 0);
//	__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, 0);
//	printf("STANDBY\r\n");
//	HAL_Delay(2000);
//
//	/* Test 3 : M1 REV -> CH2 = 20 % */
	__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, arr);
	__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, 0);

	printf("M1 REV 20%% on CH2\r\n");
	HAL_Delay(5000);


  /* USER CODE END 2 */

  /* Call init function for freertos objects (in cmsis_os2.c) */
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
		//		//To test in term write for ex : M1 50
		//		if (HAL_UART_Receive(&huart1, &c, 1, 1) == HAL_OK)
		//		{
		//			// Élimination des CR/LF successifs
		//			if (c == '\r' || c == '\n')
		//			{
		//				if (rx_index == 0)
		//					continue;  // ignore les lignes vides
		//
		//				rx_line[rx_index] = '\0';
		//
		//				// Parsing
		//				char *argv[8];
		//				int argc = split(rx_line, argv);
		//
		//				parse_cmd(argc, argv, &motors);
		//
		//				// Reset clean du buffer !
		//				memset(rx_line, 0, sizeof(rx_line));
		//				rx_index = 0;
		//			}
		//			else
		//			{
		//				// Accumulation des caractères
		//				if (rx_index < sizeof(rx_line) - 1)
		//					rx_line[rx_index++] = c;
		//
		//				// Echo
		//				HAL_UART_Transmit(&huart1, &c, 1, 10);
		//			}
		//		}
		//
		//		// Update moteur
		//		if (HAL_GetTick() - last_update >= 10)
		//		{
		//			Motor_UpdateSpeed(&motors);
		//			last_update = HAL_GetTick();
		//		}

				//TEST MOTEUR SIMPLE
//				// --- Moteur 1 : tourne en sens reverse doucement ---
//				    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, 0);    // FWD OFF
//				    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, 300);  // REV ON
//
////				    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_3, 0);    // M2 FWD OFF
////				    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_4, 300);  // M2 REV ON
//
//				    HAL_Delay(3000);
//
//				    // --- STANDBY pour les 2 moteurs ---
//				    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, 0);
//				    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, 0);
////				    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_3, 0);
////				    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_4, 0);
//
//				    HAL_Delay(500);
//
//				    // --- Moteur 1 : sens forward ---
//				    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, 0);
//				    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, 300);
//
////				    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_4, 0);
////				    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_3, 300);
//
//				    HAL_Delay(3000);
//
//				    // --- Retour Standby ---
//				    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, 0);
//				    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, 0);
////				    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_3, 0);
////				    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_4, 0);
//
//				    HAL_Delay(1000);
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
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = RCC_PLLM_DIV2;
  RCC_OscInitStruct.PLL.PLLN = 16;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLR = RCC_PLLR_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = RCC_PLLQ_DIV2;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure the SYSCLKSource, HCLK, PCLK1 and PCLK2 clocks dividers
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK4|RCC_CLOCKTYPE_HCLK2
                              |RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.AHBCLK2Divider = RCC_SYSCLK_DIV2;
  RCC_ClkInitStruct.AHBCLK4Divider = RCC_SYSCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_3) != HAL_OK)
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
  if (htim->Instance == TIM16)
  {
    HAL_IncTick();
  }
  /* USER CODE BEGIN Callback 1 */

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
#ifdef USE_FULL_ASSERT
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
