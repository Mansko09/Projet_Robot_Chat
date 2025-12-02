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
#include "motors.h"
#include "TOFs.h"
#include "encodeur.h"
#include "odometry.h"
#include <string.h>
#include <stdlib.h>
#include <stdio.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define STACK_SIZE_SMALL   256
#define STACK_SIZE_MEDIUM  384

#define PRIO_MOTOR   10
#define PRIO_CTRL    5
#define PRIO_TOF     4
#define PRIO_ODOM    6

#define TOF_TRESHHOLD 40
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
h_Motor_t hMotors;
h_tof_t hTof;

//task handles
TaskHandle_t xTaskMotorsHandle = NULL;
TaskHandle_t xTaskControlHandle = NULL;
TaskHandle_t xTaskToFHandle = NULL;
TaskHandle_t xTaskOdomHandle = NULL;

Encodeur_t enc;
Odom_t odom;

Odom_Params_t odom_params = {
		.wheel_radius = 0.03f,    // 3 cm
		.wheel_base   = 0.15f,    // 15 cm entre roues
		.ticks_per_rev = 48.0f    // à vérifier pour les encodeurs
};

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
//static void set_pwm_percent(TIM_HandleTypeDef *htim, uint32_t channel, float percent)
//{
//	if (percent < 0.0f) percent = 0.0f;
//	if (percent > 100.0f) percent = 100.0f;
//	uint32_t arr = htim->Init.Period;
//	uint32_t ccr = (uint32_t)((percent/100.0f) * (float)arr);
//	__HAL_TIM_SET_COMPARE(htim, channel, ccr);
//}

/* ============================= */
/*        TASK   MOTORS          */
/* ============================= */
void Task_Motors(void * unsused)
{
	printf("task_Motors\r\n");

	while (1)
	{
		//get distance tofs
		Motor_UpdateSpeed(&hMotors);
		vTaskDelay(1);
	}
}

/* ============================= */
/*       TASK   CONTROL          */
/* ============================= */
void Task_Control(void *unused)
{
	printf("task_Control\r\n");

	while (1)
	{
		/* Print the 4 ToF distances */
		printf(
				"FL: %d mm, FC: %d mm, FR: %d mm, BC: %d mm\r\n",
				hTof.distance_tof1,    // Front Left
				hTof.distance_tof2,    // Front Center
				hTof.distance_tof3,    // Front Right
				hTof.distance_tof4     // Back Center
		);

		/* --------------------- Robot Obstacle Logic ----------------------
		 * We consider:
		 *  - distance_tof1 = front left
		 *  - distance_tof2 = front center
		 *  - distance_tof3 = front right
		 *  - distance_tof4 = back center
		 *
		 * Example strategy:
		 *  - If something in front → try to turn according to FL/FR
		 *  - If fully blocked → go backwards
		 *  - If back blocked → prevent reverse
		 *-----------------------------------------------------------------*/

		int front_left   = (hTof.distance_tof1 <= TOF_TRESHHOLD);
		int front_center = (hTof.distance_tof2 <= TOF_TRESHHOLD);
		int front_right  = (hTof.distance_tof3 <= TOF_TRESHHOLD);
		int back_center  = (hTof.distance_tof4 <= TOF_TRESHHOLD);

		/* ---- CASE 1 : NO OBSTACLE IN FRONT → GO FORWARD ---- */
		if (!front_left && !front_center && !front_right)
		{
			hMotors.mode_mot1 = FORWARD_MODE;
			hMotors.mode_mot2 = FORWARD_MODE;
		}

		/* ---- CASE 2 : OBSTACLE IN FRONT CENTER ONLY → CHOOSE BEST SIDE ---- */
		else if (front_center && !front_left && front_right)
		{
			// Turn LEFT (mot1 backward, mot2 forward)
			hMotors.mode_mot1 = REVERSE_MODE;
			hMotors.mode_mot2 = FORWARD_MODE;
		}
		else if (front_center && front_left && !front_right)
		{
			// Turn RIGHT
			hMotors.mode_mot1 = FORWARD_MODE;
			hMotors.mode_mot2 = REVERSE_MODE;
		}

		/* ---- CASE 3 : OBSTACLE ON LEFT ONLY → TURN RIGHT ---- */
		else if (front_left && !front_center && !front_right)
		{
			hMotors.mode_mot1 = FORWARD_MODE;
			hMotors.mode_mot2 = REVERSE_MODE;
		}

		/* ---- CASE 4 : OBSTACLE ON RIGHT ONLY → TURN LEFT ---- */
		else if (!front_left && !front_center && front_right)
		{
			hMotors.mode_mot1 = REVERSE_MODE;
			hMotors.mode_mot2 = FORWARD_MODE;
		}

		/* ---- CASE 5 : FULLY BLOCKED (ALL 3 FRONT) → GO BACK ---- */
		else if (front_left && front_center && front_right)
		{
			// BUT check if back is free
			if (!back_center)
			{
				hMotors.mode_mot1 = REVERSE_MODE;
				hMotors.mode_mot2 = REVERSE_MODE;
			}
			else
			{
				// Stuck → stop motors to avoid crash
				hMotors.mode_mot1 = BRAKE_MODE;
				hMotors.mode_mot2 = BRAKE_MODE;
			}
		}

		/* ---- Any other random case → safe backward ---- */
		else
		{
			if (!back_center)
			{
				hMotors.mode_mot1 = REVERSE_MODE;
				hMotors.mode_mot2 = REVERSE_MODE;
			}
			else
			{
				hMotors.mode_mot1 = BRAKE_MODE;
				hMotors.mode_mot2 = BRAKE_MODE;
			}
		}

		/* Apply motor commands */
		Motor_SetMode(&hMotors);
		Motor_SetSpeed_percent(&hMotors, 40, 40);

		printf("Mot1 speed: %d, Mot2 speed: %d\r\n",
				hMotors.current_speed1,
				hMotors.current_speed2);

		vTaskDelay(1);
	}
}

/* ============================= */
/*       TASK   TOFs             */
/* ============================= */
void Task_ToFs(void *unused)
{
    printf("Task_ToFs started\r\n");

    for(;;)
    {

        vTaskDelay(50);
    }
}


/* ============================= */
/*        TASK ODOM              */
/* ============================= */
void Task_Odom(void *unused)
{
    printf("Task_Odom started\r\n");

    for(;;)
    {
        Encodeur_Read(&enc);
        Odom_Update(&odom, &enc, &odom_params);

        printf("x=%.2f  y=%.2f  th=%.1f°\r\n",
                odom.x, odom.y, odom.theta * 180 / M_PI);

        vTaskDelay(10);
    }
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
	  /* ======================== INIT ========================= */
	Motor_Init(&hMotors, &htim1);
	hMotors.m1_forward_channel = TIM_CHANNEL_1;
	hMotors.m1_reverse_channel = TIM_CHANNEL_2;
	hMotors.m2_forward_channel = TIM_CHANNEL_4; //J'ai échangé 3 et 4 pour que quand on soit en mode fwd, les deux roues sont dans le mm sens
	hMotors.m2_reverse_channel = TIM_CHANNEL_3;

	HAL_TIM_Encoder_Start(&htim2, TIM_CHANNEL_ALL);
	HAL_LPTIM_Encoder_Start(&hlptim1, 0xFFFF);
	Encodeur_Init();
	Odom_Init(&odom);
    /* ======================== START PWM ========================== */
    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_2);
    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_3);
    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_4);

    /* ======================== CREATE TASKS ========================== */

    xTaskCreate(Task_Motors, "Motors", STACK_SIZE_MEDIUM, NULL, PRIO_MOTOR, &xTaskMotorsHandle);
    xTaskCreate(Task_Control, "Control", STACK_SIZE_MEDIUM, NULL, PRIO_CTRL, &xTaskControlHandle);
    xTaskCreate(Task_ToFs,  "ToFs",    STACK_SIZE_SMALL,  NULL, PRIO_TOF, &xTaskToFHandle);
    xTaskCreate(Task_Odom,  "Odom",    STACK_SIZE_SMALL,  NULL, PRIO_ODOM, &xTaskOdomHandle);



	printf("\r\n================== TEST CONTROL MOTORs  ===============\r\n");
	HAL_TIM_Base_Start(&htim1);
	HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
	HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_2);
	HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_3);
	HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_4);


	__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, 0);   // M1 FWD
	__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, 0);     // M1 REV OFF
	__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_3, 0);     // M1 REV OFF
	__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_4, 0);     // M1 REV OFF

	uint32_t arr = htim1.Init.Period;
	arr *= 0.80f;   // 20% duty cycle SAFE

	/* ------------------ M1 FWD ------------------ */
	//	__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, arr);   // M1 FWD
	//	__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, 0);     // M1 REV OFF
	//	printf("M1 FWD 80%%\r\n");
	//	HAL_Delay(3000);

	//	/* ------- STANDBY sécuritaire avant inversion ------- */
	//	__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, 0);
	//	__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, 0);
	//	printf("M1 STANDBY\r\n");
	//	HAL_Delay(1000);
	//
	//	/* ------------------ M1 REV ------------------ */
	//	__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, arr);   // M1 REV
	//	__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, 0);     // M1 FWD OFF
	//	printf("M1 REV 20%%\r\n");
	//	HAL_Delay(3000);

	//	/* ------------------ M2 FWD ------------------ */
	//	__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_3, arr);   // M2 FWD
	//	__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_4, 0);     // M2 REV OFF
	//	printf("M2 FWD 80%%\r\n");
	//	HAL_Delay(3000);

	//	/* ------- STANDBY sécuritaire avant inversion ------- */
	//	__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_3, 0);
	//	__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_4, 0);
	//	printf("M2 STANDBY\r\n");
	//	HAL_Delay(1000);
	//
	//	/* ------------------ M2 REV ------------------ */
	//	__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_4, arr);   // M2 REV
	//	__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_3, 0);     // M2 FWD OFF
	//	printf("M2 REV 20%%\r\n");
	//	HAL_Delay(3000);

	//uint32_t last_update = HAL_GetTick();


	vTaskStartScheduler();
	/* USER CODE END 2 */

	//	/* Call init function for freertos objects (in cmsis_os2.c) */
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
