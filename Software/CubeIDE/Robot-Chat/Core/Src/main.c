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

#include <string.h>
#include <stdlib.h>
#include <stdio.h>
#include "control.h"
#include "semphr.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define STACK_SIZE_SMALL   256
#define STACK_SIZE_MEDIUM  384

#define PRIO_CTRL    5
#define PRIO_TOF     4
#define PRIO_ODOM_MOTOR    10
#define PRIO_ACCEL   2
#define PRIO_LIDAR 3

#define TOF_TRESHHOLD 40


/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
h_Motor_t hMotors;
h_tof_t hTof;
Odom_t odom;
Encodeur_t enc;
PID_t pid_left;
PID_t pid_right;

h_control_t hControl;
//FREERTOS
SemaphoreHandle_t controlMutex;


//task handles
TaskHandle_t xTaskControlHandle = NULL;
TaskHandle_t xTaskToFHandle = NULL;
TaskHandle_t xTaskOdomMotorHandle = NULL;
TaskHandle_t xTaskTestHandle=NULL;
TaskHandle_t TaskTestOdomPollingHandle=NULL;


Odom_Params_t odom_params = {
		.wheel_radius = 0.02f,    // 4 cm de diametre donc 2 de rayon
		.wheel_base   = 0.153f,    // 15 cm entre roues
		.ticks_per_rev =  222.4f // cf datasheet moteurs
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


/* ============================= */
/*       TASK   CONTROL          */
/* ============================= */
//void Task_Control(void *unused)
//{
//    printf("task_Control\r\n");
//
//    for (;;)
//    {
//        int vide, acc;
//
//        // Lire l’état partagé
//        xSemaphoreTake(controlMutex, portMAX_DELAY);
//        vide = hControl.vide;
//        acc  = hControl.AccData;
//        xSemaphoreGive(controlMutex);
//
//        /* --- PRIORITÉ 1 : CHOC ACCÉLÉROMÈTRE --- */
//        if (acc == 1)
//        {
//            xSemaphoreTake(controlMutex, portMAX_DELAY);
//            hControl.hMotors.mode_mot1 = BRAKE_MODE;
//            hControl.hMotors.mode_mot2 = BRAKE_MODE;
//            xSemaphoreGive(controlMutex);
//
//            Motor_SetMode(&hControl.hMotors);
//            Motor_SetSpeed_percent(&hControl.hMotors, 0, 0);
//
//            printf("[CONTROL] ACCEL SHOCK -> FULL STOP\n");
//
//            vTaskDelay(pdMS_TO_TICKS(20));
//            continue;
//        }
//
//        /* --- PRIORITÉ 2 : OBSTACLES LIDAR/ToF --- */
//
//        xSemaphoreTake(controlMutex, portMAX_DELAY);
//
//        switch (vide)
//        {
//        case 2: // Front → recule
//            hControl.hMotors.mode_mot1 = REVERSE_MODE;
//            hControl.hMotors.mode_mot2 = REVERSE_MODE;
//            break;
//
//        case 1: // Obstacle gauche → tourne à droite
//            hControl.hMotors.mode_mot1 = FORWARD_MODE;
//            hControl.hMotors.mode_mot2 = REVERSE_MODE;
//            break;
//
//        case 3: // Obstacle droite → tourne à gauche
//            hControl.hMotors.mode_mot1 = REVERSE_MODE;
//            hControl.hMotors.mode_mot2 = FORWARD_MODE;
//            break;
//
//        case 4: // Obstacle arrière → avance
//            hControl.hMotors.mode_mot1 = FORWARD_MODE;
//            hControl.hMotors.mode_mot2 = FORWARD_MODE;
//            break;
//
//        case 0: // Aucun obstacle
//            hControl.hMotors.mode_mot1 = FORWARD_MODE;
//            hControl.hMotors.mode_mot2 = FORWARD_MODE;
//            break;
//
//        default:
//            hControl.hMotors.mode_mot1 = BRAKE_MODE;
//            hControl.hMotors.mode_mot2 = BRAKE_MODE;
//            break;
//        }
//
//        xSemaphoreGive(controlMutex);
//
//        /* Envoie de la commande moteur */
//        Motor_SetMode(&hControl.hMotors);
//        Motor_SetSpeed_percent(&hControl.hMotors, 40, 40);
//
//        // Debug
//        printf("[CONTROL] vide=%d mot1=%d mot2=%d\n",
//               vide,
//               hControl.hMotors.mode_mot1,
//               hControl.hMotors.mode_mot2);
//
//        vTaskDelay(pdMS_TO_TICKS(10));
//    }
//}
//
///* ============================= */
///*       TASK   TOFs             */
///* ============================= */
//void Task_ToFs(void *unused)
//{
//    for(;;)
//    {
//        int vide_detecte = 0;
//
//// logique tofs de David
//
//        xSemaphoreTake(controlMutex, portMAX_DELAY);
//        hControl.vide = vide_detecte;
//        xSemaphoreGive(controlMutex);
//
//        vTaskDelay(20);
//    }
//}
//
///* ============================= */
///*        TASK ODOM et Moteurs (toutes les 10ms)             */
///* ============================= */
//void Task_OdomMotor(void *unused)
//{

//    for (;;)
//    {
//        Encodeur_Read(&enc);
//
//        xSemaphoreTake(controlMutex, portMAX_DELAY);
//
//        // Mise à jour odométrie
//        Odom_Update(&hControl.odom, &enc, &odom_params);
//		  Control_UpdateSpeed(&hControl, &enc, &odom_params, dt);
//		  Control_RunPID(&hControl, dt);
//
//        // Mise à jour interne des contrôleurs moteurs
//        Motor_UpdateSpeed(&hControl.hMotors);


//        xSemaphoreGive(controlMutex);
//
//        // Debug (lecture sans modifier)
//        printf("x=%.2f y=%.2f th=%.1f°\n",
//               hControl.odom.x,
//               hControl.odom.y,
//               hControl.odom.theta * 180.0f / M_PI);
//
//        vTaskDelay(10);
//    }
//}


//TASK TEST POUR MOTEURS ET ODOMETRIE
//void Task_TestMotors(void *unused)
//{
//	printf("=== TEST MOTORS START ===\r\n");
//
//	// ---------- RESET ODOM ----------
//	memset(&hControl.odom, 0, sizeof(hControl.odom));
//
//	// ---------- INITIALISATION MOTEURS ----------
//	xSemaphoreTake(controlMutex, portMAX_DELAY);
//	hControl.hMotors.mode_mot1 = FORWARD_MODE;
//	hControl.hMotors.mode_mot2 = FORWARD_MODE;
//	Motor_SetMode(&hControl.hMotors);
//
//	Motor_SetSpeed_percent(&hControl.hMotors, 40, 40);
//	hControl.hMotors.current_speed1 = hControl.hMotors.target_speed1;
//	hControl.hMotors.current_speed2 = hControl.hMotors.target_speed2;
//	xSemaphoreGive(controlMutex);
//
//	// ---------- PARAMÈTRES ----------
//	const TickType_t updatePeriod = pdMS_TO_TICKS(100); // 100 ms
//	const TickType_t printPeriod  = pdMS_TO_TICKS(1000); // 1 s
//	const int totalDurationMs = 10000; // 10 s
//	int elapsedMs = 0;
//	TickType_t lastPrintTick = xTaskGetTickCount();
//
//	// ---------- BOUCLE DE TEST ----------
//	while (elapsedMs < totalDurationMs)
//	{
//		xSemaphoreTake(controlMutex, portMAX_DELAY);
//
//		// Mise à jour moteurs
//		Motor_UpdateSpeed(&hControl.hMotors);
//
//		// Lecture encodeurs + odométrie
//		Encodeur_Read(&enc);
//		Odom_Update(&hControl.odom, &enc, &odom_params);
//
//		// Capture valeurs localement
//		float x = hControl.odom.x;
//		float y = hControl.odom.y;
//		float th = hControl.odom.theta;
//
//		xSemaphoreGive(controlMutex);
//
//		// Affichage toutes les 1 seconde
//		if ((xTaskGetTickCount() - lastPrintTick) >= printPeriod)
//		{
//			printf("[TEST] Hold 40%%\r\n");
//			printf("x=%.2f y=%.2f th=%.1f degres \r\n",
//					x, y, th * 180.0f / M_PI);
//			//printf("dL=%d dR=%d\n", enc.delta_1, enc.delta_2);
//
//
//			lastPrintTick = xTaskGetTickCount();
//		}
//
//		vTaskDelay(updatePeriod);
//		elapsedMs += updatePeriod * portTICK_PERIOD_MS;
//
//	}
//
//	// ---------- ARRET DES MOTEURS ----------
//	xSemaphoreTake(controlMutex, portMAX_DELAY);
//	Motor_SetSpeed_percent(&hControl.hMotors, 0, 0);
//	hControl.hMotors.mode_mot1 = BRAKE_MODE;
//	hControl.hMotors.mode_mot2 = BRAKE_MODE;
//	Motor_SetMode(&hControl.hMotors);
//	xSemaphoreGive(controlMutex);
//
//	printf("=== TEST MOTORS STOP ===\r\n");
//
//	vTaskDelete(NULL);
//}
//

//				----- TEST ASSERVISSEMENT ----

//void Task_TestMotors(void *p)
//{
//    const float dt = 0.01f; // 10 ms
//    // ---------------------------
//    // Consigne de test (10 cm/s en avant)
//    // ---------------------------
//
//
//    // ---------------------------
//    // Boucle périodique FreeRTOS
//    // ---------------------------
//    for (;;)
//    {
//        // 1. Lecture encodeurs et calcul des vitesses mesurées
//        Encodeur_Read(&enc);  // met à jour enc.delta_1 / enc.delta_2
//        Control_UpdateSpeed(&hControl, &enc, &odom_params, dt);
//
//        // 2. Calcul PID et commande moteurs
//        Control_RunPID(&hControl, dt, odom_params.wheel_base);
//
//        // 3. Mise à jour PWM / rampes
//        Motor_UpdateSpeed(&hControl.hMotors);
//
//        // 4. Affichage pour debug
//        printf("Target v: %.3f m/s, w: %.3f rad/s | "
//               "Measured v_left: %.3f m/s, v_right: %.3f m/s | "
//               "PWM left: %.1f, PWM right: %.1f\r\n",
//               hControl.v_ref,
//               hControl.w_ref,
//               hControl.v_left_meas,
//               hControl.v_right_meas,
//               hControl.hMotors.current_speed1,
//               hControl.hMotors.current_speed2);
//
//        // 5. Attente périodique
//        vTaskDelay(pdMS_TO_TICKS(100));
//    }
//}






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
	printf(" ------ FELIX READY ------\r\n ");
	/* ======================== INIT ========================= */
	controlMutex = xSemaphoreCreateMutex();
	//Initialisation moteurs
	hControl.hMotors.m1_forward_channel = TIM_CHANNEL_1;
	hControl.hMotors.m1_reverse_channel = TIM_CHANNEL_2;
	hControl.hMotors.m2_forward_channel = TIM_CHANNEL_4; //J'ai échangé 3 et 4 pour que quand on veuille être en mode fwd, on met les deux roues sont dans le mm sens
	hControl.hMotors.m2_reverse_channel = TIM_CHANNEL_3;
	Motor_Init(&hControl.hMotors, &htim1);
	Motor_SetSpeed(&hControl.hMotors,0.15f);


//	hControl.hMotors.current_speed1 = hControl.hMotors.target_speed1; // directement pour le test
//	hControl.hMotors.current_speed2 = hControl.hMotors.target_speed2;
//	Motor_UpdateSpeed(&hControl.hMotors);
//ici controle de la vitesse

	//	hControl.hMotors.mode_mot1 = FORWARD_MODE;
	//	hControl.hMotors.mode_mot2 = FORWARD_MODE;
	//	Motor_SetMode(&hControl.hMotors);
	//	Motor_SetSpeed_percent(&hControl.hMotors, 40, 40);
	//	hControl.hMotors.current_speed1 = hControl.hMotors.target_speed1; // directement pour le test
	//	hControl.hMotors.current_speed2 = hControl.hMotors.target_speed2;
	//	Motor_UpdateSpeed(&hControl.hMotors);

	//initialisation encodeurs - odométrie
	Encodeur_Init();
	Odom_Init(&hControl.odom);
	//initialisation asservissement
	PID_Init(&pid_left,
	         0.0f,     // Kp
	         0.0f,     // Ki
	         -100.0f,
	         100.0f);

	PID_Init(&pid_right,
	         0.0f,
	         0.0f,
	         -100.0f,
	         100.0f);


	//initialisation carac vide pour les tofs
	//initialisation lidar
	//initialisation accelero

	/* ======================== CREATION TACHES ========================== */
	//  xTaskCreate(Task_OdomMotor,  "Odom et Moteurs",    STACK_SIZE_SMALL,  NULL, PRIO_ODOM_MOTOR, &xTaskOdomMotorHandle); //tâche odom motor
	//    xTaskCreate(Task_ToFs,  "ToFs",    STACK_SIZE_SMALL,  NULL, PRIO_TOF, &xTaskToFHandle);//Tâche tofs
	//tâche accelero, lidar
	//    xTaskCreate(Task_Control, "Control", STACK_SIZE_MEDIUM, NULL, PRIO_CTRL, &xTaskControlHandle); //Tâche Controle central


	//tâches tests
	//xTaskCreate(Task_TestMotors,  "Test moteurs et asservissement ",    STACK_SIZE_SMALL,  NULL, PRIO_ODOM_MOTOR, &xTaskTestHandle);



	vTaskStartScheduler();
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
	RCC_OscInitStruct.PLL.PLLM = RCC_PLLM_DIV1;
	RCC_OscInitStruct.PLL.PLLN = 8;
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
