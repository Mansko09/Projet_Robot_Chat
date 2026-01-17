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
#include <math.h>
#include <stdio.h>
#include "../drivers/control.h"
#include "semphr.h"

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define STACK_SIZE_SMALL   256
#define STACK_SIZE_MEDIUM  384

#define PRIO_CTRL    4
#define PRIO_TOF     3
#define PRIO_ODOM_MOTOR 5
#define PRIO_ACCEL   1
#define PRIO_LIDAR 2

#define TOF_TRESHHOLD 40


/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
h_Motor_t hMotors;
Odom_t odom;
Encodeur_t enc;
PID_t pid_left;
PID_t pid_right;

h_control_t hControl;
//FREERTOS
SemaphoreHandle_t sem_TimerTOF;
SemaphoreHandle_t sem_ADXL;

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


extern i2c_mux_t mux;

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

static void ControlData_Init(void)
{
	hControl.AccData = 0;
	hControl.vide    = 0;
}

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
	//printf("glapitouADXL\r\n");
	if (GPIO_Pin == GPIO_PIN_0 || GPIO_Pin == GPIO_PIN_1){
		ADXL_IntProto();
		int16_t acc[3] = {0,0,0};
		ADXL_getAccel(acc ,OUTPUT_SIGNED);
		//printf("x : %d, y : %d, z : %d\r\n",(int)acc[0],(int)acc[1],(int)acc[2]);
		ADXL_disableSingleTap();
		BaseType_t higher_priority_task_woken = pdFALSE;
		xSemaphoreGiveFromISR(sem_ADXL, &higher_priority_task_woken);
		portYIELD_FROM_ISR(higher_priority_task_woken);
	}
}



/* ============================= */
/*       TASK   ACCELERO          */
/* ============================= */
void taskAccelDetection(void * unused){
	printf("[ACC] Start \r\n");
	uint8_t mesured_axes = X_axes | Y_axes;
	uint8_t duration_choc =0x1B;
	uint8_t threshold_choc = 0x21;

	for (;;){
		ADXL_enableSingleTap(INT2, mesured_axes,duration_choc, threshold_choc);
		xSemaphoreTake(sem_ADXL,portMAX_DELAY);
		hControl.AccData=!hControl.AccData;
		//printf("Choc \r\n");
		vTaskDelay(50);
	}
}

/* ============================= */
/*       TASK   TOFS          */
/* ============================= */
void taskTOFDetection(void *unused)
{
	printf("[TOF] Task Loop Start\r\n");

	// Le démarrage du timer ici est correct car TOF_Init est déjà fait
	if (HAL_TIM_Base_Start_IT(&htim17) != HAL_OK)
		printf("[TOF] Erreur Timer 17\r\n");
	for (;;)
	{
		// On attend le top du Timer 17 (20ms)
		if (xSemaphoreTake(sem_TimerTOF, portMAX_DELAY) != pdTRUE)
		{
			printf("[TOF] timeout sem_TimerTOF\r\n");
			continue;
		}

		int new_vide = 0;
		// Canal 0 : Gauche
		if (data_read_TOF(VL53L0X_DEFAULT_ADDRESS, 0) == 1) {
			new_vide |= 1;
		}
		// Canal 1 : Centre / Avant
		if (data_read_TOF(VL53L0X_DEFAULT_ADDRESS, 1) == 1) {
			new_vide |= 2;
		}
		// Canal 2 : Droite
		if (data_read_TOF(VL53L0X_DEFAULT_ADDRESS, 2) == 1) {
			new_vide |= 4;
		}
		// Canal 3 : Arrière (On ne le lit que si l'avant est "safe")
		if (data_read_TOF(VL53L0X_DEFAULT_ADDRESS, 3) == 1) {
			new_vide |= 8;
		}

		if (new_vide != 0) {
			// PRIORITÉ ABSOLUE : Si on voit du vide, on coupe les moteurs DIRECTEMENT
			// Cela gagne les 20ms de latence de la Task_Control
			//Motor_CommandVelLR(&hControl.hMotors, -0.1f, -0.1f); // Petite pichenette arrière parce que sinon il prend trop de temps à s'arrêter
			hControl.vide = new_vide;
		}
		else {
			// On ne remet à zéro hControl.vide QUE si aucun capteur ne voit de vide
			hControl.vide = 0;
		}

		// --- DEBUG : Décommente la ligne ci-dessous pour voir les détections en temps réel ---
		// if(new_vide != 0) printf("[TOF] Detection Vide sur Canal : %d\r\n", new_vide);
	}
}

/* ============================= */
/*       TASK  MOTOR et odométrie      */
/* ============================= */
void Task_Motor(void *argument)
{
	Odom_t odom_local;
	odom_local=hControl.odom;
	for(;;)
	{
		Encodeur_Read(&enc);//lecture encodeurs
		Odom_Update(&odom_local,&enc,&odom_params);//mise à jour odométrie
		hControl.odom=odom_local;
		//printf("Position : x=%f,y=%f, theta=%f degres \r\n",hControl.odom.x,hControl.odom.y, hControl.odom.theta * 180.0f / M_PI);
		Motor_UpdateSpeed(&hControl.hMotors);
		vTaskDelay(pdMS_TO_TICKS(10));
	}
}



void Task_Control(void *unused)
{
	printf("[CTRL] start\r\n");
	// --- INITIALISATION DES LEDS ---
	// Au démarrage, le robot est inactif (Rouge allumé, Vert éteint)
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_SET);   // Rouge ON
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, GPIO_PIN_RESET); // Vert OFF
	// Vitesses augmentées
	const float V_FWD  = 0.18f;
	const float V_BACK = -0.14f;
	const float V_TURN = 0.14f;
	typedef enum { MOVE_FWD, MOVE_BRAKE, MOVE_BACKWARD, MOVE_TURN } state_t;
	state_t current_state = MOVE_FWD;
	int robot_active = 0;    // Le robot commence arrêté
	int last_acc_val = 0;    // Pour détecter le changement (front montant) sur AccData
	int timer_state = 0;
	int side_memory = 0;
	int last_turn_dir = 0;
	for (;;)
	{
		int vide;
		int current_acc;
		float vL = 0.0f;
		float vR = 0.0f;
		//lecture données
		vide = hControl.vide;
		current_acc = hControl.AccData; // Supposons que AccData passe à 1 lors d'un choc
		// --- LOGIQUE START/STOP (ACCÉLÉRO) ---
		if (current_acc == 1 && last_acc_val == 0) {
			robot_active = !robot_active;
			printf("[CTRL] ACC CHOC ! Robot %s\n", robot_active ? "START" : "STOP");

			if (robot_active) {
				// PASSAGE EN MARCHE : Vert ON, Rouge OFF
				HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, GPIO_PIN_SET);   // Vert
				HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_RESET); // Rouge

				current_state = MOVE_FWD;
			} else {
				// PASSAGE EN ARRÊT : Rouge ON, Vert OFF
				HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_SET);   // Rouge
				HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, GPIO_PIN_RESET); // Vert

				current_state = MOVE_BRAKE;
				timer_state = 1;
			}
		}
		last_acc_val = current_acc;
		// --- MACHINE À ÉTATS ---
		if (robot_active) {
			switch (current_state)
			{
			case MOVE_FWD:
				if ((vide & 1) || (vide & 2) || (vide & 4)) {
					side_memory = vide;
					current_state = MOVE_BRAKE;
					timer_state = 1; // Freinage immédiat
					printf("[CTRL] DANGER (%d) -> STOP\r\n", vide);
				} else {
					vL = V_FWD; vR = V_FWD;
				}
				break;
			case MOVE_BRAKE:
				// Envoie juste une commande de vitesse nulle.
				vL = -0.05f;
				vR = -0.05f;

				timer_state--;
				if (timer_state <= 0) {
					current_state = MOVE_BACKWARD;
					timer_state = 60;
					printf("[CTRL] Debut Recul 1.2s\r\n");
				}
				break;
			case MOVE_BACKWARD:
				if (vide & 8) { // Sécurité Arrière
					vL = 0.0f; vR = 0.0f;
					current_state = MOVE_TURN;
					printf("[CTRL] DANGER (%d) -> STOP\r\n", vide);
					timer_state = 30; // Rotation plus rapide
				} else {
					vL = V_BACK; vR = V_BACK;
					timer_state--;
					if (timer_state <= 0) {
						current_state = MOVE_TURN;
						timer_state = 35; // Environ 90° à 0.15f
					}
				}
				break;
			case MOVE_TURN:
				// Si vide détecté pendant rotation, on repart en arrière
				if (((vide & 1) || (vide & 2) || (vide & 4)) && ((vide & 8) == 0)) {
					current_state = MOVE_BRAKE;
					timer_state = 1;
					break;
				}

				if (side_memory & 1 || side_memory & 2) { // Gauche ou Centre -> Droite
					vL = V_TURN; vR = -V_TURN;
					last_turn_dir = 2;
				} else if (side_memory & 4) { // Droite -> Gauche
					vL = -V_TURN; vR = V_TURN;
					last_turn_dir = 1;
				} else {
					if (last_turn_dir == 1) { vL = -V_TURN; vR = V_TURN; }
					else { vL = V_TURN; vR = -V_TURN; }
				}

				timer_state--;
				if (timer_state <= 0) {
					current_state = MOVE_FWD;
					side_memory = 0;
				}
				break;
			}
		} else {
		    // Robot inactif : On coupe tout pour le silence et la batterie
		    vL = 0.0f; vR = 0.0f;
		    hControl.hMotors.mode_mot1 = STANDBY_MODE;
		    hControl.hMotors.mode_mot2 = STANDBY_MODE;
		    Motor_SetMode(&hControl.hMotors);
		}
		// --- ENVOI COMMANDES ---
		Motor_CommandVelLR(&hControl.hMotors, vL, vR);
		vTaskDelay(pdMS_TO_TICKS(20));
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
	MX_TIM17_Init();
	/* USER CODE BEGIN 2 */
	printf(" ------ FELIX READY ------\r\n ");
	/* ======================== INIT ========================= */
	/* 1. RESET LOGICIEL I2C (Pour débloquer le bus si gelé) */
	__HAL_I2C_DISABLE(&hi2c3);
	HAL_Delay(10);
	__HAL_I2C_ENABLE(&hi2c3);

	//initialisation lidar

	//intialisation TOFS

	if (TOF_Init() != 1) {
		printf("[TOF] FATAL ERROR: Panne materielle I2C\r\n");
		// On ne bloque pas tout le robot, mais on met un flag d'erreur
		hControl.vide = -1;
	} else {
		printf("[TOF] Initialisation reussie !\r\n");
	}

	sem_TimerTOF = xSemaphoreCreateBinary();
	sem_ADXL = xSemaphoreCreateBinary();

	//Initialisation moteurs
	hControl.hMotors.m1_forward_channel = TIM_CHANNEL_1;
	hControl.hMotors.m1_reverse_channel = TIM_CHANNEL_2;
	hControl.hMotors.m2_forward_channel = TIM_CHANNEL_4; //J'ai échangé 3 et 4 pour que quand on veuille être en mode fwd, les deux roues sont dans le mm sens
	hControl.hMotors.m2_reverse_channel = TIM_CHANNEL_3;
	Motor_Init(&hControl.hMotors, &htim1);

//	hControl.hMotors.speed_ramp1 = 1500; // 3200 / 1500 => ~2 itérations pour l'arrêt
//	hControl.hMotors.speed_ramp2 = 1500;

	//initialisation encodeurs - odométrie
	ControlData_Init();
	Encodeur_Init();
	//initialisation accelero
	hControl.AccData=0;
	ADXL_Init(&adxl);
	ADXL_SetOffset(2,0,-63);
	ADXL_Measure(ON);

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

	/* ======================== CONFIGURATION INITIALE MOUVEMENT ========================= */

//	hControl.hMotors.target_speed1 = 0;
//	hControl.hMotors.target_speed2 = 0;
//	hControl.hMotors.current_speed1 = 0;
//	hControl.hMotors.current_speed2 = 0;
//	hControl.hMotors.mode_mot1 = STANDBY_MODE;
//	hControl.hMotors.mode_mot2 = STANDBY_MODE;
//	Motor_SetMode(&hControl.hMotors);


	/* ======================== CREATION TACHES ========================== */
	if(xTaskCreate(taskTOFDetection,   "TOF",  1024, NULL, 2, NULL) != pdPASS){
		printf("Error creating task detection\r\n");
		Error_Handler();
	}
	if(xTaskCreate(taskAccelDetection, "ACC",   512, NULL, 1, NULL)!= pdPASS){
		printf("Error creating task accel\r\n");
		Error_Handler();
	}

	if(xTaskCreate(Task_Control, "CTRL", 1024, NULL, 3, NULL)!= pdPASS){
		printf("Error creating task ctrl\r\n");
		Error_Handler();
	}
	if(xTaskCreate(Task_Motor, "Task_Motor", 1024, NULL, 4, NULL)!= pdPASS){
		printf("Error creating task motor\r\n");
		Error_Handler();
	}


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
	else if (htim->Instance == TIM17)
	{
		if (sem_TimerTOF != NULL)
		{
			BaseType_t hpw = pdFALSE;
			xSemaphoreGiveFromISR(sem_TimerTOF, &hpw);
			portYIELD_FROM_ISR(hpw);
		}
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
