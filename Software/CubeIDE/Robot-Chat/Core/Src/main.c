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
SemaphoreHandle_t controlMutex;
SemaphoreHandle_t sem_TOF;
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
			printf("x : %d, y : %d, z : %d\r\n",(int)acc[0],(int)acc[1],(int)acc[2]);
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
	ADXL_Init(&adxl);
	uint8_t mesured_axes = X_axes | Y_axes;
	uint8_t duration_choc = 0x1B;
	uint8_t threshold_choc = 0x21;
	ADXL_SetOffset(2,0,-63);
	ADXL_Measure(ON);
	for (;;){
		ADXL_enableSingleTap(INT2, mesured_axes,duration_choc, threshold_choc);
		xSemaphoreTake(sem_ADXL,portMAX_DELAY);
		if (xSemaphoreTake(controlMutex, pdMS_TO_TICKS(5)) == pdTRUE)
		        {
		            hControl.AccData = 1;
		            xSemaphoreGive(controlMutex);
		        }

		        vTaskDelay(pdMS_TO_TICKS(200));

		        if (xSemaphoreTake(controlMutex, pdMS_TO_TICKS(5)) == pdTRUE)
		        {
		            hControl.AccData = 0;
		            xSemaphoreGive(controlMutex);
		        }
		printf("Choc \r\n");
		vTaskDelay(500);
	}
}

/* ============================= */
/*       TASK   TOFS          */
/* ============================= */
void taskTOFDetection(void *unused)
{
    printf("[TOF] start\r\n");
    TOF_Init();

    if (HAL_TIM_Base_Start_IT(&htim17) != HAL_OK)
        printf("[TOF] Erreur Timer 17\r\n");

    // Debug : débloque une première fois
    xSemaphoreGive(sem_TOF);

    for (;;)
    {
        if (xSemaphoreTake(sem_TOF, portMAX_DELAY) != pdTRUE)
        {
            printf("[TOF] timeout sem_TOF (timer?)\r\n");
        }

        int new_vide = 0;

        if (data_read_TOF(VL53L0X_DEFAULT_ADDRESS, CHANNEL_0) == 1) new_vide = 1;
        if (new_vide == 0 && data_read_TOF(VL53L0X_DEFAULT_ADDRESS, CHANNEL_1) == 1) new_vide = 2;
        if (new_vide == 0 && data_read_TOF(VL53L0X_DEFAULT_ADDRESS, CHANNEL_2) == 1) new_vide = 3;
        if (new_vide == 0 && data_read_TOF(VL53L0X_DEFAULT_ADDRESS, CHANNEL_3) == 1) new_vide = 4;

        if (xSemaphoreTake(controlMutex, pdMS_TO_TICKS(5)) == pdTRUE)
        {
            hControl.vide = new_vide;
            xSemaphoreGive(controlMutex);
        }

        printf("[TOF] vide=%d\r\n", new_vide);
    }
}

/* ============================= */
/*       TASK  MOTOR       */
/* ============================= */
void Task_Motor(void *argument)
{
    for(;;)
    {
        // copies locales
        float t1, t2;
        int m1, m2;

        if (xSemaphoreTake(controlMutex, pdMS_TO_TICKS(10)) == pdTRUE)
        {
            t1 = hControl.hMotors.target_speed1;
            t2 = hControl.hMotors.target_speed2;
            m1 = hControl.hMotors.mode_mot1;
            m2 = hControl.hMotors.mode_mot2;
            xSemaphoreGive(controlMutex);

            // appliquer hors mutex
            hControl.hMotors.mode_mot1 = m1;
            hControl.hMotors.mode_mot2 = m2;
            Motor_SetMode(&hControl.hMotors);

            // si Motor_UpdateSpeed utilise target_speedX, assure-toi que t1/t2 sont déjà dans hControl.hMotors
            Motor_UpdateSpeed(&hControl.hMotors);
        }

        vTaskDelay(pdMS_TO_TICKS(20));
    }
}


/* ============================= */
/*       TASK   CONTROL          */
/* ============================= */
//void Task_Control(void *unused)
//{
//    printf("[CTRL] start\r\n");
//    for (;;)
//    {
//        int vide = 0;
//        int acc  = 0;
//
//        if (xSemaphoreTake(controlMutex, pdMS_TO_TICKS(5)) == pdTRUE)
//        {
//            vide = hControl.vide;
//            acc  = hControl.AccData;
//            xSemaphoreGive(controlMutex);
//			//process TOF
//            if (vide != 0) {
//				switch(vide){
//				case 1:
//					// Vide à gauche -> On tourne à droite
//					 hControl.hMotors.mode_mot1 = FORWARD_MODE;
//					 hControl.hMotors.mode_mot2 = REVERSE_MODE;
//					 Motor_SetMode(&hControl.hMotors);
//					 Motor_SetSpeed(&hControl.hMotors, 0.15f);
//					 break;
//				case 2:
//					//vide devant -> on recule
//					hControl.hMotors.mode_mot1 = REVERSE_MODE;
//					hControl.hMotors.mode_mot2 = REVERSE_MODE;
//					Motor_SetMode(&hControl.hMotors);
//					Motor_SetSpeed(&hControl.hMotors, 0.15f);
//					break;
//				case 3:
//					//vide à droite -> On tourne à gauche
//					hControl.hMotors.mode_mot1 = REVERSE_MODE;
//					hControl.hMotors.mode_mot2 = FORWARD_MODE;
//					Motor_SetMode(&hControl.hMotors);
//					Motor_SetSpeed(&hControl.hMotors, 0.15f);
//					break;
//				case 4:
//					//vide derrière -> On avance
//					hControl.hMotors.mode_mot1 = FORWARD_MODE;
//					hControl.hMotors.mode_mot2 = FORWARD_MODE;
//					Motor_SetMode(&hControl.hMotors);
//					Motor_SetSpeed(&hControl.hMotors, 0.15f);
//					break;
//				}
//            }
//            else{
//            	hControl.hMotors.mode_mot1 = FORWARD_MODE;
//            	hControl.hMotors.mode_mot2 = FORWARD_MODE;
//            	Motor_SetMode(&hControl.hMotors);
//            	Motor_SetSpeed(&hControl.hMotors, 0.15f);
//                //Process accelerometre : toujours forward (ou stop si choc)
//                if (acc == 1)
//                {
//                	//traitement avec lidar, voir comment est valeur de retour
//        //            hControl.hMotors.mode_mot1 = BRAKE_MODE;
//        //            hControl.hMotors.mode_mot2 = BRAKE_MODE;
//        //            Motor_SetMode(&hControl.hMotors);
//        //            Motor_SetSpeed_percent(&hControl.hMotors, 0, 0);
//                    printf("[CTRL] STOP (acc=1) vide=%d\r\n", vide);
//                }
//                else
//                {
//        //            hControl.hMotors.mode_mot1 = FORWARD_MODE;
//        //            hControl.hMotors.mode_mot2 = FORWARD_MODE;
//        //            Motor_SetMode(&hControl.hMotors);
//        //            Motor_SetSpeed_percent(&hControl.hMotors, 40, 40);
//                    printf("[CTRL] FWD vide=%d acc=%d\r\n", vide, acc);
//                }
//            }
//        }
//        vTaskDelay(pdMS_TO_TICKS(20));
//    }
//}

void Task_Control(void *unused)
{
    printf("[CTRL] start\r\n");

    for (;;)
    {
        int vide, acc;

        xSemaphoreTake(controlMutex, portMAX_DELAY);
        vide = hControl.vide;
        acc  = hControl.AccData;

        if (acc == 1)
        {
            hControl.hMotors.mode_mot1 = BRAKE_MODE;
            hControl.hMotors.mode_mot2 = BRAKE_MODE;
            Motor_SetMode(&hControl.hMotors);
            Motor_SetSpeed_percent(&hControl.hMotors, 0, 0);
            printf("[CTRL] STOP choc\r\n");
        }
        else if (vide == 1)
        {
            hControl.hMotors.mode_mot1 = FORWARD_MODE;
            hControl.hMotors.mode_mot2 = REVERSE_MODE;
            Motor_SetMode(&hControl.hMotors);
            Motor_SetSpeed(&hControl.hMotors, 0.15f);
            //printf("[CTRL] vide=1 -> droite\r\n");
        }
        else if (vide == 2)
        {
            hControl.hMotors.mode_mot1 = REVERSE_MODE;
            hControl.hMotors.mode_mot2 = REVERSE_MODE;
            Motor_SetMode(&hControl.hMotors);
            Motor_SetSpeed(&hControl.hMotors, 0.15f);
            //printf("[CTRL] vide=2 -> recule\r\n");
        }
        else if (vide == 3)
        {
            hControl.hMotors.mode_mot1 = REVERSE_MODE;
            hControl.hMotors.mode_mot2 = FORWARD_MODE;
            Motor_SetMode(&hControl.hMotors);
            Motor_SetSpeed(&hControl.hMotors, 0.15f);
            //printf("[CTRL] vide=3 -> gauche\r\n");
        }
        else if (vide == 4)
        {
            hControl.hMotors.mode_mot1 = FORWARD_MODE;
            hControl.hMotors.mode_mot2 = FORWARD_MODE;
            Motor_SetMode(&hControl.hMotors);
            Motor_SetSpeed(&hControl.hMotors, 0.15f);
            //printf("[CTRL] vide=4 -> avance\r\n");
        }
        else
        {
            // CAS NORMAL: avance
            hControl.hMotors.mode_mot1 = FORWARD_MODE;
            hControl.hMotors.mode_mot2 = FORWARD_MODE;
            Motor_SetMode(&hControl.hMotors);
            Motor_SetSpeed(&hControl.hMotors, 0.15f);
            //printf("[CTRL] avance\r\n");
        }

        xSemaphoreGive(controlMutex);

        vTaskDelay(pdMS_TO_TICKS(50));
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

	controlMutex = xSemaphoreCreateMutex();
	sem_TOF  = xSemaphoreCreateBinary();
	sem_ADXL = xSemaphoreCreateBinary();

	//initialisation lidar

	//Initialisation moteurs
	hControl.hMotors.m1_forward_channel = TIM_CHANNEL_1;
	hControl.hMotors.m1_reverse_channel = TIM_CHANNEL_2;
	hControl.hMotors.m2_forward_channel = TIM_CHANNEL_4; //J'ai échangé 3 et 4 pour que quand on veuille être en mode fwd, les deux roues sont dans le mm sens
	hControl.hMotors.m2_reverse_channel = TIM_CHANNEL_3;
	Motor_Init(&hControl.hMotors, &htim1);

	//lancement mode fwd
//		hControl.hMotors.mode_mot1 = FORWARD_MODE;
//		hControl.hMotors.mode_mot2 = FORWARD_MODE;
//		Motor_SetMode(&hControl.hMotors);
//		Motor_SetSpeed(&hControl.hMotors,0.15f);
//		hControl.hMotors.current_speed1 = hControl.hMotors.target_speed1; // directement pour le test
//		hControl.hMotors.current_speed2 = hControl.hMotors.target_speed2;
//		Motor_UpdateSpeed(&hControl.hMotors);

	//initialisation encodeurs - odométrie
	ControlData_Init();
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




	/* ======================== CREATION TACHES ========================== */
	if(xTaskCreate(taskTOFDetection,   "TOF",  1024, NULL, 4, NULL) != pdPASS){
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
//	if(xTaskCreate(Task_Motor, "Task_Motor", 1024, NULL, 4, NULL)!= pdPASS){
//		  printf("Error creating task motor\r\n");
//		  Error_Handler();
//}


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
