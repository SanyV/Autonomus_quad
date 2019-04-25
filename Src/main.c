
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * This notice applies to any and all portions of this file
  * that are not between comment pairs USER CODE BEGIN and
  * USER CODE END. Other portions of this file, whether 
  * inserted by the user or by software development tools
  * are owned by their respective copyright owners.
  *
  * Copyright (c) 2018 STMicroelectronics International N.V. 
  * All rights reserved.
  *
  * Redistribution and use in source and binary forms, with or without 
  * modification, are permitted, provided that the following conditions are met:
  *
  * 1. Redistribution of source code must retain the above copyright notice, 
  *    this list of conditions and the following disclaimer.
  * 2. Redistributions in binary form must reproduce the above copyright notice,
  *    this list of conditions and the following disclaimer in the documentation
  *    and/or other materials provided with the distribution.
  * 3. Neither the name of STMicroelectronics nor the names of other 
  *    contributors to this software may be used to endorse or promote products 
  *    derived from this software without specific written permission.
  * 4. This software, including modifications and/or derivative works of this 
  *    software, must execute solely and exclusively on microcontroller or
  *    microprocessor devices manufactured by or for STMicroelectronics.
  * 5. Redistribution and use of this software other than as permitted under 
  *    this license is void and will automatically terminate your rights under 
  *    this license. 
  *
  * THIS SOFTWARE IS PROVIDED BY STMICROELECTRONICS AND CONTRIBUTORS "AS IS" 
  * AND ANY EXPRESS, IMPLIED OR STATUTORY WARRANTIES, INCLUDING, BUT NOT 
  * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY, FITNESS FOR A 
  * PARTICULAR PURPOSE AND NON-INFRINGEMENT OF THIRD PARTY INTELLECTUAL PROPERTY
  * RIGHTS ARE DISCLAIMED TO THE FULLEST EXTENT PERMITTED BY LAW. IN NO EVENT 
  * SHALL STMICROELECTRONICS OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
  * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
  * LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, 
  * OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF 
  * LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING 
  * NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
  * EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
  *
  ******************************************************************************
  */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "stm32h7xx_hal.h"
#include "cmsis_os.h"
//#include "cmsis_os.h"

/* USER CODE BEGIN Includes */
#include "Config.h"
#include "MPU6500.h"
#include "ublox.h"
#include "ms5611_spi.h"
#include "hmc5983_spi.h"
#include "PWM_PPM.h"
#include "UKF_lib.h"
#include "ADC.h"

/* USER CODE END Includes */

/* Private variables ---------------------------------------------------------*/

extern SemaphoreHandle_t  xMPU_UKF_Mutex, xUKF_PID_Mutex, xMPU_Mutex, xSPI_Mutex,xMAG_Mutex,xBARO_Mutex, xADC_Mutex  ;

SPI_HandleTypeDef hspi1;

TIM_HandleTypeDef htim5;
TIM_HandleTypeDef htim6;
TIM_HandleTypeDef htim7;

UART_HandleTypeDef huart1;
UART_HandleTypeDef huart2;
DMA_HandleTypeDef hdma_usart1_rx;
DMA_HandleTypeDef hdma_usart1_tx;

DMA_HandleTypeDef hdma_usart2_rx;
DMA_HandleTypeDef hdma_usart2_tx;

osThreadId defaultTaskHandle;

/* USER CODE BEGIN PV */
/* Private variables ---------------------------------------------------------*/
extern uint32_t deltaTimeCOMM1, executionTimeCOMM1, previousCOMM1Time;
extern uint32_t deltaTimeCOMM2, executionTimeCOMM2, previousCOMM2Time;
extern uint32_t deltaTimeCLI,  	executionTimeCLI,  	previousCLITime;
extern uint32_t deltaTimeMPU,  	executionTimeMPU,  	previousMPUTime;
extern uint32_t deltaTimeGPS,  	executionTimeGPS,  	previousGPSTime;
extern uint32_t deltaTimeLED,   executionTimeLED,   previousLEDTime;
extern uint32_t deltaTimeUKF,   executionTimeUKF,   previousUKFTime;
extern uint32_t deltaTimeINS,   executionTimeINS,   previousINSTime;
extern uint32_t deltaTime1Hz,   executionTime1Hz,    previous1HzTime;
extern uint32_t deltaTimeBARO,   executionTimeBARO ,   previousBAROTime;
extern float dtBARO, dtCOMM1, dtCOMM2, dtCLI, dtMPU, dtGPS, dtLED, dtUKF, dtINS;
/* USER CODE END PV */



/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_SPI1_Init(void);
static void MX_TIM5_Init(void);
static void MX_TIM6_Init(void);
void MX_TIM7_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_USART2_UART_Init(void);
void StartDefaultTask(void const * argument);

/* USER CODE BEGIN PFP */
/* Private function prototypes -----------------------------------------------*/

/* USER CODE END PFP */

/* USER CODE BEGIN 0 */
void Leds_task( void *pvParameters )
    {
	while(1) {
		Led1_Toggle;
		//Led2_Toggle;
		//Led3_Toggle;
		//Led4_Toggle;
		vTaskDelay(250);
		}
}
void TaskLed_GPS( void *pvParameters )
{
	while(1)
	    {
		if (GPS_hAccf < NAV_MIN_GPS_ACC)
		    {
			Led2_On;
			vTaskDelay(600);
			Led2_Off;
			vTaskDelay(200);
		    }
		else
		{
			Led2_Off;
			vTaskDelay(100);
		}

	    }
}
void TaskLed_Arming( void *pvParameters )
{
	while(1)
	    {
		if (supervisorState&STATE_ARMED)
		    {
			Led3_On;
			vTaskDelay(900);
			Led3_Off;
			vTaskDelay(100);
		    }
		else
		    {
			Led3_Off;
			vTaskDelay(400);
		    }

	    }
}
/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  *
  * @retval None
  */
int main(void)
{
  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* Enable I-Cache-------------------------------------------------------------*/
  SCB_EnableICache();

  /* Enable D-Cache-------------------------------------------------------------*/
  SCB_EnableDCache();

  /* MCU Configuration----------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */
  /* add mutexes, ... */

    xUKF_PID_Mutex = xSemaphoreCreateMutex();
  	xGPS_UKF_Mutex = xSemaphoreCreateMutex();
  	xMPU_UKF_Mutex = xSemaphoreCreateMutex();
  	xMPU_Mutex = xSemaphoreCreateMutex();
 	xMAG_Mutex = xSemaphoreCreateMutex();
  	xBARO_Mutex = xSemaphoreCreateMutex();
	xSPI_Mutex = xSemaphoreCreateMutex();
	xADC_Mutex = xSemaphoreCreateMutex();
	StateNavUKF = 0;
  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_SPI1_Init();
  MX_TIM5_Init();
  MX_TIM6_Init();
  MX_TIM7_Init();
  MX_USART1_UART_Init();
  MX_USART2_UART_Init();
  /* USER CODE BEGIN 2 */
  Led_init();
  CommInit();
  CliInit();
  GPS_init();
  initPressure();
  initMag();
  MPU6500_Init_Full();
 // Baro_Mag_Init_Full();
  /******************************************/
  	Init_PWM_Reciever1();
  /******************************************/
  	Init_PWM_Motor(57100,40000);
	Init_SYS_ADC();

	xTaskCreate(prvArming,(signed char*)"Arming", 100, NULL, PRIORITET_TASK_ARMING, ( xTaskHandle * ) NULL);
  	xTaskCreate(TaskLed_GPS,(signed char*)"LED_GPS",70, NULL, PRIORITET_TASK_LEDS, ( xTaskHandle * ) NULL);
	xTaskCreate(TaskLed_Arming,(signed char*)"LED_Arm",70, NULL, PRIORITET_TASK_LEDS, ( xTaskHandle * ) NULL);

  	/* USER CODE END 2 */

  /* USER CODE BEGIN RTOS_MUTEX */


  //	xADC_Mutex = xSemaphoreCreateMutex();
  /* USER CODE END RTOS_MUTEX */

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* add semaphores, ... */
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
  cliPortPrintF("FreeHeapSize =  %6d, before Led_task \r\n", xPortGetFreeHeapSize());
  xTaskCreate(Leds_task,(char*)"LEDS",70, NULL, 5, ( xTaskHandle * ) NULL);
  /* USER CODE END RTOS_TIMERS */

  /* Create the thread(s) */
  /* definition and creation of defaultTask */
  cliPortPrintF("FreeHeapSize =  %6d, before Default_task \r\n", xPortGetFreeHeapSize());
  osThreadDef(defaultTask, StartDefaultTask, osPriorityNormal, 0, 128);
  defaultTaskHandle = osThreadCreate(osThread(defaultTask), NULL);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  cliPortPrintF("FreeHeapSize =  %6d, before Kernel \r\n", xPortGetFreeHeapSize());

  /* USER CODE END RTOS_THREADS */

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */
 

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

  RCC_OscInitTypeDef RCC_OscInitStruct;
  RCC_ClkInitTypeDef RCC_ClkInitStruct;
  RCC_PeriphCLKInitTypeDef PeriphClkInitStruct;

    /**Supply configuration update enable 
    */
  MODIFY_REG(PWR->CR3, PWR_CR3_SCUEN, 0);

    /**Configure the main internal regulator output voltage 
    */
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  while ((PWR->D3CR & (PWR_D3CR_VOSRDY)) != PWR_D3CR_VOSRDY) 
  {
    
  }
    /**Macro to configure the PLL clock source 
    */
  __HAL_RCC_PLL_PLLSOURCE_CONFIG(RCC_PLLSOURCE_HSE);

    /**Initializes the CPU, AHB and APB busses clocks 
    */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 4; //4;
  RCC_OscInitStruct.PLL.PLLN = 400; // 400 - рекомендуема¤, 512  - Overclocked; // 480 - наиболее стабильна¤ low jitter for UKF and MPU dT
  RCC_OscInitStruct.PLL.PLLP = 2;
  RCC_OscInitStruct.PLL.PLLQ = 9; //10; дл¤ 512 //8; дл¤ 400 // - пределы делител¤ дл¤ работы компаса datasheet утверждает о стабильной работе на SPI clock 3 ћгц у мен¤ получаетс¤ работа вплоть до 20ћгц с небольшим кр¤ком в обработчике прерывани¤/данных от компаса
  RCC_OscInitStruct.PLL.PLLR = 2;
  RCC_OscInitStruct.PLL.PLLRGE = RCC_PLL1VCIRANGE_1;
  RCC_OscInitStruct.PLL.PLLVCOSEL = RCC_PLL1VCOWIDE;
  RCC_OscInitStruct.PLL.PLLFRACN = 0;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Initializes the CPU, AHB and APB busses clocks 
    */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2
                              |RCC_CLOCKTYPE_D3PCLK1|RCC_CLOCKTYPE_D1PCLK1;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.SYSCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_HCLK_DIV2; //DIV2
  RCC_ClkInitStruct.APB3CLKDivider = RCC_APB3_DIV4;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_APB1_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_APB2_DIV4;
  RCC_ClkInitStruct.APB4CLKDivider = RCC_APB4_DIV4;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)  //2 latence normal at 400Mhz
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  PeriphClkInitStruct.PeriphClockSelection = RCC_PERIPHCLK_USART2|RCC_PERIPHCLK_USART1
                              |RCC_PERIPHCLK_SPI1;
  PeriphClkInitStruct.PLL2.PLL2M = 4;
  PeriphClkInitStruct.PLL2.PLL2N = 400;
  PeriphClkInitStruct.PLL2.PLL2P = 8;
  PeriphClkInitStruct.PLL2.PLL2Q = 8;
  PeriphClkInitStruct.PLL2.PLL2R = 2;
  PeriphClkInitStruct.PLL2.PLL2RGE = RCC_PLL2VCIRANGE_1;
  PeriphClkInitStruct.PLL2.PLL2VCOSEL = RCC_PLL2VCOWIDE;
  PeriphClkInitStruct.PLL2.PLL2FRACN = 0;
  PeriphClkInitStruct.Spi123ClockSelection = RCC_SPI123CLKSOURCE_PLL;
  PeriphClkInitStruct.Usart234578ClockSelection = RCC_USART234578CLKSOURCE_D2PCLK1; //RCC_USART234578CLKSOURCE_PLL2;
  PeriphClkInitStruct.Usart16ClockSelection = RCC_USART16CLKSOURCE_PLL2;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInitStruct) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Configure the Systick interrupt time 
    */
  HAL_SYSTICK_Config(SystemCoreClock/1000);

    /**Configure the Systick 
    */
  HAL_SYSTICK_CLKSourceConfig(SYSTICK_CLKSOURCE_HCLK);

  /* SysTick_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(SysTick_IRQn, 4, 0);
}

/* SPI1 init function */
static void MX_SPI1_Init(void)
{

  /* SPI1 parameter configuration*/

  hspi1.Instance = SPI1;
  hspi1.Init.Mode = SPI_MODE_MASTER;
  hspi1.Init.Direction = SPI_DIRECTION_2LINES;
  hspi1.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi1.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi1.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi1.Init.NSS = SPI_NSS_SOFT;
  hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_128;
  hspi1.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi1.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi1.Init.CRCPolynomial = 7;
  hspi1.Init.NSSPMode = SPI_NSS_PULSE_ENABLE;
  hspi1.Init.NSSPolarity = SPI_NSS_POLARITY_LOW;
  hspi1.Init.MasterKeepIOState = SPI_MASTER_KEEP_IO_STATE_ENABLE;
  hspi1.Init.FifoThreshold = SPI_FIFO_THRESHOLD_01DATA;
  hspi1.Init.TxCRCInitializationPattern = SPI_CRC_INITIALIZATION_ALL_ZERO_PATTERN;
  hspi1.Init.RxCRCInitializationPattern = SPI_CRC_INITIALIZATION_ALL_ZERO_PATTERN;
  hspi1.Init.MasterSSIdleness = SPI_MASTER_SS_IDLENESS_00CYCLE;
  hspi1.Init.MasterInterDataIdleness = SPI_MASTER_INTERDATA_IDLENESS_00CYCLE;
  hspi1.Init.MasterReceiverAutoSusp = SPI_MASTER_INTERDATA_IDLENESS_00CYCLE;
  //hspi1.Init.MasterKeepIOState = SPI_MASTER_KEEP_IO_STATE_DISABLE;
  hspi1.Init.IOSwap = SPI_IO_SWAP_DISABLE;

  if (HAL_SPI_Init(&hspi1) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

}
static void MX_TIM5_Init(void)
{
	  RCC_ClkInitTypeDef    clkconfig_tim;
	  uint32_t              uwTimclock_tim = 0;
	  uint32_t              uwPrescalerValue_tim = 0;
	  uint32_t              pFLatency_tim;
	/* Get clock configuration */
	  HAL_RCC_GetClockConfig(&clkconfig_tim, &pFLatency_tim);

	  /* Compute TIM14 clock */
	  uwTimclock_tim = 2*HAL_RCC_GetPCLK1Freq();

	  /* Compute the prescaler value to have TIM14 counter clock equal to 1MHz */
	  uwPrescalerValue_tim = (uint32_t) ((uwTimclock_tim / 10000000) - 1);

	  htim5.Instance = TIM5;
	  /* Initialize TIMx peripheral as follow:
	  + Period = [(TIM6CLK/10000) - 1]. to have a (0.1 mks) s time base.
	  + Prescaler = (uwTimclock/1000000 - 1) to have a 10MHz counter clock.
	  + ClockDivision = 0
	  + Counter direction = Up
	  */
	  htim5.Init.Period = 0xFFFFFFFF; //(10000000 / 1000) - 1;
	  htim5.Init.Prescaler = uwPrescalerValue_tim;
	  htim5.Init.ClockDivision = 0;
	  htim5.Init.CounterMode = TIM_COUNTERMODE_UP;
	  if(HAL_TIM_Base_Init(&htim5) != HAL_OK)
	  {
	    /* Start the TIM time Base generation in interrupt mode */
		    _Error_Handler(__FILE__, __LINE__);
	  }
	  HAL_TIM_Base_Start(&htim5);
}

/* TIM6 init function */
static void MX_TIM6_Init(void)
{
	  RCC_ClkInitTypeDef    clkconfig_tim;
	  uint32_t              uwTimclock_tim = 0;
	  uint32_t              uwPrescalerValue_tim = 0;
	  uint32_t              pFLatency_tim;
	/* Get clock configuration */
	  HAL_RCC_GetClockConfig(&clkconfig_tim, &pFLatency_tim);

	  /* Compute TIM6 clock */
	  uwTimclock_tim = 2*HAL_RCC_GetPCLK1Freq();

	  /* Compute the prescaler value to have TIM14 counter clock equal to 1MHz */
	  uwPrescalerValue_tim = (uint32_t) ((uwTimclock_tim / 10000000) - 1);

	  htim6.Instance = TIM6;
	  /* Initialize TIMx peripheral as follow:
	  + Period = [(TIM6CLK/10000) - 1]. to have a (0.1 mks) s time base.
	  + Prescaler = (uwTimclock/1000000 - 1) to have a 10MHz counter clock.
	  + ClockDivision = 0
	  + Counter direction = Up
	  */
	  htim6.Init.Period = (10000000 / 1000) - 1;
	  htim6.Init.Prescaler = uwPrescalerValue_tim;
	  htim6.Init.ClockDivision = 0;
	  htim6.Init.CounterMode = TIM_COUNTERMODE_UP;
	  if(HAL_TIM_Base_Init(&htim6) != HAL_OK)
	  {
	    /* Start the TIM time Base generation in interrupt mode */
		    _Error_Handler(__FILE__, __LINE__);
	  }
	  HAL_TIM_Base_Start(&htim6);
}
/* TIM7 init function */
void MX_TIM7_Init(void)
{

 // TIM_MasterConfigTypeDef sMasterConfig;

	  htim7.Instance = TIM7;
	  htim7.Init.Prescaler = (( HAL_RCC_GetPCLK2Freq()  / 1000000 ) - 1);	// PCLK2 / 2 = 2 MHz = 0.5 uSec Tick
	  htim7.Init.CounterMode = TIM_COUNTERMODE_UP;
	  htim7.Init.Period = 5000 - 1; //10000 тиков дл¤ 200√ц // 5000 тиков дл¤ 400√ц;
	  htim7.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
	  htim7.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
	//	htim6.Init.RepetitionCounter
	  if (HAL_TIM_Base_Init(&htim7) != HAL_OK)
	  {
	    Error_Handler();
	  }
	//  __HAL_TIM_SET_COUNTER(&htim6, 10000);// First pass value  200Hz
		HAL_TIM_Base_Start_IT(&htim7);
}


/* USART1 init function */
static void MX_USART1_UART_Init(void)
{

  huart1.Instance = USART1;
  huart1.Init.BaudRate = 230400; // 9600 дл¤ калибровки 115200; //921600; //230400;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  huart1.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart1.Init.Prescaler = UART_PRESCALER_DIV1;
  huart1.Init.FIFOMode = UART_FIFOMODE_DISABLE;
  huart1.Init.TXFIFOThreshold = UART_TXFIFO_THRESHOLD_1_8;
  huart1.Init.RXFIFOThreshold = UART_RXFIFO_THRESHOLD_1_8;
  huart1.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

}

/* USART2 init function */
static void MX_USART2_UART_Init(void)
{

  huart2.Instance = USART2;
  huart2.Init.BaudRate = 38400;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  huart2.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart2.Init.Prescaler = UART_PRESCALER_DIV1;
  huart2.Init.FIFOMode = UART_FIFOMODE_DISABLE;
  huart2.Init.TXFIFOThreshold = UART_TXFIFO_THRESHOLD_1_8;
  huart2.Init.RXFIFOThreshold = UART_RXFIFO_THRESHOLD_1_8;
  huart2.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

}

/** 
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void) 
{
  /* DMA controller clock enable */
  __HAL_RCC_DMA1_CLK_ENABLE();
  __HAL_RCC_DMA2_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA2_Stream2_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Stream0_IRQn, 14, 0);
  HAL_NVIC_EnableIRQ(DMA1_Stream0_IRQn);
  /* DMA2_Stream7_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Stream1_IRQn, 14, 0);
  HAL_NVIC_EnableIRQ(DMA1_Stream1_IRQn);
  /* DMA2_Stream2_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA2_Stream2_IRQn, 13, 0);
  HAL_NVIC_EnableIRQ(DMA2_Stream2_IRQn);
  /* DMA2_Stream7_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA2_Stream7_IRQn, 13, 0);
  HAL_NVIC_EnableIRQ(DMA2_Stream7_IRQn);

}
/** Configure pins as 
        * Analog 
        * Input 
        * Output
        * EVENT_OUT
        * EXTI
*/
static void MX_GPIO_Init(void)
{

  GPIO_InitTypeDef GPIO_InitStruct;

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOE_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOE, BARO_CS_Pin|MAG_CS_Pin, GPIO_PIN_SET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GYRO_CS_GPIO_Port, GYRO_CS_Pin, GPIO_PIN_SET);

  /*Configure GPIO pins : BARO_CS_Pin MAG_CS_Pin */
  GPIO_InitStruct.Pin = BARO_CS_Pin|MAG_CS_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

  /*Configure GPIO pin : MAG_INT_Pin */
//  GPIO_InitStruct.Pin = MAG_INT_Pin;
//  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
//  GPIO_InitStruct.Pull = GPIO_NOPULL;
//  HAL_GPIO_Init(MAG_INT_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : GYRO_INT_Pin */
  GPIO_InitStruct.Pin = GYRO_INT_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GYRO_INT_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : GYRO_CS_Pin */
  GPIO_InitStruct.Pin = GYRO_CS_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  HAL_GPIO_Init(GYRO_CS_GPIO_Port, &GPIO_InitStruct);

  /*
  __HAL_RCC_GPIOD_CLK_ENABLE();
  GPIO_InitStruct.Pin = SPI_MPU_Pin_CS;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  HAL_GPIO_Init(SPI_MPU_Port_Group_CS, &GPIO_InitStruct);
  CS_LOW;
*/
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/* StartDefaultTask function */
void StartDefaultTask(void const * argument)
{

  /* USER CODE BEGIN 5 */
	vTaskDelete(defaultTaskHandle);
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END 5 */ 
}

/**
  * @brief  Period elapsed callback in non blocking mode
  * @note   This function is called  when TIM14 interrupt took place, inside
  * HAL_TIM_IRQHandler(). It makes a direct call to HAL_IncTick() to increment
  * a global variable "uwTick" used as application time base.
  * @param  htim : TIM handle
  * @retval None
  */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  /* USER CODE BEGIN Callback 0 */

  /* USER CODE END Callback 0 */
  if (htim->Instance == TIM14) {
    HAL_IncTick();
  }
  /* USER CODE BEGIN Callback 1 */

  /* USER CODE END Callback 1 */
}

/**
  * @brief  This function is executed in case of error occurrence.
  * @param  file: The file name as string.
  * @param  line: The line in file as a number.
  * @retval None
  */
void _Error_Handler(char *file, int line)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  while(1)
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
void assert_failed(uint8_t* file, uint32_t line)
{ 
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/**
  * @}
  */

/**
  * @}
  */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
