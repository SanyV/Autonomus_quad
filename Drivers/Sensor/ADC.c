#include "ADC.h"
#include "stm32h7xx_hal.h"
#include "Config.h"

//Коэффициенты преоюразования АЦП
#define K_ADC_VOLTAGE 					0.000712890625f//ADC to Volt

#define K_VOLTAGE_TO_ALL_CURRENT		2.0f*54.0f//VOLTAGE to ALL CURRENT


#define K_PRESCALER_VOLTAGE 			6.0204f//PRESCALER ALL VOLTAGE

#define K_ADC_ALL_CURRENT 				K_VOLTAGE_TO_ALL_CURRENT*K_ADC_VOLTAGE


#define K_ADC_ALL_VOLTAGE 				K_PRESCALER_VOLTAGE*K_ADC_VOLTAGE
//***************************************************************************************************//

#define CAPACITY_BATT	 				4500.0f
//***************************************************************************************************//
//Коэффициенты экспоненциального сглаживания
#define DT_ADC							1.0f/100.0f
#define F_CUT_DIFF_ADC_CURRENT			4.61f
#define K_EXP_DIFF_ADC_CURRENT			(float)(1.0f - 2.0f*M_PI*F_CUT_DIFF_ADC_CURRENT*DT_ADC/(1.0f+2.0f*M_PI*F_CUT_DIFF_ADC_CURRENT*DT_ADC))

#define F_CUT_DIFF_ADC_VOLTAGE			20.61f
#define K_EXP_DIFF_ADC_VOLTAGE			(float)(1.0f - 2.0f*M_PI*F_CUT_DIFF_ADC_VOLTAGE*DT_ADC/(1.0f+2.0f*M_PI*F_CUT_DIFF_ADC_VOLTAGE*DT_ADC))

#define F_CUT_DIFF_ADC_VOLTAGE_SM		0.08f
#define K_EXP_DIFF_ADC_VOLTAGE_SM		(float)(1.0f - 2.0f*M_PI*F_CUT_DIFF_ADC_VOLTAGE_SM*DT_ADC/(1.0f+2.0f*M_PI*F_CUT_DIFF_ADC_VOLTAGE_SM*DT_ADC))
//***************************************************************************************************//

//Вспомогательные промежуточные переменные
//uint16_t aADCxConvertedData[6]; /* буфер системного АЦП */
//****************************************************//
//Для ОС
SemaphoreHandle_t xADC_Semaphore = NULL;
SemaphoreHandle_t xADC_Mutex;
TaskHandle_t adc_handle;
uint32_t ovf_adc_stack;
//****************************************************//
//Счетчики цикла
uint32_t cnt_cycle_adc, cnt_ticks_adc;
//****************************************************//


//Главные перменные
float All_voltage, All_power, All_action, All_voltage_smooth, All_Current, All_percent_cap, All_capacity_batt;
//****************************************************//
/* Definition of ADCx conversions data table size */
#define ADC_CONVERTED_DATA_BUFFER_SIZE   ((uint32_t)  32)   /* Size of array aADCxConvertedData[] */

/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* ADC handle declaration */
ADC_HandleTypeDef             AdcHandle;

/* ADC channel configuration structure declaration */
ADC_ChannelConfTypeDef        sConfig;

/* Variable containing ADC conversions data */
ALIGN_32BYTES (static uint16_t   aADCxConvertedData[ADC_CONVERTED_DATA_BUFFER_SIZE]);
static DMA_HandleTypeDef         DmaHandle;

void HAL_ADC_MspInit(ADC_HandleTypeDef *hadc)
{
  GPIO_InitTypeDef          GPIO_InitStruct;


  /*##-1- Enable peripherals and GPIO Clocks #################################*/
  /* Enable GPIO clock ****************************************/
  __HAL_RCC_GPIOA_CLK_ENABLE();
  /* ADC Periph clock enable */
  __HAL_RCC_ADC12_CLK_ENABLE();
  /* ADC Periph interface clock configuration */
  __HAL_RCC_ADC_CONFIG(RCC_ADCCLKSOURCE_CLKP);
  /* Enable DMA clock */
  __HAL_RCC_DMA1_CLK_ENABLE();

  /*##- 2- Configure peripheral GPIO #########################################*/
  /* ADC Channel GPIO pin configuration */
  GPIO_InitStruct.Pin = GPIO_PIN_0 | GPIO_PIN_1 | GPIO_PIN_2 | GPIO_PIN_3;
  GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
  /*##- 3- Configure DMA #####################################################*/

  /*********************** Configure DMA parameters ***************************/
  DmaHandle.Instance                 = DMA1_Stream2;
  DmaHandle.Init.Request             = DMA_REQUEST_ADC1;
  DmaHandle.Init.Direction           = DMA_PERIPH_TO_MEMORY;
  DmaHandle.Init.PeriphInc           = DMA_PINC_DISABLE;
  DmaHandle.Init.MemInc              = DMA_MINC_ENABLE;
  DmaHandle.Init.PeriphDataAlignment = DMA_PDATAALIGN_HALFWORD;
  DmaHandle.Init.MemDataAlignment    = DMA_MDATAALIGN_HALFWORD;
  DmaHandle.Init.Mode                = DMA_CIRCULAR;
  DmaHandle.Init.Priority            = DMA_PRIORITY_MEDIUM;
  /* Deinitialize  & Initialize the DMA for new transfer */
  HAL_DMA_DeInit(&DmaHandle);
  HAL_DMA_Init(&DmaHandle);

  /* Associate the DMA handle */
  __HAL_LINKDMA(hadc, DMA_Handle, DmaHandle);

  /* NVIC configuration for DMA Input data interrupt */
  HAL_NVIC_SetPriority(DMA1_Stream2_IRQn, 13, 0);
  HAL_NVIC_EnableIRQ(DMA1_Stream2_IRQn);
}

/**
  * @brief ADC MSP De-Initialization
  *        This function frees the hardware resources used in this example:
  *          - Disable the Peripheral's clock
  *          - Revert GPIO to their default state
  * @param hadc: ADC handle pointer
  * @retval None
  */
void HAL_ADC_MspDeInit(ADC_HandleTypeDef *hadc)
{
  /*##-1- Reset peripherals ##################################################*/
 // ADCx_FORCE_RESET();
 // ADCx_RELEASE_RESET();
  /* ADC Periph clock disable
   (automatically reset all ADC instances of the ADC common group) */
  __HAL_RCC_ADC12_CLK_DISABLE();

  /*##-2- Disable peripherals and GPIO Clocks ################################*/
  /* De-initialize the ADC Channel GPIO pin */
  HAL_GPIO_DeInit(GPIOA, GPIO_PIN_0 | GPIO_PIN_1 | GPIO_PIN_2 | GPIO_PIN_3);
}
void Init_SYS_ADC(void)
{

	//создание двоичного семафора для передачи обработки данных
	vSemaphoreCreateBinary(xADC_Semaphore);
	//задача обработки данных АЦП
	xTaskCreate(prvOperation_with_ADC_results,(char*)"ADC_Processing", 150, NULL, PRIORITET_TASK_OPERATION_ADC, adc_handle);
	//сглаженному напряжению присвоено значнение батареи
	All_voltage_smooth = 16.8f;

	 /* ### - 1 - Initialize ADC peripheral #################################### */
	  AdcHandle.Instance          = ADC1;
	  if (HAL_ADC_DeInit(&AdcHandle) != HAL_OK)
	  {
	    /* ADC de-initialization Error */
	    Error_Handler();
	  }

	  AdcHandle.Init.ClockPrescaler           = ADC_CLOCK_SYNC_PCLK_DIV4;      /* Synchronous clock mode, input ADC clock divided by 4*/
	  AdcHandle.Init.Resolution               = ADC_RESOLUTION_12B;            /* 16-bit resolution for converted data */
	  AdcHandle.Init.ScanConvMode             = ENABLE;                       /* Sequencer disabled (ADC conversion on only 1 channel: channel set on rank 1) */
	  AdcHandle.Init.EOCSelection             = ADC_EOC_SINGLE_CONV;           /* EOC flag picked-up to indicate conversion end */
	  AdcHandle.Init.LowPowerAutoWait         = DISABLE;                       /* Auto-delayed conversion feature disabled */
	  AdcHandle.Init.ContinuousConvMode       = ENABLE;                        /* Continuous mode enabled (automatic conversion restart after each conversion) */
	  AdcHandle.Init.NbrOfConversion          = 4;                             /* Parameter discarded because sequencer is disabled */
	  AdcHandle.Init.DiscontinuousConvMode    = DISABLE;                       /* Parameter discarded because sequencer is disabled */
	  AdcHandle.Init.NbrOfDiscConversion      = 1;                             /* Parameter discarded because sequencer is disabled */
	  AdcHandle.Init.ExternalTrigConv         = ADC_SOFTWARE_START;            /* Software start to trig the 1st conversion manually, without external event */
	  AdcHandle.Init.ExternalTrigConvEdge     = ADC_EXTERNALTRIGCONVEDGE_NONE; /* Parameter discarded because software trigger chosen */
	  AdcHandle.Init.ConversionDataManagement = ADC_CONVERSIONDATA_DMA_CIRCULAR; /* ADC DMA circular requested */
	  AdcHandle.Init.Overrun                  = ADC_OVR_DATA_OVERWRITTEN;      /* DR register is overwritten with the last conversion result in case of overrun */
	  AdcHandle.Init.OversamplingMode         = DISABLE;                       /* No oversampling */
	  AdcHandle.Init.BoostMode                = ENABLE;                        /* Enable Boost mode as ADC clock frequency is bigger than 20 MHz */
	  /* Initialize ADC peripheral according to the passed parameters */
	  if (HAL_ADC_Init(&AdcHandle) != HAL_OK)
	  {
	    Error_Handler();
	  }


	  /* ### - 2 - Start calibration ############################################ */
	  if (HAL_ADCEx_Calibration_Start(&AdcHandle, ADC_CALIB_OFFSET, ADC_SINGLE_ENDED) != HAL_OK)
	  {
	    Error_Handler();
	  }

	  /* ### - 3 - Channel configuration ######################################## */
	  sConfig.Channel      = ADC_CHANNEL_1;                /* Sampled channel number */
	  sConfig.Rank         = ADC_REGULAR_RANK_1;          /* Rank of sampled channel number ADCx_CHANNEL */
	  sConfig.SamplingTime = ADC_SAMPLETIME_387CYCLES_5;   /* Sampling time (number of clock cycles unit) */
	  sConfig.SingleDiff   = ADC_SINGLE_ENDED;            /* Single-ended input channel */
	  sConfig.OffsetNumber = ADC_OFFSET_NONE;             /* No offset subtraction */
	  sConfig.Offset = 0;                                 /* Parameter discarded because offset correction is disabled */
	  if (HAL_ADC_ConfigChannel(&AdcHandle, &sConfig) != HAL_OK)
	  {
	    Error_Handler();
	  }

	  sConfig.Channel      = ADC_CHANNEL_2;                /* Sampled channel number */
	  sConfig.Rank         = ADC_REGULAR_RANK_2;          /* Rank of sampled channel number ADCx_CHANNEL */
	  if (HAL_ADC_ConfigChannel(&AdcHandle, &sConfig) != HAL_OK)
	  {
	    Error_Handler();
	  }

	  sConfig.Channel      = ADC_CHANNEL_3;                /* Sampled channel number */
	  sConfig.Rank         = ADC_REGULAR_RANK_3;          /* Rank of sampled channel number ADCx_CHANNEL */
	  if (HAL_ADC_ConfigChannel(&AdcHandle, &sConfig) != HAL_OK)
	  {
	    Error_Handler();
	  }
	  sConfig.Channel      = ADC_CHANNEL_4;                /* Sampled channel number */
	  sConfig.Rank         = ADC_REGULAR_RANK_4;          /* Rank of sampled channel number ADCx_CHANNEL */
	  if (HAL_ADC_ConfigChannel(&AdcHandle, &sConfig) != HAL_OK)
	  {
	    Error_Handler();
	  }
	  /* ### - 4 - Start conversion in DMA mode ################################# */
	  if (HAL_ADC_Start_DMA(&AdcHandle,
	                        (uint32_t *)aADCxConvertedData,
	                        ADC_CONVERTED_DATA_BUFFER_SIZE
	                       ) != HAL_OK)
	  {
	    Error_Handler();
	  }
}

void DMA1_Stream2_IRQHandler(void)
{
	static portBASE_TYPE xHigherPriorityTaskWoken_ADC = pdFALSE;
  /* USER CODE BEGIN DMA1_Stream1_IRQn 0 */

  /* USER CODE END DMA1_Stream1_IRQn 0 */
  HAL_DMA_IRQHandler(&DmaHandle);
  /* USER CODE BEGIN DMA1_Stream1_IRQn 1 */
	xSemaphoreGiveFromISR(xADC_Semaphore, &xHigherPriorityTaskWoken_ADC);
      //принудительное переключение контекста для уменьшения времени реакции на прерывание
      if( xHigherPriorityTaskWoken_ADC != pdFALSE )
      {
      	taskYIELD();
      }

  /* USER CODE END DMA1_Stream1_IRQn 1 */
}

void prvOperation_with_ADC_results(void *pvParameters)
    {
	float volt, volt_s=16.8f, curr_all = 0.0f, action=0.0f, power=0.0f, capacity_batt = CAPACITY_BATT, percent_cap;
	float al_c_cal;
	static float all_current_sum_calib;
	static float all_current_offset;
	static uint8_t cnt_current_calib, cnt_delay_start_calib;


	while(1)
	    {
		xSemaphoreTake(xADC_Semaphore, portMAX_DELAY);
	//	cnt_cycle_adc = (uint32_t)DWT->CYCCNT - cnt_ticks_adc;
	//	cnt_ticks_adc = (uint32_t)DWT->CYCCNT;


		al_c_cal = -(float)(aADCxConvertedData[0])*K_ADC_ALL_CURRENT;

		//задержка для установления постоянного напряжения
		if (cnt_delay_start_calib <255)
		    {
			cnt_delay_start_calib++;
		    }
		else
		    {
			if (cnt_current_calib < 128)
			    {
				//находим смещение
				cnt_current_calib++;
				all_current_sum_calib+=al_c_cal;
			    }
			else
			    {
				if (cnt_current_calib == 128)
				    {
					all_current_offset = (float)(all_current_sum_calib/(float)cnt_current_calib);
					cnt_current_calib++;
				    }

				//считаем с учетом смещения
				curr_all = ((float)K_EXP_DIFF_ADC_CURRENT*curr_all+(1.0f-(float)K_EXP_DIFF_ADC_CURRENT)*(al_c_cal-all_current_offset));
			    }
		    }

		volt =K_EXP_DIFF_ADC_VOLTAGE*volt+(1.0f-K_EXP_DIFF_ADC_VOLTAGE)*(K_ADC_ALL_VOLTAGE*(float)aADCxConvertedData[1]);

		power = curr_all*volt;
		action = action+(curr_all*0.277777f)*DT_ADC;

		volt_s = K_EXP_DIFF_ADC_VOLTAGE_SM*volt_s+(1.0f-K_EXP_DIFF_ADC_VOLTAGE_SM)*volt;
		if (capacity_batt>action)
		percent_cap = 100.0f*(capacity_batt - action)/capacity_batt;


		if ((counter_sys > 15000)&&(volt < 11.5f)&&(volt > 8.0f))
		    {
			supervisorState |= STATE_LOW_BATTERY;
		//	buzzer_flag = BUZZER_FLAG_VOLTAGE_LOW;
		    }
		//защитить
		xSemaphoreTake(xADC_Mutex, portMAX_DELAY);
		All_Current = curr_all;
		All_voltage = volt;
		All_action = action;
		All_power = power;
		All_voltage_smooth = volt_s;
		All_percent_cap = percent_cap;
		xSemaphoreGive(xADC_Mutex);
		//taskEXIT_CRITICAL();

		ovf_adc_stack = uxTaskGetStackHighWaterMark(adc_handle);

	//	xSemaphoreGive(xSD_ADC_collect_Semaphore);
		////////////////////
	    }
    }
