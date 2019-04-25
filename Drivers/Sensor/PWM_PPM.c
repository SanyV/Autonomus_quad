


#include "stm32h7xx.h"
#include "stm32h7xx_hal.h"
#include "Config.h"

#include "PWM_PPM.h"
#include "ublox.h"
#include "MPU6500.h"
#include "IMU_INS.h"
#include "UKF_lib.h"
#include "hmc5983_spi.h"
#include "ms5611_spi.h"
#include "NAV_POINTS.h"

TIM_HandleTypeDef htim4;
TIM_HandleTypeDef htim1;


void HAL_TIM_IC_MspInit(TIM_HandleTypeDef* htim_ic);
void HAL_TIM_IC_MspDeInit(TIM_HandleTypeDef* htim_ic);
void HAL_TIM_PWM_MspInit(TIM_HandleTypeDef *htim);

//Вспомогательыне промежуточные данные
#define PPM_NCNT 16		 /* макс количество пропусков ППМ */
uint8_t	ppm1_nimp[10];		/* значения счетчиков пропусков ППМ */
uint16_t ppm1_buf[10]; 		/* буфер для значений каналов ППМ */
uint16_t RSSI;
//****************************************************//
//Главные данные с приемника
volatile float throtle_ppm, yaw_rate_ppm, pitch_ppm, roll_ppm;
volatile float k_vel_auto_ppm;
uint16_t option_channel_1, option_channel_2, option_channel_3;
uint16_t M1_lev_front, M2_prav_back, M3_prav_front, M4_lev_back;
//****************************************************//
//Счетчики циклов
uint32_t cnt_cycle_ppm, cnt_ticks_ppm;
uint32_t cnt_cycle_pwm, cnt_ticks_pwm;

volatile uint8_t flagSavePoint;
volatile uint8_t rc_channel, data_ready;

void Init_PWM_Reciever1(void)
{

	  TIM_ClockConfigTypeDef sClockSourceConfig;
	  TIM_SlaveConfigTypeDef sSlaveConfig;
	  TIM_MasterConfigTypeDef sMasterConfig;
	  TIM_IC_InitTypeDef sConfigIC;

	  data_ready = 0;
	  data_ready = 0;
	  flagSavePoint = 0;

	  htim4.Instance = TIM4;
	  htim4.Init.Prescaler = 127;
	  htim4.Init.CounterMode = TIM_COUNTERMODE_UP;
	  htim4.Init.Period = 65535;
	  htim4.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
	  htim4.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
	  if (HAL_TIM_Base_Init(&htim4) != HAL_OK)
	  {
	    _Error_Handler(__FILE__, __LINE__);
	  }

	  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
	  if (HAL_TIM_ConfigClockSource(&htim4, &sClockSourceConfig) != HAL_OK)
	  {
	    _Error_Handler(__FILE__, __LINE__);
	  }

	  if (HAL_TIM_IC_Init(&htim4) != HAL_OK)
	  {
	    _Error_Handler(__FILE__, __LINE__);
	  }

	  sSlaveConfig.SlaveMode = TIM_SLAVEMODE_RESET;
	  sSlaveConfig.InputTrigger = TIM_TS_TI1FP1;
	  sSlaveConfig.TriggerPolarity = TIM_INPUTCHANNELPOLARITY_BOTHEDGE; //TIM_INPUTCHANNELPOLARITY_RISING;
	  sSlaveConfig.TriggerFilter = 0;
	  if (HAL_TIM_SlaveConfigSynchronization(&htim4, &sSlaveConfig) != HAL_OK)
	  {
	    _Error_Handler(__FILE__, __LINE__);
	  }

	  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
	  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
	  if (HAL_TIMEx_MasterConfigSynchronization(&htim4, &sMasterConfig) != HAL_OK)
	  {
	    _Error_Handler(__FILE__, __LINE__);
	  }

	  sConfigIC.ICPolarity = TIM_INPUTCHANNELPOLARITY_RISING;
	  sConfigIC.ICSelection = TIM_ICSELECTION_DIRECTTI;
	  sConfigIC.ICPrescaler = TIM_ICPSC_DIV1;
	  sConfigIC.ICFilter = 0;
	  if (HAL_TIM_IC_ConfigChannel(&htim4, &sConfigIC, TIM_CHANNEL_1) != HAL_OK)
	  {
	    _Error_Handler(__FILE__, __LINE__);
	  }

	  if (HAL_TIM_IC_ConfigChannel(&htim4, &sConfigIC, TIM_CHANNEL_2) != HAL_OK)
	  {
	    _Error_Handler(__FILE__, __LINE__);
	  }
	  /*##-3- Start the Input Capture in interrupt mode ##########################*/
	   if(HAL_TIM_IC_Start_IT(&htim4, TIM_CHANNEL_1) != HAL_OK)
	   {
	     /* Starting Error */
		    _Error_Handler(__FILE__, __LINE__);
	   }

}
void Init_PWM_Motor(uint16_t Period1, uint16_t Dp1)
{
	/* Timer Output Compare Configuration Structure declaration */
	TIM_OC_InitTypeDef sConfig;
//	uint32_t Prescaler;

	M1_lev_front  = Dp1;
	M2_prav_back  = Dp1;
	M3_prav_front = Dp1;
	M4_lev_back   = Dp1;

	  htim1.Instance = PWM_M;
//	  Prescaler = (SystemCoreClock / (2*20000000)) - 1;
	  htim1.Init.Prescaler         = 4; //Prescaler;
	  htim1.Init.Period            = Period1;
	  htim1.Init.ClockDivision     = 0;
	  htim1.Init.CounterMode       = TIM_COUNTERMODE_UP;
	  htim1.Init.RepetitionCounter = 0;
	  if (HAL_TIM_PWM_Init(&htim1) != HAL_OK)
	  {
	    /* Initialization Error */
		    _Error_Handler(__FILE__, __LINE__);
	  }

	  /*##-2- Configure the PWM channels #########################################*/
	  /* Common configuration for all channels */
	  sConfig.OCMode       = TIM_OCMODE_PWM1;
	  sConfig.OCPolarity   = TIM_OCPOLARITY_HIGH;
	  sConfig.OCFastMode   = TIM_OCFAST_DISABLE;
	  sConfig.OCNPolarity  = TIM_OCNPOLARITY_HIGH;
	  sConfig.OCNIdleState = TIM_OCNIDLESTATE_RESET;
	  sConfig.OCIdleState  = TIM_OCIDLESTATE_RESET;
	  /* Set the pulse value for channel 1 */
	  sConfig.Pulse = M1_lev_front;
	  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfig, TIM_CHANNEL_1) != HAL_OK)
	  {
	    /* Configuration Error */
		    _Error_Handler(__FILE__, __LINE__);
	  }

	  /* Set the pulse value for channel 2 */
	  sConfig.Pulse = M2_prav_back;
	  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfig, TIM_CHANNEL_2) != HAL_OK)
	  {
	    /* Configuration Error */
		    _Error_Handler(__FILE__, __LINE__);
	  }

	  /* Set the pulse value for channel 3 */
	  sConfig.Pulse = M3_prav_front;
	  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfig, TIM_CHANNEL_3) != HAL_OK)
	  {
	    /* Configuration Error */
		    _Error_Handler(__FILE__, __LINE__);
	  }

	  /* Set the pulse value for channel 4 */
	  sConfig.Pulse = M4_lev_back;
	  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfig, TIM_CHANNEL_4) != HAL_OK)
	  {
	    /* Configuration Error */
		    _Error_Handler(__FILE__, __LINE__);
	  }

	  /*##-3- Start PWM signals generation #######################################*/
	  /* Start channel 1 */
	  if (HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1) != HAL_OK)
	  {
	    /* PWM Generation Error */
		    _Error_Handler(__FILE__, __LINE__);
	  }
	  /* Start channel 2 */
	  if (HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_2) != HAL_OK)
	  {
	    /* PWM Generation Error */
		    _Error_Handler(__FILE__, __LINE__);
	  }
	  /* Start channel 3 */
	  if (HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_3) != HAL_OK)
	  {
	    /* PWM generation Error */
		    _Error_Handler(__FILE__, __LINE__);
	  }
	  /* Start channel 4 */
	  if (HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_4) != HAL_OK)
	  {
	    /* PWM generation Error */
		    _Error_Handler(__FILE__, __LINE__);
	  }
	  __HAL_TIM_ENABLE_IT(&htim1, TIM_IT_UPDATE);

}

void TIM4_IRQHandler(void)
{
  /* USER CODE BEGIN TIM3_IRQn 0 */

  /* USER CODE END TIM3_IRQn 0 */
  HAL_TIM_IRQHandler(&htim4);
  /* USER CODE BEGIN TIM3_IRQn 1 */

  /* USER CODE END TIM3_IRQn 1 */
}

void HAL_TIM_IC_CaptureCallback(TIM_HandleTypeDef *htim)
{
    static uint16_t temp;
    static uint8_t flag_save_point = 0;

    // считываем значение из регистра захвата
    temp = HAL_TIM_ReadCapturedValue(htim, TIM_CHANNEL_1);
    // если интервал слишком длинный, значит, пакет принят
    if (temp > 2800)
    {
        rc_channel = 0;
    }
    else // сохраняем одно принятое значение в буфер
    {
    	ppm1_buf[rc_channel] = temp;

	    switch(rc_channel) //обработка каналов
		{
		    case 3:
		    yaw_rate_ppm = (float)YAW_RATE*(ppm1_buf[3] - MED_PPM);
		    break;
		    case 1:
		    pitch_ppm = (float)PITCH_ROLL_K*(ppm1_buf[1] - MED_PPM);
		    break;
		    case 2:
		    if (ppm1_buf[2] < MIN_PPM) ppm1_buf[2] = MIN_PPM;
		    throtle_ppm = (float)THROTLE_K*(ppm1_buf[2] - MIN_PPM);
		    break;
		    case 0:
		    roll_ppm = (float)PITCH_ROLL_K*(ppm1_buf[0] - MED_PPM);
		    break;
		    case 4:
		    option_channel_1 = ppm1_buf[4];
		    if((option_channel_1 > 1520)&&(option_channel_1 < 1550))
			    {
				Mode = MODE_ESC_CALIB;

			    }
		    if((option_channel_1 > 1120)&&(option_channel_1 < 1149))
			    {
				Mode = MODE_STAB;

			    }
		    if((option_channel_1 > 1250)&&(option_channel_1 < 1281))
			    {
//							if (supervisorState&GPS_GLITCH) Mode = MODE_STAB;
//							else
				Mode = MODE_ALT_HOLD;

			    }
		    if((option_channel_1 > 1400)&&(option_channel_1 < 1425))
			    {
				if ((supervisorState&GPS_GLITCH)||(GPS_hAccf>2.5f)||(StateNavUKF&UKF_BAD_GPS))
				    {
//					buzzer_flag = BUZZER_FLAG_NOT_MODE;
					Mode = MODE_ALT_HOLD;
				    }
				else
				Mode = MODE_POS_HOLD;

			    }
		    if((option_channel_1 > 1802)&&(option_channel_1 < 1830))
			    {
				if ((supervisorState&GPS_GLITCH)||(GPS_hAccf>2.5f)||(StateNavUKF&UKF_BAD_GPS))
				    {
	//				buzzer_flag = BUZZER_FLAG_NOT_MODE;
					Mode = MODE_ALT_HOLD;
				    }
				else
				Mode = MODE_HEADFREE;

			    }
		    if((option_channel_1 > 1650)&&(option_channel_1 < 1690))
			    {
				if ((supervisorState&GPS_GLITCH)||(GPS_hAccf>2.5f)||(StateNavUKF&UKF_BAD_GPS))
				    {
		//			buzzer_flag = BUZZER_FLAG_NOT_MODE;
					Mode = MODE_ALT_HOLD;
				    }
				else
				Mode = MODE_AUTO;

			    }
		    break;
		    case 5:
		    option_channel_2 = ppm1_buf[5];
		    if (option_channel_2 > 1800)
			{
			    if (flag_save_point == 0)
				{

				    flagSavePoint = 1;
				    flag_save_point = 1;
				}
			}
		    else
			{
			    flag_save_point = 0;
		    }
		    break;
		    case 6:
		    option_channel_3 = ppm1_buf[6];
		    k_vel_auto_ppm = (float)option_channel_3/1000.0f-0.980f;
		    k_vel_auto_ppm = 0.2f + k_vel_auto_ppm*3.725f;
		    break;
		    default: break;
		}
        rc_channel++;
    };

}

void HAL_TIM_PWM_PulseFinishedCallback(TIM_HandleTypeDef *htim)
{
	if ( htim->Instance == TIM1 ){
	__HAL_TIM_SET_COMPARE(htim,TIM_CHANNEL_1, M1_lev_front);
	__HAL_TIM_SET_COMPARE(htim,TIM_CHANNEL_2, M2_prav_back);
	__HAL_TIM_SET_COMPARE(htim,TIM_CHANNEL_3, M3_prav_front);
	__HAL_TIM_SET_COMPARE(htim,TIM_CHANNEL_4, M4_lev_back);
	}
}
void prvArming(void *pvParameters)
    {
	static uint16_t schet_arming, schet_disarming;
	uint16_t yaw_raw_ppm;
	float thr_ppm;
	while(1)
	    {
		//защититить
		taskENTER_CRITICAL();
		thr_ppm = throtle_ppm;
		yaw_raw_ppm = ppm1_buf[3];
		taskEXIT_CRITICAL();
		////////////

//		if((supervisorState&STATE_ARMED)&&(thr_ppm>0.3f)) supervisorState|=STATE_FLYING;
//		else supervisorState &= ~STATE_FLYING;

		if (((thr_ppm < 0.05f)&&(yaw_raw_ppm > (MAX_PPM + 30)))&&!(supervisorState&STATE_ARMED))	schet_arming++;
		else schet_arming = 0;
		if (((thr_ppm < 0.05f)&&(yaw_raw_ppm < (MIN_PPM - 30)))&&(supervisorState&STATE_ARMED))	schet_disarming++;
		else schet_disarming = 0;
		if ((schet_arming >= TIME_FOR_ARMING)&&!(supervisorState&STATE_ARMED))
		    {
			//защитить
			taskENTER_CRITICAL();
			supervisorState |= STATE_ARMED;

			GPS_X_home = GPS_X;
			GPS_Y_home = GPS_Y;
			//flag_start_home = 0;
			//flag_start_home_baro = 0;
//			//buzzer_flag = BUZZER_FLAG_ARMING_DISARMING;
			//flag_start_arming_imu = 1;
			//////////
			schet_arming = 0;
			taskEXIT_CRITICAL();
		    }
		if ((schet_disarming >= TIME_FOR_ARMING)&&(supervisorState&STATE_ARMED))
		    {
			//защитить
			taskENTER_CRITICAL();
			supervisorState &= ~STATE_ARMED;
	//		buzzer_flag = BUZZER_FLAG_ARMING_DISARMING;
			alt_offset = 0.0f;
			freeAllPoints();
			///////////
			schet_disarming = 0;
			taskEXIT_CRITICAL();
		    }
		vTaskDelay(50);
	    }
    }

/* TIM3 Interrupt Handler */

void TIM1_UP_IRQHandler(void){
	__HAL_TIM_SET_COMPARE(&htim1,TIM_CHANNEL_1, M1_lev_front);
	__HAL_TIM_SET_COMPARE(&htim1,TIM_CHANNEL_2, M2_prav_back);
	__HAL_TIM_SET_COMPARE(&htim1,TIM_CHANNEL_3, M3_prav_front);
	__HAL_TIM_SET_COMPARE(&htim1,TIM_CHANNEL_4, M4_lev_back);
	__HAL_TIM_CLEAR_FLAG(&htim1,TIM_FLAG_UPDATE);
}
void HAL_TIM_IC_MspInit(TIM_HandleTypeDef* htim_ic)
{
  GPIO_InitTypeDef GPIO_InitStruct;
  if(htim_ic->Instance==TIM4)
  {
  /* USER CODE BEGIN TIM4_MspInit 0 */
	  /* Enable all GPIO Channels Clock requested */
	  __HAL_RCC_GPIOD_CLK_ENABLE();
  /* USER CODE END TIM4_MspInit 0 */
    /* Peripheral clock enable */
    __HAL_RCC_TIM4_CLK_ENABLE();
    /**TIM4 GPIO Configuration
    PD12     ------> TIM4_CH1
    PD13     ------> TIM4_CH2
    */
    GPIO_InitStruct.Pin = GPIO_PIN_12;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    GPIO_InitStruct.Alternate = GPIO_AF2_TIM4;
    HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

    /* TIM4 interrupt Init */
    HAL_NVIC_SetPriority(TIM4_IRQn, 9, 0);
    HAL_NVIC_EnableIRQ(TIM4_IRQn);
  /* USER CODE BEGIN TIM4_MspInit 1 */

  /* USER CODE END TIM4_MspInit 1 */
  }

}

void HAL_TIM_IC_MspDeInit(TIM_HandleTypeDef* htim_ic)
{

  if(htim_ic->Instance==TIM4)
  {
  /* USER CODE BEGIN TIM4_MspDeInit 0 */

  /* USER CODE END TIM4_MspDeInit 0 */
    /* Peripheral clock disable */
    __HAL_RCC_TIM4_CLK_DISABLE();

    /**TIM4 GPIO Configuration
    PD12     ------> TIM4_CH1
    PD13     ------> TIM4_CH2
    */
    HAL_GPIO_DeInit(GPIOD, GPIO_PIN_12);

    /* TIM4 interrupt DeInit */
    HAL_NVIC_DisableIRQ(TIM4_IRQn);
  /* USER CODE BEGIN TIM4_MspDeInit 1 */

  /* USER CODE END TIM4_MspDeInit 1 */
  }

}


void HAL_TIM_PWM_MspInit(TIM_HandleTypeDef *htim)
{
  GPIO_InitTypeDef   GPIO_InitStruct;
  /*##-1- Enable peripherals and GPIO Clocks #################################*/
  if(htim->Instance==TIM1)
  {
  /* TIM1 Peripheral clock enable */
  __HAL_RCC_TIM1_CLK_ENABLE();

  /* Enable all GPIO Channels Clock requested */
  __HAL_RCC_GPIOE_CLK_ENABLE();

  /* Configure
  PA.08 (CN12 connector) (TIM1_Channel1),
  PA.09 (CN12 connector) (TIM1_Channel2),
  PA.10 (CN12 connector) (TIM1_Channel3),
  PA.11 (CN12 connector) (TIM1_Channel4) in output, push-pull, alternate function mode
  */
  GPIO_InitStruct.Pin = GPIO_PIN_9|GPIO_PIN_11|GPIO_PIN_13|GPIO_PIN_14;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  GPIO_InitStruct.Alternate = GPIO_AF1_TIM1;
  HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

  /* Peripheral clock enable */
  __HAL_RCC_TIM1_CLK_ENABLE();
  /* TIM1 interrupt Init */
  HAL_NVIC_SetPriority(TIM1_UP_IRQn, 8, 0);
  HAL_NVIC_EnableIRQ(TIM1_UP_IRQn);
  }

}


