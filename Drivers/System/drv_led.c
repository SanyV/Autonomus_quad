#include "stm32h7xx.h"
#include "drv_led.h"
void Led_init(void)
		{
  GPIO_InitTypeDef GPIO_InitStruct;
  //__HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOG_CLK_ENABLE();

  GPIO_InitStruct.Pin = Led1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  HAL_GPIO_Init(Led1_Port, &GPIO_InitStruct);

  GPIO_InitStruct.Pin = Led2_Pin;
  HAL_GPIO_Init(Led2_Port, &GPIO_InitStruct);

  GPIO_InitStruct.Pin = Led3_Pin;
  HAL_GPIO_Init(Led3_Port, &GPIO_InitStruct);

  GPIO_InitStruct.Pin = Led4_Pin;
  HAL_GPIO_Init(Led4_Port, &GPIO_InitStruct);
  Led4_Off;

		}
