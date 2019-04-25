/*
  Alex Valov 2017

*/

///////////////////////////////////////////////////////////////////////////////

#pragma once
//Полезные данные
extern int16_t mx, my, mz;
extern float mxf, myf, mzf;
extern float M_X, M_Y, M_Z;

#define MAG_CS_LOW       HAL_GPIO_WritePin(GPIOE, MAG_CS_Pin, GPIO_PIN_RESET)
#define MAG_CS_HIGH      HAL_GPIO_WritePin(GPIOE, MAG_CS_Pin, GPIO_PIN_SET)
///////////////////////////////////////////////////////////////////////////////

#define	EXTI_MAG_IRQ	 				EXTI3_IRQn
#define EXTI_MAG_IRQHandler				EXTI3_IRQHandler
#define EXTI_Line_MAG					EXTI_LINE3

extern float magScaleFactor[3];

extern uint8_t magDataUpdate;

extern uint8_t newMagData;

extern int16_t rawMag[3];

extern float nonRotatedMagData[3];

///////////////////////////////////////////////////////////////////////////////

void readMag(void *unused);

///////////////////////////////////////////////////////////////////////////////

void initMag(void);

///////////////////////////////////////////////////////////////////////////////
