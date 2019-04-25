
#ifndef _BARO_MAG_H_
#define _BARO_MAG_H_

#include "Config.h"

#define HMC5883_DEFAULT_ADDRESS     0x3C
#define MS5611_DEFAULT_ADDRESS      0xEE

//***************************************************************************************************//

//***************************************************************************************************//


#define MAG_CS_LOW       HAL_GPIO_WritePin(GPIOE, MAG_CS_Pin, GPIO_PIN_RESET)
#define MAG_CS_HIGH      HAL_GPIO_WritePin(GPIOE, MAG_CS_Pin, GPIO_PIN_SET)

#define BARO_CS_LOW       HAL_GPIO_WritePin(GPIOE, BARO_CS_Pin, GPIO_PIN_RESET)
#define BARO_CS_HIGH      HAL_GPIO_WritePin(GPIOE, BARO_CS_Pin, GPIO_PIN_SET)

//extern SemaphoreHandle_t xSD_FLAGS_collect_Semaphore;

//экспорт
extern float mxf, myf, mzf;
extern float Temperature, Altitude, Pressure;

extern uint8_t flag_start_home_baro;
extern uint16_t err_num_i2c_def, err_num_i2c_line;
extern uint8_t compass_not_health, baro_not_health;

extern uint32_t cnt_ticks_baro, cnt_cycle_baro;
extern uint32_t cnt_cycle_mag, cnt_ticks_mag;

extern enum Err_Sensor Err_Mag, Err_Baro;

extern int16_t mx, my, mz;

/////
//extern SemaphoreHandle_t xSD_MAG_BARO_collect_Semaphore;

void Baro_Mag_Init_Full(void);





#endif
