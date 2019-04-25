#ifndef _CONFIG_
#define _CONFIG_

#include "stm32h7xx_hal.h"
#include "FreeRTOS.h"
#include "task.h"
#include "timers.h"
#include "queue.h"
#include "semphr.h"
#include "event_groups.h"
#include "arm_math.h"
#include "stdio.h"
#include "stdint.h"
#include "stdlib.h"
#include "stdbool.h"
#include "math.h"
#include "drv_led.h"
#include "cli.h"
#include "comm.h"
#include "drv_uart1.h"
#include "drv_uart2.h"

// need for cache syncing on F7/H7
#define dmaBufferInvalidate(saddr, n) {                                     \
  uint8_t *start = (uint8_t *)(saddr);                                      \
  uint8_t *end = start + (size_t)(n);                                       \
  __DSB();                                                                  \
  while (start < end) {                                                     \
    SCB->DCIMVAC = (uint32_t)start;                                         \
    start += 32U;                                                           \
  }                                                                         \
  __DSB();                                                                  \
  __ISB();                                                                  \
}

#define dmaBufferFlush(saddr, n) {                                          \
  uint8_t *start = (uint8_t *)(saddr);                                      \
  uint8_t *end = start + (size_t)(n);                                       \
  __DSB();                                                                  \
  while (start < end) {                                                     \
    SCB->DCCIMVAC = (uint32_t)start;                                        \
    start += 32U;                                                           \
  }                                                                         \
  __DSB();                                                                  \
  __ISB();                                                                  \
}



#ifndef u32
typedef uint32_t  u32;
#endif
#ifndef u16
typedef uint16_t u16;
#endif
#ifndef u8
typedef uint8_t  u8;
#endif

#define UART1_TX_PIN        GPIO_PIN_9
#define UART1_RX_PIN        GPIO_PIN_10
#define UART1_GPIO_TX       GPIOA
#define UART1_GPIO_RX       GPIOA

//QUAD
//HAL systick timer
#define timerMicros()				TIM14->CNT

#define FRQ_MPU						500.0f			// sample frequency in Hz
#define DT_CYCLE_MPU              			(float)(1.0f/500.0f)
#define DT_UKF             				(float)(1.0f/200.0f)
#define RAD2DEG 					(float)(180.0/M_PI)
#define DEG2RAD						(float)(M_PI/180.0)
/*** PRIORITET INTERRUPTS ***/
/*
 * 	���������� ����������, ������������ ������� FRERRTOS ������ ������ ����(������ �����),
 * 	��� configLIBRARY_MAX_SYSCALL_INTERRUPT_PRIORITY
 */
#define PRIORITET_I2C 				2
#define PRIORITET_SPI_INS 			5
#define PRIORITET_ADC 				6
#define PRIORITET_UART_BLUETOOTH 	7
#define PRIORITET_UART_FRSKY	 	7
#define PRIORITET_UART_GPS 			4
#define PRIORITET_PPM 				3
#define PRIORITET_PWM 				3
/**PRIORITET TASKS**/
/*
 * ��� ���� ���������, ��� ���� �����(�������� � ������� �� ����������)
 *
 *
 */
#define PRIORITET_TASK_OPERATION_ADC				4
#define PRIORITET_TASK_REQUEST_ADC					2
#define PRIORITET_TASK_PROCESSING_MAG_BARO			4
#define PRIORITET_TASK_REQUEST_MAG_BARO				3
#define PRIORITET_TASK_ERR_MAG_BARO					3

#define PRIORITET_TASK_COLLECT_SD_PID				5
#define PRIORITET_TASK_COLLECT_SD_MAG_BARO			2
#define PRIORITET_TASK_COLLECT_SD_ADC				2
#define PRIORITET_TASK_COLLECT_SD_PPM				2
#define PRIORITET_TASK_COLLECT_SD_PWM				2
#define PRIORITET_TASK_COLLECT_SD_FLAGS				2
#define PRIORITET_TASK_COLLECT_SD_GPS				2
#define PRIORITET_TASK_COLLECT_SD_UKF				3
#define PRIORITET_TASK_TRANSIVER_SD					1

#define PRIORITET_TASK_PARCEL_TO_COMP				4
#define PRIORITET_TASK_GPS							5
#define PRIORITET_TASK_INS							6
#define PRIORITET_TASK_UKF							3
#define PRIORITET_TASK_ARMING						2
#define PRIORITET_TASK_BUZZER						2
#define PRIORITET_TASK_LEDS							4

////////////////////

/***BUZZER***/
#define BUZZER_FLAG_VALUE_TRANSMITTED 				1
#define BUZZER_DLIT_SIGNAL_VALUE_TRANSMITTED 		5
#define BUZZER_DLIT_PAUSE_VALUE_TRANSMITTED 		2
#define BUZZER_NUMBER_POVTOR_VALUE_TRANSMITTED 		2

#define BUZZER_FLAG_VOLTAGE_LOW 				2
#define BUZZER_DLIT_SIGNAL_VOLTAGE_LOW  		40
#define BUZZER_DLIT_PAUSE_VOLTAGE_LOW 			20
#define BUZZER_NUMBER_POVTOR_VOLTAGE_LOW 		2

#define BUZZER_FLAG_FAIL_SENSOR					3
#define BUZZER_DLIT_SIGNAL_FAIL_SENSOR			2
#define BUZZER_DLIT_PAUSE_FAIL_SENSOR	 		2
#define BUZZER_NUMBER_POVTOR_FAIL_SENSOR		-1

#define BUZZER_FLAG_ARMING_DISARMING			4
#define BUZZER_DLIT_SIGNAL_ARMING_DISARMING 		10
#define BUZZER_DLIT_PAUSE_ARMING_DISARMING 		2
#define BUZZER_NUMBER_POVTOR_ARMING_DISARMING		2

#define BUZZER_FLAG_FAIL_SD					5
#define BUZZER_DLIT_SIGNAL_FAIL_SD	 		10
#define BUZZER_DLIT_PAUSE_FAIL_SD	 		10
#define BUZZER_NUMBER_POVTOR_FAIL_SD			-1

#define BUZZER_FLAG_LIFT_COMMAND 				6
#define BUZZER_DLIT_SIGNAL_LIFT_COMMAND 		3
#define BUZZER_DLIT_PAUSE_LIFT_COMMAND 			10
#define BUZZER_NUMBER_POVTOR_LIFT_COMMAND  		1

#define BUZZER_FLAG_GLITCH 					7
#define BUZZER_DLIT_SIGNAL_GLITCH 	 		20
#define BUZZER_DLIT_PAUSE_GLITCH  			10
#define BUZZER_NUMBER_POVTOR_GLITCH 	  		4

#define BUZZER_FLAG_NOT_MODE 					8
#define BUZZER_DLIT_SIGNAL_NOT_MODE 	 		60
#define BUZZER_DLIT_PAUSE_NOT_MODE  			1
#define BUZZER_NUMBER_POVTOR_NOT_MODE 	  		1


#define __sqrtf(a)	sqrtf(a)
#ifndef MIN
#define MIN(a, b) ((a < b) ? a : b)
#endif
#ifndef MAX
#define MAX(a, b) ((a > b) ? a : b)
#endif


//��������� ������� ��� ��������� ��������
enum supervisorStates {
    STATE_INITIALIZING	= 0x00,
    STATE_CALIBRATION	= 0x01,
    STATE_DISARMED	= 0x02,
    STATE_ARMED		= 0x04,
    STATE_FLYING	= 0x08,
    STATE_RADIO_LOSS1	= 0x10,
    GPS_GLITCH	= 0x20,
    STATE_LOW_BATTERY	= 0x40,
    ERR_SENSOR	= 0x80
};
enum supervisorStates supervisorState;

//����� ������� ��� ��������� ��������
enum Modes {
	MODE_STAB	= 0x00,
	MODE_ESC_CALIB	= 0x01,
	MODE_ALT_HOLD	= 0x02,
	MODE_HEADFREE	= 0x04,
	MODE_AUTO	= 0x08,
	MODE_POS_HOLD	= 0x10,
};
enum Modes Mode;

//��������� ���������
enum StateNavUKFs{
    UKF_NORM = 0x00,
    UKF_BAD_GPS = 0x01,
    UKF_GLITCH_GPS = 0x02,
    UKF_NAN = 0x04
};
enum StateNavUKFs StateNavUKF;


//���-�� ��������� ������(����) �����
size_t ovf_full_stack;
volatile uint32_t main_stack;
//������� ��
volatile uint32_t counter_sys;
extern float axf, ayf, azf, gxf, gyf, gzf;//���������� ������ � �������� � g � ���/���
extern float mxf, myf, mzf;//������ � �������
//������ ��������
enum Err_Sensor {
    OK,
    Err_Init,
    Err_value,
    Err_synq
};

enum Err_Serial {
    S_OK,
    S_Init,
    Err_Overfull,
    Err_Send,
	Err_Receive
};

#define SQR(x)  ((x) * (x))
#endif
