
#ifndef _PWM_IN_OUT_H_
#define _PWM_IN_OUT_H_


//***************************************************************************************************//
#define	PPM_R_IRQ TIM4_IRQHandler
#define	PPM_R	  TIM4


//***************************************************************************************************//
#define	PWM_M_IRQ	TIM1_UP_IRQHandler
#define	PWM_M		TIM1


//***************************************************************************************************//


#define MED_PPM       		 1405
#define DEAD_ZONE_PPM        5
#define MIN_PPM       		 970
#define MAX_PPM        		 1800
#define YAW_RATE       		 0.004f
#define MAX_PITCH_ROLL		 M_PI/3.0f
#define PITCH_ROLL_K       	 MAX_PITCH_ROLL/((MAX_PPM - MIN_PPM)/2.0)
#define THROTLE_K      		 1.0/(MAX_PPM - MIN_PPM)
//#define THROTLE_K      		 1.0/(MAX_PPM+200 - MIN_PPM)


#define TIME_FOR_ARMING 		20*1

#define DEAD_ZONE_ALT_STICK		0.08f
extern volatile uint8_t flagSavePoint;
extern volatile float throtle_ppm, yaw_rate_ppm, pitch_ppm, roll_ppm;
extern volatile float k_vel_auto_ppm;
extern uint16_t option_channel_1, option_channel_2;
extern uint16_t M1_lev_front, M2_prav_back, M3_prav_front, M4_lev_back;
extern uint16_t ppm1_buf[10]; 		/* буфер для значений каналов ППМ */
 //Счетчики циклов
extern uint32_t cnt_cycle_ppm, cnt_ticks_ppm;
extern uint32_t cnt_cycle_pwm, cnt_ticks_pwm;
//импорт
extern SemaphoreHandle_t xSD_PPM_collect_Semaphore;
extern SemaphoreHandle_t xSD_PWM_collect_Semaphore;
//extern data_ukf DataUKF;
//extern SemaphoreHandle_t xSD_FLAGS_collect_Semaphore;
//extern uint8_t flag_start_home;
//extern uint8_t flag_start_home_baro;


void Init_PWM_Reciever1(void);
void Init_PWM_Motor(uint16_t Period1, uint16_t Dp1);
void prvArming(void *pvParameters);


#endif /* _PWM_IN_OUT_H_ */
