
#include <BARO_MAG.h>
#include "cmsis_os.h"
#include <math.h>
#include "Config.h"
#include "stdbool.h"
#define _MAG_INTERNAL
//#define _MAG_EXTERNAL

extern SemaphoreHandle_t  xSPI_Mutex ;

extern SPI_HandleTypeDef hspi1;
extern enum Err_Sensor Err_Baro;

uint32_t deltaTimeBARO,   executionTimeBARO ,   previousBAROTime;
float dtBARO;

extern uint8_t SPIx_SendByte(uint8_t byte);
extern uint8_t spi_recieve(void);


//Вспомогательные промежуточные переменные
uint8_t i2c_inx, i2c_dev_adr, i2c_cnt;
uint8_t i2c_buf[15], i2c_buf_mag[7], i2c_buf_baro[7];
uint8_t Flag_bar_temper;
uint16_t C1, C2, C3, C4, C5, C6, manufactura, crc;
uint16_t err_num_i2c_def, err_num_i2c_line;
uint8_t flag_err_i2c;
//****************************************************//

// Хандл и Стек
TaskHandle_t mag_baro_handle;
uint32_t ovf_mag_baro_stack;

SemaphoreHandle_t xMAG_BARO_Semaphore = NULL;
SemaphoreHandle_t xBAROMAG_UKF_Mutex;
//****************************************************//

//Полезные данные
int16_t mx, my, mz;
float mxf, myf, mzf;
float M_X, M_Y, M_Z;
float Temperature, Altitude, Pressure;
//****************************************************//
//Флаги
uint8_t flag_start_home_baro;
uint8_t compass_not_health, baro_not_health;
enum Err_Sensor Err_Mag, Err_Baro;

//счетчик тактов для измерения цикла
uint32_t cnt_cycle_mag, cnt_ticks_mag;
uint32_t cnt_cycle_baro, cnt_ticks_baro;

//калибровочные коэффициенты
#ifdef _MAG_INTERNAL
//внутренний компас
const float mag_x_bios = 235.095428f, mag_y_bios = -308.824699f, mag_z_bios = 0.407281f;
const float m1 = 1.050349f, 	m2 = 0.013182f, 	m3 = 0.018234f;
const float 			m5 = 1.080057f, 	m6 = -0.000538f;
const float 						m9 = 1.186722f;
#endif
//внешний компас
//на проводе дома
//const float mag_x_bios = -12.597431f, mag_y_bios = 95.272002f, mag_z_bios = 3.337495f;
//const float m1 = 5.544633f, m2 = 0.044157f, m3 = -0.048782f;
//const float m4 = 0.044157f, m5 = 5.450562f, m6 = -0.052568f;
//const float m7 = -0.048782f, m8 = -0.052568f, m9 = 6.340859f;

#ifdef _MAG_EXTERNAL
//в деревне
//const float mag_x_bios = -33.845827f, mag_y_bios = 65.105180f, mag_z_bios = -8.810466f;
//const float m1 = 5.084115f, m2 = 0.051629f, m3 = -0.013368f;
//const float m4 = 0.051629f, m5 = 5.037634f, m6 = -0.048618f;
//const float m7 = -0.013368f, m8 = -0.048618f, m9 = 5.803905f;
//в деревне (ровная сфера)
//const float mag_x_bios = -34.740234f, mag_y_bios = 64.616275f, mag_z_bios = -5.849274f;
//const float m1 = 5.059997f, m2 = 0.051608f, m3 = -0.011655f;
//const float m4 = 0.051608f, m5 = 5.024446f, m6 = -0.058183f;
//const float m7 = -0.011655f, m8 = -0.058183f, m9 = 5.778027f;
//за храмом
//const float mag_x_bios = -14.138508f, mag_y_bios = 101.658827f, mag_z_bios = 9.686203f;
//const float m1 = 5.033572f, m2 = 0.029271f, m3 = -0.050326f;
//const float m4 = 0.029271f, m5 = 4.959995f, m6 = -0.037160f;
//const float m7 = -0.050326f, m8 = -0.037160f, m9 = 5.746589f;
//на стадионе
const float mag_x_bios = -30.722438f, mag_y_bios = 59.441092f, mag_z_bios = -15.127328f;
const float m1 = 5.155095f, m2 = 0.043244f, m3 = -0.031934f;
const float m4 = 0.043244f, m5 = 5.084305f, m6 = -0.048353f;
const float m7 = -0.031934f, m8 = -0.048353f, m9 = 5.930254f;
#endif

#define I2C_DEFAULT_TIMEOUT  1000
//--------------------------------------------

uint8_t Baro_read( void )
{
	uint8_t res;
	xSemaphoreTake(xSPI_Mutex, portMAX_DELAY);
	BARO_CS_LOW;
	SPIx_SendByte(0x80);
	res = spi_recieve();
	BARO_CS_HIGH;
	xSemaphoreGive(xSPI_Mutex);
	return res;
}
void Baro_reads(uint8_t reg_addr,  uint8_t *pData, uint16_t Size)
{
	static uint8_t stx[10];
	BARO_CS_LOW;
	//stx[0] = reg_addr; //0x80 | reg_addr;
	SPIx_SendByte( reg_addr);
    if (HAL_SPI_Receive(&hspi1, (uint8_t *)pData, Size, 1000) != HAL_OK)
    {
    	Err_Baro = Err_synq;
//    	return -1;
    }
	BARO_CS_HIGH;
}
void Baro_write_cmd( uint8_t value1 )
{
	xSemaphoreTake(xSPI_Mutex, portMAX_DELAY);
	BARO_CS_LOW;
	SPIx_SendByte(value1);
	BARO_CS_HIGH;
	xSemaphoreGive(xSPI_Mutex);
}
uint8_t Mag_read_register( uint8_t reg1 )
{
	uint8_t res;
	xSemaphoreTake(xSPI_Mutex, portMAX_DELAY);
	MAG_CS_LOW;
	SPIx_SendByte(reg1|0x80);
	res = spi_recieve();
	MAG_CS_HIGH;
	xSemaphoreGive(xSPI_Mutex);
	return res;
}
void Mag_write_register( uint8_t reg1, uint8_t value1 )
{
	xSemaphoreTake(xSPI_Mutex, portMAX_DELAY);
	MAG_CS_LOW;
	SPIx_SendByte(reg1);
	SPIx_SendByte(value1);
	MAG_CS_HIGH;
	xSemaphoreGive(xSPI_Mutex);
}

void Mag_reads(uint8_t reg_addr,  uint8_t *pData, uint16_t Size)
{
	MAG_CS_LOW;
	SPIx_SendByte(reg_addr | 0x80 | 0x40);
    if (HAL_SPI_Receive(&hspi1, pData, Size, 1000) != HAL_OK)
    {
    	Err_Baro = Err_synq;
//    	return -1;
    }
	MAG_CS_HIGH;
}

void Obrabotka_Mag(void)
    {

		int16_t mx1, my1, mz1;
		static int16_t mx_last, my_last, mz_last;
		static uint8_t first=0, one = 0;
		int16_t mx_r, my_r, mz_r;

		float mx_bios, my_bios, mz_bios;
		float mx_cal, my_cal, mz_cal;



		cnt_cycle_mag = (uint32_t)DWT->CYCCNT - cnt_ticks_mag;
		cnt_ticks_mag = (uint32_t)DWT->CYCCNT;

		#ifdef _MAG_EXTERNAL
		mx1 = (int16_t)(((uint16_t)i2c_buf_mag[1]<<8)+i2c_buf_mag[2]);
		mz1 = -(int16_t)(((uint16_t)i2c_buf_mag[3]<<8)+i2c_buf_mag[4]);
		my1 = -(int16_t)(((uint16_t)i2c_buf_mag[5]<<8)+i2c_buf_mag[6]);
		#endif
		#ifdef _MAG_INTERNAL
		mx1 = (int16_t)(((uint16_t)i2c_buf_mag[1]<<8)+i2c_buf_mag[2]);
		mz1 = (int16_t)(((uint16_t)i2c_buf_mag[3]<<8)+i2c_buf_mag[4]);
		my1 = (int16_t)(((uint16_t)i2c_buf_mag[5]<<8)+i2c_buf_mag[6]);
		#endif

		if (first < 30)
		    {
			mx_last = mx1;
			my_last = my1;
			mz_last = mz1;
			first++;
		    }
		if (mx1>mx_last) mx_r = mx1 - mx_last; else mx_r = mx_last - mx1;
		if (my1>my_last) my_r = my1 - my_last; else my_r = my_last - my1;
		if (mz1>mz_last) mz_r = mz1 - mz_last; else mz_r = mz_last - mz1;

		if (((mx_r > 120)||(my_r > 120)||(mz_r > 120))&& one == 0)
		    {
			mx = mx_last;
			my = my_last;
			mz = mz_last;
			one = 1;
		    }
		else
		{
			mx = mx1;
			my = my1;
			mz = mz1;
			mx_last = mx1;
			my_last = my1;
			mz_last = mz1;
			one = 0;
		}

		mx_bios = (float)mx - mag_x_bios;
		my_bios = (float)my - mag_y_bios;
		mz_bios = (float)mz - mag_z_bios;

//		mx_cal = m1*mx_bios + m2*my_bios + m3*mz_bios;
//		my_cal = m4*mx_bios + m5*my_bios + m6*mz_bios;
//		mz_cal = m7*mx_bios + m8*my_bios + m9*mz_bios;
		mx_cal = m1*mx_bios + m2*my_bios + m3*mz_bios;
		my_cal = m2*mx_bios + m5*my_bios + m6*mz_bios;
		mz_cal = m3*mx_bios + m6*my_bios + m9*mz_bios;



		//защитить
		//taskENTER_CRITICAL();
		xSemaphoreTake(xBAROMAG_UKF_Mutex, portMAX_DELAY);
		mxf = mx_cal;
		myf = my_cal;
		mzf = mz_cal;
		xSemaphoreGive(xBAROMAG_UKF_Mutex);
		//taskEXIT_CRITICAL();
		//
	//	 if (compass_not_health == 0)
		// xSemaphoreGive(xSD_MAG_BARO_collect_Semaphore);

	   // }
    }
void Obrabotka_Baro(void)
    {
		static uint32_t ADC_Temp, ADC_Press;
		static int32_t dt_baro, TEMP_baro, TEMP_baro2;
		static int32_t TEMP_baro_act, TEMP_baro_last, deltat;
		static int64_t OFF_baro, sens_baro, OFF_baro2, sens_baro2;
		static float press_last, alt_last = 0.0f, temp11;
		static uint16_t shet_alt;
		static double Altitude_s;
		static float Bios_alt;
		static float press_act, pressik, altik, deltaalt;
		static uint8_t first_temp = 0, first_alt = 0;

		if (Flag_bar_temper == 1)//temp
		    {
			ADC_Temp = (uint32_t)(i2c_buf_baro[1] << 16 | i2c_buf_baro[2] << 8 | i2c_buf_baro[3]);

			dt_baro = (int32_t)(ADC_Temp - (uint32_t)C5*256);
			TEMP_baro = (int32_t)(2000+((int64_t)dt_baro*C6)/(1<<23));
			TEMP_baro2 = 0;
			if (TEMP_baro < 2000)  // < 20C
			{
				TEMP_baro2 = (dt_baro * dt_baro) / ( 1<< 31);  //Ошибка было 2в30
				OFF_baro2 = (5 *( (TEMP_baro - 2000) * (TEMP_baro - 2000))) / 2;
				sens_baro2= (5 *( (TEMP_baro - 2000) * (TEMP_baro - 2000))) / 4;
				if (TEMP_baro < -1500) //<-15C
				{
					OFF_baro2 = OFF_baro2 + 7 *( (TEMP_baro + 1500) * (TEMP_baro + 1500));
					sens_baro2= sens_baro2 + (11 *( (TEMP_baro + 1500) * (TEMP_baro + 1500))) / 2;
				}

			}

			else
			{
				TEMP_baro2 = 0;
				OFF_baro2 = 0;
				sens_baro2 = 0;
			}

			TEMP_baro = TEMP_baro - TEMP_baro2;
			OFF_baro = OFF_baro - OFF_baro2 ;
			sens_baro = sens_baro - sens_baro2;

			if (first_temp == 0)
			    {
				first_temp = 1;
				TEMP_baro_last = TEMP_baro;
				OFF_baro2 = 0;
				sens_baro2 = 0;

			    }
			if (TEMP_baro >= TEMP_baro_last)
			deltat = TEMP_baro - TEMP_baro_last;
			else
			deltat = TEMP_baro_last - TEMP_baro;

			if ((TEMP_baro != NAN)&&(deltat < 500 ))
			    {
				TEMP_baro_act = TEMP_baro;
				TEMP_baro_last = TEMP_baro_act;
			    }
			else TEMP_baro_act = TEMP_baro_last;

			temp11 = (float)TEMP_baro_act/100.0f;

		    }

		if (Flag_bar_temper == 2)//press
		    {
		//	cnt_cycle_baro = (uint32_t)DWT->CYCCNT - cnt_ticks_baro;
		//	cnt_ticks_baro = (uint32_t)DWT->CYCCNT;

			OFF_baro = (int64_t)((int64_t)C2*65536+((int64_t)C4*(int64_t)dt_baro)/128);
			sens_baro = (int64_t)((int64_t)C1*32768+((int64_t)C3*(int64_t)dt_baro)/256);
			ADC_Press = (uint32_t)(i2c_buf_baro[1] << 16 | i2c_buf_baro[2] << 8 | i2c_buf_baro[3]);

			//Pressure = (int32_t)(((int64_t)(ADC_Press*sens_baro/2097152)-(int64_t)OFF_baro)/32768);

//			OFF_baro2 = 0;
//			sens_baro2 = 0;

/*			if (TEMP_baro < 2000)
			{
			    OFF_baro2 = 5 * ((TEMP_baro_act - 2000) * (TEMP_baro_act - 2000)) / 2;
			    sens_baro2 = 5 * ((TEMP_baro_act - 2000) * (TEMP_baro_act - 2000)) / 4;
			}
			if (TEMP_baro < -1500)
			{
			    OFF_baro2 = OFF_baro2 + 7 * ((TEMP_baro_act + 1500) * (TEMP_baro_act + 1500));
			    sens_baro2 = sens_baro2 + 11 * ((TEMP_baro_act + 1500) * (TEMP_baro_act + 1500)) / 2;
			}
			OFF_baro = OFF_baro - OFF_baro2;
			sens_baro = sens_baro - sens_baro2;
			*/

			pressik = (float)((int64_t)ADC_Press * sens_baro / (1<<21) - OFF_baro)*(1.0f/32768.0f);
			if (!(isnanf(pressik)))
			    {
				press_act = pressik;
				press_last = press_act;
			    }
			else press_act = press_last;
			Pressure = press_act;
			if (flag_start_home_baro == 0)
			    {
				shet_alt = 0;
				Altitude_s = 0.0f;
				first_alt = 0;
				flag_start_home_baro = 1;
			    }
			if (shet_alt < 10) shet_alt++;
			else
			    {
				if ((shet_alt < 10+15)&&(shet_alt >= 10))
				    {
					Altitude = 0;
					Altitude_s +=(float)44330.0f * (float)((float)1.0f - (float)powf((float)press_act/101325.0f, 0.19029495f));
					shet_alt++;
				    }
				else
				    {
					if (shet_alt == 10+15)
					    {
						Bios_alt = (float)((double)Altitude_s/(double)(shet_alt-10));
						shet_alt++;
						//flag_start_home_baro = 1;

					    }
					altik = (float)44330.0f * (float)((float)1.0f - (float)powf((float)press_act/101325.0f, 0.19029495f));
					altik -= Bios_alt;

					if (first_alt == 0)
					    {
						first_alt = 1;
						alt_last = altik;
					    }
					if (altik >= alt_last)
					deltaalt = altik - alt_last;
					else
					deltaalt = alt_last - altik;

					if ((!(isnanf(altik)))&&(deltaalt < 5.0f))
					    {
						alt_last = altik;
						//защитить
						//taskENTER_CRITICAL();
						xSemaphoreTake(xBAROMAG_UKF_Mutex, portMAX_DELAY);
						Temperature = temp11;
						Altitude = altik;
						xSemaphoreGive(xBAROMAG_UKF_Mutex);
						//taskEXIT_CRITICAL();
						////////////////////
						// if ((compass_not_health == 1)&&(baro_not_health == 0))
						// xSemaphoreGive(xSD_MAG_BARO_collect_Semaphore);
					    }

				    }
			    }

		    }

	  //  }
    }
void prvMAG_BARO_Processing(void *pvParameters)
    {


	while(1)
	    {

		xSemaphoreTake(xMAG_BARO_Semaphore, portMAX_DELAY);
		Obrabotka_Mag();
		Obrabotka_Baro();
		ovf_mag_baro_stack = uxTaskGetStackHighWaterMark(mag_baro_handle);
		ovf_full_stack = xPortGetFreeHeapSize();

	    }
    }
void prvMAG_BARO_Request(void *pvParameters)
    {
//	static portBASE_TYPE xHigherPriorityTaskWoken_MAG_BARO = pdFALSE;
//	static portBASE_TYPE xHigherPriorityTaskWoken_flag = pdFALSE;
	static float last_bar;
	static uint8_t schet1, schet2;
	static int16_t mxl, myl, mzl;
	static uint32_t currentTime;
//	uint8_t buf_w[1];
//	uint8_t buf_read[16];

	while(1)
	    {
        currentTime       = TIM5->CNT;
        deltaTimeBARO    = currentTime - previousBAROTime;
        previousBAROTime = currentTime;
        dtBARO = (float)deltaTimeBARO * 0.0000001f;

		if ((mx == mxl)&&(my == myl)&&(mz == mzl)&&(schet1<5)) schet1++;
		if ((mx != mxl)||(my != myl)||(mz != mzl)) schet1=0;
		if ((last_bar == Pressure)&&(schet2<5)) schet2++;
				if (last_bar != Pressure) schet2=0;

		if (schet1 == 5)
		    {
			schet1 = 6;
			Err_Mag = Err_value;
			supervisorState |= ERR_SENSOR;
			compass_not_health = 1;
	//		buzzer_flag = BUZZER_FLAG_FAIL_SENSOR;
	//		xSemaphoreGive(xSD_FLAGS_collect_Semaphore);

		    }
		if (schet2 == 5)
		    {
			schet2 = 6;
			Err_Baro = Err_value;
			supervisorState |= ERR_SENSOR;
			baro_not_health = 1;
	//		buzzer_flag = BUZZER_FLAG_FAIL_SENSOR;
	//		xSemaphoreGive(xSD_FLAGS_collect_Semaphore);
		    }
		mxl = mx;
		myl = my;
		mzl = mz;
		last_bar = Pressure;
	//	 if ((compass_not_health == 1)&&(baro_not_health == 1))
	//	 xSemaphoreGive(xSD_MAG_BARO_collect_Semaphore);
	//	vTaskDelay(2);

		Mag_reads( 0x03, &i2c_buf_mag[1], 6);
		Obrabotka_Mag();
		vTaskDelay(10);

		Baro_write_cmd (0x58);
		Flag_bar_temper = 1;
		vTaskDelay(10);
		Baro_reads(0x00, &i2c_buf_baro[1], 3);
		Obrabotka_Baro();
//		vTaskDelay(20);
//		Mag_reads(0x03, &i2c_buf_mag[1], 6);
//		Obrabotka_Mag();
//		vTaskDelay(6);

		Baro_write_cmd (0x48);
		Flag_bar_temper = 2;
		vTaskDelay(10);
		Baro_reads(0x00, &i2c_buf_baro[1], 3);
		Obrabotka_Baro();
	//	vTaskDelay(100);

		ovf_mag_baro_stack = uxTaskGetStackHighWaterMark(mag_baro_handle);
		ovf_full_stack = xPortGetFreeHeapSize();

	    executionTimeBARO = TIM5->CNT - previousBAROTime;
	    }
    }



void MS5611_Get_Coef_Calib(void)
    {
	u8 tmpBuffer[2];
	int i = 0;
	Baro_write_cmd(0x1E); // Reset Device
	HAL_Delay(10);
	Baro_reads( 0xA0 , tmpBuffer, 2);
	manufactura = (uint16_t)(tmpBuffer[i] << 8 | tmpBuffer[i + 1]);
	Baro_reads( 0xA2 , tmpBuffer,  2);
	C1 = (uint16_t)(tmpBuffer[i] << 8 | tmpBuffer[i + 1]);
	Baro_reads( 0xA4 , tmpBuffer,  2);
	C2 = (uint16_t)(tmpBuffer[i] << 8 | tmpBuffer[i + 1]);
	Baro_reads( 0xA6 , tmpBuffer,  2);
	C3 = (uint16_t)(tmpBuffer[i] << 8 | tmpBuffer[i + 1]);
	Baro_reads( 0xA8 , tmpBuffer,  2);
	C4 = (uint16_t)(tmpBuffer[i] << 8 | tmpBuffer[i + 1]);
	Baro_reads( 0xAA , tmpBuffer,  2);
	C5 = (uint16_t)(tmpBuffer[i] << 8 | tmpBuffer[i + 1]);
	Baro_reads( 0xAC , tmpBuffer,  2);
	C6 = (uint16_t)(tmpBuffer[i] << 8 | tmpBuffer[i + 1]);
	Baro_reads( 0xAE , tmpBuffer,  1);
	crc = (uint16_t)(tmpBuffer[i]);
	if (!(C1&&C2&&C3&&C4&&C5&&C6)) Err_Baro = Err_Init;

    }
void Baro_Mag_Init_Full(void)
{
	//создание двоичного семафора для передачи обработки данных
	vSemaphoreCreateBinary(xMAG_BARO_Semaphore);
	xBAROMAG_UKF_Mutex = xSemaphoreCreateMutex();
	//vSemaphoreCreateBinary(xMAG_BARO_Err_Semaphore);
	//xTaskCreate(prvErrToggling,(char*)"MAG_BARO_err", 50, NULL, PRIORITET_TASK_ERR_MAG_BARO, ( xTaskHandle * ) NULL);

	u8 buf1 = 0x70; // CRa = 0-r 11-8 samples 100-15Hz refresh rate  00-Normal measurement configuration
	u8 buf2 = 0xA0; // CRb = 101- resolution ± 4.7 Ga  00000-r
	u8 buf3 = 0;	// Mode = 0-i2c speed normal 00000-r 00-continuous operation
	u8 u1,u2,u3, res;
	uint16_t pause_init;

	Mag_write_register(0x00, buf1);
	Mag_write_register(0x01, buf2);
	Mag_write_register(0x02, buf3);
	HAL_Delay(10);
	res = Mag_read_register(0x02);
	u1 = Mag_read_register(10);
	u2 = Mag_read_register(11);
	u3 = Mag_read_register(12);

	if ( u1 != 'H' && u2 !='4' && u3 !='3') Err_Mag = Err_Init; //HMC5883L IDs

	MS5611_Get_Coef_Calib();

	HAL_Delay(100);
	pause_init = 0;
	//задача обработки данных с датчиков
//	xTaskCreate(prvMAG_BARO_Processing,(char*)"MAG_BARO_Processing", 130, NULL, PRIORITET_TASK_PROCESSING_MAG_BARO, mag_baro_handle);

	//создание задачи переодического вызова магнетометра барометра
	xTaskCreate(prvMAG_BARO_Request,(char*)"MAG_BARO_Request", 70, NULL, PRIORITET_TASK_REQUEST_MAG_BARO, ( xTaskHandle * ) NULL);

}


