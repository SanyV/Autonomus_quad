/*
  Alex Valov 2017

*/
///////////////////////////////////////////////////////////////////////////////

#include "Config.h"
#include "ms5611_spi.h"
#include "MPU6500.h"
#include <math.h>

///////////////////////////////////////////////////////////////////////////////
SemaphoreHandle_t xBARO_Semaphore = NULL;
SemaphoreHandle_t xBARO_Mutex;
uint32_t deltaTimeBARO,   executionTimeBARO ,   previousBAROTime;
float dtBARO;

// Хандл и Стек
TaskHandle_t baro_handle;
uint32_t baro_stack;
///////////////////////////////////////

//#define OSR  256  // 0.60 mSec conversion time (1666.67 Hz)
//#define OSR  512  // 1.17 mSec conversion time ( 854.70 Hz)
//#define OSR 1024  // 2.28 mSec conversion time ( 357.14 Hz)
//#define OSR 2048  // 4.54 mSec conversion time ( 220.26 Hz)
  #define OSR 4096  // 9.04 mSec conversion time ( 110.62 Hz)

///////////////////////////////////////

extern SemaphoreHandle_t  xSPI_Mutex ;

extern SPI_HandleTypeDef hspi1;

float Temperature, Altitude, Pressure;

uint16_t c1, c2, c3, c4, c5, c6;

uint32_t d1;

uint32_t d1Value;

uint32_t d2;

uint32_t d2Value;

int32_t dT;

int32_t ms5611Temperature;

uint8_t newPressureReading = false;

uint8_t newTemperatureReading = false;
float base_alt = 0;
float altitude = 0;

////////////////////////////////////////////////
int64_t offset;
int64_t offset2 = 0;

int64_t sensitivity;
int64_t sensitivity2 = 0;

int64_t f;

int64_t p;
//double p;

int64_t ms5611Temp2  = 0;
////////////////////////////////////////////////

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
void Baro_reads(uint8_t reg_addr,  uint8_t *pdata_buf, uint16_t Size)
{
	xSemaphoreTake(xSPI_Mutex, portMAX_DELAY);
	BARO_CS_LOW;
	SPIx_SendByte( reg_addr);
    if (HAL_SPI_Receive(&hspi1, (uint8_t *)pdata_buf, Size, 1000) != HAL_OK)
    {
   // 	Err_Baro = Err_synq;
//    	return -1;
    }
	BARO_CS_HIGH;
	xSemaphoreGive(xSPI_Mutex);
}
void Baro_write_cmd( uint8_t value1 )
{
	xSemaphoreTake(xSPI_Mutex, portMAX_DELAY);
	BARO_CS_LOW;
	SPIx_SendByte(value1);
	BARO_CS_HIGH;
	xSemaphoreGive(xSPI_Mutex);
}
///////////////////////////////////////////////////////////////////////////////
// Read Temperature Request Pressure
///////////////////////////////////////////////////////////////////////////////

void readTemperatureRequestPressure(void)
{
	static uint8_t data_buf[3];

    Baro_reads(0x00, &data_buf[0], 3); 			// Request temperature read

    d2 = (uint32_t)(data_buf[0] << 16 | data_buf[1] << 8 | data_buf[2]);

    #if   (OSR ==  256)
	    Baro_write_cmd(  0x40);  // Request pressure conversion
	#elif (OSR ==  512)
	    Baro_write_cmd(  0x42);
	#elif (OSR == 1024)
	    Baro_write_cmd(  0x44);
	#elif (OSR == 2048)
	    Baro_write_cmd(  0x46);
	#elif (OSR == 4096)
	    Baro_write_cmd(  0x48);
    #endif
}

///////////////////////////////////////////////////////////////////////////////
// ReadPressureRequestPressure
///////////////////////////////////////////////////////////////////////////////

void readPressureRequestPressure(void)
{
    static uint8_t data_buf[3];

    Baro_reads(0x00, &data_buf[0], 3);    // Request pressure read

    d1 = (uint32_t)(data_buf[0] << 16 | data_buf[1] << 8 | data_buf[2]);

    #if   (OSR ==  256)
	    Baro_write_cmd(  0x40);  // Request pressure conversion
	#elif (OSR ==  512)
		Baro_write_cmd(  0x42);
	#elif (OSR == 1024)
	    Baro_write_cmd(  0x44);
	#elif (OSR == 2048)
	    Baro_write_cmd(  0x46);
	#elif (OSR == 4096)
	    Baro_write_cmd(  0x48);
    #endif
}

///////////////////////////////////////////////////////////////////////////////
// Read Pressure Request Temperature
///////////////////////////////////////////////////////////////////////////////

void readPressureRequestTemperature(void)
{
    uint8_t data_buf[3];

    Baro_reads(0x00, &data_buf[0], 3);    // Request pressure read

    d1 = (uint32_t)(data_buf[0] << 16 | data_buf[1] << 8 | data_buf[2]);

    #if   (OSR ==  256)
	    Baro_write_cmd(  0x50);   // Request temperature converison
	#elif (OSR ==  512)
	    Baro_write_cmd(  0x52);
	#elif (OSR == 1024)
	    Baro_write_cmd(  0x54);
	#elif (OSR == 2048)
	    Baro_write_cmd(  0x56);
	#elif (OSR == 4096)
	    Baro_write_cmd(  0x58);
    #endif
}

///////////////////////////////////////////////////////////////////////////////
// Calculate Temperature
///////////////////////////////////////////////////////////////////////////////

void calculateTemperature(void)
{
    dT                = (int32_t)d2Value - ((int32_t)c5 << 8);
    ms5611Temperature = (int32_t)2000 + (int32_t)(((int64_t)dT * c6) >> 23);
    Temperature = (double)ms5611Temperature / (double)100.0f;

}

///////////////////////////////////////////////////////////////////////////////
// Calculate Pressure Altitude
///////////////////////////////////////////////////////////////////////////////

void calculatePressureAltitude(void)
{
	offset      = ((int64_t)c2<< 16) + (((int64_t)c4* dT) >> 7);
	sensitivity = ((int64_t)c1<< 15) + (((int64_t)c3* dT) >> 8);

	if (ms5611Temperature < 2000)
	{
		ms5611Temp2  = SQR(dT) >> 31;

		f	 		 = SQR(ms5611Temperature - 2000);
		offset2      = 5 * f >> 1;
		sensitivity2 = 5 * f >> 2;

		if (ms5611Temperature < -1500)
		{
			f 			  = SQR(ms5611Temperature + 1500);
			offset2      +=  7 * f;
			sensitivity2 += 11 * f >> 1;
		}

		ms5611Temperature -= ms5611Temp2;
		offset -= offset2;
		sensitivity -= sensitivity2;
	}
	else
	{
		ms5611Temp2 = 0;
		offset2 = 0;
		sensitivity2 = 0;
	}
	//p = (float)((((int64_t)d1Value * sensitivity) >> 21) - offset)*(1.0f/32768.0f);
	p = ((((int64_t)d1Value * sensitivity) >> 21) - offset) >> 15;
	//p = (double)((((int64_t)d1Value * sensitivity) >> 21) - offset) *(double) (1.0f/32768.0f);

	Pressure = p;
	//remove sonar support to debug  AltHold
	altitude = 44330.0f * (1.0f - pow( (double)p / 101325.0f, 1.0f / 5.255f));
	Altitude = altitude - base_alt;

}

///////////////////////////////////////////////////////////////////////////
void BARO_Processing(void *pvParameters)
    {
	static uint32_t currentTime;

while(1)
    {
    currentTime       = TIM5->CNT;
    deltaTimeBARO    = currentTime - previousBAROTime;
    previousBAROTime = currentTime;
    dtBARO = (float)deltaTimeBARO * 0.0000001f;

    if (!newTemperatureReading)
	{
		readTemperatureRequestPressure();
		vTaskDelay(10);
	    newTemperatureReading = true;
	}
	else
	{
	    readPressureRequestTemperature();
	    vTaskDelay(10);
	    newPressureReading = true;
	}

    // Update altitude/temperature measurements
    if (newTemperatureReading && newPressureReading)
    {
        d1Value = d1;
        d2Value = d2;

        calculateTemperature();
        calculatePressureAltitude();

        newTemperatureReading = false;
        newPressureReading    = false;
    //    vTaskDelay(30);
    }
	//baro_stack = uxTaskGetStackHighWaterMark(baro_handle);
	//ovf_full_stack = xPortGetFreeHeapSize();
    executionTimeBARO = TIM5->CNT - previousBAROTime;

   }
 }
///////////////////////////////////////////////////////////////////////////////
// Pressure Initialization
///////////////////////////////////////////////////////////////////////////////

void initPressure(void)
{
    static uint8_t data_buf[2];
    static uint16_t samples;
    float temp_alt = 0.0f;

    Baro_write_cmd(  0x1E);      // Reset Device

    HAL_Delay(10);

    Baro_reads(0xA2, data_buf, 2);    // Read Calibration data_buf C1
    c1 = (uint16_t)(data_buf[0] << 8 | data_buf[1]);

    Baro_reads(0xA4, data_buf, 2);    // Read Calibration data_buf C2
    c2 = (uint16_t)(data_buf[0] << 8 | data_buf[1]);

    Baro_reads(0xA6, data_buf, 2);    // Read Calibration data_buf C3
    c3 = (uint16_t)(data_buf[0] << 8 | data_buf[1]);

    Baro_reads(0xA8, data_buf, 2);    // Read Calibration data_buf C4
    c4 = (uint16_t)(data_buf[0] << 8 | data_buf[1]);

    Baro_reads(0xAA, data_buf, 2);    // Read Calibration data_buf C5
    c5 = (uint16_t)(data_buf[0] << 8 | data_buf[1]);

    Baro_reads(0xAC, data_buf, 2);    // Read Calibration data_buf C6
    c6 = (uint16_t)(data_buf[0] << 8 | data_buf[1]);

	HAL_Delay(1000);
    #if   (OSR ==  256)
	    Baro_write_cmd 0x50);  // Request temperature conversion
	#elif (OSR ==  512)
	    Baro_write_cmd 0x52);
	#elif (OSR == 1024)
	    Baro_write_cmd(0x54);
	#elif (OSR == 2048)
	    Baro_write_cmd(0x56);
	#elif (OSR == 4096)
	    Baro_write_cmd(0x58);
    #endif

	HAL_Delay(10);
    for (samples = 0; samples < 100; samples++)
    {
    readTemperatureRequestPressure();
	HAL_Delay(10);

    readPressureRequestTemperature();
	HAL_Delay(10);

    d1Value = d1;
    d2Value = d2;

    calculateTemperature();
    calculatePressureAltitude();
    temp_alt += altitude;
    }

    base_alt = temp_alt / 100.0f;

    newTemperatureReading = false;
    newPressureReading    = false;
	cliPortPrintF("FreeHeapSize =  %6d, before BARO_task \r\n", xPortGetFreeHeapSize());
	//создание задачи переодического вызова магнетометра барометра
	xTaskCreate(BARO_Processing,(char*)"BARO", 1000, NULL, PRIORITET_TASK_REQUEST_MAG_BARO, baro_handle);

}

///////////////////////////////////////////////////////////////////////////////
