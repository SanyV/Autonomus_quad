/*
  Alex Valov 2017
*/

///////////////////////////////////////////////////////////////////////////////

#include "Config.h"
#include "hmc5983_spi.h"
#include "MPU6500.h"

///////////////////////////////////////////////////////////////////////////////
extern SemaphoreHandle_t  xSPI_Mutex ;

SemaphoreHandle_t xMAG_Semaphore = NULL;
SemaphoreHandle_t xMAG_Mutex;

QueueSetHandle_t xQueueSet_MAG;
QueueSetMemberHandle_t xActivatedMember;

portBASE_TYPE xHigherPriorityTaskWoken_MAG = pdFALSE;

uint32_t deltaTimeMAG,   executionTimeMAG ,   previousMAGTime;
float dtMAG;

// Хандл и Стек
TaskHandle_t mag_handle;
uint32_t mag_stack;

extern SPI_HandleTypeDef hspi1;
//Полезные данные
int16_t mx, my, mz;
float mxf, myf, mzf;

//калибровочные коэффициенты
float calibration_matrix[9] = {1.172212f, 0.009437f, 0.023501f, 0.009437f, 1.119674f, 0.029901f, 0.023501f, 0.029901f, 1.157116f };
float bias[3] = { -33.32254f, 184.597310f, -19.555526f};

//float calibration_matrix[9] = {1.58f, -0.012f, 0.001f, -0.379f, 1.609f, -0.083f, -0.574f, 0.064, 1.572f };
//float bias[3] = { 12.375f, -59.357f, -61.673f};

//float calibration_matrix[9] = {1.191266f, 0.003326f, -0.000823f, 0.003326f, 1.141729f, 0.023241f, -0.000823f, 0.023241f, 1.167051f };
//float bias[3] = { -35.918275f, 173.613946f, -27.843390f};

float nonCalbratedMagdata [3];
float CalibratedMagdata [3];

#define HMC5883_ADDRESS 0x1E

#define HMC5883_CONFIG_REG_A    0x00
#define HMC5883_CONFIG_REG_B    0x01
#define HMC5883_MODE_REG        0x02
#define HMC5883_DATA_X_MSB_REG  0x03
#define HMC5883_STATUS_REG      0x09

///////////////////////////////////////////////////////////////////////////////

//#define SENSOR_CONFIG 0x18  // 1 Sample average, 75 Hz
//#define SENSOR_CONFIG 0x38  // 2 Sample average, 75 Hz
//#define SENSOR_CONFIG 0x58  // 4 Sample average, 75 Hz
#define SENSOR_CONFIG 0x78  // 8 Sample average, 75 Hz

//#define SENSOR_CONFIG 0x10  // 1 Sample average, 15 Hz
//#define SENSOR_CONFIG 0x30  // 2 Sample average, 15 Hz
//#define SENSOR_CONFIG 0x50  // 4 Sample average, 15 Hz
//#define SENSOR_CONFIG 0x70      // 8 Sample average, 15 Hz

#define NORMAL_MEASUREMENT_CONFIGURATION 0x00
#define POSITIVE_BIAS_CONFIGURATION      0x01

///////////////////////////////////////////////////////////////////////////////

//#define SENSOR_GAIN 0x00  // +/- 0.88 Ga
//#define SENSOR_GAIN 0x20        // +/- 1.3  Ga (default)
//#define SENSOR_GAIN 0x40  // +/- 1.9  Ga
//#define SENSOR_GAIN 0x60  // +/- 2.5  Ga
//#define SENSOR_GAIN 0x80  // +/- 4.0  Ga
#define SENSOR_GAIN 0xA0  // +/- 4.7  Ga
//#define SENSOR_GAIN 0xC0  // +/- 5.6  Ga
//#define SENSOR_GAIN 0xE0  // +/- 8.1  Ga

///////////////////////////////////////////////////////////////////////////////

#define OP_MODE_CONTINUOUS 0x00 // Continuous conversion
#define OP_MODE_SINGLE     0x01 // Single conversion

#define STATUS_RDY         0x01 // Data Ready

///////////////////////////////////////////////////////////////////////////////

// Matrix variables
arm_matrix_instance_f32 a;
arm_matrix_instance_f32 b;
arm_matrix_instance_f32 x;


float magScaleFactor[3];

uint8_t magDataUpdate = false;

uint8_t newMagData = false;

int16_t rawMag[3];

float nonRotatedMagData[3];
uint8_t Mag_Buffer_Rx[6];
uint8_t Err_MAG;


///////////////////////////////////////////////////////////////////////////////

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
	xSemaphoreTake(xSPI_Mutex, portMAX_DELAY);
	MAG_CS_LOW;
	SPIx_SendByte(reg_addr | 0x80 | 0x40);
    if (HAL_SPI_Receive(&hspi1, pData, Size, 1000) != HAL_OK)
    {
    	Err_MAG = 1;
    }
    Err_MAG = 0;
	MAG_CS_HIGH;
	xSemaphoreGive(xSPI_Mutex);
}


///////////////////////////////////////////////////////////////////////////////
// Read Magnetometer
///////////////////////////////////////////////////////////////////////////////
uint8_t MAG_Flag;
///////////////////////////////////////////////////////////////////////////////
void EXTI_MAG_IRQHandler(void)
{
//	static uint16_t led_counter;
	 if(__HAL_GPIO_EXTI_GET_IT(MAG_INT_Pin) != RESET)
	 {
		 __HAL_GPIO_EXTI_CLEAR_IT(MAG_INT_Pin);
		//данные готовы, теперь надо их забрать, отправляем запрос , установить флаг готовности данных
	// xSemaphoreGiveFromISR(xMAG_Semaphore,  &xHigherPriorityTaskWoken_MAG);
		 MAG_Flag = 1;
	 }

}


void readMag(void *unused)
{

	static uint32_t currentTime;
	while (1)
	{
	//  if( xSemaphoreTake(xMAG_Semaphore, portMAX_DELAY ) == pdTRUE )
		if (MAG_Flag)
	  {
	    currentTime       = TIM5->CNT;
	    deltaTimeMAG    = currentTime - previousMAGTime;
	    previousMAGTime = currentTime;
	    dtMAG = (float)deltaTimeMAG * 0.0000001f;

       Mag_reads( HMC5883_DATA_X_MSB_REG, &Mag_Buffer_Rx[0], 6);

    //raw mag data
    mx = (int16_t)(((uint16_t)Mag_Buffer_Rx[0]<<8)+Mag_Buffer_Rx[1]);
    mz = (int16_t)(((uint16_t)Mag_Buffer_Rx[2]<<8)+Mag_Buffer_Rx[3]);
    my = (int16_t)(((uint16_t)Mag_Buffer_Rx[4]<<8)+Mag_Buffer_Rx[5]);

    // Apply biases
    nonCalbratedMagdata[0] = mx - bias[0];
    nonCalbratedMagdata[1] = my - bias[1];
    nonCalbratedMagdata[2] = mz - bias[2];

    // Калибровка показаний магнетометра
	  arm_mat_init_f32(&a, 3, 3, (float *)calibration_matrix);
	  arm_mat_init_f32(&b, 3, 1, (float *)nonCalbratedMagdata);
	  arm_mat_init_f32(&x, 3, 1, (float *)CalibratedMagdata);
	  arm_mat_mult_f32(&a, &b, &x);
		//защитить
	  xSemaphoreTake(xMAG_Mutex, portMAX_DELAY);
	  mxf = CalibratedMagdata[0];
	  myf = CalibratedMagdata[1];
	  mzf = CalibratedMagdata[2];
      xSemaphoreGive(xMAG_Mutex);

    Mag_write_register (HMC5883_MODE_REG, OP_MODE_SINGLE );
    executionTimeMAG = TIM5->CNT - previousMAGTime;
    MAG_Flag = 0;
	}

	  else  //обходной путь для решения вопроса с залипанием компаса на высокой SPI CLK > 8Мгц
	  {
	//	  timeout_mag++;
		  if (abs(previousMAGTime - TIM5->CNT) > 500000)
		  { // reinit mag
			    Mag_write_register (HMC5883_MODE_REG,  OP_MODE_SINGLE );
		  }
		  vTaskDelay(2);
	  }
	}
}


///////////////////////////////////////////////////////////////////////////////
// Initialize Magnetometer
///////////////////////////////////////////////////////////////////////////////

void EXTI_Init_MAG(void)
{

  GPIO_InitTypeDef GPIO_InitStruct;
  __HAL_RCC_GPIOD_CLK_ENABLE();

  /*Configure GPIO pin : MAG_INT_Pin */
  GPIO_InitStruct.Pin = MAG_INT_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING  ;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(MAG_INT_GPIO_Port, &GPIO_InitStruct);

  HAL_NVIC_SetPriority(EXTI_MAG_IRQ, 5, 0x00);
  HAL_NVIC_EnableIRQ(EXTI_MAG_IRQ);


}


void initMag(void)
{
  //  vSemaphoreCreateBinary(xMAG_Semaphore);
  //  xSemaphoreTake(xMAG_Semaphore, portMAX_DELAY );

    //readMag();
    MAG_Flag = 0;
	cliPortPrintF("FreeHeapSize =  %6d, before MAG_task \r\n", xPortGetFreeHeapSize());
	//создание задачи переодического вызова магнетометра барометра
	xTaskCreate(readMag, (char*)"MAG_proces", 700, NULL, 9, mag_handle);

	EXTI_Init_MAG();

    Mag_write_register (HMC5883_CONFIG_REG_A, SENSOR_CONFIG | NORMAL_MEASUREMENT_CONFIGURATION ); //Не работает с POSITIVE_BIAS_CONFIGURATION
    HAL_Delay(50);
    Mag_write_register (HMC5883_CONFIG_REG_B, SENSOR_GAIN);
    HAL_Delay(20);
    Mag_write_register (HMC5883_MODE_REG, OP_MODE_SINGLE); // OP_MODE_SINGLE
    HAL_Delay(20);
//    Err_MAG = 0;

}

///////////////////////////////////////////////////////////////////////////////
