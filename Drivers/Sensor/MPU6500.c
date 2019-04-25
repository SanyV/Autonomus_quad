
#include "stm32h7xx_hal.h"
#include "cmsis_os.h"
#include "MPU6500.h"
#include "IMU_INS.h"
#include "UKF_lib.h"
#include "Config.h"


//#include "UKF_lib.h"

SPI_HandleTypeDef hspi1;
float acc_divider;
float gyro_divider;

//������ � ������������ ��������
int16_t ax, ay, az, gx, gy, gz;//����� ����� � 16 ��������
float temp_gyro;//����������� �������
float axf, ayf, azf, gxf, gyf, gzf;//���������� ������ � �������� � g � ���/���
//****************************************************//

//�������������� ��������� ���������
uint8_t flag_end_of_gyro_calib;//���� ����� ����������
uint32_t cnt_cycle_ins, cnt_ticks_ins;//�������� �����
uint32_t ovf_ins_stack;//��������� ����

//****************************************************//

//������ ��� UKF
//������� ��� ����� � ������� ���������

SemaphoreHandle_t  xMPU_UKF_Mutex, xUKF_PID_Mutex, xMPU_Mutex, xSPI_Mutex;
TaskHandle_t ukf_handle;
uint32_t ovf_ukf_stack;
//****************************************************//

//������������� ���������
const float Z_plus = 4160.0f, Z_minus = 4175.0f, Y_plus = 4120.0f, Y_minus = 4090.0f, X_plus = 4200.0f, X_minus = 4000.0f;

const float ax_bias = 89.564543f, ay_bias = 11.174040f, az_bias = -10.346222f;
const float am1 = 0.999500f, 	am2 = -0.000151f, 	am3 = 0.003176f;
const float			 am5 = 0.999091f, 	am6 = 0.005234f;
const float  						am9 = 0.983432f;
//****************************************************//

uint32_t deltaTimeINS,   executionTimeINS,   previousINSTime;
uint32_t deltaTimeMPU,   executionTimeMPU ,   previousMPUTime;
float dtMPU, dtINS;
uint32_t deltaTimeGPScalib,   executionTimeGPScalib ,   previousGPSTimecalib;

//��������������� �������� ��� �������
uint8_t spi_counter_byte;//���-�� ���������� ���� � ������
uint32_t counter_work_int_exti, counter_work_int_spi;
//****************************************************//
enum { x, y, z };
uint8_t acc_bias_data_save[6];
int32_t acc_offset[3];
//������ �������
enum Err_Sensor Err_MPU;
//****************************************************//

//������� ��� ����� � ������� ���������
SemaphoreHandle_t xMPU_Semaphore = NULL;
SemaphoreHandle_t xMPUDataReady_Semaphore = NULL;

#define ACC_ERR_COLLECT_COUNT 200

TaskHandle_t ins_handle;
TaskHandle_t readmpu_handle;
//****************************************************//
extern SPI_HandleTypeDef hspi1;
portBASE_TYPE xHigherPriorityTaskWoken_MPU6500 = pdFALSE;

//�������� ����� ������
uint8_t MPU_buff[14];
//****************************************************//
uint16_t led_counter;
void EXTI_IRQHandler(void)
{

	 if(__HAL_GPIO_EXTI_GET_IT(EXTI_SPI_MPU_Pin) != RESET)
	 {
		 __HAL_GPIO_EXTI_CLEAR_IT(EXTI_SPI_MPU_Pin);
	//	 if (led_counter==249) Led2_Toggle;
	//	 led_counter = (led_counter + 1) % 1000;
		//������ ������, ������ ���� �� �������, ���������� ������ , ��������� ���� ���������� ������
		 xSemaphoreGiveFromISR(xMPUDataReady_Semaphore,  &xHigherPriorityTaskWoken_MPU6500);
		 portYIELD_FROM_ISR(xHigherPriorityTaskWoken_MPU6500);

	 }

}

//������ �������������� �������� ����� � 1 ������� � ������� ���������� �� ������� GPS TimePulse ��� ����� ����� ������� dtUKF
void EXTI9_5_IRQHandler(void)
{
	  static volatile uint32_t currentTime;
	  __disable_irq();
	 if(__HAL_GPIO_EXTI_GET_IT(GPIO_PIN_7) != RESET)
	 {
		 __HAL_GPIO_EXTI_CLEAR_IT(GPIO_PIN_7);
         currentTime       = TIM5->CNT;
         deltaTimeGPScalib    = abs(currentTime - previousGPSTimecalib);
         previousGPSTimecalib = currentTime;
	 }
	 __enable_irq();
}
void readMPU(void *unused)
{
	  static uint8_t sTxBuf[14];
	  static volatile uint32_t currentTime;
	  static volatile uint32_t mpu_count = 0;
	  static float axf1, ayf1, azf1, gxf1, gyf1, gzf1;//���������� ������ � �������� � g � ���/���
//	  static uint8_t sRxBuf[22] ;

		while (1)
	{
		//	������� �������� ���������� ������ ����/������ ��� ������������� ���� 1000��
	  if( xSemaphoreTake(xMPUDataReady_Semaphore, portMAX_DELAY ) == pdTRUE )
	  {

          currentTime       = TIM5->CNT;
          deltaTimeMPU    = currentTime - previousMPUTime;
          previousMPUTime = currentTime;
          dtMPU = (float)deltaTimeMPU * 0.0000001f;


//		LED_Toggle(3);
	  sTxBuf[0]= ACCEL_XOUT_H_REG|0x80;
	  xSemaphoreTake(xSPI_Mutex, portMAX_DELAY);
	  CS_LOW;
    if (HAL_SPI_TransmitReceive(&hspi1, (uint8_t*)sTxBuf, (uint8_t*)MPU_buff, 1, 1000) != HAL_OK)
    {
    	Err_MPU = Err_synq;
//    	LED_On(3);
    	return;
    }

    if (HAL_SPI_Receive(&hspi1, (uint8_t*)MPU_buff, 14, 1000) != HAL_OK)
    {
    	Err_MPU = Err_synq;
//  	LED_On(3);
    	return;
    }
	CS_HIGH;
	xSemaphoreGive(xSPI_Mutex);
/*
	ax = (int16_t)(((uint16_t)MPU_buff[0]<<8)+MPU_buff[1]);
	ay = (int16_t)(((uint16_t)MPU_buff[2]<<8)+MPU_buff[3]);
	az = (int16_t)(((uint16_t)MPU_buff[4]<<8)+MPU_buff[5]);

	gx = (int16_t)(((uint16_t)MPU_buff[8]<<8)+MPU_buff[9]);
	gy = (int16_t)(((uint16_t)MPU_buff[10]<<8)+MPU_buff[11]);
	gz = (int16_t)(((uint16_t)MPU_buff[12]<<8)+MPU_buff[13]);

	temp_gyro = (int16_t)(((uint16_t)MPU_buff[6]<<8)+MPU_buff[7]);

	gyf1 += (float)gx / gyro_divider; //��� ��������� � ������ ��������� ������  x � y
	gxf1 += (float)gy / gyro_divider;
	gzf1 += -(float)gz / gyro_divider; // Yaw ������������� ��� ���������� � ������� ��������� NED

	ayf1 += ((float)ax / acc_divider);
	axf1 += ((float)ay / acc_divider);
	azf1 += (-(float)az / acc_divider); // Yaw ������������� ��� ���������� � ������� ��������� NED


	mpu_count++;
	if (!(mpu_count % 5 )) {
		gxf = gxf1 / 5.0f;  //������ ��������� ��� ���������� ��������� ���������??? ����� ���������
		gyf = gyf1 / 5.0f;
		gzf = gzf1 / 5.0f;
		gxf1 = 0.0f;
		gyf1 = 0.0f;
		gzf1 = 0.0f;
		axf = axf1 / 5.0f;
		ayf = ayf1 / 5.0f;
		azf = azf1 / 5.0f;
		axf1 = 0.0f;
		ayf1 = 0.0f;
		azf1 = 0.0f;
		xSemaphoreGive(xMPU_Semaphore);
	}
	*/
	xSemaphoreGive(xMPU_Semaphore);
	  }
      // Calculate execution tick process time
	//  xSemaphoreTake(xMPU_Mutex, portMAX_DELAY);
      executionTimeMPU = TIM5->CNT - previousMPUTime;
  //    xSemaphoreGive(xMPU_Mutex);
	}
}

uint8_t SPIx_SendByte(uint8_t byte)
{
	uint8_t sTxBuf[1];
	uint8_t sRxBuf[1];

	sTxBuf[0]= byte;
    if (HAL_SPI_TransmitReceive(&hspi1, (uint8_t*)sTxBuf, (uint8_t*)sRxBuf, 1, 1000) != HAL_OK)
    {
    	Err_MPU = Err_synq;
    	return -1;
    }


    return sRxBuf[0];
}


uint8_t MPU_Write(uint8_t reg_addr, uint8_t data){
	uint8_t byte;
	xSemaphoreTake(xSPI_Mutex, portMAX_DELAY);
	CS_LOW;
	SPIx_SendByte(reg_addr);
	byte = SPIx_SendByte(data);
	CS_HIGH;
	xSemaphoreGive(xSPI_Mutex);
	return byte;
}

int SPIx_Writes(uint8_t reg_addr, uint8_t* data, uint8_t len){
	uint32_t i = 0;
	xSemaphoreTake(xSPI_Mutex, portMAX_DELAY);
	CS_LOW;
	SPIx_SendByte(reg_addr);
	while(i < len){
		SPIx_SendByte(data[i++]);
	}
	CS_HIGH;
	xSemaphoreGive(xSPI_Mutex);
	return 0;
}

uint8_t SPIx_Read(uint8_t addr, uint8_t reg_addr)
{
	uint8_t dummy = 0;
	uint8_t data = 0;
	xSemaphoreTake(xSPI_Mutex, portMAX_DELAY);
	CS_LOW;
	SPIx_SendByte(0x80 | reg_addr);
	data = SPIx_SendByte(dummy);
	CS_HIGH;
	xSemaphoreGive(xSPI_Mutex);
	return data;
}


void EXTI_Init_MPU6500(void)
{

  GPIO_InitTypeDef GPIO_InitStruct;


  __HAL_RCC_GPIOD_CLK_ENABLE();

  /*Configure GPIO pins : INT_MPU_Pin INT_MAG_Pin */
  GPIO_InitStruct.Pin = GPIO_PIN_13 | GPIO_PIN_7;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(EXTI_SPI_MPU_Port_Group, &GPIO_InitStruct);

	HAL_NVIC_SetPriority(EXTI_MPU_IRQ, 6, 0x00);
	HAL_NVIC_EnableIRQ(EXTI_MPU_IRQ);

	HAL_NVIC_SetPriority(EXTI9_5_IRQn, 10, 0x00);  // ��������� 10
	HAL_NVIC_EnableIRQ(EXTI9_5_IRQn);

}


uint8_t spi_recieve(void)
{
	//HAL_Delay(10);
	return SPIx_SendByte(0x00);
}
uint8_t SPI_read_register( uint8_t reg1 )
{
	uint8_t res;
	xSemaphoreTake(xSPI_Mutex, portMAX_DELAY);
	CS_LOW;
	SPIx_SendByte(reg1|0x80);
	res = spi_recieve();
	CS_HIGH;
	xSemaphoreGive(xSPI_Mutex);
	return res;


}
void SPI_write_register( uint8_t reg1, uint8_t value1 )
{
	xSemaphoreTake(xSPI_Mutex, portMAX_DELAY);
	CS_LOW;
	SPIx_SendByte(reg1);
	SPIx_SendByte(value1);
	CS_HIGH;
	xSemaphoreGive(xSPI_Mutex);
}
void SPI_write_bits( uint8_t reg, uint8_t mask )
{
	uint8_t registr, value;
	uint16_t pause_init;
	for (pause_init = 0; pause_init < 1600; pause_init++);
	pause_init = 0;
	CS_LOW;
	registr = SPI_read_register(reg);
	value = registr|mask;
	for (pause_init = 0; pause_init < 1600; pause_init++);
	pause_init = 0;
	SPI_write_register(reg, value);
	CS_HIGH;
	for (pause_init = 0; pause_init < 1600; pause_init++);
	pause_init = 0;
}

static void mpu6000ReliablySetReg(uint8_t reg, uint8_t val) {
 uint8_t tmp;

	do {
        HAL_Delay(10);
        SPI_write_register(reg, val);
        HAL_Delay(10);
        tmp = SPI_read_register(reg);
    } while (tmp != val);
}
uint16_t mpuid=0;

void MPU_get_acc_divider(void)
{
    uint8_t scale;
    scale = SPI_read_register(MPUREG_ACCEL_CONFIG);
    scale &= 0x18;
    switch (scale)
    {
    case BITS_FS_2G:
        acc_divider = 16384;
        break;
    case BITS_FS_4G:
        acc_divider = 8192;
        break;
    case BITS_FS_8G:
        acc_divider = 4096;
        break;
    case BITS_FS_16G:
        acc_divider = 2048;
        break;
    }
}

void MPU_get_gyro_divider(void)
{
    uint8_t scale;
    scale = SPI_read_register(MPUREG_GYRO_CONFIG);
    scale &= 0x18;
    switch (scale)
    {
    case BITS_FS_250DPS:
        gyro_divider = 131;
        break;
    case BITS_FS_500DPS:
        gyro_divider = 65.5;
        break;
    case BITS_FS_1000DPS:
        gyro_divider = 32.8;
        break;
    case BITS_FS_2000DPS:
        gyro_divider = 16.4;
        break;
    }
}

MPU_ReadRegs (uint8_t reg, uint8_t *MPU_buff, uint16_t len)
{
uint8_t 	sTxBuf[1];

sTxBuf[0]= reg|0x80;
CS_LOW;
if (HAL_SPI_TransmitReceive(&hspi1, (uint8_t*)sTxBuf, (uint8_t*)MPU_buff, 1, 1000) != HAL_OK)
{
	Err_MPU = Err_synq;
//    	LED_On(3);
//	return;
}
if (HAL_SPI_Receive(&hspi1, (uint8_t*)MPU_buff, len, 1000) != HAL_OK)
{
	Err_MPU = Err_synq;
//  	LED_On(3);
//	return;
}
CS_HIGH;
}

void MPU_GyroCalibration(void)
{
    uint8_t response[6], divider;
    uint8_t i, j;
    int16_t offset_tmp[3] = { 0, 0, 0 };
    int32_t offset[3] = { 0, 0, 0 };
    uint8_t data[6] = { 0, 0, 0, 0, 0, 0 };

    for (j = 0; j < 200; j++)
    {
        MPU_ReadRegs(MPUREG_GYRO_XOUT_H, response, 6);
        for (i = x; i <= z; i++)
        {
            offset_tmp[i] = ((int16_t) response[i * 2] << 8) | response[i * 2 + 1];
            offset[i] -= (int32_t) offset_tmp[i];
        }
        HAL_Delay(5);
    }

    divider = MPU_Write(MPUREG_GYRO_CONFIG | READ_FLAG, 0x00);
    divider &= 0x18;

    switch (divider)
    {

    case BITS_FS_250DPS:
        divider = 8;
        break;
    case BITS_FS_500DPS:
        divider = 4;
        break;
    case BITS_FS_1000DPS:
        divider = 2;
        break;
    case BITS_FS_2000DPS:
        divider = 1;
        break;
    }

    // offset register referred to 1000 DPS
    // From Invensense Motion Driver, avoid round errors
    offset[x] = (int32_t) ( ( (int64_t) offset[x] ) / divider / 100 );
    offset[y] = (int32_t) ( ( (int64_t) offset[y] ) / divider / 100 );
    offset[z] = (int32_t) ( ( (int64_t) offset[z] ) / divider / 100 );

    // This works too
    //offset[x] /= (int32_t) (100 * divider);
    //offset[y] /= (int32_t) (100 * divider);
    //offset[z] /= (int32_t) (100 * divider);

    // swapped bytes
    data[0] = (offset[x] >> 8) & 0xFF;
    data[1] = offset[x] & 0xFF;
    data[2] = (offset[y] >> 8) & 0xFF;
    data[3] = offset[y] & 0xFF;
    data[4] = (offset[z] >> 8) & 0xFF;
    data[5] = offset[z] & 0xFF;

    SPIx_Writes(MPUREG_XG_OFFS_USRH, (uint8_t*) &data[0], 2);
    SPIx_Writes(MPUREG_YG_OFFS_USRH, (uint8_t*) &data[2], 2);
    SPIx_Writes(MPUREG_ZG_OFFS_USRH, (uint8_t*) &data[4], 2);
}

void MPU_AccCalibration(int32_t *acc_offset)
{
    int16_t bias[3] = { 0, 0, 0 };
    uint8_t data[6] = { 0, 0, 0, 0, 0, 0 };

    bias[0] = ((int16_t) acc_bias_data_save[0] << 8) | acc_bias_data_save[1];
    bias[1] = ((int16_t) acc_bias_data_save[2] << 8) | acc_bias_data_save[3];
    bias[2] = ((int16_t) acc_bias_data_save[4] << 8) | acc_bias_data_save[5];

    bias[0] -= (acc_offset[0] & ~1);
    bias[1] -= (acc_offset[1] & ~1);
    bias[2] -= (acc_offset[2] & ~1);

    // swapped bytes
    data[0] = (bias[x] >> 8) & 0xFF;
    data[1] = bias[x] & 0xFF;
    data[2] = (bias[y] >> 8) & 0xFF;
    data[3] = bias[y] & 0xFF;
    data[4] = (bias[z] >> 8) & 0xFF;
    data[5] = bias[z] & 0xFF;

    SPIx_Writes(MPUREG_XA_OFFSET_H, (uint8_t*) &data[0], 2);
    SPIx_Writes(MPUREG_YA_OFFSET_H, (uint8_t*) &data[2], 2);
    SPIx_Writes(MPUREG_ZA_OFFSET_H, (uint8_t*) &data[4], 2);
}

uint8_t Acc_Collect_Error_cal(int32_t *acc_offset, uint8_t count)
{
    uint8_t response[6], divider;
    int16_t acc_offset_tmp[3];
    uint8_t i;

    MPU_ReadRegs(MPUREG_ACCEL_XOUT_H, response, 6);
    for (i = x; i <= z; i++)
    {
        acc_offset_tmp[i] = ((int16_t) response[i * 2] << 8) | response[i * 2 + 1];
        acc_offset[i] += (int32_t) acc_offset_tmp[i];
    }

    if (count == 0) // finally process average offsets
    {
        divider = SPI_read_register(MPUREG_ACCEL_CONFIG);
        divider &= 0x18;

        switch (divider)
        {
        case BITS_FS_2G:
            divider = 8;
            break;
        case BITS_FS_4G:
            divider = 4;
            break;
        case BITS_FS_8G:
            divider = 2;
            break;
        case BITS_FS_16G:
            divider = 1;
            break;
        }

        // offset register referred to 16G (against example from Invensense which says 8G)
        // from Invensense Motion Driver, avoid round errors
        acc_offset[x] = (int32_t) ( ( (int64_t) acc_offset[x] ) / divider / ACC_ERR_COLLECT_COUNT );
        acc_offset[y] = (int32_t) ( ( (int64_t) acc_offset[y] ) / divider / ACC_ERR_COLLECT_COUNT );
        acc_offset[z] = (int32_t) ( ( (int64_t) acc_offset[z] ) / divider / ACC_ERR_COLLECT_COUNT );

        // One axis must be very near to vertical, subtract value for 1G from it
        for (i = 0; i <= 2; i++)
        {
            if (fabsf(acc_offset[i]) > 1000)
            {
                if (acc_offset[i] > 1000)
                {
                    acc_offset[i] -= 2048;
                }
                else if (acc_offset[i] < 1000)
                {
                    acc_offset[i] += 2048;
                }
            }
        }

        MPU_AccCalibration(acc_offset);

        return 0;
    }

    return 1;
}

void Revert_Acc_Cancel()
{
    // restore saved cancellation registers
	SPIx_Writes(MPUREG_XA_OFFSET_H, (uint8_t*) &acc_bias_data_save[0], 2);
	SPIx_Writes(MPUREG_YA_OFFSET_H, (uint8_t*) &acc_bias_data_save[2], 2);
	SPIx_Writes(MPUREG_ZA_OFFSET_H, (uint8_t*) &acc_bias_data_save[4], 2);
}

void Get_MPU_Acc_Offset(int32_t *acc_offset)
{
    uint8_t i, j;
    uint8_t response[6], divider;
    int16_t acc_offset_tmp[3];

    // restore saved cancellation registers
    SPIx_Writes(MPUREG_XA_OFFSET_H, (uint8_t*) &acc_bias_data_save[0], 2);
    SPIx_Writes(MPUREG_YA_OFFSET_H, (uint8_t*) &acc_bias_data_save[2], 2);
    SPIx_Writes(MPUREG_ZA_OFFSET_H, (uint8_t*) &acc_bias_data_save[4], 2);

    for (j = 0; j < 200; j++)
    {
        MPU_ReadRegs(MPUREG_ACCEL_XOUT_H, response, 6);
        for (i = x; i <= z; i++)
        {
            acc_offset_tmp[i] = ((int16_t) response[i * 2] << 8) | response[i * 2 + 1];
            acc_offset[i] += (int32_t) acc_offset_tmp[i];
        }

        HAL_Delay(5);
    }

    divider = SPI_read_register(MPUREG_ACCEL_CONFIG);
    divider &= 0x18;

    switch (divider)
    {
    case BITS_FS_2G:
        divider = 8;
        break;
    case BITS_FS_4G:
        divider = 4;
        break;
    case BITS_FS_8G:
        divider = 2;
        break;
    case BITS_FS_16G:
        divider = 1;
        break;
    }

    // offset register referred to 16G (against example from Invensense which says 8G)
    // from Invensense Motion Driver, avoid round errors
    acc_offset[x] = (int32_t) ( ( (int64_t) acc_offset[x] ) / divider / 200 );
    acc_offset[y] = (int32_t) ( ( (int64_t) acc_offset[y] ) / divider / 200 );
    acc_offset[z] = (int32_t) ( ( (int64_t) acc_offset[z] ) / divider / 200 );

    /*
     * this works too
     acc_offset[x] /= (int32_t) (200 * divider);
     acc_offset[y] /= (int32_t) (200 * divider);
     acc_offset[z] /= (int32_t) (200 * divider);
     */

    // One axis must be very near to vertical, subtract value for 1G from it
    for (i = 0; i <= 2; i++)
    {
        if (fabsf(acc_offset[i]) > 1000)
        {
            if (acc_offset[i] > 1000)
            {
                acc_offset[i] -= 2048;
            }
            else if (acc_offset[i] < 1000)
            {
                acc_offset[i] += 2048;
            }
        }
    }
}


void MPU_Init(void)
    {

	//������������� ������ �� ��������, �������, � ��.
	    // reset
		SPI_write_register(107, 0b10000000);
		HAL_Delay(200);
		SPI_write_register(104, 0b00000111);
		HAL_Delay(200);

	    while (SPI_read_register(117) != 0x68) {}   //MPU6000

	    // wake up w/ Z axis clock reg
	    mpu6000ReliablySetReg(107, 0b00000011);

	    // Enable Acc & Gyro
	    mpu6000ReliablySetReg(108, 0b00000000);

	    // Sample rate divider 8KHz
	    mpu6000ReliablySetReg(25, 0b00000000);

	    // DLPF set 0 Accel 260Hz 0 �������� Gyro 256Hz 0.98ms ��������
	    //mpu6000ReliablySetReg(26, 0b00000000); //��� DLPF
	    mpu6000ReliablySetReg(26, 0x02); //DLPF_CFG_98HZ

	    // +-2000dps, Bit 0-1 unset (FCHOICE_B) enables LPF (FCHOICE set
	    mpu6000ReliablySetReg(27, 0b10000);
	    // +-8G
	    mpu6000ReliablySetReg(28, 0b10000);

	    // disable I2C interface
	    mpu6000ReliablySetReg(106,0b10000);

	    // Interrupt setup
	    mpu6000ReliablySetReg(55, 0x01<<4);
	    mpu6000ReliablySetReg(56, 0x01);

		if(Err_MPU)
		    {
			supervisorState |= ERR_SENSOR;
		//	buzzer_flag = BUZZER_FLAG_FAIL_SENSOR;
		    }
  //Bump spi clock to 25��� (100/4) - 12,5��� (100/8) - 16��� �� 512
		hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_4;
		  if (HAL_SPI_Init(&hspi1) != HAL_OK)
		  {
		    _Error_Handler(__FILE__, __LINE__);
		  }

    }
/* SPI1 init function */
void prvIMU_INS(void *pvParameters)
    {
	  static volatile uint32_t currentTime;
	while(1)
	    {

		xSemaphoreTake(xMPU_Semaphore, portMAX_DELAY);

        currentTime       = TIM5->CNT;
        deltaTimeINS    = currentTime - previousINSTime;
        previousINSTime = currentTime;
        dtINS = (float)deltaTimeINS * 0.0000001f;

		ax = (int16_t)(((uint16_t)MPU_buff[0]<<8)+MPU_buff[1]);
		ay = (int16_t)(((uint16_t)MPU_buff[2]<<8)+MPU_buff[3]);
		az = (int16_t)(((uint16_t)MPU_buff[4]<<8)+MPU_buff[5]);

		gx = (int16_t)(((uint16_t)MPU_buff[8]<<8)+MPU_buff[9]);
		gy = (int16_t)(((uint16_t)MPU_buff[10]<<8)+MPU_buff[11]);
		gz = (int16_t)(((uint16_t)MPU_buff[12]<<8)+MPU_buff[13]);

		temp_gyro = (int16_t)(((uint16_t)MPU_buff[6]<<8)+MPU_buff[7]);

		xSemaphoreTake(xMPU_UKF_Mutex, portMAX_DELAY);
		gyf = (float)gx / gyro_divider; //��� ��������� � ������ ��������� ������  x � y
		gxf = (float)gy / gyro_divider;
		gzf = -(float)gz / gyro_divider; // Yaw ������������� ��� ���������� � ������� ��������� NED

	    gyf *= 0.0174533f;  // to radian
	    gxf *= 0.0174533f;
	    gzf *= 0.0174533f;

		ayf = ((float)ax / acc_divider);
		axf = ((float)ay / acc_divider);
		azf = (-(float)az / acc_divider); // Yaw ������������� ��� ���������� � ������� ��������� NED

		ayf *= GRAVITY;
		axf *= GRAVITY;
		azf *= GRAVITY; // Yaw ������������� ��� ���������� � ������� ��������� NED
		xSemaphoreGive(xMPU_UKF_Mutex);

		Control();

        executionTimeINS = TIM5->CNT - currentTime;
	    }
}

void MPU6500_Init_Full(void)
    {

	//�������� ������ ��������� ������ � ������������� � ���������
	cliPortPrintF("FreeHeapSize =  %6d, before IMU_INS \r\n", xPortGetFreeHeapSize());
	xTaskCreate(prvIMU_INS,(char*)"IMU_INS", 600, NULL, 9, ins_handle);
	//�������� ������� ��� �������� ������ �� ���������� ������ ��������� ���� ������
	vSemaphoreCreateBinary(xMPU_Semaphore);
	vSemaphoreCreateBinary(xMPUDataReady_Semaphore);  //������������ ���������� ������ - ����������
	cliPortPrintF("FreeHeapSize =  %6d, before Read_MPU \r\n", xPortGetFreeHeapSize());
	xTaskCreate(readMPU,(char*)"Read_MPU", 600, NULL, 10 , readmpu_handle);

	////
	xTaskCreate(prvIMU_INS_UKF,(char*)"UKF", 400, NULL, 8, ukf_handle);
	//�������� ������� ��� �������� ������ �� ���������� ������ ��������� ���� ������
	vSemaphoreCreateBinary(xUKF_Semaphore);


	navUkfInit();
	altUkfInit();

	MPU_Init();
    MPU_get_acc_divider();
    MPU_get_gyro_divider();

    Acc_Collect_Error_cal(&acc_offset[0],250);

    // save values of acc cancellation registers
    MPU_ReadRegs(MPUREG_XA_OFFSET_H, (uint8_t*) &acc_bias_data_save[0], 2);
    MPU_ReadRegs(MPUREG_YA_OFFSET_H, (uint8_t*) &acc_bias_data_save[2], 2);
    MPU_ReadRegs(MPUREG_ZA_OFFSET_H, (uint8_t*) &acc_bias_data_save[4], 2);
    HAL_Delay(4000); // wait for silence after batteries plugged in
    MPU_GyroCalibration();
    MPU_AccCalibration(&acc_offset[0]);

	led_counter = 0;
	EXTI_Init_MPU6500();
/*
	gxf =1.0f;
	gyf =1.0f;
	gzf =1.0f;
	axf =1.0f;
	ayf =1.0f;
	azf =1.0f;
*/
    }

