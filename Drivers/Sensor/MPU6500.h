/* библиотека для получения данных с MPU6500 - акселерометр, гироскоп */

#ifndef _MPU6500_H_
#define _MPU6500_H_
// Configuration bits mpu9250
#define BIT_SLEEP                   0x40
#define BIT_H_RESET                 0x80
#define BITS_CLKSEL                 0x07
#define MPU_CLK_SEL_PLLGYROX        0x01
#define MPU_CLK_SEL_PLLGYROZ        0x03
#define MPU_EXT_SYNC_GYROX          0x02
#define BITS_FS_250DPS              0x00
#define BITS_FS_500DPS              0x08
#define BITS_FS_1000DPS             0x10
#define BITS_FS_2000DPS             0x18
#define BITS_FS_2G                  0x00
#define BITS_FS_4G                  0x08
#define BITS_FS_8G                  0x10
#define BITS_FS_16G                 0x18
#define BITS_FS_MASK                0x18

#define BITS_DLPF_CFG_256HZ_NOLPF2  0x00
#define BITS_DLPF_CFG_188HZ         0x01
#define BITS_DLPF_CFG_98HZ          0x02
#define BITS_DLPF_CFG_42HZ          0x03
#define BITS_DLPF_CFG_20HZ          0x04
#define BITS_DLPF_CFG_10HZ          0x05
#define BITS_DLPF_CFG_5HZ           0x06
#define BITS_DLPF_CFG_2100HZ_NOLPF  0x07
#define BITS_DLPF_CFG_MASK          0x07

#define BIT_INT_ANYRD_2CLEAR        0x10
#define BIT_RAW_RDY_EN              0x01

// mpu9250 registers
#define MPU_InitRegNum 19

#define READ_FLAG                   0x80

#define MPUREG_XG_OFFS_TC          0x00
#define MPUREG_YG_OFFS_TC          0x01
#define MPUREG_ZG_OFFS_TC          0x02
#define MPUREG_X_FINE_GAIN         0x03
#define MPUREG_Y_FINE_GAIN         0x04
#define MPUREG_Z_FINE_GAIN         0x05
#define MPUREG_XA_OFFS_H           0x06
#define MPUREG_XA_OFFS_L           0x07
#define MPUREG_YA_OFFS_H           0x08
#define MPUREG_YA_OFFS_L           0x09
#define MPUREG_ZA_OFFS_H           0x0A
#define MPUREG_ZA_OFFS_L           0x0B
#define MPUREG_PRODUCT_ID          0x0C
#define MPUREG_SELF_TEST_X         0x0D
#define MPUREG_SELF_TEST_Y         0x0E
#define MPUREG_SELF_TEST_Z         0x0F
#define MPUREG_SELF_TEST_A         0x10
#define MPUREG_XG_OFFS_USRH        0x13
#define MPUREG_XG_OFFS_USRL        0x14
#define MPUREG_YG_OFFS_USRH        0x15
#define MPUREG_YG_OFFS_USRL        0x16
#define MPUREG_ZG_OFFS_USRH        0x17
#define MPUREG_ZG_OFFS_USRL        0x18
#define MPUREG_SMPLRT_DIV          0x19
#define MPUREG_CONFIG              0x1A
#define MPUREG_GYRO_CONFIG         0x1B
#define MPUREG_ACCEL_CONFIG        0x1C
#define MPUREG_ACCEL_CONFIG_2      0x1D
#define MPUREG_LP_ACCEL_ODR        0x1E
#define MPUREG_MOT_THR             0x1F
#define MPUREG_FIFO_EN             0x23
#define MPUREG_I2C_MST_CTRL        0x24
#define MPUREG_I2C_SLV0_ADDR       0x25
#define MPUREG_I2C_SLV0_REG        0x26
#define MPUREG_I2C_SLV0_CTRL       0x27
#define MPUREG_I2C_SLV1_ADDR       0x28
#define MPUREG_I2C_SLV1_REG        0x29
#define MPUREG_I2C_SLV1_CTRL       0x2A
#define MPUREG_I2C_SLV2_ADDR       0x2B
#define MPUREG_I2C_SLV2_REG        0x2C
#define MPUREG_I2C_SLV2_CTRL       0x2D
#define MPUREG_I2C_SLV3_ADDR       0x2E
#define MPUREG_I2C_SLV3_REG        0x2F
#define MPUREG_I2C_SLV3_CTRL       0x30
#define MPUREG_I2C_SLV4_ADDR       0x31
#define MPUREG_I2C_SLV4_REG        0x32
#define MPUREG_I2C_SLV4_DO         0x33
#define MPUREG_I2C_SLV4_CTRL       0x34
#define MPUREG_I2C_SLV4_DI         0x35
#define MPUREG_I2C_MST_STATUS      0x36
#define MPUREG_INT_PIN_CFG         0x37
#define MPUREG_INT_ENABLE          0x38
#define MPUREG_ACCEL_XOUT_H        0x3B
#define MPUREG_ACCEL_XOUT_L        0x3C
#define MPUREG_ACCEL_YOUT_H        0x3D
#define MPUREG_ACCEL_YOUT_L        0x3E
#define MPUREG_ACCEL_ZOUT_H        0x3F
#define MPUREG_ACCEL_ZOUT_L        0x40
#define MPUREG_TEMP_OUT_H          0x41
#define MPUREG_TEMP_OUT_L          0x42
#define MPUREG_GYRO_XOUT_H         0x43
#define MPUREG_GYRO_XOUT_L         0x44
#define MPUREG_GYRO_YOUT_H         0x45
#define MPUREG_GYRO_YOUT_L         0x46
#define MPUREG_GYRO_ZOUT_H         0x47
#define MPUREG_GYRO_ZOUT_L         0x48
#define MPUREG_EXT_SENS_DATA_00    0x49
#define MPUREG_EXT_SENS_DATA_01    0x4A
#define MPUREG_EXT_SENS_DATA_02    0x4B
#define MPUREG_EXT_SENS_DATA_03    0x4C
#define MPUREG_EXT_SENS_DATA_04    0x4D
#define MPUREG_EXT_SENS_DATA_05    0x4E
#define MPUREG_EXT_SENS_DATA_06    0x4F
#define MPUREG_EXT_SENS_DATA_07    0x50
#define MPUREG_EXT_SENS_DATA_08    0x51
#define MPUREG_EXT_SENS_DATA_09    0x52
#define MPUREG_EXT_SENS_DATA_10    0x53
#define MPUREG_EXT_SENS_DATA_11    0x54
#define MPUREG_EXT_SENS_DATA_12    0x55
#define MPUREG_EXT_SENS_DATA_13    0x56
#define MPUREG_EXT_SENS_DATA_14    0x57
#define MPUREG_EXT_SENS_DATA_15    0x58
#define MPUREG_EXT_SENS_DATA_16    0x59
#define MPUREG_EXT_SENS_DATA_17    0x5A
#define MPUREG_EXT_SENS_DATA_18    0x5B
#define MPUREG_EXT_SENS_DATA_19    0x5C
#define MPUREG_EXT_SENS_DATA_20    0x5D
#define MPUREG_EXT_SENS_DATA_21    0x5E
#define MPUREG_EXT_SENS_DATA_22    0x5F
#define MPUREG_EXT_SENS_DATA_23    0x60
#define MPUREG_I2C_SLV0_DO         0x63
#define MPUREG_I2C_SLV1_DO         0x64
#define MPUREG_I2C_SLV2_DO         0x65
#define MPUREG_I2C_SLV3_DO         0x66
#define MPUREG_I2C_MST_DELAY_CTRL  0x67
#define MPUREG_SIGNAL_PATH_RESET   0x68
#define MPUREG_MOT_DETECT_CTRL     0x69
#define MPUREG_USER_CTRL           0x6A
#define MPUREG_PWR_MGMT_1          0x6B
#define MPUREG_PWR_MGMT_2          0x6C
#define MPUREG_BANK_SEL            0x6D
#define MPUREG_MEM_START_ADDR      0x6E
#define MPUREG_MEM_R_W             0x6F
#define MPUREG_DMP_CFG_1           0x70
#define MPUREG_DMP_CFG_2           0x71
#define MPUREG_FIFO_COUNTH         0x72
#define MPUREG_FIFO_COUNTL         0x73
#define MPUREG_FIFO_R_W            0x74
#define MPUREG_WHOAMI              0x75
#define MPUREG_XA_OFFSET_H         0x77
#define MPUREG_XA_OFFSET_L         0x78
#define MPUREG_YA_OFFSET_H         0x7A
#define MPUREG_YA_OFFSET_L         0x7B
#define MPUREG_ZA_OFFSET_H         0x7D
#define MPUREG_ZA_OFFSET_L         0x7E

#define GRAVITY		9.80665f	// m/s^2
#define DECL 		12.39f
#define INCL 		71.0f
#define TG_DECL 	0.2196813f
#define LPF_ACC_SR	0.98f
#define K_GYRO 		(32767.0f/1000.0f*RAD2DEG)

#define constrainInt(v, lo, hi)	    (((int)(v) < (int)(lo)) ? (int)(lo) : (((int)(v) > (int)(hi)) ? (int)(hi) : (int)(v)))
#define constrainFloat(v, lo, hi)   (((float)(v) < (float)(lo)) ? (float)(lo) : (((float)(v) > (float)(hi)) ? (float)(hi) : (float)(v)))


//***************************************************************************************************//

#define MPU_CS_Pin GPIO_PIN_14
#define MPU_CS_GPIO_Port GPIOD
#define MPU_SPI_SCK_Pin GPIO_PIN_5
#define MPU_SPI_SCK_GPIO_Port GPIOA
#define MPU_SPI_MISO_Pin GPIO_PIN_6
#define MPU_SPI_MISO_GPIO_Port GPIOA
#define MPU_SPI_MOSI_Pin GPIO_PIN_7
#define MPU_SPI_MOSI_GPIO_Port GPIOA


#define	SPI_MPU_Pin_CS					GPIO_PIN_14
#define	SPI_MPU_Port_Group_CS			GPIOD

#define	EXTI_MPU_IRQ	 				EXTI15_10_IRQn
#define EXTI_IRQHandler					EXTI15_10_IRQHandler
#define EXTI_Line_MPU					EXTI_LINE13

#define	EXTI_SPI_MPU_Pin				GPIO_PIN_13
#define	EXTI_SPI_MPU_Port_Group			GPIOD

#define	EXTI_SPI_MPU_Pin_Source_Group	EXTI_PortSourceGPIOD
#define	EXTI_SPI_MPU_Pin_Source			EXTI_PinSource13
//***************************************************************************************************//



#define CS_LOW       HAL_GPIO_WritePin(SPI_MPU_Port_Group_CS, SPI_MPU_Pin_CS, GPIO_PIN_RESET)
#define CS_HIGH      HAL_GPIO_WritePin(SPI_MPU_Port_Group_CS, SPI_MPU_Pin_CS, GPIO_PIN_SET)


#define DLPF_CFG_GYRO_250Hz 		0	//delay = 0.97 ms
#define DLPF_CFG_GYRO_184Hz 		1	//delay = 2.9 ms
#define DLPF_CFG_GYRO_92Hz 		2	//delay = 3.9 ms
#define DLPF_CFG_GYRO_41Hz 		3	//delay = 5.9 ms
#define DLPF_CFG_GYRO_20Hz 		4	//delay = 9.9 ms
#define DLPF_CFG_GYRO_10Hz 		5	//delay = 17.85 ms
#define DLPF_CFG_GYRO_5Hz 		6	//delay = 33.48 ms
#define DLPF_CFG_GYRO_3600Hz 		7	//delay = 0.17 ms

#define F_CHOICE_B_GYRO_FAST 		1	//8800Hz
#define F_CHOICE_B_GYRO_LOW 		2	//3600Hz
#define F_CHOICE_B_GYRO_DLPF 		0	//DLPF

#define GYRO_FS_SEL_250	 		0
#define GYRO_FS_SEL_500	 		8
#define GYRO_FS_SEL_1000	 	16
#define GYRO_FS_SEL_2000	 	24

#define ACCEL_FS_SEL_2	 		0
#define ACCEL_FS_SEL_4	 		8
#define ACCEL_FS_SEL_8		 	16
#define ACCEL_FS_SEL_16		 	24

#define ACCEL_F_CHOICE_B_USE_DLPF 	0	//DLPF
#define ACCEL_F_CHOICE_B_NOT_USE_DLPF 	8	//1130Hz

#define ACCEL_CFG_GYRO_460Hz 		0	//delay = 1.94 ms
#define ACCEL_CFG_GYRO_184Hz 		1	//delay = 5.8 ms
#define ACCEL_CFG_GYRO_92Hz 		2	//delay = 7.8 ms
#define ACCEL_CFG_GYRO_41Hz 		3	//delay = 11.8 ms
#define ACCEL_CFG_GYRO_20Hz 		4	//delay = 19.8 ms
#define ACCEL_CFG_GYRO_10Hz 		5	//delay = 35.7 ms
#define ACCEL_CFG_GYRO_5Hz 		6	//delay = 66.96 ms
//#define ACCEL_CFG_GYRO_460Hz 		7	//delay = 1.94 ms

#define RAW_RDY_EN	 		1

#define SMPLRT_DIV_REG	 		25
#define CONFIG_REG 			26
#define GYRO_CONFIG_REG 		27
#define ACCEL_CONFIG_REG 		28
#define ACCEL_CONFIG_REG2 		29

#define INT_ENABLE_REG	 		56

#define ACCEL_XOUT_H_REG 		59
#define ACCEL_XOUT_L_REG 		60
#define ACCEL_YOUT_H_REG 		61
#define ACCEL_YOUT_L_REG 		62
#define ACCEL_ZOUT_H_REG 		63
#define ACCEL_ZOUT_L_REG 		64

#define TEMP_OUT_H_REG 			65
#define TEMP_OUT_L_REG 			66

#define GYRO_XOUT_H_REG 		67
#define GYRO_XOUT_L_REG 		68
#define GYRO_YOUT_H_REG 		69
#define GYRO_YOUT_L_REG 		70
#define GYRO_ZOUT_H_REG 		71
#define GYRO_ZOUT_L_REG 		72


//Импорт
void prvIMU_INS(void *pvParameters);
extern void prvIMU_INS_UKF(void *pvParameters);
extern SemaphoreHandle_t xUKF_Semaphore;
extern SemaphoreHandle_t  xMPU_UKF_Mutex;
extern TaskHandle_t ukf_handle;
//Экспрот
 /*FreeRTOS*/
extern SemaphoreHandle_t xMPU_Semaphore;
extern TaskHandle_t ins_handle;
/*FreeRTOS*/
//буффер данных с датчика
extern uint8_t MPU_buff[14];

extern enum Err_Sensor Err_MPU;
uint8_t SPIx_SendByte(uint8_t byte);
uint8_t spi_recieve(void);

//функция настройки и инициализации датчика
void MPU6500_Init_Full(void);
void SPI_write_register( uint8_t reg1, uint8_t value1 );
uint8_t SPI_read_register( uint8_t reg1 );


#endif /* _MPU6500_H_ */
