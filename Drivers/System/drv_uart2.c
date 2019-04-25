/*
 Sany V. 2017
*/

///////////////////////////////////////////////////////////////////////////////
#include "drv_uart2.h"
#include "Config.h"
#include "stdbool.h"
#include "stdarg.h"

extern uint8_t  (*gpsPortAvailable)(void);
extern void     (*gpsPortClearBuffer)(void);
extern uint8_t  (*gpsPortRead)(void);
extern void     (*gpsPortPrint)(char *str);
extern void     (*gpsPortPrintCh)(uint8_t ch);
extern void     (*gpsPortPrintF)(const char * fmt, ...);
extern void     (*gpsPortPrintBinary)(uint8_t *buf, uint16_t length);
extern uint16_t (*gpsPortNumCharsAvailable)(void);


extern UART_HandleTypeDef huart2;
extern DMA_HandleTypeDef hdma_usart2_rx;
extern DMA_HandleTypeDef hdma_usart2_tx;

enum Err_Serial Err_serial2;

SemaphoreHandle_t TX2_Buf_flag = NULL;
SemaphoreHandle_t TX2_Mutex;

// Receive buffer
uint8_t  RX2Buffer[USART2_BUFFER_SIZE] __attribute__((aligned (32)));
uint32_t rx2DMAPos = 0;

bool TX2BufferFull;
uint8_t  TX2Buffer[USART2_BUFFER_SIZE] __attribute__((aligned (32)));
uint16_t TX2BufferTail = 0;
uint16_t TX2BufferHead = 0;

///////////////////////////////////////////////////////////////////////////////
// usart1 DMA
///////////////////////////////////////////////////////////////////////////////
/* USART1 init function */
void serial2_Init(void)
{
 // static char Message[14] = "Test message\r\n";
  Err_serial2 = S_OK;
  TX2_Mutex = xSemaphoreCreateMutex();
  vSemaphoreCreateBinary(TX2_Buf_flag);

  //osEventFlagsSet(TX1_UR_flag, FLAGS_MSK2);
	
 // HAL_UART_Transmit_IT(&huart2, (uint8_t *)Message, 14);

  gpsPortAvailable        = &serial2Available;
  gpsPortPrint            = &serial2Print;
  gpsPortPrintF           = &serial2PrintF;
  gpsPortRead             = &serial2Read;
  gpsPortPrintCh		  = &serial2Write;
  gpsPortPrintBinary	  = &serial2PrintBinary;
  gpsPortClearBuffer	  = &serial2ClearBuffer;
  gpsPortNumCharsAvailable= &serial2NumCharsAvailable;

  /*
  cliPortAvailable       = &serial2Available;
  cliPortPrint           = &serial2Print;
  cliPortPrintF          = &serial2PrintF;
  cliPortRead            = &serial2Read;
  cliPortPrintCh		 = &serial2Write;
  cliPortPrintBinary	 = &serial2PrintBinary;
*/
 // HAL_UART_Transmit(&huart2, (uint8_t *)Message, 14, 0x1000);

  HAL_UART_Receive_DMA(&huart2, (uint8_t *)RX2Buffer, (uint16_t) USART2_BUFFER_SIZE);

  serial2ClearBuffer();
  TX2BufferFull = false;
}

///////////////////////////////////////////////////////////////////////////////
// usart1 Available
///////////////////////////////////////////////////////////////////////////////
uint8_t serial2Available(void)
{
    return (hdma_usart2_rx.Instance->NDTR != rx2DMAPos) ? true : false;
}

///////////////////////////////////////////////////////////////////////////////
// usart1 Clear Buffer
///////////////////////////////////////////////////////////////////////////////
void serial2ClearBuffer(void)
{
    rx2DMAPos = hdma_usart2_rx.Instance->NDTR;
}

///////////////////////////////////////////////////////////////////////////////
// usart1 Number of Characters Available
///////////////////////////////////////////////////////////////////////////////
uint16_t serial2NumCharsAvailable(void)
{
  int32_t number;

  number = rx2DMAPos - hdma_usart2_rx.Instance->NDTR;

  if (number >= 0)
      return (uint16_t)number;
  else
      return (uint16_t)(USART2_BUFFER_SIZE + number);
}

///////////////////////////////////////////////////////////////////////////////
// usart1 Read
///////////////////////////////////////////////////////////////////////////////
uint8_t serial2Read(void)
{
    uint8_t ch;
	dmaBufferInvalidate(&RX2Buffer, USART2_BUFFER_SIZE); //Р В Р Р‹Р В РЎвЂ�Р В Р вЂ¦Р РЋРІР‚В¦Р РЋР вЂљР В РЎвЂўР В Р вЂ¦Р В РЎвЂ�Р В Р’В·Р В Р’В°Р РЋРІР‚В Р В РЎвЂ�Р РЋР РЏ Р В РЎвЂќР В Р’ВµР РЋРІвЂљВ¬Р В Р’В° Р В РІР‚СњР В РЎС™Р В РЎвЂ™ Р В Р’В±Р РЋРЎвЂњР РЋРІР‚С›Р В Р’ВµР РЋР вЂљР В Р’В°
    ch = RX2Buffer[USART2_BUFFER_SIZE - rx2DMAPos];
    // go back around the buffer
    if (--rx2DMAPos == 0)
     rx2DMAPos = USART2_BUFFER_SIZE;

    return ch;
}

///////////////////////////////////////////////////////////////////////////////
// usart1 Read Poll
///////////////////////////////////////////////////////////////////////////////
uint8_t serial2ReadPoll(void)
{
    while (!serial2Available()); // wait for some bytes
    return serial2Read();
}

///////////////////////////////////////////////////////////////////////////////
// usart1 Write
///////////////////////////////////////////////////////////////////////////////
void serial2Write(uint8_t ch)
{
	if (!TX2BufferFull)
{
  xSemaphoreTake(TX2_Mutex, portMAX_DELAY);
  TX2Buffer[TX2BufferHead] = ch;
  TX2BufferHead  =  (TX2BufferHead + 1) % USART2_BUFFER_SIZE;
	if (TX2BufferHead == TX2BufferTail)
	{
		TX2BufferFull = true;
		//Err_serial2 = Err_Overfull;
		xSemaphoreGive(TX2_Mutex);
		return;
	}
  xSemaphoreGive(TX2_Mutex);
  xSemaphoreGive(TX2_Buf_flag);
}
}

///////////////////////////////////////////////////////////////////////////////
// usart1 Print
///////////////////////////////////////////////////////////////////////////////

void serial2Print(char *str)
{
  if (!TX2BufferFull)
	{
	  xSemaphoreTake(TX2_Mutex, portMAX_DELAY);
	while (*str)
    {
		TX2Buffer[TX2BufferHead] = *str++;
    	TX2BufferHead  =  (TX2BufferHead + 1) % USART2_BUFFER_SIZE;
			if (TX2BufferHead == TX2BufferTail)
			{
				TX2BufferFull = true;
				//Err_serial2 = Err_Overfull;
				xSemaphoreGive(TX2_Mutex);
				return;
			}
    }
	  xSemaphoreGive(TX2_Mutex);
	  xSemaphoreGive(TX2_Buf_flag);
	}
}

///////////////////////////////////////////////////////////////////////////////
// usart1 Print Formatted - Print formatted string to usart1
// From Ala42
///////////////////////////////////////////////////////////////////////////////

void serial2PrintF(const char * fmt, ...)
{
	char buf[256];
	uint16_t len;

	va_list  vlist;
	va_start (vlist, fmt);

	len = vsnprintf(buf, sizeof(buf), fmt, vlist);
	serial2PrintBinary((uint8_t *)buf, len);
	va_end(vlist);
}

///////////////////////////////////////////////////////////////////////////////
// usart1 Print Binary String
///////////////////////////////////////////////////////////////////////////////

void serial2PrintBinary(uint8_t *buf, uint16_t length)
{
    static uint16_t i;
    if (!TX2BufferFull)
  	{
	xSemaphoreTake(TX2_Mutex, portMAX_DELAY);
    for (i = 0; i < length; i++)
    {
    	TX2Buffer[TX2BufferHead] = buf[i];
    	TX2BufferHead = (TX2BufferHead + 1) % USART2_BUFFER_SIZE;
			if (TX2BufferHead == TX2BufferTail)
			{
				TX2BufferFull = true;
				Err_serial2 = Err_Overfull;
	//			xSemaphoreGive(TX2_Mutex);
				return;
			}
    }
	  xSemaphoreGive(TX2_Mutex);
	  xSemaphoreGive(TX2_Buf_flag);
  	}
}


///////////////////////////////////////////////////////////////////////////////
