/*
 Sany V. 2017
*/

///////////////////////////////////////////////////////////////////////////////
#include "drv_uart1.h"
#include "Config.h"
#include "stdbool.h"
#include "stdarg.h"

extern uint16_t (*cliPortNumCharsAvailable)(void);
extern uint8_t (*cliPortAvailable)(void);
extern void     (*cliPortClearBuffer)(void);
extern uint8_t  (*cliPortRead)(void);
extern void     (*cliPortPrint)(char *str);
extern void     (*cliPortPrintCh)(uint8_t ch);
extern void     (*cliPortPrintF)(const char * fmt, ...);
extern void     (*cliPortPrintBinary)(uint8_t *buf, uint16_t length);


extern UART_HandleTypeDef huart1;
extern DMA_HandleTypeDef hdma_usart1_rx;
extern DMA_HandleTypeDef hdma_usart1_tx;

enum Err_Serial Err_Serial1;

SemaphoreHandle_t TX1_Buf_flag = NULL;
SemaphoreHandle_t TX1_Mutex;

// Receive buffer
uint8_t  RX1Buffer[USART1_BUFFER_SIZE] __attribute__((aligned (32)));
uint32_t rx1DMAPos = 0;

bool TX1BufferFull;
uint8_t  TX1Buffer[USART1_BUFFER_SIZE] __attribute__((aligned (32)));
uint16_t TX1BufferTail = 0;
uint16_t TX1BufferHead = 0;

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
  /* Turn LED2 on: Transfer in reception process is correct */
    __HAL_UART_FLUSH_DRREGISTER(huart);
    __HAL_UART_CLEAR_OREFLAG(huart);
}

void HAL_UART_RxHalfCpltCallback(UART_HandleTypeDef *huart)
{

    __HAL_UART_FLUSH_DRREGISTER(huart);
    __HAL_UART_CLEAR_OREFLAG(huart);
  /* NOTE: This function should not be modified, when the callback is needed,
           the HAL_UART_RxHalfCpltCallback can be implemented in the user file
   */
}

///////////////////////////////////////////////////////////////////////////////
// usart1 DMA
///////////////////////////////////////////////////////////////////////////////
/* USART1 init function */
void serial1_Init(void)
{
//  static char Message[14] = "Test message\r\n";
  Err_Serial1 = S_OK;
  TX1_Mutex = xSemaphoreCreateMutex();
  vSemaphoreCreateBinary(TX1_Buf_flag);

  //osEventFlagsSet(TX1_UR_flag, FLAGS_MSK2);
	
 // HAL_UART_Transmit_IT(&huart1, (uint8_t *)Message, 14);

  cliPortAvailable       = &serial1Available;
  cliPortPrint           = &serial1Print;
  cliPortPrintF          = &serial1PrintF;
  cliPortRead            = &serial1Read;
  cliPortPrintCh		 = &serial1Write;
  cliPortPrintBinary	 = &serial1PrintBinary;
  cliPortClearBuffer	  = &serial1ClearBuffer;
  cliPortNumCharsAvailable= &serial1NumCharsAvailable;


	HAL_UART_Receive_DMA(&huart1, (uint8_t *)RX1Buffer, (uint16_t) USART1_BUFFER_SIZE);

	serial1ClearBuffer();
	TX1BufferFull = false;
}

///////////////////////////////////////////////////////////////////////////////
// usart1 Available
///////////////////////////////////////////////////////////////////////////////
uint8_t serial1Available(void)
{
    return (hdma_usart1_rx.Instance->NDTR != rx1DMAPos) ? true : false;
}

///////////////////////////////////////////////////////////////////////////////
// usart1 Clear Buffer
///////////////////////////////////////////////////////////////////////////////
void serial1ClearBuffer(void)
{
    rx1DMAPos = hdma_usart1_rx.Instance->NDTR;
}

///////////////////////////////////////////////////////////////////////////////
// usart1 Number of Characters Available
///////////////////////////////////////////////////////////////////////////////
uint16_t serial1NumCharsAvailable(void)
{
  int32_t number;

  number = rx1DMAPos - hdma_usart1_rx.Instance->NDTR;

  if (number >= 0)
      return (uint16_t)number;
  else
      return (uint16_t)(USART1_BUFFER_SIZE + number);
}

///////////////////////////////////////////////////////////////////////////////
// usart1 Read
///////////////////////////////////////////////////////////////////////////////
uint8_t serial1Read(void)
{
    uint8_t ch;
	dmaBufferInvalidate(&RX1Buffer, USART1_BUFFER_SIZE); //Р В Р Р‹Р В РЎвЂ�Р В Р вЂ¦Р РЋРІР‚В¦Р РЋР вЂљР В РЎвЂўР В Р вЂ¦Р В РЎвЂ�Р В Р’В·Р В Р’В°Р РЋРІР‚В Р В РЎвЂ�Р РЋР РЏ Р В РЎвЂќР В Р’ВµР РЋРІвЂљВ¬Р В Р’В° Р В РІР‚СњР В РЎС™Р В РЎвЂ™ Р В Р’В±Р РЋРЎвЂњР РЋРІР‚С›Р В Р’ВµР РЋР вЂљР В Р’В°
    ch = RX1Buffer[USART1_BUFFER_SIZE - rx1DMAPos];
    // go back around the buffer
    if (--rx1DMAPos == 0)
      rx1DMAPos = USART1_BUFFER_SIZE;

    return ch;
}

///////////////////////////////////////////////////////////////////////////////
// usart1 Read Poll
///////////////////////////////////////////////////////////////////////////////
uint8_t serial1ReadPoll(void)
{
    while (!serial1Available()); // wait for some bytes
    return serial1Read();
}

///////////////////////////////////////////////////////////////////////////////
// usart1 Write
///////////////////////////////////////////////////////////////////////////////
void serial1Write(uint8_t ch)
{
	if (!TX1BufferFull)
{
  xSemaphoreTake(TX1_Mutex, portMAX_DELAY);
  TX1Buffer[TX1BufferHead] = ch;
  TX1BufferHead  =  (TX1BufferHead + 1) % USART1_BUFFER_SIZE;
	if (TX1BufferHead == TX1BufferTail)
	{
		TX1BufferFull = true;
		//Err_Serial1 = Err_Overfull;
		xSemaphoreGive(TX1_Mutex);
		return;
	}
  xSemaphoreGive(TX1_Mutex);
  xSemaphoreGive(TX1_Buf_flag);
}
}

///////////////////////////////////////////////////////////////////////////////
// usart1 Print
///////////////////////////////////////////////////////////////////////////////

void serial1Print(char *str)
{
  if (!TX1BufferFull)
	{
	  xSemaphoreTake(TX1_Mutex, portMAX_DELAY);
	while (*str)
    {
		TX1Buffer[TX1BufferHead] = *str++;
    	TX1BufferHead  =  (TX1BufferHead + 1) % USART1_BUFFER_SIZE;
			if (TX1BufferHead == TX1BufferTail)
			{
				TX1BufferFull = true;
				//Err_Serial1 = Err_Overfull;
				xSemaphoreGive(TX1_Mutex);
				return;
			}
    }
	  xSemaphoreGive(TX1_Mutex);
	  xSemaphoreGive(TX1_Buf_flag);
	}
}

///////////////////////////////////////////////////////////////////////////////
// usart1 Print Formatted - Print formatted string to usart1
// From Ala42
///////////////////////////////////////////////////////////////////////////////

void serial1PrintF(const char * fmt, ...)
{
	char buf[256];
	uint16_t len;

	va_list  vlist;
	va_start (vlist, fmt);

	len = vsnprintf(buf, sizeof(buf), fmt, vlist);
	serial1PrintBinary((uint8_t *)buf, len);
	va_end(vlist);
}

///////////////////////////////////////////////////////////////////////////////
// usart1 Print Binary String
///////////////////////////////////////////////////////////////////////////////

void serial1PrintBinary(uint8_t *buf, uint16_t length)
{
    static uint16_t i;
    if (!TX1BufferFull)
  	{
	xSemaphoreTake(TX1_Mutex, portMAX_DELAY);
    for (i = 0; i < length; i++)
    {
    	TX1Buffer[TX1BufferHead] = buf[i];
    	TX1BufferHead = (TX1BufferHead + 1) % USART1_BUFFER_SIZE;
			if (TX1BufferHead == TX1BufferTail)
			{
				TX1BufferFull = true;
				Err_Serial1 = Err_Overfull;
	//			xSemaphoreGive(TX1_Mutex);
				return;
			}
    }
	  xSemaphoreGive(TX1_Mutex);
	  xSemaphoreGive(TX1_Buf_flag);
  	}
}


///////////////////////////////////////////////////////////////////////////////
