
#include "Config.h"
#include "stdbool.h"
#include "drv_uart1.h"
#include "drv_uart2.h"

//семафор для связи с модулем орбаботки
extern SemaphoreHandle_t TX1_Buf_flag;
extern SemaphoreHandle_t TX1_Mutex;
extern UART_HandleTypeDef huart1;

extern SemaphoreHandle_t TX2_Buf_flag;
extern SemaphoreHandle_t TX2_Mutex;

extern UART_HandleTypeDef huart2;
//ошибка датчика
extern enum Err_Sensor Err_Serial1;
extern enum Err_Sensor Err_Serial2;

extern bool TX1BufferFull;
extern uint8_t  TX1Buffer[USART1_BUFFER_SIZE] __attribute__((aligned (32)));
extern uint16_t TX1BufferTail;
extern uint16_t TX1BufferHead;

extern bool TX2BufferFull;
extern uint8_t  TX2Buffer[USART2_BUFFER_SIZE] __attribute__((aligned (32)));
extern uint16_t TX2BufferTail;
extern uint16_t TX2BufferHead;

TaskHandle_t comm1_handle;
TaskHandle_t comm2_handle;

uint32_t deltaTimeCOMM1, executionTimeCOMM1, previousCOMM1Time;
uint32_t deltaTimeCOMM2, executionTimeCOMM2, previousCOMM2Time;
float dtCOMM1, dtCOMM2;

void comm1Task(void *unused)
{
	static char uartState;
	static uint32_t currentTime;
	while (1)
	{
		if( xSemaphoreTake(TX1_Buf_flag, portMAX_DELAY ) == pdTRUE )
		  {
	            currentTime       = TIM5->CNT;
	            deltaTimeCOMM1    = currentTime - previousCOMM1Time;
	            previousCOMM1Time = currentTime;
	            dtCOMM1 = (float)deltaTimeCOMM1 * 0.0000001f;

		xSemaphoreTake(TX1_Mutex, portMAX_DELAY);
		dmaBufferFlush (&TX1Buffer[0], USART1_BUFFER_SIZE);
		if (TX1BufferTail > TX1BufferHead)
		{
//			xSemaphoreTake(TX1_Int_flag, portMAX_DELAY );
		uartState = HAL_UART_GetState(&huart1);
		  while ((uartState!= HAL_UART_STATE_READY) && (uartState != HAL_UART_STATE_BUSY_RX))
		  {
			  uartState = HAL_UART_GetState(&huart1);
		  }
		  if (HAL_UART_Transmit_DMA(&huart1, &TX1Buffer[TX1BufferTail] , USART1_BUFFER_SIZE - TX1BufferTail) != HAL_OK )
		       {
		    	   Err_Serial1 = Err_Send;
		       }
			uartState = HAL_UART_GetState(&huart1);
			  while ((uartState!= HAL_UART_STATE_READY) && (uartState != HAL_UART_STATE_BUSY_RX))
			  {
				  uartState = HAL_UART_GetState(&huart1);
			  }
		   		if (HAL_UART_Transmit_DMA(&huart1, &TX1Buffer[0], TX1BufferHead) != HAL_OK )
		       {
			    	   Err_Serial1 = Err_Send;
		       }
		   	}
	//	   	vTaskDelay(1000);

		else
		{
 //			   osMutexRelease(TX1_UR_flag);
	uartState = HAL_UART_GetState(&huart1);
  while ((uartState!= HAL_UART_STATE_READY) && (uartState != HAL_UART_STATE_BUSY_RX))   //?
  {
uartState = HAL_UART_GetState(&huart1);
	}

  if (HAL_UART_Transmit_DMA(&huart1, &TX1Buffer[TX1BufferTail], TX1BufferHead - TX1BufferTail) != HAL_OK )
	       {
		    	   Err_Serial1 = Err_Send;
	       }
		}
		TX1BufferTail = TX1BufferHead;
		TX1BufferFull = false;
	  xSemaphoreGive(TX1_Mutex);

	}
        // Calculate execution tick process time
        executionTimeCOMM1 = TIM5->CNT - currentTime;
	}
}

void comm2Task(void *unused)
{
	static char uartState;
	static uint32_t currentTime;
	while (1)
	{
		  if( xSemaphoreTake(TX2_Buf_flag, portMAX_DELAY ) == pdTRUE )
		  {
	            currentTime       = TIM5->CNT;
	            deltaTimeCOMM2    = currentTime - previousCOMM2Time;
	            previousCOMM2Time = currentTime;
	            dtCOMM2 = (float)deltaTimeCOMM2 * 0.0000001f;

		xSemaphoreTake(TX2_Mutex, portMAX_DELAY);
		dmaBufferFlush (&TX2Buffer[0], USART2_BUFFER_SIZE);
		if (TX2BufferTail > TX2BufferHead)
		{
		uartState = HAL_UART_GetState(&huart2);
		  while ((uartState!= HAL_UART_STATE_READY) && (uartState != HAL_UART_STATE_BUSY_RX))
		  {
			  uartState = HAL_UART_GetState(&huart2);
		  }
		  if (HAL_UART_Transmit_DMA(&huart2, &TX2Buffer[TX2BufferTail] , USART2_BUFFER_SIZE - TX2BufferTail) != HAL_OK )
		       {
		    	   Err_Serial1 = Err_Send;
		       }
			uartState = HAL_UART_GetState(&huart2);
			  while ((uartState!= HAL_UART_STATE_READY) && (uartState != HAL_UART_STATE_BUSY_RX))
			  {
				  uartState = HAL_UART_GetState(&huart2);
			  }
		   		if (HAL_UART_Transmit_DMA(&huart2, &TX2Buffer[0], TX2BufferHead) != HAL_OK )
		       {
			    	   Err_Serial1 = Err_Send;
		       }
		   	}
	//	   	vTaskDelay(1000);

		else
		{

	uartState = HAL_UART_GetState(&huart2);
  while ((uartState!= HAL_UART_STATE_READY) && (uartState != HAL_UART_STATE_BUSY_RX))   //?
  {
uartState = HAL_UART_GetState(&huart2);
	}

  if (HAL_UART_Transmit_DMA(&huart2, &TX2Buffer[TX2BufferTail], TX2BufferHead - TX2BufferTail) != HAL_OK )
	       {
	  	  	  	  Err_Serial1 = Err_Send;
	       }
		}
		TX2BufferTail = TX2BufferHead;
		TX2BufferFull = false;
	  xSemaphoreGive(TX2_Mutex);

	}
	        // Calculate execution tick process time
	        executionTimeCOMM2 = TIM5->CNT - currentTime;
	}
}

int CommInit (void) {
 
	serial1_Init();
	serial2_Init();
	//создание задачи сериал портов
	cliPortPrintF("FreeHeapSize =  %6d, before Console \r\n", xPortGetFreeHeapSize());
	xTaskCreate(comm1Task,(char*)"COMM1", 600, NULL, 3, comm1_handle);
	cliPortPrintF("FreeHeapSize =  %6d, before GPS_serial \r\n", xPortGetFreeHeapSize());
	xTaskCreate(comm2Task,(char*)"COMM2", 600, NULL, 3, comm2_handle);
  return(0);
}
