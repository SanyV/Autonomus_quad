#ifndef _ADC_H_
#define _ADC_H_


//***************************************************************************************************//

//***************************************************************************************************//

//—четчики цикла
//extern uint32_t cnt_cycle_adc, cnt_ticks_adc;
//****************************************************//


void Init_SYS_ADC(void);
void prvOperation_with_ADC_results(void *pvParameters);
void prvADC_Request(void *pvParameters);


#endif
