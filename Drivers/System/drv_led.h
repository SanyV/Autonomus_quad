#define Led1_Pin GPIO_PIN_0
#define Led1_Port GPIOB
#define Led2_Pin GPIO_PIN_7
#define Led2_Port GPIOB
#define Led3_Pin GPIO_PIN_14
#define Led3_Port GPIOB
#define Led4_Pin GPIO_PIN_7
#define Led4_Port GPIOG

#define Led1_On       HAL_GPIO_WritePin(Led1_Port, Led1_Pin, GPIO_PIN_SET)
#define Led1_Off      HAL_GPIO_WritePin(Led1_Port, Led1_Pin, GPIO_PIN_RESET)
#define Led1_Toggle   HAL_GPIO_TogglePin(Led1_Port, Led1_Pin)

#define Led2_On       HAL_GPIO_WritePin(Led2_Port, Led2_Pin, GPIO_PIN_SET)
#define Led2_Off      HAL_GPIO_WritePin(Led2_Port, Led2_Pin, GPIO_PIN_RESET)
#define Led2_Toggle   HAL_GPIO_TogglePin(Led2_Port, Led2_Pin)

#define Led3_On       HAL_GPIO_WritePin(Led3_Port, Led3_Pin, GPIO_PIN_SET)
#define Led3_Off      HAL_GPIO_WritePin(Led3_Port, Led3_Pin, GPIO_PIN_RESET)
#define Led3_Toggle   HAL_GPIO_TogglePin(Led3_Port, Led3_Pin)

#define Led4_Off      HAL_GPIO_WritePin(Led4_Port, Led4_Pin, GPIO_PIN_SET)
#define Led4_On       HAL_GPIO_WritePin(Led4_Port, Led4_Pin, GPIO_PIN_RESET)
#define Led4_Toggle   HAL_GPIO_TogglePin(Led4_Port, Led4_Pin)

void Led_init(void);
