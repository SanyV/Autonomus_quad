################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Src/freertos.c \
../Src/main.c \
../Src/stm32h7xx_hal_msp.c \
../Src/stm32h7xx_hal_timebase_TIM.c \
../Src/stm32h7xx_it.c \
../Src/system_stm32h7xx.c 

OBJS += \
./Src/freertos.o \
./Src/main.o \
./Src/stm32h7xx_hal_msp.o \
./Src/stm32h7xx_hal_timebase_TIM.o \
./Src/stm32h7xx_it.o \
./Src/system_stm32h7xx.o 

C_DEPS += \
./Src/freertos.d \
./Src/main.d \
./Src/stm32h7xx_hal_msp.d \
./Src/stm32h7xx_hal_timebase_TIM.d \
./Src/stm32h7xx_it.d \
./Src/system_stm32h7xx.d 


# Each subdirectory must supply rules for building sources it contributes
Src/%.o: ../Src/%.c
	@echo 'Building file: $<'
	@echo 'Invoking: MCU GCC Compiler'
	@echo $(PWD)
	arm-none-eabi-gcc -mcpu=cortex-m7 -mthumb -mfloat-abi=hard -mfpu=fpv5-d16 '-D__weak=__attribute__((weak))' '-D__packed="__attribute__((__packed__))"' -DUSE_HAL_DRIVER -DSTM32H743xx -DARM_MATH_CM7 -I"/Users/sanyv/code/Projects/code/AC6/H7-newlib/Inc" -I"/Users/sanyv/code/Projects/code/AC6/H7-newlib/Drivers/STM32H7xx_HAL_Driver/Inc" -I"/Users/sanyv/code/Projects/code/AC6/H7-newlib/Drivers/System" -I"/Users/sanyv/code/Projects/code/AC6/H7-newlib/Drivers/Sensor" -I"/Users/sanyv/code/Projects/code/AC6/H7-newlib/Drivers/STM32H7xx_HAL_Driver/Inc/Legacy" -I"/Users/sanyv/code/Projects/code/AC6/H7-newlib/Drivers/CMSIS/Device/ST/STM32H7xx/Include" -I"/Users/sanyv/code/Projects/code/AC6/H7-newlib/Drivers/CMSIS/Include" -I"/Users/sanyv/code/Projects/code/AC6/H7-newlib/Middlewares/Third_Party/FreeRTOS/Source/portable/GCC/ARM_CM7/r0p1" -I"/Users/sanyv/code/Projects/code/AC6/H7-newlib/Middlewares/Third_Party/FreeRTOS/Source/include" -I"/Users/sanyv/code/Projects/code/AC6/H7-newlib/Middlewares/Third_Party/FreeRTOS/Source/CMSIS_RTOS"  -O2 -Wall -fmessage-length=0 -c -fmessage-length=0 -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '


