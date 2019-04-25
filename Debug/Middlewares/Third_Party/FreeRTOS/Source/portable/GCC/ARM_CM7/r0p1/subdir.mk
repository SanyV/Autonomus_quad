################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Middlewares/Third_Party/FreeRTOS/Source/portable/GCC/ARM_CM7/r0p1/port.c 

OBJS += \
./Middlewares/Third_Party/FreeRTOS/Source/portable/GCC/ARM_CM7/r0p1/port.o 

C_DEPS += \
./Middlewares/Third_Party/FreeRTOS/Source/portable/GCC/ARM_CM7/r0p1/port.d 


# Each subdirectory must supply rules for building sources it contributes
Middlewares/Third_Party/FreeRTOS/Source/portable/GCC/ARM_CM7/r0p1/%.o: ../Middlewares/Third_Party/FreeRTOS/Source/portable/GCC/ARM_CM7/r0p1/%.c
	@echo 'Building file: $<'
	@echo 'Invoking: MCU GCC Compiler'
	@echo $(PWD)
	arm-none-eabi-gcc -mcpu=cortex-m7 -mthumb -mfloat-abi=hard -mfpu=fpv5-d16 '-D__weak=__attribute__((weak))' '-D__packed="__attribute__((__packed__))"' -DUSE_HAL_DRIVER -DSTM32H743xx -DARM_MATH_CM7 -I"/Users/sanyv/code/Projects/code/AC6/H7-newlib/Inc" -I"/Users/sanyv/code/Projects/code/AC6/H7-newlib/Drivers/STM32H7xx_HAL_Driver/Inc" -I"/Users/sanyv/code/Projects/code/AC6/H7-newlib/Drivers/System" -I"/Users/sanyv/code/Projects/code/AC6/H7-newlib/Drivers/Sensor" -I"/Users/sanyv/code/Projects/code/AC6/H7-newlib/Drivers/STM32H7xx_HAL_Driver/Inc/Legacy" -I"/Users/sanyv/code/Projects/code/AC6/H7-newlib/Drivers/CMSIS/Device/ST/STM32H7xx/Include" -I"/Users/sanyv/code/Projects/code/AC6/H7-newlib/Drivers/CMSIS/Include" -I"/Users/sanyv/code/Projects/code/AC6/H7-newlib/Middlewares/Third_Party/FreeRTOS/Source/portable/GCC/ARM_CM7/r0p1" -I"/Users/sanyv/code/Projects/code/AC6/H7-newlib/Middlewares/Third_Party/FreeRTOS/Source/include" -I"/Users/sanyv/code/Projects/code/AC6/H7-newlib/Middlewares/Third_Party/FreeRTOS/Source/CMSIS_RTOS"  -O2 -Wall -fmessage-length=0 -c -fmessage-length=0 -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '


