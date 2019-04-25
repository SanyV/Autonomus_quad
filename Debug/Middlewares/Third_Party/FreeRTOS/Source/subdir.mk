################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Middlewares/Third_Party/FreeRTOS/Source/croutine.c \
../Middlewares/Third_Party/FreeRTOS/Source/event_groups.c \
../Middlewares/Third_Party/FreeRTOS/Source/list.c \
../Middlewares/Third_Party/FreeRTOS/Source/queue.c \
../Middlewares/Third_Party/FreeRTOS/Source/tasks.c \
../Middlewares/Third_Party/FreeRTOS/Source/timers.c 

OBJS += \
./Middlewares/Third_Party/FreeRTOS/Source/croutine.o \
./Middlewares/Third_Party/FreeRTOS/Source/event_groups.o \
./Middlewares/Third_Party/FreeRTOS/Source/list.o \
./Middlewares/Third_Party/FreeRTOS/Source/queue.o \
./Middlewares/Third_Party/FreeRTOS/Source/tasks.o \
./Middlewares/Third_Party/FreeRTOS/Source/timers.o 

C_DEPS += \
./Middlewares/Third_Party/FreeRTOS/Source/croutine.d \
./Middlewares/Third_Party/FreeRTOS/Source/event_groups.d \
./Middlewares/Third_Party/FreeRTOS/Source/list.d \
./Middlewares/Third_Party/FreeRTOS/Source/queue.d \
./Middlewares/Third_Party/FreeRTOS/Source/tasks.d \
./Middlewares/Third_Party/FreeRTOS/Source/timers.d 


# Each subdirectory must supply rules for building sources it contributes
Middlewares/Third_Party/FreeRTOS/Source/%.o: ../Middlewares/Third_Party/FreeRTOS/Source/%.c
	@echo 'Building file: $<'
	@echo 'Invoking: MCU GCC Compiler'
	@echo $(PWD)
	arm-none-eabi-gcc -mcpu=cortex-m7 -mthumb -mfloat-abi=hard -mfpu=fpv5-d16 '-D__weak=__attribute__((weak))' '-D__packed="__attribute__((__packed__))"' -DUSE_HAL_DRIVER -DSTM32H743xx -DARM_MATH_CM7 -I"/Users/sanyv/code/Projects/code/AC6/H7-newlib/Inc" -I"/Users/sanyv/code/Projects/code/AC6/H7-newlib/Drivers/STM32H7xx_HAL_Driver/Inc" -I"/Users/sanyv/code/Projects/code/AC6/H7-newlib/Drivers/System" -I"/Users/sanyv/code/Projects/code/AC6/H7-newlib/Drivers/Sensor" -I"/Users/sanyv/code/Projects/code/AC6/H7-newlib/Drivers/STM32H7xx_HAL_Driver/Inc/Legacy" -I"/Users/sanyv/code/Projects/code/AC6/H7-newlib/Drivers/CMSIS/Device/ST/STM32H7xx/Include" -I"/Users/sanyv/code/Projects/code/AC6/H7-newlib/Drivers/CMSIS/Include" -I"/Users/sanyv/code/Projects/code/AC6/H7-newlib/Middlewares/Third_Party/FreeRTOS/Source/portable/GCC/ARM_CM7/r0p1" -I"/Users/sanyv/code/Projects/code/AC6/H7-newlib/Middlewares/Third_Party/FreeRTOS/Source/include" -I"/Users/sanyv/code/Projects/code/AC6/H7-newlib/Middlewares/Third_Party/FreeRTOS/Source/CMSIS_RTOS"  -O2 -Wall -fmessage-length=0 -c -fmessage-length=0 -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '


