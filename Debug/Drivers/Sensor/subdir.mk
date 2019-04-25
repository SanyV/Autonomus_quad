################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Drivers/Sensor/ADC.c \
../Drivers/Sensor/IMU_INS.c \
../Drivers/Sensor/MPU6500.c \
../Drivers/Sensor/NAV_POINTS.c \
../Drivers/Sensor/PID.c \
../Drivers/Sensor/PWM_PPM.c \
../Drivers/Sensor/UKF_lib.c \
../Drivers/Sensor/hmc5983_spi.c \
../Drivers/Sensor/ms5611_spi.c \
../Drivers/Sensor/ublox.c 

OBJS += \
./Drivers/Sensor/ADC.o \
./Drivers/Sensor/IMU_INS.o \
./Drivers/Sensor/MPU6500.o \
./Drivers/Sensor/NAV_POINTS.o \
./Drivers/Sensor/PID.o \
./Drivers/Sensor/PWM_PPM.o \
./Drivers/Sensor/UKF_lib.o \
./Drivers/Sensor/hmc5983_spi.o \
./Drivers/Sensor/ms5611_spi.o \
./Drivers/Sensor/ublox.o 

C_DEPS += \
./Drivers/Sensor/ADC.d \
./Drivers/Sensor/IMU_INS.d \
./Drivers/Sensor/MPU6500.d \
./Drivers/Sensor/NAV_POINTS.d \
./Drivers/Sensor/PID.d \
./Drivers/Sensor/PWM_PPM.d \
./Drivers/Sensor/UKF_lib.d \
./Drivers/Sensor/hmc5983_spi.d \
./Drivers/Sensor/ms5611_spi.d \
./Drivers/Sensor/ublox.d 


# Each subdirectory must supply rules for building sources it contributes
Drivers/Sensor/%.o: ../Drivers/Sensor/%.c
	@echo 'Building file: $<'
	@echo 'Invoking: MCU GCC Compiler'
	@echo $(PWD)
	arm-none-eabi-gcc -mcpu=cortex-m7 -mthumb -mfloat-abi=hard -mfpu=fpv5-d16 '-D__weak=__attribute__((weak))' '-D__packed="__attribute__((__packed__))"' -DUSE_HAL_DRIVER -DSTM32H743xx -DARM_MATH_CM7 -I"/Users/sanyv/code/Projects/code/AC6/H7-newlib/Inc" -I"/Users/sanyv/code/Projects/code/AC6/H7-newlib/Drivers/STM32H7xx_HAL_Driver/Inc" -I"/Users/sanyv/code/Projects/code/AC6/H7-newlib/Drivers/System" -I"/Users/sanyv/code/Projects/code/AC6/H7-newlib/Drivers/Sensor" -I"/Users/sanyv/code/Projects/code/AC6/H7-newlib/Drivers/STM32H7xx_HAL_Driver/Inc/Legacy" -I"/Users/sanyv/code/Projects/code/AC6/H7-newlib/Drivers/CMSIS/Device/ST/STM32H7xx/Include" -I"/Users/sanyv/code/Projects/code/AC6/H7-newlib/Drivers/CMSIS/Include" -I"/Users/sanyv/code/Projects/code/AC6/H7-newlib/Middlewares/Third_Party/FreeRTOS/Source/portable/GCC/ARM_CM7/r0p1" -I"/Users/sanyv/code/Projects/code/AC6/H7-newlib/Middlewares/Third_Party/FreeRTOS/Source/include" -I"/Users/sanyv/code/Projects/code/AC6/H7-newlib/Middlewares/Third_Party/FreeRTOS/Source/CMSIS_RTOS"  -O2 -Wall -fmessage-length=0 -c -fmessage-length=0 -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '


