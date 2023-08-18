################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (10.3-2021.10)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Drivers/BNO055_driver/bno055.c \
../Drivers/BNO055_driver/bno055_support.c 

OBJS += \
./Drivers/BNO055_driver/bno055.o \
./Drivers/BNO055_driver/bno055_support.o 

C_DEPS += \
./Drivers/BNO055_driver/bno055.d \
./Drivers/BNO055_driver/bno055_support.d 


# Each subdirectory must supply rules for building sources it contributes
Drivers/BNO055_driver/%.o Drivers/BNO055_driver/%.su: ../Drivers/BNO055_driver/%.c Drivers/BNO055_driver/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m7 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32F756xx -c -I../Core/Inc -I../Drivers/STM32F7xx_HAL_Driver/Inc -I../Drivers/STM32F7xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32F7xx/Include -I../Drivers/CMSIS/Include -I"E:/Tsemi/HaruRobo/STM32/HaruRobo/I2Ctest/Drivers/BNO055_driver" -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv5-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-Drivers-2f-BNO055_driver

clean-Drivers-2f-BNO055_driver:
	-$(RM) ./Drivers/BNO055_driver/bno055.d ./Drivers/BNO055_driver/bno055.o ./Drivers/BNO055_driver/bno055.su ./Drivers/BNO055_driver/bno055_support.d ./Drivers/BNO055_driver/bno055_support.o ./Drivers/BNO055_driver/bno055_support.su

.PHONY: clean-Drivers-2f-BNO055_driver

