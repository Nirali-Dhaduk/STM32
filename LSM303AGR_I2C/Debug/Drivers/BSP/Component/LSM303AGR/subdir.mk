################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (10.3-2021.10)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Drivers/BSP/Component/LSM303AGR/lsm303agr.c 

OBJS += \
./Drivers/BSP/Component/LSM303AGR/lsm303agr.o 

C_DEPS += \
./Drivers/BSP/Component/LSM303AGR/lsm303agr.d 


# Each subdirectory must supply rules for building sources it contributes
Drivers/BSP/Component/LSM303AGR/%.o Drivers/BSP/Component/LSM303AGR/%.su: ../Drivers/BSP/Component/LSM303AGR/%.c Drivers/BSP/Component/LSM303AGR/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m0plus -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32L053xx -c -I../Core/Inc -I../Drivers/STM32L0xx_HAL_Driver/Inc -I../Drivers/STM32L0xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32L0xx/Include -I../Drivers/CMSIS/Include -I"D:/pragnesh/02_project/Nirali/LSM303AGR_I2C/Drivers/BSP/Component/LSM303AGR" -I"D:/pragnesh/02_project/Nirali/LSM303AGR_I2C/Debug/Drivers/BSP/Component/LSM303AGR" -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfloat-abi=soft -mthumb -o "$@"

clean: clean-Drivers-2f-BSP-2f-Component-2f-LSM303AGR

clean-Drivers-2f-BSP-2f-Component-2f-LSM303AGR:
	-$(RM) ./Drivers/BSP/Component/LSM303AGR/lsm303agr.d ./Drivers/BSP/Component/LSM303AGR/lsm303agr.o ./Drivers/BSP/Component/LSM303AGR/lsm303agr.su

.PHONY: clean-Drivers-2f-BSP-2f-Component-2f-LSM303AGR

