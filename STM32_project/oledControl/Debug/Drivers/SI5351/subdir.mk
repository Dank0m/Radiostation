################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (13.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Drivers/SI5351/si5351.c 

OBJS += \
./Drivers/SI5351/si5351.o 

C_DEPS += \
./Drivers/SI5351/si5351.d 


# Each subdirectory must supply rules for building sources it contributes
Drivers/SI5351/%.o Drivers/SI5351/%.su Drivers/SI5351/%.cyclo: ../Drivers/SI5351/%.c Drivers/SI5351/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m0 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32F051x8 -c -I../Core/Inc -I"C:/Users/User/Desktop/STM32_project/oledControl/Drivers/SI5351" -I"C:/Users/User/Desktop/STM32_project/oledControl/Drivers/OLED" -I../Drivers/STM32F0xx_HAL_Driver/Inc -I../Drivers/STM32F0xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32F0xx/Include -I../Drivers/CMSIS/Include -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfloat-abi=soft -mthumb -o "$@"

clean: clean-Drivers-2f-SI5351

clean-Drivers-2f-SI5351:
	-$(RM) ./Drivers/SI5351/si5351.cyclo ./Drivers/SI5351/si5351.d ./Drivers/SI5351/si5351.o ./Drivers/SI5351/si5351.su

.PHONY: clean-Drivers-2f-SI5351

