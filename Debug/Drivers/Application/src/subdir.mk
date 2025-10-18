################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (11.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Drivers/Application/src/BLDC.c 

OBJS += \
./Drivers/Application/src/BLDC.o 

C_DEPS += \
./Drivers/Application/src/BLDC.d 


# Each subdirectory must supply rules for building sources it contributes
Drivers/Application/src/%.o Drivers/Application/src/%.su Drivers/Application/src/%.cyclo: ../Drivers/Application/src/%.c Drivers/Application/src/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m3 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32F103xB -c -I../Core/Inc -I"C:/Users/01226/Desktop/apex25codes/ADC_test/Drivers/Application/inc" -I"C:/Users/01226/Desktop/apex25codes/ADC_test/Drivers/Service/inc" -I"C:/Users/01226/Desktop/apex25codes/ADC_test/Drivers/HAL/inc" -I../Drivers/STM32F1xx_HAL_Driver/Inc/Legacy -I../Drivers/STM32F1xx_HAL_Driver/Inc -I../Drivers/CMSIS/Device/ST/STM32F1xx/Include -I../Drivers/CMSIS/Include -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfloat-abi=soft -mthumb -o "$@"

clean: clean-Drivers-2f-Application-2f-src

clean-Drivers-2f-Application-2f-src:
	-$(RM) ./Drivers/Application/src/BLDC.cyclo ./Drivers/Application/src/BLDC.d ./Drivers/Application/src/BLDC.o ./Drivers/Application/src/BLDC.su

.PHONY: clean-Drivers-2f-Application-2f-src

