################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (11.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Drivers/Service/Src/Delay.c \
../Drivers/Service/Src/SpeedCal.c 

OBJS += \
./Drivers/Service/Src/Delay.o \
./Drivers/Service/Src/SpeedCal.o 

C_DEPS += \
./Drivers/Service/Src/Delay.d \
./Drivers/Service/Src/SpeedCal.d 


# Each subdirectory must supply rules for building sources it contributes
Drivers/Service/Src/%.o Drivers/Service/Src/%.su Drivers/Service/Src/%.cyclo: ../Drivers/Service/Src/%.c Drivers/Service/Src/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m3 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32F103xB -c -I../Core/Inc -I"C:/Users/01226/Desktop/apex25codes/ADC_test/Drivers/Application/inc" -I"C:/Users/01226/Desktop/apex25codes/ADC_test/Drivers/Service/inc" -I"C:/Users/01226/Desktop/apex25codes/ADC_test/Drivers/HAL/inc" -I../Drivers/STM32F1xx_HAL_Driver/Inc/Legacy -I../Drivers/STM32F1xx_HAL_Driver/Inc -I../Drivers/CMSIS/Device/ST/STM32F1xx/Include -I../Drivers/CMSIS/Include -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfloat-abi=soft -mthumb -o "$@"

clean: clean-Drivers-2f-Service-2f-Src

clean-Drivers-2f-Service-2f-Src:
	-$(RM) ./Drivers/Service/Src/Delay.cyclo ./Drivers/Service/Src/Delay.d ./Drivers/Service/Src/Delay.o ./Drivers/Service/Src/Delay.su ./Drivers/Service/Src/SpeedCal.cyclo ./Drivers/Service/Src/SpeedCal.d ./Drivers/Service/Src/SpeedCal.o ./Drivers/Service/Src/SpeedCal.su

.PHONY: clean-Drivers-2f-Service-2f-Src

