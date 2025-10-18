################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (11.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Drivers/HAL/Src/ACS758-050B.c \
../Drivers/HAL/Src/CAN.c \
../Drivers/HAL/Src/DMA_ADC.c \
../Drivers/HAL/Src/Hall_Sensor.c \
../Drivers/HAL/Src/NixtonLcd.c \
../Drivers/HAL/Src/Throttle.c \
../Drivers/HAL/Src/Voltage_sensor.c 

OBJS += \
./Drivers/HAL/Src/ACS758-050B.o \
./Drivers/HAL/Src/CAN.o \
./Drivers/HAL/Src/DMA_ADC.o \
./Drivers/HAL/Src/Hall_Sensor.o \
./Drivers/HAL/Src/NixtonLcd.o \
./Drivers/HAL/Src/Throttle.o \
./Drivers/HAL/Src/Voltage_sensor.o 

C_DEPS += \
./Drivers/HAL/Src/ACS758-050B.d \
./Drivers/HAL/Src/CAN.d \
./Drivers/HAL/Src/DMA_ADC.d \
./Drivers/HAL/Src/Hall_Sensor.d \
./Drivers/HAL/Src/NixtonLcd.d \
./Drivers/HAL/Src/Throttle.d \
./Drivers/HAL/Src/Voltage_sensor.d 


# Each subdirectory must supply rules for building sources it contributes
Drivers/HAL/Src/%.o Drivers/HAL/Src/%.su Drivers/HAL/Src/%.cyclo: ../Drivers/HAL/Src/%.c Drivers/HAL/Src/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m3 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32F103xB -c -I../Core/Inc -I"C:/Users/01226/Desktop/apex25codes/ADC_test/Drivers/Application/inc" -I"C:/Users/01226/Desktop/apex25codes/ADC_test/Drivers/Service/inc" -I"C:/Users/01226/Desktop/apex25codes/ADC_test/Drivers/HAL/inc" -I../Drivers/STM32F1xx_HAL_Driver/Inc/Legacy -I../Drivers/STM32F1xx_HAL_Driver/Inc -I../Drivers/CMSIS/Device/ST/STM32F1xx/Include -I../Drivers/CMSIS/Include -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfloat-abi=soft -mthumb -o "$@"

clean: clean-Drivers-2f-HAL-2f-Src

clean-Drivers-2f-HAL-2f-Src:
	-$(RM) ./Drivers/HAL/Src/ACS758-050B.cyclo ./Drivers/HAL/Src/ACS758-050B.d ./Drivers/HAL/Src/ACS758-050B.o ./Drivers/HAL/Src/ACS758-050B.su ./Drivers/HAL/Src/CAN.cyclo ./Drivers/HAL/Src/CAN.d ./Drivers/HAL/Src/CAN.o ./Drivers/HAL/Src/CAN.su ./Drivers/HAL/Src/DMA_ADC.cyclo ./Drivers/HAL/Src/DMA_ADC.d ./Drivers/HAL/Src/DMA_ADC.o ./Drivers/HAL/Src/DMA_ADC.su ./Drivers/HAL/Src/Hall_Sensor.cyclo ./Drivers/HAL/Src/Hall_Sensor.d ./Drivers/HAL/Src/Hall_Sensor.o ./Drivers/HAL/Src/Hall_Sensor.su ./Drivers/HAL/Src/NixtonLcd.cyclo ./Drivers/HAL/Src/NixtonLcd.d ./Drivers/HAL/Src/NixtonLcd.o ./Drivers/HAL/Src/NixtonLcd.su ./Drivers/HAL/Src/Throttle.cyclo ./Drivers/HAL/Src/Throttle.d ./Drivers/HAL/Src/Throttle.o ./Drivers/HAL/Src/Throttle.su ./Drivers/HAL/Src/Voltage_sensor.cyclo ./Drivers/HAL/Src/Voltage_sensor.d ./Drivers/HAL/Src/Voltage_sensor.o ./Drivers/HAL/Src/Voltage_sensor.su

.PHONY: clean-Drivers-2f-HAL-2f-Src

