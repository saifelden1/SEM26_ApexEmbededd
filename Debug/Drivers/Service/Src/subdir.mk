################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (11.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Drivers/Service/Src/Delay.c \
../Drivers/Service/Src/PowerCal.c \
../Drivers/Service/Src/SoftThrottle.c \
../Drivers/Service/Src/SpeedDWT.c \
../Drivers/Service/Src/TimeDif.c 

OBJS += \
./Drivers/Service/Src/Delay.o \
./Drivers/Service/Src/PowerCal.o \
./Drivers/Service/Src/SoftThrottle.o \
./Drivers/Service/Src/SpeedDWT.o \
./Drivers/Service/Src/TimeDif.o 

C_DEPS += \
./Drivers/Service/Src/Delay.d \
./Drivers/Service/Src/PowerCal.d \
./Drivers/Service/Src/SoftThrottle.d \
./Drivers/Service/Src/SpeedDWT.d \
./Drivers/Service/Src/TimeDif.d 


# Each subdirectory must supply rules for building sources it contributes
Drivers/Service/Src/%.o Drivers/Service/Src/%.su Drivers/Service/Src/%.cyclo: ../Drivers/Service/Src/%.c Drivers/Service/Src/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m3 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32F103xB -c -I../Core/Inc -I"C:/Users/01226/Desktop/apex25codes/ADC_test/Drivers/Application/inc" -I"C:/Users/01226/Desktop/apex25codes/ADC_test/Drivers/Service/inc" -I"C:/Users/01226/Desktop/apex25codes/ADC_test/Drivers/HAL/inc" -I../Drivers/STM32F1xx_HAL_Driver/Inc/Legacy -I../Drivers/STM32F1xx_HAL_Driver/Inc -I../Drivers/CMSIS/Device/ST/STM32F1xx/Include -I../Drivers/CMSIS/Include -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfloat-abi=soft -mthumb -o "$@"

clean: clean-Drivers-2f-Service-2f-Src

clean-Drivers-2f-Service-2f-Src:
	-$(RM) ./Drivers/Service/Src/Delay.cyclo ./Drivers/Service/Src/Delay.d ./Drivers/Service/Src/Delay.o ./Drivers/Service/Src/Delay.su ./Drivers/Service/Src/PowerCal.cyclo ./Drivers/Service/Src/PowerCal.d ./Drivers/Service/Src/PowerCal.o ./Drivers/Service/Src/PowerCal.su ./Drivers/Service/Src/SoftThrottle.cyclo ./Drivers/Service/Src/SoftThrottle.d ./Drivers/Service/Src/SoftThrottle.o ./Drivers/Service/Src/SoftThrottle.su ./Drivers/Service/Src/SpeedDWT.cyclo ./Drivers/Service/Src/SpeedDWT.d ./Drivers/Service/Src/SpeedDWT.o ./Drivers/Service/Src/SpeedDWT.su ./Drivers/Service/Src/TimeDif.cyclo ./Drivers/Service/Src/TimeDif.d ./Drivers/Service/Src/TimeDif.o ./Drivers/Service/Src/TimeDif.su

.PHONY: clean-Drivers-2f-Service-2f-Src

