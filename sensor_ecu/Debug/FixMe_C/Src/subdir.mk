################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (10.3-2021.10)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../FixMe_C/Src/led.c 

C_DEPS += \
./FixMe_C/Src/led.d 

OBJS += \
./FixMe_C/Src/led.o 


# Each subdirectory must supply rules for building sources it contributes
FixMe_C/Src/%.o FixMe_C/Src/%.su FixMe_C/Src/%.cyclo: ../FixMe_C/Src/%.c FixMe_C/Src/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32F407xx -c -I../Core/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc/Legacy -I../Middlewares/Third_Party/FreeRTOS/Source/include -I../Middlewares/Third_Party/FreeRTOS/Source/CMSIS_RTOS_V2 -I../Middlewares/Third_Party/FreeRTOS/Source/portable/GCC/ARM_CM4F -I../Drivers/CMSIS/Device/ST/STM32F4xx/Include -I../Drivers/CMSIS/Include -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-FixMe_C-2f-Src

clean-FixMe_C-2f-Src:
	-$(RM) ./FixMe_C/Src/led.cyclo ./FixMe_C/Src/led.d ./FixMe_C/Src/led.o ./FixMe_C/Src/led.su

.PHONY: clean-FixMe_C-2f-Src

