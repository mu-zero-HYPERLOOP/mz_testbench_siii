################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (10.3-2021.10)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
CPP_SRCS += \
../Applications/Src/main_entry.cpp 

OBJS += \
./Applications/Src/main_entry.o 

CPP_DEPS += \
./Applications/Src/main_entry.d 


# Each subdirectory must supply rules for building sources it contributes
Applications/Src/%.o Applications/Src/%.su Applications/Src/%.cyclo: ../Applications/Src/%.cpp Applications/Src/subdir.mk
	arm-none-eabi-g++ "$<" -mcpu=cortex-m4 -std=gnu++14 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32F407xx -c -I../Core/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc/Legacy -I../Middlewares/Third_Party/FreeRTOS/Source/include -I../Middlewares/Third_Party/FreeRTOS/Source/CMSIS_RTOS_V2 -I../Middlewares/Third_Party/FreeRTOS/Source/portable/GCC/ARM_CM4F -I../Drivers/CMSIS/Device/ST/STM32F4xx/Include -I../Drivers/CMSIS/Include -I"C:/Users/OfficeLaptop/Documents/GitHub/mz_testbench_siii/track/Lib/Inc" -I"C:/Users/OfficeLaptop/Documents/GitHub/mz_testbench_siii/track/Applications/Inc" -O0 -ffunction-sections -fdata-sections -fno-exceptions -fno-rtti -fno-use-cxa-atexit -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-Applications-2f-Src

clean-Applications-2f-Src:
	-$(RM) ./Applications/Src/main_entry.cyclo ./Applications/Src/main_entry.d ./Applications/Src/main_entry.o ./Applications/Src/main_entry.su

.PHONY: clean-Applications-2f-Src

