################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (10.3-2021.10)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
CPP_SRCS += \
../Lib/Src/AdcConvCpltCallbackHandler.cpp \
../Lib/Src/AdcConvHalfCpltCallbackHandler.cpp \
../Lib/Src/GPIOExtiCallbackHandler.cpp \
../Lib/Src/TimPeriodElapsedCallbackHandler.cpp \
../Lib/Src/estdio.cpp \
../Lib/Src/putchar.cpp 

OBJS += \
./Lib/Src/AdcConvCpltCallbackHandler.o \
./Lib/Src/AdcConvHalfCpltCallbackHandler.o \
./Lib/Src/GPIOExtiCallbackHandler.o \
./Lib/Src/TimPeriodElapsedCallbackHandler.o \
./Lib/Src/estdio.o \
./Lib/Src/putchar.o 

CPP_DEPS += \
./Lib/Src/AdcConvCpltCallbackHandler.d \
./Lib/Src/AdcConvHalfCpltCallbackHandler.d \
./Lib/Src/GPIOExtiCallbackHandler.d \
./Lib/Src/TimPeriodElapsedCallbackHandler.d \
./Lib/Src/estdio.d \
./Lib/Src/putchar.d 


# Each subdirectory must supply rules for building sources it contributes
Lib/Src/%.o Lib/Src/%.su Lib/Src/%.cyclo: ../Lib/Src/%.cpp Lib/Src/subdir.mk
	arm-none-eabi-g++ "$<" -mcpu=cortex-m4 -std=gnu++14 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32F407xx -c -I../Drivers/STM32F4xx_HAL_Driver/Inc/Legacy -I../Middlewares/Third_Party/FreeRTOS/Source/include -I../Middlewares/Third_Party/FreeRTOS/Source/CMSIS_RTOS_V2 -I../Middlewares/Third_Party/FreeRTOS/Source/portable/GCC/ARM_CM4F -I../Drivers/CMSIS/Device/ST/STM32F4xx/Include -I../Drivers/CMSIS/Include -I"/home/karlsassie/Documents/mu-zero/git/mz_testbench_siii/sensor_ecu/Lib/Inc" -I"/home/karlsassie/Documents/mu-zero/git/mz_testbench_siii/sensor_ecu/Applications/CANzero/dbc-cpp" -I"/home/karlsassie/Documents/mu-zero/git/mz_testbench_siii/sensor_ecu/Core/Inc" -I"/home/karlsassie/Documents/mu-zero/git/mz_testbench_siii/sensor_ecu/Drivers/CMSIS/Include" -I"/home/karlsassie/Documents/mu-zero/git/mz_testbench_siii/sensor_ecu/Drivers/STM32F4xx_HAL_Driver/Inc" -I"/home/karlsassie/Documents/mu-zero/git/mz_testbench_siii/sensor_ecu/Applications/CANzero/include" -I"/home/karlsassie/Documents/mu-zero/git/mz_testbench_siii/sensor_ecu/Applications/CANZero_static/Inc" -I../Core/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc -I"/home/karlsassie/Documents/mu-zero/git/mz_testbench_siii/sensor_ecu/Applications/Inc" -O0 -ffunction-sections -fdata-sections -fno-exceptions -fno-rtti -fno-use-cxa-atexit -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-Lib-2f-Src

clean-Lib-2f-Src:
	-$(RM) ./Lib/Src/AdcConvCpltCallbackHandler.cyclo ./Lib/Src/AdcConvCpltCallbackHandler.d ./Lib/Src/AdcConvCpltCallbackHandler.o ./Lib/Src/AdcConvCpltCallbackHandler.su ./Lib/Src/AdcConvHalfCpltCallbackHandler.cyclo ./Lib/Src/AdcConvHalfCpltCallbackHandler.d ./Lib/Src/AdcConvHalfCpltCallbackHandler.o ./Lib/Src/AdcConvHalfCpltCallbackHandler.su ./Lib/Src/GPIOExtiCallbackHandler.cyclo ./Lib/Src/GPIOExtiCallbackHandler.d ./Lib/Src/GPIOExtiCallbackHandler.o ./Lib/Src/GPIOExtiCallbackHandler.su ./Lib/Src/TimPeriodElapsedCallbackHandler.cyclo ./Lib/Src/TimPeriodElapsedCallbackHandler.d ./Lib/Src/TimPeriodElapsedCallbackHandler.o ./Lib/Src/TimPeriodElapsedCallbackHandler.su ./Lib/Src/estdio.cyclo ./Lib/Src/estdio.d ./Lib/Src/estdio.o ./Lib/Src/estdio.su ./Lib/Src/putchar.cyclo ./Lib/Src/putchar.d ./Lib/Src/putchar.o ./Lib/Src/putchar.su

.PHONY: clean-Lib-2f-Src

