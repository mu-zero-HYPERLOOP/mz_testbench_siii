################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (10.3-2021.10)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
CPP_SRCS += \
../Applications/FlashAccess.cpp \
../Applications/Sensor.cpp \
../Applications/TaskManager.cpp \
../Applications/Watchdog.cpp \
../Applications/application.cpp 

OBJS += \
./Applications/FlashAccess.o \
./Applications/Sensor.o \
./Applications/TaskManager.o \
./Applications/Watchdog.o \
./Applications/application.o 

CPP_DEPS += \
./Applications/FlashAccess.d \
./Applications/Sensor.d \
./Applications/TaskManager.d \
./Applications/Watchdog.d \
./Applications/application.d 


# Each subdirectory must supply rules for building sources it contributes
Applications/%.o Applications/%.su Applications/%.cyclo: ../Applications/%.cpp Applications/subdir.mk
	arm-none-eabi-g++ "$<" -mcpu=cortex-m4 -std=gnu++14 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32F407xx -c -I../Core/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc/Legacy -I../Middlewares/Third_Party/FreeRTOS/Source/include -I../Middlewares/Third_Party/FreeRTOS/Source/CMSIS_RTOS_V2 -I../Middlewares/Third_Party/FreeRTOS/Source/portable/GCC/ARM_CM4F -I../Drivers/CMSIS/Device/ST/STM32F4xx/Include -I../Drivers/CMSIS/Include -I"/home/karlsassie/Documents/mu-zero/git/mz_testbench_siii/pdu/Applications/CANzero/dbc-cpp" -I"/home/karlsassie/Documents/mu-zero/git/mz_testbench_siii/pdu/Applications/CANzero/include" -I"/home/karlsassie/Documents/mu-zero/git/mz_testbench_siii/pdu/Applications/include" -O0 -ffunction-sections -fdata-sections -fno-exceptions -fno-rtti -fno-use-cxa-atexit -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-Applications

clean-Applications:
	-$(RM) ./Applications/FlashAccess.cyclo ./Applications/FlashAccess.d ./Applications/FlashAccess.o ./Applications/FlashAccess.su ./Applications/Sensor.cyclo ./Applications/Sensor.d ./Applications/Sensor.o ./Applications/Sensor.su ./Applications/TaskManager.cyclo ./Applications/TaskManager.d ./Applications/TaskManager.o ./Applications/TaskManager.su ./Applications/Watchdog.cyclo ./Applications/Watchdog.d ./Applications/Watchdog.o ./Applications/Watchdog.su ./Applications/application.cyclo ./Applications/application.d ./Applications/application.o ./Applications/application.su

.PHONY: clean-Applications

