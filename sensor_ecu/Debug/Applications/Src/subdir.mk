################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (10.3-2021.10)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
CPP_SRCS += \
../Applications/Src/FiducialSensor.cpp \
../Applications/Src/NTCSensor.cpp \
../Applications/Src/SDC.cpp \
../Applications/Src/StartupState.cpp \
../Applications/Src/testbench.cpp 

OBJS += \
./Applications/Src/FiducialSensor.o \
./Applications/Src/NTCSensor.o \
./Applications/Src/SDC.o \
./Applications/Src/StartupState.o \
./Applications/Src/testbench.o 

CPP_DEPS += \
./Applications/Src/FiducialSensor.d \
./Applications/Src/NTCSensor.d \
./Applications/Src/SDC.d \
./Applications/Src/StartupState.d \
./Applications/Src/testbench.d 


# Each subdirectory must supply rules for building sources it contributes
Applications/Src/%.o Applications/Src/%.su Applications/Src/%.cyclo: ../Applications/Src/%.cpp Applications/Src/subdir.mk
	arm-none-eabi-g++ "$<" -mcpu=cortex-m4 -std=gnu++14 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32F407xx -c -I../Drivers/STM32F4xx_HAL_Driver/Inc/Legacy -I../Middlewares/Third_Party/FreeRTOS/Source/include -I../Middlewares/Third_Party/FreeRTOS/Source/CMSIS_RTOS_V2 -I../Middlewares/Third_Party/FreeRTOS/Source/portable/GCC/ARM_CM4F -I../Drivers/CMSIS/Device/ST/STM32F4xx/Include -I../Drivers/CMSIS/Include -I"C:/Users/OfficeLaptop/Documents/GitHub/mz_testbench_siii/sensor_ecu/Lib/Inc" -I"C:/Users/OfficeLaptop/Documents/GitHub/mz_testbench_siii/sensor_ecu/Applications/CANzero/dbc-cpp" -I"C:/Users/OfficeLaptop/Documents/GitHub/mz_testbench_siii/sensor_ecu/Core/Inc" -I"C:/Users/OfficeLaptop/Documents/GitHub/mz_testbench_siii/sensor_ecu/Drivers/CMSIS/Include" -I"C:/Users/OfficeLaptop/Documents/GitHub/mz_testbench_siii/sensor_ecu/Drivers/STM32F4xx_HAL_Driver/Inc" -I"C:/Users/OfficeLaptop/Documents/GitHub/mz_testbench_siii/sensor_ecu/Applications/CANzero/include" -I"C:/Users/OfficeLaptop/Documents/GitHub/mz_testbench_siii/sensor_ecu/Applications/CANZero_static/Inc" -I../Core/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc -I"C:/Users/OfficeLaptop/Documents/GitHub/mz_testbench_siii/sensor_ecu/Applications/Inc" -O0 -ffunction-sections -fdata-sections -fno-exceptions -fno-rtti -fno-use-cxa-atexit -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-Applications-2f-Src

clean-Applications-2f-Src:
	-$(RM) ./Applications/Src/FiducialSensor.cyclo ./Applications/Src/FiducialSensor.d ./Applications/Src/FiducialSensor.o ./Applications/Src/FiducialSensor.su ./Applications/Src/NTCSensor.cyclo ./Applications/Src/NTCSensor.d ./Applications/Src/NTCSensor.o ./Applications/Src/NTCSensor.su ./Applications/Src/SDC.cyclo ./Applications/Src/SDC.d ./Applications/Src/SDC.o ./Applications/Src/SDC.su ./Applications/Src/StartupState.cyclo ./Applications/Src/StartupState.d ./Applications/Src/StartupState.o ./Applications/Src/StartupState.su ./Applications/Src/testbench.cyclo ./Applications/Src/testbench.d ./Applications/Src/testbench.o ./Applications/Src/testbench.su

.PHONY: clean-Applications-2f-Src

