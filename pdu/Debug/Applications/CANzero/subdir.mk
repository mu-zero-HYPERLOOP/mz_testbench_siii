################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (10.3-2021.10)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
CPP_SRCS += \
../Applications/CANzero/CANZERO_OD_overwrites.cpp \
../Applications/CANzero/Emergency.cpp \
../Applications/CANzero/Heartbeat.cpp \
../Applications/CANzero/canzero_emcy.cpp \
../Applications/CANzero/canzero_od.cpp \
../Applications/CANzero/cz_interface.cpp \
../Applications/CANzero/cz_processRx.cpp 

OBJS += \
./Applications/CANzero/CANZERO_OD_overwrites.o \
./Applications/CANzero/Emergency.o \
./Applications/CANzero/Heartbeat.o \
./Applications/CANzero/canzero_emcy.o \
./Applications/CANzero/canzero_od.o \
./Applications/CANzero/cz_interface.o \
./Applications/CANzero/cz_processRx.o 

CPP_DEPS += \
./Applications/CANzero/CANZERO_OD_overwrites.d \
./Applications/CANzero/Emergency.d \
./Applications/CANzero/Heartbeat.d \
./Applications/CANzero/canzero_emcy.d \
./Applications/CANzero/canzero_od.d \
./Applications/CANzero/cz_interface.d \
./Applications/CANzero/cz_processRx.d 


# Each subdirectory must supply rules for building sources it contributes
Applications/CANzero/%.o Applications/CANzero/%.su Applications/CANzero/%.cyclo: ../Applications/CANzero/%.cpp Applications/CANzero/subdir.mk
	arm-none-eabi-g++ "$<" -mcpu=cortex-m4 -std=gnu++14 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32F407xx -c -I../Core/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc/Legacy -I../Middlewares/Third_Party/FreeRTOS/Source/include -I../Middlewares/Third_Party/FreeRTOS/Source/CMSIS_RTOS_V2 -I../Middlewares/Third_Party/FreeRTOS/Source/portable/GCC/ARM_CM4F -I../Drivers/CMSIS/Device/ST/STM32F4xx/Include -I../Drivers/CMSIS/Include -I"/home/karlsassie/Documents/mu-zero/git/mz_testbench_siii/pdu/Applications/CANzero/dbc-cpp" -I"/home/karlsassie/Documents/mu-zero/git/mz_testbench_siii/pdu/Applications/CANzero/include" -I"/home/karlsassie/Documents/mu-zero/git/mz_testbench_siii/pdu/Applications/include" -O0 -ffunction-sections -fdata-sections -fno-exceptions -fno-rtti -fno-use-cxa-atexit -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-Applications-2f-CANzero

clean-Applications-2f-CANzero:
	-$(RM) ./Applications/CANzero/CANZERO_OD_overwrites.cyclo ./Applications/CANzero/CANZERO_OD_overwrites.d ./Applications/CANzero/CANZERO_OD_overwrites.o ./Applications/CANzero/CANZERO_OD_overwrites.su ./Applications/CANzero/Emergency.cyclo ./Applications/CANzero/Emergency.d ./Applications/CANzero/Emergency.o ./Applications/CANzero/Emergency.su ./Applications/CANzero/Heartbeat.cyclo ./Applications/CANzero/Heartbeat.d ./Applications/CANzero/Heartbeat.o ./Applications/CANzero/Heartbeat.su ./Applications/CANzero/canzero_emcy.cyclo ./Applications/CANzero/canzero_emcy.d ./Applications/CANzero/canzero_emcy.o ./Applications/CANzero/canzero_emcy.su ./Applications/CANzero/canzero_od.cyclo ./Applications/CANzero/canzero_od.d ./Applications/CANzero/canzero_od.o ./Applications/CANzero/canzero_od.su ./Applications/CANzero/cz_interface.cyclo ./Applications/CANzero/cz_interface.d ./Applications/CANzero/cz_interface.o ./Applications/CANzero/cz_interface.su ./Applications/CANzero/cz_processRx.cyclo ./Applications/CANzero/cz_processRx.d ./Applications/CANzero/cz_processRx.o ./Applications/CANzero/cz_processRx.su

.PHONY: clean-Applications-2f-CANzero

