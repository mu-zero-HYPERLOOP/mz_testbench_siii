################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (10.3-2021.10)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
CPP_SRCS += \
../Applications/CANZero_static/Src/cz_emergency.cpp \
../Applications/CANZero_static/Src/cz_handles.cpp \
../Applications/CANZero_static/Src/cz_heartbeat.cpp \
../Applications/CANZero_static/Src/cz_message_receiver.cpp \
../Applications/CANZero_static/Src/cz_processRx.cpp \
../Applications/CANZero_static/Src/cz_receive_queue.cpp \
../Applications/CANZero_static/Src/cz_receive_task.cpp \
../Applications/CANZero_static/Src/cz_send_queue.cpp \
../Applications/CANZero_static/Src/cz_send_task.cpp \
../Applications/CANZero_static/Src/cz_statemaschine.cpp \
../Applications/CANZero_static/Src/cz_taskmaster.cpp \
../Applications/CANZero_static/Src/cz_weak.cpp 

OBJS += \
./Applications/CANZero_static/Src/cz_emergency.o \
./Applications/CANZero_static/Src/cz_handles.o \
./Applications/CANZero_static/Src/cz_heartbeat.o \
./Applications/CANZero_static/Src/cz_message_receiver.o \
./Applications/CANZero_static/Src/cz_processRx.o \
./Applications/CANZero_static/Src/cz_receive_queue.o \
./Applications/CANZero_static/Src/cz_receive_task.o \
./Applications/CANZero_static/Src/cz_send_queue.o \
./Applications/CANZero_static/Src/cz_send_task.o \
./Applications/CANZero_static/Src/cz_statemaschine.o \
./Applications/CANZero_static/Src/cz_taskmaster.o \
./Applications/CANZero_static/Src/cz_weak.o 

CPP_DEPS += \
./Applications/CANZero_static/Src/cz_emergency.d \
./Applications/CANZero_static/Src/cz_handles.d \
./Applications/CANZero_static/Src/cz_heartbeat.d \
./Applications/CANZero_static/Src/cz_message_receiver.d \
./Applications/CANZero_static/Src/cz_processRx.d \
./Applications/CANZero_static/Src/cz_receive_queue.d \
./Applications/CANZero_static/Src/cz_receive_task.d \
./Applications/CANZero_static/Src/cz_send_queue.d \
./Applications/CANZero_static/Src/cz_send_task.d \
./Applications/CANZero_static/Src/cz_statemaschine.d \
./Applications/CANZero_static/Src/cz_taskmaster.d \
./Applications/CANZero_static/Src/cz_weak.d 


# Each subdirectory must supply rules for building sources it contributes
Applications/CANZero_static/Src/%.o Applications/CANZero_static/Src/%.su Applications/CANZero_static/Src/%.cyclo: ../Applications/CANZero_static/Src/%.cpp Applications/CANZero_static/Src/subdir.mk
	arm-none-eabi-g++ "$<" -mcpu=cortex-m4 -std=gnu++14 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32F407xx -c -I../Core/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc/Legacy -I../Middlewares/Third_Party/FreeRTOS/Source/include -I../Middlewares/Third_Party/FreeRTOS/Source/CMSIS_RTOS_V2 -I../Middlewares/Third_Party/FreeRTOS/Source/portable/GCC/ARM_CM4F -I../Drivers/CMSIS/Device/ST/STM32F4xx/Include -I../Drivers/CMSIS/Include -I"C:/Users/OfficeLaptop/Documents/GitHub/mz_testbench_siii/brake_ecu/Applications/Inc" -I"C:/Users/OfficeLaptop/Documents/GitHub/mz_testbench_siii/brake_ecu/Lib/Inc" -I"C:/Users/OfficeLaptop/Documents/GitHub/mz_testbench_siii/brake_ecu/Applications/CANzero/dbc-cpp" -I"C:/Users/OfficeLaptop/Documents/GitHub/mz_testbench_siii/brake_ecu/Applications/CANzero/include" -I"C:/Users/OfficeLaptop/Documents/GitHub/mz_testbench_siii/brake_ecu/Applications/CANZero_static/Inc" -O0 -ffunction-sections -fdata-sections -fno-exceptions -fno-rtti -fno-use-cxa-atexit -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-Applications-2f-CANZero_static-2f-Src

clean-Applications-2f-CANZero_static-2f-Src:
	-$(RM) ./Applications/CANZero_static/Src/cz_emergency.cyclo ./Applications/CANZero_static/Src/cz_emergency.d ./Applications/CANZero_static/Src/cz_emergency.o ./Applications/CANZero_static/Src/cz_emergency.su ./Applications/CANZero_static/Src/cz_handles.cyclo ./Applications/CANZero_static/Src/cz_handles.d ./Applications/CANZero_static/Src/cz_handles.o ./Applications/CANZero_static/Src/cz_handles.su ./Applications/CANZero_static/Src/cz_heartbeat.cyclo ./Applications/CANZero_static/Src/cz_heartbeat.d ./Applications/CANZero_static/Src/cz_heartbeat.o ./Applications/CANZero_static/Src/cz_heartbeat.su ./Applications/CANZero_static/Src/cz_message_receiver.cyclo ./Applications/CANZero_static/Src/cz_message_receiver.d ./Applications/CANZero_static/Src/cz_message_receiver.o ./Applications/CANZero_static/Src/cz_message_receiver.su ./Applications/CANZero_static/Src/cz_processRx.cyclo ./Applications/CANZero_static/Src/cz_processRx.d ./Applications/CANZero_static/Src/cz_processRx.o ./Applications/CANZero_static/Src/cz_processRx.su ./Applications/CANZero_static/Src/cz_receive_queue.cyclo ./Applications/CANZero_static/Src/cz_receive_queue.d ./Applications/CANZero_static/Src/cz_receive_queue.o ./Applications/CANZero_static/Src/cz_receive_queue.su ./Applications/CANZero_static/Src/cz_receive_task.cyclo ./Applications/CANZero_static/Src/cz_receive_task.d ./Applications/CANZero_static/Src/cz_receive_task.o ./Applications/CANZero_static/Src/cz_receive_task.su ./Applications/CANZero_static/Src/cz_send_queue.cyclo ./Applications/CANZero_static/Src/cz_send_queue.d ./Applications/CANZero_static/Src/cz_send_queue.o ./Applications/CANZero_static/Src/cz_send_queue.su ./Applications/CANZero_static/Src/cz_send_task.cyclo ./Applications/CANZero_static/Src/cz_send_task.d ./Applications/CANZero_static/Src/cz_send_task.o ./Applications/CANZero_static/Src/cz_send_task.su ./Applications/CANZero_static/Src/cz_statemaschine.cyclo ./Applications/CANZero_static/Src/cz_statemaschine.d ./Applications/CANZero_static/Src/cz_statemaschine.o ./Applications/CANZero_static/Src/cz_statemaschine.su ./Applications/CANZero_static/Src/cz_taskmaster.cyclo ./Applications/CANZero_static/Src/cz_taskmaster.d ./Applications/CANZero_static/Src/cz_taskmaster.o ./Applications/CANZero_static/Src/cz_taskmaster.su ./Applications/CANZero_static/Src/cz_weak.cyclo ./Applications/CANZero_static/Src/cz_weak.d ./Applications/CANZero_static/Src/cz_weak.o ./Applications/CANZero_static/Src/cz_weak.su

.PHONY: clean-Applications-2f-CANZero_static-2f-Src

