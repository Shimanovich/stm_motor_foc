################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (14.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
CPP_SRCS += \
../Core/Src/common/base_classes/FOCMotor.cpp 

OBJS += \
./Core/Src/common/base_classes/FOCMotor.o 

CPP_DEPS += \
./Core/Src/common/base_classes/FOCMotor.d 


# Each subdirectory must supply rules for building sources it contributes
Core/Src/common/base_classes/%.o Core/Src/common/base_classes/%.su Core/Src/common/base_classes/%.cyclo: ../Core/Src/common/base_classes/%.cpp Core/Src/common/base_classes/subdir.mk
	arm-none-eabi-g++ "$<" -mcpu=cortex-m3 -std=gnu++14 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32F103xE -c -I../Core/Inc -I"D:/work/my_stabilization_project/storm_stab/stab_controller/Core/Src/motor_drivers" -I"D:/work/my_stabilization_project/storm_stab/stab_controller/Core/Src/common/base_classes" -I"D:/work/my_stabilization_project/storm_stab/stab_controller/Core/Src/common" -I../Drivers/STM32F1xx_HAL_Driver/Inc/Legacy -I../Drivers/STM32F1xx_HAL_Driver/Inc -I../Middlewares/Third_Party/FreeRTOS/Source/include -I../Middlewares/Third_Party/FreeRTOS/Source/CMSIS_RTOS_V2 -I../Middlewares/Third_Party/FreeRTOS/Source/portable/GCC/ARM_CM3 -I../Drivers/CMSIS/Device/ST/STM32F1xx/Include -I../Drivers/CMSIS/Include -O0 -ffunction-sections -fdata-sections -fno-exceptions -fno-rtti -fno-use-cxa-atexit -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfloat-abi=soft -mthumb -o "$@"

clean: clean-Core-2f-Src-2f-common-2f-base_classes

clean-Core-2f-Src-2f-common-2f-base_classes:
	-$(RM) ./Core/Src/common/base_classes/FOCMotor.cyclo ./Core/Src/common/base_classes/FOCMotor.d ./Core/Src/common/base_classes/FOCMotor.o ./Core/Src/common/base_classes/FOCMotor.su

.PHONY: clean-Core-2f-Src-2f-common-2f-base_classes

