################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (14.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
CPP_SRCS += \
../Core/Src/motor_drivers/BLDCDriver3PWM.cpp 

OBJS += \
./Core/Src/motor_drivers/BLDCDriver3PWM.o 

CPP_DEPS += \
./Core/Src/motor_drivers/BLDCDriver3PWM.d 


# Each subdirectory must supply rules for building sources it contributes
Core/Src/motor_drivers/%.o Core/Src/motor_drivers/%.su Core/Src/motor_drivers/%.cyclo: ../Core/Src/motor_drivers/%.cpp Core/Src/motor_drivers/subdir.mk
	arm-none-eabi-g++ "$<" -mcpu=cortex-m3 -std=gnu++14 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32F103xE -c -I../Core/Inc -I"D:/work/my_stabilization_project/storm_stab/stab_controller/Core/Src/motor_drivers" -I"D:/work/my_stabilization_project/storm_stab/stab_controller/Core/Src/common/base_classes" -I"D:/work/my_stabilization_project/storm_stab/stab_controller/Core/Src/common" -I../Drivers/STM32F1xx_HAL_Driver/Inc/Legacy -I../Drivers/STM32F1xx_HAL_Driver/Inc -I../Middlewares/Third_Party/FreeRTOS/Source/include -I../Middlewares/Third_Party/FreeRTOS/Source/CMSIS_RTOS_V2 -I../Middlewares/Third_Party/FreeRTOS/Source/portable/GCC/ARM_CM3 -I../Drivers/CMSIS/Device/ST/STM32F1xx/Include -I../Drivers/CMSIS/Include -O0 -ffunction-sections -fdata-sections -fno-exceptions -fno-rtti -fno-use-cxa-atexit -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfloat-abi=soft -mthumb -o "$@"

clean: clean-Core-2f-Src-2f-motor_drivers

clean-Core-2f-Src-2f-motor_drivers:
	-$(RM) ./Core/Src/motor_drivers/BLDCDriver3PWM.cyclo ./Core/Src/motor_drivers/BLDCDriver3PWM.d ./Core/Src/motor_drivers/BLDCDriver3PWM.o ./Core/Src/motor_drivers/BLDCDriver3PWM.su

.PHONY: clean-Core-2f-Src-2f-motor_drivers

