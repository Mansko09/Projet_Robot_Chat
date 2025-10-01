################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (11.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../VL53L1X_ULD_API/core/VL53L1X_api.c \
../VL53L1X_ULD_API/core/VL53L1X_calibration.c 

OBJS += \
./VL53L1X_ULD_API/core/VL53L1X_api.o \
./VL53L1X_ULD_API/core/VL53L1X_calibration.o 

C_DEPS += \
./VL53L1X_ULD_API/core/VL53L1X_api.d \
./VL53L1X_ULD_API/core/VL53L1X_calibration.d 


# Each subdirectory must supply rules for building sources it contributes
VL53L1X_ULD_API/core/%.o VL53L1X_ULD_API/core/%.su VL53L1X_ULD_API/core/%.cyclo: ../VL53L1X_ULD_API/core/%.c VL53L1X_ULD_API/core/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32WB55xx -c -I../VL53L1X_ULD_API/platform -I../VL53L1X_ULD_API/core -I../Core/Inc -I../Drivers/STM32WBxx_HAL_Driver/Inc -I../Drivers/STM32WBxx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32WBxx/Include -I../Drivers/CMSIS/Include -I"C:/Users/david/OneDrive/Bureau/Projet_ESE/Projet_Robot_Chat/Software/Core/Src" -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-VL53L1X_ULD_API-2f-core

clean-VL53L1X_ULD_API-2f-core:
	-$(RM) ./VL53L1X_ULD_API/core/VL53L1X_api.cyclo ./VL53L1X_ULD_API/core/VL53L1X_api.d ./VL53L1X_ULD_API/core/VL53L1X_api.o ./VL53L1X_ULD_API/core/VL53L1X_api.su ./VL53L1X_ULD_API/core/VL53L1X_calibration.cyclo ./VL53L1X_ULD_API/core/VL53L1X_calibration.d ./VL53L1X_ULD_API/core/VL53L1X_calibration.o ./VL53L1X_ULD_API/core/VL53L1X_calibration.su

.PHONY: clean-VL53L1X_ULD_API-2f-core

