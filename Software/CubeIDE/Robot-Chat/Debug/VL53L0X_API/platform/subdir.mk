################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (11.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../VL53L0X_API/platform/vl53l0x_platform.c \
../VL53L0X_API/platform/vl53l0x_platform_log.c 

OBJS += \
./VL53L0X_API/platform/vl53l0x_platform.o \
./VL53L0X_API/platform/vl53l0x_platform_log.o 

C_DEPS += \
./VL53L0X_API/platform/vl53l0x_platform.d \
./VL53L0X_API/platform/vl53l0x_platform_log.d 


# Each subdirectory must supply rules for building sources it contributes
VL53L0X_API/platform/%.o VL53L0X_API/platform/%.su VL53L0X_API/platform/%.cyclo: ../VL53L0X_API/platform/%.c VL53L0X_API/platform/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32WB55xx -c -I../Core/Inc -I../VL53L1X_ULD_API/core -I../VL53L1X_ULD_API/platform -I../Drivers/STM32WBxx_HAL_Driver/Inc -I../Drivers/STM32WBxx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32WBxx/Include -I../Drivers/CMSIS/Include -I../Middlewares/Third_Party/FreeRTOS/Source/include -I../Middlewares/Third_Party/FreeRTOS/Source/CMSIS_RTOS -I../Middlewares/Third_Party/FreeRTOS/Source/portable/GCC/ARM_CM4F -I"C:/Users/david/OneDrive/Bureau/Projet_ESE/Projet_Robot_Chat/Software/VL53L0X_API/core" -I"C:/Users/david/OneDrive/Bureau/Projet_ESE/Projet_Robot_Chat/Software/VL53L0X_API/platform" -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-VL53L0X_API-2f-platform

clean-VL53L0X_API-2f-platform:
	-$(RM) ./VL53L0X_API/platform/vl53l0x_platform.cyclo ./VL53L0X_API/platform/vl53l0x_platform.d ./VL53L0X_API/platform/vl53l0x_platform.o ./VL53L0X_API/platform/vl53l0x_platform.su ./VL53L0X_API/platform/vl53l0x_platform_log.cyclo ./VL53L0X_API/platform/vl53l0x_platform_log.d ./VL53L0X_API/platform/vl53l0x_platform_log.o ./VL53L0X_API/platform/vl53l0x_platform_log.su

.PHONY: clean-VL53L0X_API-2f-platform

