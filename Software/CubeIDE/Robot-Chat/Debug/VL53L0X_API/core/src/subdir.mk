################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (11.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../VL53L0X_API/core/src/vl53l0x_api.c \
../VL53L0X_API/core/src/vl53l0x_api_calibration.c \
../VL53L0X_API/core/src/vl53l0x_api_core.c \
../VL53L0X_API/core/src/vl53l0x_api_ranging.c \
../VL53L0X_API/core/src/vl53l0x_api_strings.c 

OBJS += \
./VL53L0X_API/core/src/vl53l0x_api.o \
./VL53L0X_API/core/src/vl53l0x_api_calibration.o \
./VL53L0X_API/core/src/vl53l0x_api_core.o \
./VL53L0X_API/core/src/vl53l0x_api_ranging.o \
./VL53L0X_API/core/src/vl53l0x_api_strings.o 

C_DEPS += \
./VL53L0X_API/core/src/vl53l0x_api.d \
./VL53L0X_API/core/src/vl53l0x_api_calibration.d \
./VL53L0X_API/core/src/vl53l0x_api_core.d \
./VL53L0X_API/core/src/vl53l0x_api_ranging.d \
./VL53L0X_API/core/src/vl53l0x_api_strings.d 


# Each subdirectory must supply rules for building sources it contributes
VL53L0X_API/core/src/%.o VL53L0X_API/core/src/%.su VL53L0X_API/core/src/%.cyclo: ../VL53L0X_API/core/src/%.c VL53L0X_API/core/src/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32WB55xx -c -I../Core/Inc -I"C:/Users/david/OneDrive/Bureau/Projet_ESE/Projet_Robot_Chat/Software/VL53L0X_API/platform/src" -I"C:/Users/david/OneDrive/Bureau/Projet_ESE/Projet_Robot_Chat/Software/VL53L0X_API/core/inc" -I"C:/Users/david/OneDrive/Bureau/Projet_ESE/Projet_Robot_Chat/Software/VL53L0X_API/core/src" -I"C:/Users/david/OneDrive/Bureau/Projet_ESE/Projet_Robot_Chat/Software/VL53L0X_API/platform/inc" -I../VL53L1X_ULD_API/core -I../VL53L1X_ULD_API/platform -I../Drivers/STM32WBxx_HAL_Driver/Inc -I../Drivers/STM32WBxx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32WBxx/Include -I../Drivers/CMSIS/Include -I../Middlewares/Third_Party/FreeRTOS/Source/include -I../Middlewares/Third_Party/FreeRTOS/Source/CMSIS_RTOS -I../Middlewares/Third_Party/FreeRTOS/Source/portable/GCC/ARM_CM4F -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-VL53L0X_API-2f-core-2f-src

clean-VL53L0X_API-2f-core-2f-src:
	-$(RM) ./VL53L0X_API/core/src/vl53l0x_api.cyclo ./VL53L0X_API/core/src/vl53l0x_api.d ./VL53L0X_API/core/src/vl53l0x_api.o ./VL53L0X_API/core/src/vl53l0x_api.su ./VL53L0X_API/core/src/vl53l0x_api_calibration.cyclo ./VL53L0X_API/core/src/vl53l0x_api_calibration.d ./VL53L0X_API/core/src/vl53l0x_api_calibration.o ./VL53L0X_API/core/src/vl53l0x_api_calibration.su ./VL53L0X_API/core/src/vl53l0x_api_core.cyclo ./VL53L0X_API/core/src/vl53l0x_api_core.d ./VL53L0X_API/core/src/vl53l0x_api_core.o ./VL53L0X_API/core/src/vl53l0x_api_core.su ./VL53L0X_API/core/src/vl53l0x_api_ranging.cyclo ./VL53L0X_API/core/src/vl53l0x_api_ranging.d ./VL53L0X_API/core/src/vl53l0x_api_ranging.o ./VL53L0X_API/core/src/vl53l0x_api_ranging.su ./VL53L0X_API/core/src/vl53l0x_api_strings.cyclo ./VL53L0X_API/core/src/vl53l0x_api_strings.d ./VL53L0X_API/core/src/vl53l0x_api_strings.o ./VL53L0X_API/core/src/vl53l0x_api_strings.su

.PHONY: clean-VL53L0X_API-2f-core-2f-src

