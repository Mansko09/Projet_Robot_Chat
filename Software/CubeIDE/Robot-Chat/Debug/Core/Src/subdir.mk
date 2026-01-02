################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (11.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Core/Src/ADXL.c \
../Core/Src/TOF_Control.c \
../Core/Src/app_freertos.c \
../Core/Src/dma.c \
../Core/Src/gpio.c \
../Core/Src/i2c-mux.c \
../Core/Src/i2c.c \
../Core/Src/lptim.c \
../Core/Src/main.c \
../Core/Src/memorymap.c \
../Core/Src/stm32wbxx_hal_msp.c \
../Core/Src/stm32wbxx_hal_timebase_tim.c \
../Core/Src/stm32wbxx_it.c \
../Core/Src/syscalls.c \
../Core/Src/sysmem.c \
../Core/Src/system_stm32wbxx.c \
../Core/Src/tim.c \
../Core/Src/usart.c 

OBJS += \
./Core/Src/ADXL.o \
./Core/Src/TOF_Control.o \
./Core/Src/app_freertos.o \
./Core/Src/dma.o \
./Core/Src/gpio.o \
./Core/Src/i2c-mux.o \
./Core/Src/i2c.o \
./Core/Src/lptim.o \
./Core/Src/main.o \
./Core/Src/memorymap.o \
./Core/Src/stm32wbxx_hal_msp.o \
./Core/Src/stm32wbxx_hal_timebase_tim.o \
./Core/Src/stm32wbxx_it.o \
./Core/Src/syscalls.o \
./Core/Src/sysmem.o \
./Core/Src/system_stm32wbxx.o \
./Core/Src/tim.o \
./Core/Src/usart.o 

C_DEPS += \
./Core/Src/ADXL.d \
./Core/Src/TOF_Control.d \
./Core/Src/app_freertos.d \
./Core/Src/dma.d \
./Core/Src/gpio.d \
./Core/Src/i2c-mux.d \
./Core/Src/i2c.d \
./Core/Src/lptim.d \
./Core/Src/main.d \
./Core/Src/memorymap.d \
./Core/Src/stm32wbxx_hal_msp.d \
./Core/Src/stm32wbxx_hal_timebase_tim.d \
./Core/Src/stm32wbxx_it.d \
./Core/Src/syscalls.d \
./Core/Src/sysmem.d \
./Core/Src/system_stm32wbxx.d \
./Core/Src/tim.d \
./Core/Src/usart.d 


# Each subdirectory must supply rules for building sources it contributes
Core/Src/%.o Core/Src/%.su Core/Src/%.cyclo: ../Core/Src/%.c Core/Src/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32WB55xx -c -I../Core/Inc -I../VL53L1X_ULD_API/core -I../VL53L1X_ULD_API/platform -I../Drivers/STM32WBxx_HAL_Driver/Inc -I../Drivers/STM32WBxx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32WBxx/Include -I../Drivers/CMSIS/Include -I../Middlewares/Third_Party/FreeRTOS/Source/include -I../Middlewares/Third_Party/FreeRTOS/Source/CMSIS_RTOS -I../Middlewares/Third_Party/FreeRTOS/Source/portable/GCC/ARM_CM4F -I"C:/Users/david/OneDrive/Bureau/Projet_ESE/Projet_Robot_Chat/Software/VL53L0X_API" -I"C:/Users/david/OneDrive/Bureau/Projet_ESE/Projet_Robot_Chat/Software/VL53L0X_API/platform" -I"C:/Users/david/OneDrive/Bureau/Projet_ESE/Projet_Robot_Chat/Software/VL53L0X_API/core" -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-Core-2f-Src

clean-Core-2f-Src:
	-$(RM) ./Core/Src/ADXL.cyclo ./Core/Src/ADXL.d ./Core/Src/ADXL.o ./Core/Src/ADXL.su ./Core/Src/TOF_Control.cyclo ./Core/Src/TOF_Control.d ./Core/Src/TOF_Control.o ./Core/Src/TOF_Control.su ./Core/Src/app_freertos.cyclo ./Core/Src/app_freertos.d ./Core/Src/app_freertos.o ./Core/Src/app_freertos.su ./Core/Src/dma.cyclo ./Core/Src/dma.d ./Core/Src/dma.o ./Core/Src/dma.su ./Core/Src/gpio.cyclo ./Core/Src/gpio.d ./Core/Src/gpio.o ./Core/Src/gpio.su ./Core/Src/i2c-mux.cyclo ./Core/Src/i2c-mux.d ./Core/Src/i2c-mux.o ./Core/Src/i2c-mux.su ./Core/Src/i2c.cyclo ./Core/Src/i2c.d ./Core/Src/i2c.o ./Core/Src/i2c.su ./Core/Src/lptim.cyclo ./Core/Src/lptim.d ./Core/Src/lptim.o ./Core/Src/lptim.su ./Core/Src/main.cyclo ./Core/Src/main.d ./Core/Src/main.o ./Core/Src/main.su ./Core/Src/memorymap.cyclo ./Core/Src/memorymap.d ./Core/Src/memorymap.o ./Core/Src/memorymap.su ./Core/Src/stm32wbxx_hal_msp.cyclo ./Core/Src/stm32wbxx_hal_msp.d ./Core/Src/stm32wbxx_hal_msp.o ./Core/Src/stm32wbxx_hal_msp.su ./Core/Src/stm32wbxx_hal_timebase_tim.cyclo ./Core/Src/stm32wbxx_hal_timebase_tim.d ./Core/Src/stm32wbxx_hal_timebase_tim.o ./Core/Src/stm32wbxx_hal_timebase_tim.su ./Core/Src/stm32wbxx_it.cyclo ./Core/Src/stm32wbxx_it.d ./Core/Src/stm32wbxx_it.o ./Core/Src/stm32wbxx_it.su ./Core/Src/syscalls.cyclo ./Core/Src/syscalls.d ./Core/Src/syscalls.o ./Core/Src/syscalls.su ./Core/Src/sysmem.cyclo ./Core/Src/sysmem.d ./Core/Src/sysmem.o ./Core/Src/sysmem.su ./Core/Src/system_stm32wbxx.cyclo ./Core/Src/system_stm32wbxx.d ./Core/Src/system_stm32wbxx.o ./Core/Src/system_stm32wbxx.su ./Core/Src/tim.cyclo ./Core/Src/tim.d ./Core/Src/tim.o ./Core/Src/tim.su ./Core/Src/usart.cyclo ./Core/Src/usart.d ./Core/Src/usart.o ./Core/Src/usart.su

.PHONY: clean-Core-2f-Src

