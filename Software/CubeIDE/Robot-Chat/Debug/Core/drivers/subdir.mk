################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (13.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Core/drivers/encodeur.c \
../Core/drivers/motors.c \
../Core/drivers/odometry.c 

OBJS += \
./Core/drivers/encodeur.o \
./Core/drivers/motors.o \
./Core/drivers/odometry.o 

C_DEPS += \
./Core/drivers/encodeur.d \
./Core/drivers/motors.d \
./Core/drivers/odometry.d 


# Each subdirectory must supply rules for building sources it contributes
Core/drivers/%.o Core/drivers/%.su Core/drivers/%.cyclo: ../Core/drivers/%.c Core/drivers/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32WB55xx -c -I../Core/Inc -I../Drivers/STM32WBxx_HAL_Driver/Inc -I../Drivers/STM32WBxx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32WBxx/Include -I../Drivers/CMSIS/Include -I../Middlewares/Third_Party/FreeRTOS/Source/include -I../Middlewares/Third_Party/FreeRTOS/Source/CMSIS_RTOS -I../Middlewares/Third_Party/FreeRTOS/Source/portable/GCC/ARM_CM4F -I../Drivers/CMSIS/RTOS2/Include -I"C:/Users/mbeng/Documents/ENSEA_3e_ESE/Projet_ESE/Projet_Robot_Chat/Software/CubeIDE/Robot-Chat/Core/drivers" -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-Core-2f-drivers

clean-Core-2f-drivers:
	-$(RM) ./Core/drivers/encodeur.cyclo ./Core/drivers/encodeur.d ./Core/drivers/encodeur.o ./Core/drivers/encodeur.su ./Core/drivers/motors.cyclo ./Core/drivers/motors.d ./Core/drivers/motors.o ./Core/drivers/motors.su ./Core/drivers/odometry.cyclo ./Core/drivers/odometry.d ./Core/drivers/odometry.o ./Core/drivers/odometry.su

.PHONY: clean-Core-2f-drivers

