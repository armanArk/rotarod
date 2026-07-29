################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (13.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Core/Src/74hc165.c \
../Core/Src/TM1637.c \
../Core/Src/cli.c \
../Core/Src/ds3231.c \
../Core/Src/fs_logger.c \
../Core/Src/main.c \
../Core/Src/motor_control.c \
../Core/Src/settings.c \
../Core/Src/staging.c \
../Core/Src/stm32f4xx_hal_msp.c \
../Core/Src/stm32f4xx_it.c \
../Core/Src/syscalls.c \
../Core/Src/sysmem.c \
../Core/Src/system_stm32f4xx.c \
../Core/Src/ui.c \
../Core/Src/w25q64.c 

OBJS += \
./Core/Src/74hc165.o \
./Core/Src/TM1637.o \
./Core/Src/cli.o \
./Core/Src/ds3231.o \
./Core/Src/fs_logger.o \
./Core/Src/main.o \
./Core/Src/motor_control.o \
./Core/Src/settings.o \
./Core/Src/staging.o \
./Core/Src/stm32f4xx_hal_msp.o \
./Core/Src/stm32f4xx_it.o \
./Core/Src/syscalls.o \
./Core/Src/sysmem.o \
./Core/Src/system_stm32f4xx.o \
./Core/Src/ui.o \
./Core/Src/w25q64.o 

C_DEPS += \
./Core/Src/74hc165.d \
./Core/Src/TM1637.d \
./Core/Src/cli.d \
./Core/Src/ds3231.d \
./Core/Src/fs_logger.d \
./Core/Src/main.d \
./Core/Src/motor_control.d \
./Core/Src/settings.d \
./Core/Src/staging.d \
./Core/Src/stm32f4xx_hal_msp.d \
./Core/Src/stm32f4xx_it.d \
./Core/Src/syscalls.d \
./Core/Src/sysmem.d \
./Core/Src/system_stm32f4xx.d \
./Core/Src/ui.d \
./Core/Src/w25q64.d 


# Each subdirectory must supply rules for building sources it contributes
Core/Src/%.o Core/Src/%.su Core/Src/%.cyclo: ../Core/Src/%.c Core/Src/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32F401xC -c -I../Core/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32F4xx/Include -I../Drivers/CMSIS/Include -I../USB_DEVICE/App -I../USB_DEVICE/Target -I../Middlewares/ST/STM32_USB_Device_Library/Core/Inc -I../Middlewares/ST/STM32_USB_Device_Library/Class/MSC/Inc -I../FATFS/Target -I../FATFS/App -I../Middlewares/Third_Party/FatFs/src -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-Core-2f-Src

clean-Core-2f-Src:
	-$(RM) ./Core/Src/74hc165.cyclo ./Core/Src/74hc165.d ./Core/Src/74hc165.o ./Core/Src/74hc165.su ./Core/Src/TM1637.cyclo ./Core/Src/TM1637.d ./Core/Src/TM1637.o ./Core/Src/TM1637.su ./Core/Src/cli.cyclo ./Core/Src/cli.d ./Core/Src/cli.o ./Core/Src/cli.su ./Core/Src/ds3231.cyclo ./Core/Src/ds3231.d ./Core/Src/ds3231.o ./Core/Src/ds3231.su ./Core/Src/fs_logger.cyclo ./Core/Src/fs_logger.d ./Core/Src/fs_logger.o ./Core/Src/fs_logger.su ./Core/Src/main.cyclo ./Core/Src/main.d ./Core/Src/main.o ./Core/Src/main.su ./Core/Src/motor_control.cyclo ./Core/Src/motor_control.d ./Core/Src/motor_control.o ./Core/Src/motor_control.su ./Core/Src/settings.cyclo ./Core/Src/settings.d ./Core/Src/settings.o ./Core/Src/settings.su ./Core/Src/staging.cyclo ./Core/Src/staging.d ./Core/Src/staging.o ./Core/Src/staging.su ./Core/Src/stm32f4xx_hal_msp.cyclo ./Core/Src/stm32f4xx_hal_msp.d ./Core/Src/stm32f4xx_hal_msp.o ./Core/Src/stm32f4xx_hal_msp.su ./Core/Src/stm32f4xx_it.cyclo ./Core/Src/stm32f4xx_it.d ./Core/Src/stm32f4xx_it.o ./Core/Src/stm32f4xx_it.su ./Core/Src/syscalls.cyclo ./Core/Src/syscalls.d ./Core/Src/syscalls.o ./Core/Src/syscalls.su ./Core/Src/sysmem.cyclo ./Core/Src/sysmem.d ./Core/Src/sysmem.o ./Core/Src/sysmem.su ./Core/Src/system_stm32f4xx.cyclo ./Core/Src/system_stm32f4xx.d ./Core/Src/system_stm32f4xx.o ./Core/Src/system_stm32f4xx.su ./Core/Src/ui.cyclo ./Core/Src/ui.d ./Core/Src/ui.o ./Core/Src/ui.su ./Core/Src/w25q64.cyclo ./Core/Src/w25q64.d ./Core/Src/w25q64.o ./Core/Src/w25q64.su

.PHONY: clean-Core-2f-Src

