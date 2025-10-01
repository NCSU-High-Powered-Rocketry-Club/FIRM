################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (13.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Core/Src/bmp581_spi.c \
../Core/Src/firm_utils.c \
../Core/Src/i2c_utils.c \
../Core/Src/icm45686.c \
../Core/Src/logger.c \
../Core/Src/main.c \
../Core/Src/mmc5983ma.c \
../Core/Src/spi_utils.c \
../Core/Src/stm32f4xx_hal_msp.c \
../Core/Src/stm32f4xx_it.c \
../Core/Src/syscalls.c \
../Core/Src/sysmem.c \
../Core/Src/system_stm32f4xx.c \
../Core/Src/usb_print_debug.c 

OBJS += \
./Core/Src/bmp581_spi.o \
./Core/Src/firm_utils.o \
./Core/Src/i2c_utils.o \
./Core/Src/icm45686.o \
./Core/Src/logger.o \
./Core/Src/main.o \
./Core/Src/mmc5983ma.o \
./Core/Src/spi_utils.o \
./Core/Src/stm32f4xx_hal_msp.o \
./Core/Src/stm32f4xx_it.o \
./Core/Src/syscalls.o \
./Core/Src/sysmem.o \
./Core/Src/system_stm32f4xx.o \
./Core/Src/usb_print_debug.o 

C_DEPS += \
./Core/Src/bmp581_spi.d \
./Core/Src/firm_utils.d \
./Core/Src/i2c_utils.d \
./Core/Src/icm45686.d \
./Core/Src/logger.d \
./Core/Src/main.d \
./Core/Src/mmc5983ma.d \
./Core/Src/spi_utils.d \
./Core/Src/stm32f4xx_hal_msp.d \
./Core/Src/stm32f4xx_it.d \
./Core/Src/syscalls.d \
./Core/Src/sysmem.d \
./Core/Src/system_stm32f4xx.d \
./Core/Src/usb_print_debug.d 


# Each subdirectory must supply rules for building sources it contributes
Core/Src/%.o Core/Src/%.su Core/Src/%.cyclo: ../Core/Src/%.c Core/Src/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32F405xx -c -I../Core/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32F4xx/Include -I../Drivers/CMSIS/Include -I../FATFS/Target -I../FATFS/App -I../Middlewares/Third_Party/FatFs/src -I../USB_DEVICE/App -I../USB_DEVICE/Target -I../Middlewares/ST/STM32_USB_Device_Library/Core/Inc -I../Middlewares/ST/STM32_USB_Device_Library/Class/CDC/Inc -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-Core-2f-Src

clean-Core-2f-Src:
	-$(RM) ./Core/Src/bmp581_spi.cyclo ./Core/Src/bmp581_spi.d ./Core/Src/bmp581_spi.o ./Core/Src/bmp581_spi.su ./Core/Src/firm_utils.cyclo ./Core/Src/firm_utils.d ./Core/Src/firm_utils.o ./Core/Src/firm_utils.su ./Core/Src/i2c_utils.cyclo ./Core/Src/i2c_utils.d ./Core/Src/i2c_utils.o ./Core/Src/i2c_utils.su ./Core/Src/icm45686.cyclo ./Core/Src/icm45686.d ./Core/Src/icm45686.o ./Core/Src/icm45686.su ./Core/Src/logger.cyclo ./Core/Src/logger.d ./Core/Src/logger.o ./Core/Src/logger.su ./Core/Src/main.cyclo ./Core/Src/main.d ./Core/Src/main.o ./Core/Src/main.su ./Core/Src/mmc5983ma.cyclo ./Core/Src/mmc5983ma.d ./Core/Src/mmc5983ma.o ./Core/Src/mmc5983ma.su ./Core/Src/spi_utils.cyclo ./Core/Src/spi_utils.d ./Core/Src/spi_utils.o ./Core/Src/spi_utils.su ./Core/Src/stm32f4xx_hal_msp.cyclo ./Core/Src/stm32f4xx_hal_msp.d ./Core/Src/stm32f4xx_hal_msp.o ./Core/Src/stm32f4xx_hal_msp.su ./Core/Src/stm32f4xx_it.cyclo ./Core/Src/stm32f4xx_it.d ./Core/Src/stm32f4xx_it.o ./Core/Src/stm32f4xx_it.su ./Core/Src/syscalls.cyclo ./Core/Src/syscalls.d ./Core/Src/syscalls.o ./Core/Src/syscalls.su ./Core/Src/sysmem.cyclo ./Core/Src/sysmem.d ./Core/Src/sysmem.o ./Core/Src/sysmem.su ./Core/Src/system_stm32f4xx.cyclo ./Core/Src/system_stm32f4xx.d ./Core/Src/system_stm32f4xx.o ./Core/Src/system_stm32f4xx.su ./Core/Src/usb_print_debug.cyclo ./Core/Src/usb_print_debug.d ./Core/Src/usb_print_debug.o ./Core/Src/usb_print_debug.su

.PHONY: clean-Core-2f-Src

