################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (13.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../myLib/myDiskio/src/myDiskio.c 

OBJS += \
./myLib/myDiskio/src/myDiskio.o 

C_DEPS += \
./myLib/myDiskio/src/myDiskio.d 


# Each subdirectory must supply rules for building sources it contributes
myLib/myDiskio/src/%.o myLib/myDiskio/src/%.su myLib/myDiskio/src/%.cyclo: ../myLib/myDiskio/src/%.c myLib/myDiskio/src/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32F405xx -c -I../Core/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32F4xx/Include -I../Drivers/CMSIS/Include -I../USB_DEVICE/App -I../USB_DEVICE/Target -I../Middlewares/ST/STM32_USB_Device_Library/Core/Inc -I../Middlewares/ST/STM32_USB_Device_Library/Class/CDC/Inc -I../FATFS/Target -I../FATFS/App -I../Middlewares/Third_Party/FatFs/src -I"/FIRM/myLib/myDiskio/inc" -I"/FIRM/myLib/SD/inc" -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-myLib-2f-myDiskio-2f-src

clean-myLib-2f-myDiskio-2f-src:
	-$(RM) ./myLib/myDiskio/src/myDiskio.cyclo ./myLib/myDiskio/src/myDiskio.d ./myLib/myDiskio/src/myDiskio.o ./myLib/myDiskio/src/myDiskio.su

.PHONY: clean-myLib-2f-myDiskio-2f-src

