################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (13.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../myLib/SD/src/SD.c 

OBJS += \
./myLib/SD/src/SD.o 

C_DEPS += \
./myLib/SD/src/SD.d 


# Each subdirectory must supply rules for building sources it contributes
myLib/SD/src/%.o myLib/SD/src/%.su myLib/SD/src/%.cyclo: ../myLib/SD/src/%.c myLib/SD/src/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32F405xx -c -I../Core/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32F4xx/Include -I../Drivers/CMSIS/Include -I../USB_DEVICE/App -I../USB_DEVICE/Target -I../Middlewares/ST/STM32_USB_Device_Library/Core/Inc -I../Middlewares/ST/STM32_USB_Device_Library/Class/CDC/Inc -I../FATFS/Target -I../FATFS/App -I../Middlewares/Third_Party/FatFs/src -I"/FIRM/myLib/myDiskio/inc" -I"/FIRM/myLib/SD/inc" -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-myLib-2f-SD-2f-src

clean-myLib-2f-SD-2f-src:
	-$(RM) ./myLib/SD/src/SD.cyclo ./myLib/SD/src/SD.d ./myLib/SD/src/SD.o ./myLib/SD/src/SD.su

.PHONY: clean-myLib-2f-SD-2f-src

