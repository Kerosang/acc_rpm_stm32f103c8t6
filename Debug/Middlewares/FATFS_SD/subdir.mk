################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (11.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Middlewares/FATFS_SD/FATFS_SD.c 

OBJS += \
./Middlewares/FATFS_SD/FATFS_SD.o 

C_DEPS += \
./Middlewares/FATFS_SD/FATFS_SD.d 


# Each subdirectory must supply rules for building sources it contributes
Middlewares/FATFS_SD/%.o Middlewares/FATFS_SD/%.su Middlewares/FATFS_SD/%.cyclo: ../Middlewares/FATFS_SD/%.c Middlewares/FATFS_SD/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m3 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32F103xB -c -I../USB_DEVICE/App -I../USB_DEVICE/Target -I../Core/Inc -I../Drivers/STM32F1xx_HAL_Driver/Inc -I../Drivers/STM32F1xx_HAL_Driver/Inc/Legacy -I../Middlewares/ST/STM32_USB_Device_Library/Core/Inc -I../Middlewares/ST/STM32_USB_Device_Library/Class/CDC/Inc -I../Drivers/CMSIS/Device/ST/STM32F1xx/Include -I../Drivers/CMSIS/Include -I../FATFS/Target -I../FATFS/App -I../Middlewares/Third_Party/FatFs/src -I"C:/Users/chatc/Desktop/github/acc_rpm_stm32f103c8t6/acc_rpm_stm32f103c8t6/Middlewares/FATFS_SD" -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfloat-abi=soft -mthumb -o "$@"

clean: clean-Middlewares-2f-FATFS_SD

clean-Middlewares-2f-FATFS_SD:
	-$(RM) ./Middlewares/FATFS_SD/FATFS_SD.cyclo ./Middlewares/FATFS_SD/FATFS_SD.d ./Middlewares/FATFS_SD/FATFS_SD.o ./Middlewares/FATFS_SD/FATFS_SD.su

.PHONY: clean-Middlewares-2f-FATFS_SD

