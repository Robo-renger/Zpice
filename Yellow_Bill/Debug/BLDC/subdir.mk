################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (12.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../BLDC/BLDC_program.c 

OBJS += \
./BLDC/BLDC_program.o 

C_DEPS += \
./BLDC/BLDC_program.d 


# Each subdirectory must supply rules for building sources it contributes
BLDC/%.o BLDC/%.su BLDC/%.cyclo: ../BLDC/%.c BLDC/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32F401xC -c -I../Core/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32F4xx/Include -I../Drivers/CMSIS/Include -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-BLDC

clean-BLDC:
	-$(RM) ./BLDC/BLDC_program.cyclo ./BLDC/BLDC_program.d ./BLDC/BLDC_program.o ./BLDC/BLDC_program.su

.PHONY: clean-BLDC

