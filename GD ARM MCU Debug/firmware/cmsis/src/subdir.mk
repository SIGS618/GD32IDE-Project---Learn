################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../firmware/cmsis/src/syscalls.c \
../firmware/cmsis/src/system_gd32e23x.c 

OBJS += \
./firmware/cmsis/src/syscalls.o \
./firmware/cmsis/src/system_gd32e23x.o 

C_DEPS += \
./firmware/cmsis/src/syscalls.d \
./firmware/cmsis/src/system_gd32e23x.d 


# Each subdirectory must supply rules for building sources it contributes
firmware/cmsis/src/%.o: ../firmware/cmsis/src/%.c
	@echo 'Building file: $<'
	@echo 'Invoking: GD ARM MCU C Compiler'
	arm-none-eabi-gcc -mcpu=cortex-m23 -mthumb -O0 -fmessage-length=0 -fsigned-char -ffunction-sections -fdata-sections  -g3 -std=gnu11 -DGD32E230 -I"../inc" -I"../firmware/cmsis/inc" -I"../firmware/GD32E23x_standard_peripheral/Include" -I"../firmware/GD32E23x_hal_peripheral/Include" -MMD -MP -MF"$(@:%.o=%.d)" -MT"$(@)" -Wa,-adhlns=$@.lst   -c -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '


