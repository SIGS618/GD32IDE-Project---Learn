################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../src/gd32e23x_hal_init.c \
../src/gd32e23x_hal_it.c \
../src/main.c 

OBJS += \
./src/gd32e23x_hal_init.o \
./src/gd32e23x_hal_it.o \
./src/main.o 

C_DEPS += \
./src/gd32e23x_hal_init.d \
./src/gd32e23x_hal_it.d \
./src/main.d 


# Each subdirectory must supply rules for building sources it contributes
src/%.o: ../src/%.c
	@echo 'Building file: $<'
	@echo 'Invoking: GD ARM MCU C Compiler'
	arm-none-eabi-gcc -mcpu=cortex-m23 -mthumb -O0 -fmessage-length=0 -fsigned-char -ffunction-sections -fdata-sections  -g3 -std=gnu11 -DGD32E230 -I"../inc" -I"../firmware/cmsis/inc" -I"../firmware/GD32E23x_standard_peripheral/Include" -I"../firmware/GD32E23x_hal_peripheral/Include" -MMD -MP -MF"$(@:%.o=%.d)" -MT"$(@)" -Wa,-adhlns=$@.lst   -c -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '


