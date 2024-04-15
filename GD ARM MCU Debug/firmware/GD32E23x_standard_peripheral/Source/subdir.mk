################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../firmware/GD32E23x_standard_peripheral/Source/gd32e23x_adc.c \
../firmware/GD32E23x_standard_peripheral/Source/gd32e23x_cmp.c \
../firmware/GD32E23x_standard_peripheral/Source/gd32e23x_crc.c \
../firmware/GD32E23x_standard_peripheral/Source/gd32e23x_dbg.c \
../firmware/GD32E23x_standard_peripheral/Source/gd32e23x_dma.c \
../firmware/GD32E23x_standard_peripheral/Source/gd32e23x_exti.c \
../firmware/GD32E23x_standard_peripheral/Source/gd32e23x_fmc.c \
../firmware/GD32E23x_standard_peripheral/Source/gd32e23x_fwdgt.c \
../firmware/GD32E23x_standard_peripheral/Source/gd32e23x_gpio.c \
../firmware/GD32E23x_standard_peripheral/Source/gd32e23x_i2c.c \
../firmware/GD32E23x_standard_peripheral/Source/gd32e23x_misc.c \
../firmware/GD32E23x_standard_peripheral/Source/gd32e23x_pmu.c \
../firmware/GD32E23x_standard_peripheral/Source/gd32e23x_rcu.c \
../firmware/GD32E23x_standard_peripheral/Source/gd32e23x_rtc.c \
../firmware/GD32E23x_standard_peripheral/Source/gd32e23x_spi.c \
../firmware/GD32E23x_standard_peripheral/Source/gd32e23x_syscfg.c \
../firmware/GD32E23x_standard_peripheral/Source/gd32e23x_timer.c \
../firmware/GD32E23x_standard_peripheral/Source/gd32e23x_usart.c \
../firmware/GD32E23x_standard_peripheral/Source/gd32e23x_wwdgt.c 

OBJS += \
./firmware/GD32E23x_standard_peripheral/Source/gd32e23x_adc.o \
./firmware/GD32E23x_standard_peripheral/Source/gd32e23x_cmp.o \
./firmware/GD32E23x_standard_peripheral/Source/gd32e23x_crc.o \
./firmware/GD32E23x_standard_peripheral/Source/gd32e23x_dbg.o \
./firmware/GD32E23x_standard_peripheral/Source/gd32e23x_dma.o \
./firmware/GD32E23x_standard_peripheral/Source/gd32e23x_exti.o \
./firmware/GD32E23x_standard_peripheral/Source/gd32e23x_fmc.o \
./firmware/GD32E23x_standard_peripheral/Source/gd32e23x_fwdgt.o \
./firmware/GD32E23x_standard_peripheral/Source/gd32e23x_gpio.o \
./firmware/GD32E23x_standard_peripheral/Source/gd32e23x_i2c.o \
./firmware/GD32E23x_standard_peripheral/Source/gd32e23x_misc.o \
./firmware/GD32E23x_standard_peripheral/Source/gd32e23x_pmu.o \
./firmware/GD32E23x_standard_peripheral/Source/gd32e23x_rcu.o \
./firmware/GD32E23x_standard_peripheral/Source/gd32e23x_rtc.o \
./firmware/GD32E23x_standard_peripheral/Source/gd32e23x_spi.o \
./firmware/GD32E23x_standard_peripheral/Source/gd32e23x_syscfg.o \
./firmware/GD32E23x_standard_peripheral/Source/gd32e23x_timer.o \
./firmware/GD32E23x_standard_peripheral/Source/gd32e23x_usart.o \
./firmware/GD32E23x_standard_peripheral/Source/gd32e23x_wwdgt.o 

C_DEPS += \
./firmware/GD32E23x_standard_peripheral/Source/gd32e23x_adc.d \
./firmware/GD32E23x_standard_peripheral/Source/gd32e23x_cmp.d \
./firmware/GD32E23x_standard_peripheral/Source/gd32e23x_crc.d \
./firmware/GD32E23x_standard_peripheral/Source/gd32e23x_dbg.d \
./firmware/GD32E23x_standard_peripheral/Source/gd32e23x_dma.d \
./firmware/GD32E23x_standard_peripheral/Source/gd32e23x_exti.d \
./firmware/GD32E23x_standard_peripheral/Source/gd32e23x_fmc.d \
./firmware/GD32E23x_standard_peripheral/Source/gd32e23x_fwdgt.d \
./firmware/GD32E23x_standard_peripheral/Source/gd32e23x_gpio.d \
./firmware/GD32E23x_standard_peripheral/Source/gd32e23x_i2c.d \
./firmware/GD32E23x_standard_peripheral/Source/gd32e23x_misc.d \
./firmware/GD32E23x_standard_peripheral/Source/gd32e23x_pmu.d \
./firmware/GD32E23x_standard_peripheral/Source/gd32e23x_rcu.d \
./firmware/GD32E23x_standard_peripheral/Source/gd32e23x_rtc.d \
./firmware/GD32E23x_standard_peripheral/Source/gd32e23x_spi.d \
./firmware/GD32E23x_standard_peripheral/Source/gd32e23x_syscfg.d \
./firmware/GD32E23x_standard_peripheral/Source/gd32e23x_timer.d \
./firmware/GD32E23x_standard_peripheral/Source/gd32e23x_usart.d \
./firmware/GD32E23x_standard_peripheral/Source/gd32e23x_wwdgt.d 


# Each subdirectory must supply rules for building sources it contributes
firmware/GD32E23x_standard_peripheral/Source/%.o: ../firmware/GD32E23x_standard_peripheral/Source/%.c
	@echo 'Building file: $<'
	@echo 'Invoking: GD ARM MCU C Compiler'
	arm-none-eabi-gcc -mcpu=cortex-m23 -mthumb -O0 -fmessage-length=0 -fsigned-char -ffunction-sections -fdata-sections  -g3 -std=gnu11 -DGD32E230 -I"../inc" -I"../firmware/cmsis/inc" -I"../firmware/GD32E23x_standard_peripheral/Include" -I"../firmware/GD32E23x_hal_peripheral/Include" -MMD -MP -MF"$(@:%.o=%.d)" -MT"$(@)" -Wa,-adhlns=$@.lst   -c -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '


