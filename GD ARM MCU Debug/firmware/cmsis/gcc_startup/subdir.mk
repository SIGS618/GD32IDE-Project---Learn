################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
S_UPPER_SRCS += \
../firmware/cmsis/gcc_startup/startup_gd32e23x.S 

OBJS += \
./firmware/cmsis/gcc_startup/startup_gd32e23x.o 

S_UPPER_DEPS += \
./firmware/cmsis/gcc_startup/startup_gd32e23x.d 


# Each subdirectory must supply rules for building sources it contributes
firmware/cmsis/gcc_startup/%.o: ../firmware/cmsis/gcc_startup/%.S
	@echo 'Building file: $<'
	@echo 'Invoking: GD ARM MCU Assembler'
	arm-none-eabi-gcc -mcpu=cortex-m23 -mthumb -O0 -fmessage-length=0 -fsigned-char -ffunction-sections -fdata-sections  -g3 -x assembler-with-cpp -MMD -MP -MF"$(@:%.o=%.d)" -MT"$(@)" -Wa,-adhlns=$@.lst   -c -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '


