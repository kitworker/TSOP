################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../src/RC5_IR_Emul_Receiver.c \
../src/main.c \
../src/stm3210b_eval.c \
../src/stm32f10x_it.c 

OBJS += \
./src/RC5_IR_Emul_Receiver.o \
./src/main.o \
./src/stm3210b_eval.o \
./src/stm32f10x_it.o 

C_DEPS += \
./src/RC5_IR_Emul_Receiver.d \
./src/main.d \
./src/stm3210b_eval.d \
./src/stm32f10x_it.d 


# Each subdirectory must supply rules for building sources it contributes
src/%.o: ../src/%.c
	@echo 'Building file: $<'
	@echo 'Invoking: ARM Sourcery Windows GCC C Compiler'
	arm-none-eabi-gcc -DSTM32F10X_LD_VL -DUSE_FULL_ASSERT -DUSE_STM3210B_EVAL -DUSE_STDPERIPH_DRIVER -I"C:\Users\PC108\Export_project\stm32_ld_vl\CMSIS" -I"C:\Users\PC108\Export_project\stm32_ld_vl\inc" -I"C:\Users\PC108\Export_project\stm32_ld_vl\SPL\inc" -O0 -Wall -Wa,-adhlns="$@.lst" -c -fmessage-length=0 -MMD -MP -MF"$(@:%.o=%.d)" -MT"$(@:%.o=%.d)" -mcpu=cortex-m3 -mthumb -g3 -gdwarf-2 -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '


