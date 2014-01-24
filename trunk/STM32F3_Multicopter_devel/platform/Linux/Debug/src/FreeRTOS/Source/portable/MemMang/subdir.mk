################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
/home/user/Desktop/STM32-Projekte/fh-multicopter/STM32F3_Multicopter_devel/src/FreeRTOS/Source/portable/MemMang/heap_1.c 

OBJS += \
./src/FreeRTOS/Source/portable/MemMang/heap_1.o 

C_DEPS += \
./src/FreeRTOS/Source/portable/MemMang/heap_1.d 


# Each subdirectory must supply rules for building sources it contributes
src/FreeRTOS/Source/portable/MemMang/heap_1.o: /home/user/Desktop/STM32-Projekte/fh-multicopter/STM32F3_Multicopter_devel/src/FreeRTOS/Source/portable/MemMang/heap_1.c
	@echo 'Building file: $<'
	@echo 'Invoking: ARM Summon Linux GCC C Compiler'
	arm-none-eabi-gcc -DUSE_STDPERIPH_DRIVER -DHSE_VALUE=8000000 -DSTM32F30X -I"/home/user/Desktop/STM32-Projekte/fh-multicopter/STM32F3_Multicopter_devel/src/CMSIS/Device/ST/STM32F30x/Include" -I"/home/user/Desktop/STM32-Projekte/fh-multicopter/STM32F3_Multicopter_devel/src/FreeRTOS/Source/include" -I"/home/user/Desktop/STM32-Projekte/fh-multicopter/STM32F3_Multicopter_devel/src/FreeRTOS/Source/portable/GCC/ARM_CM4F" -I"/home/user/Desktop/STM32-Projekte/fh-multicopter/STM32F3_Multicopter_devel/src/STM32F30x_StdPeriph_Driver/inc" -I"/home/user/Desktop/STM32-Projekte/fh-multicopter/STM32F3_Multicopter_devel/src/project_inc" -I"/home/user/Desktop/STM32-Projekte/fh-multicopter/STM32F3_Multicopter_devel/src/CMSIS/Include" -Os -Wall -Wa,-adhlns="$@.lst" -c -fmessage-length=0 -fsingle-precision-constant -MMD -MP -MF"$(@:%.o=%.d)" -MT"$(@:%.o=%.d)" -mcpu=cortex-m3 -mthumb -g3 -gdwarf-2 -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '


