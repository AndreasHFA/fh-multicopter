################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
/home/franz/Desktop/STM32F3_Multicopter/STM32F3_Multicopter_devel/src/FreeRTOS/source/croutine.c \
/home/franz/Desktop/STM32F3_Multicopter/STM32F3_Multicopter_devel/src/FreeRTOS/source/list.c \
/home/franz/Desktop/STM32F3_Multicopter/STM32F3_Multicopter_devel/src/FreeRTOS/source/queue.c \
/home/franz/Desktop/STM32F3_Multicopter/STM32F3_Multicopter_devel/src/FreeRTOS/source/tasks.c \
/home/franz/Desktop/STM32F3_Multicopter/STM32F3_Multicopter_devel/src/FreeRTOS/source/timers.c 

OBJS += \
./src/FreeRTOS/source/croutine.o \
./src/FreeRTOS/source/list.o \
./src/FreeRTOS/source/queue.o \
./src/FreeRTOS/source/tasks.o \
./src/FreeRTOS/source/timers.o 

C_DEPS += \
./src/FreeRTOS/source/croutine.d \
./src/FreeRTOS/source/list.d \
./src/FreeRTOS/source/queue.d \
./src/FreeRTOS/source/tasks.d \
./src/FreeRTOS/source/timers.d 


# Each subdirectory must supply rules for building sources it contributes
src/FreeRTOS/source/croutine.o: /home/franz/Desktop/STM32F3_Multicopter/STM32F3_Multicopter_devel/src/FreeRTOS/source/croutine.c
	@echo 'Building file: $<'
	@echo 'Invoking: ARM Summon Linux GCC C Compiler'
	arm-none-eabi-gcc -DUSE_STDPERIPH_DRIVER -DHSE_VALUE=8000000 -DSTM32F30X -I"/home/franz/Desktop/STM32F3_Multicopter/STM32F3_Multicopter_devel/src/CMSIS/Device/ST/STM32F30x/Include" -I"/home/franz/Desktop/STM32F3_Multicopter/STM32F3_Multicopter_devel/src/STM32F30x_StdPeriph_Driver/inc" -I"/home/franz/Desktop/STM32F3_Multicopter/STM32F3_Multicopter_devel/src/project_inc" -I"/home/franz/Desktop/STM32F3_Multicopter/STM32F3_Multicopter_devel/src/FreeRTOS/portable" -I"/home/franz/Desktop/STM32F3_Multicopter/STM32F3_Multicopter_devel/src/FreeRTOS/include" -I"/home/franz/Desktop/STM32F3_Multicopter/STM32F3_Multicopter_devel/src/FreeRTOS" -I"/home/franz/Desktop/STM32F3_Multicopter/STM32F3_Multicopter_devel/src/CMSIS/Include" -Os -Wall -Wa,-adhlns="$@.lst" -c -fmessage-length=0 -fsingle-precision-constant -MMD -MP -MF"$(@:%.o=%.d)" -MT"$(@:%.o=%.d)" -mcpu=cortex-m4 -mthumb -mfloat-abi=hard -mfpu=fpv4-sp-d16 -g3 -gdwarf-2 -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '

src/FreeRTOS/source/list.o: /home/franz/Desktop/STM32F3_Multicopter/STM32F3_Multicopter_devel/src/FreeRTOS/source/list.c
	@echo 'Building file: $<'
	@echo 'Invoking: ARM Summon Linux GCC C Compiler'
	arm-none-eabi-gcc -DUSE_STDPERIPH_DRIVER -DHSE_VALUE=8000000 -DSTM32F30X -I"/home/franz/Desktop/STM32F3_Multicopter/STM32F3_Multicopter_devel/src/CMSIS/Device/ST/STM32F30x/Include" -I"/home/franz/Desktop/STM32F3_Multicopter/STM32F3_Multicopter_devel/src/STM32F30x_StdPeriph_Driver/inc" -I"/home/franz/Desktop/STM32F3_Multicopter/STM32F3_Multicopter_devel/src/project_inc" -I"/home/franz/Desktop/STM32F3_Multicopter/STM32F3_Multicopter_devel/src/FreeRTOS/portable" -I"/home/franz/Desktop/STM32F3_Multicopter/STM32F3_Multicopter_devel/src/FreeRTOS/include" -I"/home/franz/Desktop/STM32F3_Multicopter/STM32F3_Multicopter_devel/src/FreeRTOS" -I"/home/franz/Desktop/STM32F3_Multicopter/STM32F3_Multicopter_devel/src/CMSIS/Include" -Os -Wall -Wa,-adhlns="$@.lst" -c -fmessage-length=0 -fsingle-precision-constant -MMD -MP -MF"$(@:%.o=%.d)" -MT"$(@:%.o=%.d)" -mcpu=cortex-m4 -mthumb -mfloat-abi=hard -mfpu=fpv4-sp-d16 -g3 -gdwarf-2 -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '

src/FreeRTOS/source/queue.o: /home/franz/Desktop/STM32F3_Multicopter/STM32F3_Multicopter_devel/src/FreeRTOS/source/queue.c
	@echo 'Building file: $<'
	@echo 'Invoking: ARM Summon Linux GCC C Compiler'
	arm-none-eabi-gcc -DUSE_STDPERIPH_DRIVER -DHSE_VALUE=8000000 -DSTM32F30X -I"/home/franz/Desktop/STM32F3_Multicopter/STM32F3_Multicopter_devel/src/CMSIS/Device/ST/STM32F30x/Include" -I"/home/franz/Desktop/STM32F3_Multicopter/STM32F3_Multicopter_devel/src/STM32F30x_StdPeriph_Driver/inc" -I"/home/franz/Desktop/STM32F3_Multicopter/STM32F3_Multicopter_devel/src/project_inc" -I"/home/franz/Desktop/STM32F3_Multicopter/STM32F3_Multicopter_devel/src/FreeRTOS/portable" -I"/home/franz/Desktop/STM32F3_Multicopter/STM32F3_Multicopter_devel/src/FreeRTOS/include" -I"/home/franz/Desktop/STM32F3_Multicopter/STM32F3_Multicopter_devel/src/FreeRTOS" -I"/home/franz/Desktop/STM32F3_Multicopter/STM32F3_Multicopter_devel/src/CMSIS/Include" -Os -Wall -Wa,-adhlns="$@.lst" -c -fmessage-length=0 -fsingle-precision-constant -MMD -MP -MF"$(@:%.o=%.d)" -MT"$(@:%.o=%.d)" -mcpu=cortex-m4 -mthumb -mfloat-abi=hard -mfpu=fpv4-sp-d16 -g3 -gdwarf-2 -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '

src/FreeRTOS/source/tasks.o: /home/franz/Desktop/STM32F3_Multicopter/STM32F3_Multicopter_devel/src/FreeRTOS/source/tasks.c
	@echo 'Building file: $<'
	@echo 'Invoking: ARM Summon Linux GCC C Compiler'
	arm-none-eabi-gcc -DUSE_STDPERIPH_DRIVER -DHSE_VALUE=8000000 -DSTM32F30X -I"/home/franz/Desktop/STM32F3_Multicopter/STM32F3_Multicopter_devel/src/CMSIS/Device/ST/STM32F30x/Include" -I"/home/franz/Desktop/STM32F3_Multicopter/STM32F3_Multicopter_devel/src/STM32F30x_StdPeriph_Driver/inc" -I"/home/franz/Desktop/STM32F3_Multicopter/STM32F3_Multicopter_devel/src/project_inc" -I"/home/franz/Desktop/STM32F3_Multicopter/STM32F3_Multicopter_devel/src/FreeRTOS/portable" -I"/home/franz/Desktop/STM32F3_Multicopter/STM32F3_Multicopter_devel/src/FreeRTOS/include" -I"/home/franz/Desktop/STM32F3_Multicopter/STM32F3_Multicopter_devel/src/FreeRTOS" -I"/home/franz/Desktop/STM32F3_Multicopter/STM32F3_Multicopter_devel/src/CMSIS/Include" -Os -Wall -Wa,-adhlns="$@.lst" -c -fmessage-length=0 -fsingle-precision-constant -MMD -MP -MF"$(@:%.o=%.d)" -MT"$(@:%.o=%.d)" -mcpu=cortex-m4 -mthumb -mfloat-abi=hard -mfpu=fpv4-sp-d16 -g3 -gdwarf-2 -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '

src/FreeRTOS/source/timers.o: /home/franz/Desktop/STM32F3_Multicopter/STM32F3_Multicopter_devel/src/FreeRTOS/source/timers.c
	@echo 'Building file: $<'
	@echo 'Invoking: ARM Summon Linux GCC C Compiler'
	arm-none-eabi-gcc -DUSE_STDPERIPH_DRIVER -DHSE_VALUE=8000000 -DSTM32F30X -I"/home/franz/Desktop/STM32F3_Multicopter/STM32F3_Multicopter_devel/src/CMSIS/Device/ST/STM32F30x/Include" -I"/home/franz/Desktop/STM32F3_Multicopter/STM32F3_Multicopter_devel/src/STM32F30x_StdPeriph_Driver/inc" -I"/home/franz/Desktop/STM32F3_Multicopter/STM32F3_Multicopter_devel/src/project_inc" -I"/home/franz/Desktop/STM32F3_Multicopter/STM32F3_Multicopter_devel/src/FreeRTOS/portable" -I"/home/franz/Desktop/STM32F3_Multicopter/STM32F3_Multicopter_devel/src/FreeRTOS/include" -I"/home/franz/Desktop/STM32F3_Multicopter/STM32F3_Multicopter_devel/src/FreeRTOS" -I"/home/franz/Desktop/STM32F3_Multicopter/STM32F3_Multicopter_devel/src/CMSIS/Include" -Os -Wall -Wa,-adhlns="$@.lst" -c -fmessage-length=0 -fsingle-precision-constant -MMD -MP -MF"$(@:%.o=%.d)" -MT"$(@:%.o=%.d)" -mcpu=cortex-m4 -mthumb -mfloat-abi=hard -mfpu=fpv4-sp-d16 -g3 -gdwarf-2 -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '


