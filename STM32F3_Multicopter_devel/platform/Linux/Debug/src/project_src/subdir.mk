################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
/home/user/Desktop/STM32-Projekte/STM32F3_Multicopter_devel/src/project_src/ADXL345_Lib.c \
/home/user/Desktop/STM32-Projekte/STM32F3_Multicopter_devel/src/project_src/HMC5883L_Lib.c \
/home/user/Desktop/STM32-Projekte/STM32F3_Multicopter_devel/src/project_src/IMU_algorithm.c \
/home/user/Desktop/STM32-Projekte/STM32F3_Multicopter_devel/src/project_src/L3GD20_Lib_SPI.c \
/home/user/Desktop/STM32-Projekte/STM32F3_Multicopter_devel/src/project_src/LSM303DLHC_Lib_I2C.c \
/home/user/Desktop/STM32-Projekte/STM32F3_Multicopter_devel/src/project_src/MotorPWM.c \
/home/user/Desktop/STM32-Projekte/STM32F3_Multicopter_devel/src/project_src/PID_control.c \
/home/user/Desktop/STM32-Projekte/STM32F3_Multicopter_devel/src/project_src/Vector_math.c \
/home/user/Desktop/STM32-Projekte/STM32F3_Multicopter_devel/src/project_src/filter_Lib.c \
/home/user/Desktop/STM32-Projekte/STM32F3_Multicopter_devel/src/project_src/helperFunctions.c \
/home/user/Desktop/STM32-Projekte/STM32F3_Multicopter_devel/src/project_src/main.c \
/home/user/Desktop/STM32-Projekte/STM32F3_Multicopter_devel/src/project_src/multicopter_settings.c \
/home/user/Desktop/STM32-Projekte/STM32F3_Multicopter_devel/src/project_src/printf_stdarg.c \
/home/user/Desktop/STM32-Projekte/STM32F3_Multicopter_devel/src/project_src/remote_control.c \
/home/user/Desktop/STM32-Projekte/STM32F3_Multicopter_devel/src/project_src/stm32_configuration.c \
/home/user/Desktop/STM32-Projekte/STM32F3_Multicopter_devel/src/project_src/stm32f30x_it.c \
/home/user/Desktop/STM32-Projekte/STM32F3_Multicopter_devel/src/project_src/usart.c \
/home/user/Desktop/STM32-Projekte/STM32F3_Multicopter_devel/src/project_src/xBee_Lib.c 

OBJS += \
./src/project_src/ADXL345_Lib.o \
./src/project_src/HMC5883L_Lib.o \
./src/project_src/IMU_algorithm.o \
./src/project_src/L3GD20_Lib_SPI.o \
./src/project_src/LSM303DLHC_Lib_I2C.o \
./src/project_src/MotorPWM.o \
./src/project_src/PID_control.o \
./src/project_src/Vector_math.o \
./src/project_src/filter_Lib.o \
./src/project_src/helperFunctions.o \
./src/project_src/main.o \
./src/project_src/multicopter_settings.o \
./src/project_src/printf_stdarg.o \
./src/project_src/remote_control.o \
./src/project_src/stm32_configuration.o \
./src/project_src/stm32f30x_it.o \
./src/project_src/usart.o \
./src/project_src/xBee_Lib.o 

C_DEPS += \
./src/project_src/ADXL345_Lib.d \
./src/project_src/HMC5883L_Lib.d \
./src/project_src/IMU_algorithm.d \
./src/project_src/L3GD20_Lib_SPI.d \
./src/project_src/LSM303DLHC_Lib_I2C.d \
./src/project_src/MotorPWM.d \
./src/project_src/PID_control.d \
./src/project_src/Vector_math.d \
./src/project_src/filter_Lib.d \
./src/project_src/helperFunctions.d \
./src/project_src/main.d \
./src/project_src/multicopter_settings.d \
./src/project_src/printf_stdarg.d \
./src/project_src/remote_control.d \
./src/project_src/stm32_configuration.d \
./src/project_src/stm32f30x_it.d \
./src/project_src/usart.d \
./src/project_src/xBee_Lib.d 


# Each subdirectory must supply rules for building sources it contributes
src/project_src/ADXL345_Lib.o: /home/user/Desktop/STM32-Projekte/STM32F3_Multicopter_devel/src/project_src/ADXL345_Lib.c
	@echo 'Building file: $<'
	@echo 'Invoking: ARM Summon Linux GCC C Compiler'
	arm-none-eabi-gcc -DUSE_STDPERIPH_DRIVER -DHSE_VALUE=8000000 -DSTM32F30X -I"/home/user/Desktop/STM32-Projekte/STM32F3_Multicopter_devel/src/CMSIS/Device/ST/STM32F30x/Include" -I"/home/user/Desktop/STM32-Projekte/STM32F3_Multicopter_devel/src/FreeRTOS/Source/include" -I"/home/user/Desktop/STM32-Projekte/STM32F3_Multicopter_devel/src/FreeRTOS/Source/portable/GCC/ARM_CM4F" -I"/home/user/Desktop/STM32-Projekte/STM32F3_Multicopter_devel/src/STM32F30x_StdPeriph_Driver/inc" -I"/home/user/Desktop/STM32-Projekte/STM32F3_Multicopter_devel/src/project_inc" -I"/home/user/Desktop/STM32-Projekte/STM32F3_Multicopter_devel/src/CMSIS/Include" -Os -Wall -Wa,-adhlns="$@.lst" -c -fmessage-length=0 -fsingle-precision-constant -MMD -MP -MF"$(@:%.o=%.d)" -MT"$(@:%.o=%.d)" -mcpu=cortex-m3 -mthumb -g3 -gdwarf-2 -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '

src/project_src/HMC5883L_Lib.o: /home/user/Desktop/STM32-Projekte/STM32F3_Multicopter_devel/src/project_src/HMC5883L_Lib.c
	@echo 'Building file: $<'
	@echo 'Invoking: ARM Summon Linux GCC C Compiler'
	arm-none-eabi-gcc -DUSE_STDPERIPH_DRIVER -DHSE_VALUE=8000000 -DSTM32F30X -I"/home/user/Desktop/STM32-Projekte/STM32F3_Multicopter_devel/src/CMSIS/Device/ST/STM32F30x/Include" -I"/home/user/Desktop/STM32-Projekte/STM32F3_Multicopter_devel/src/FreeRTOS/Source/include" -I"/home/user/Desktop/STM32-Projekte/STM32F3_Multicopter_devel/src/FreeRTOS/Source/portable/GCC/ARM_CM4F" -I"/home/user/Desktop/STM32-Projekte/STM32F3_Multicopter_devel/src/STM32F30x_StdPeriph_Driver/inc" -I"/home/user/Desktop/STM32-Projekte/STM32F3_Multicopter_devel/src/project_inc" -I"/home/user/Desktop/STM32-Projekte/STM32F3_Multicopter_devel/src/CMSIS/Include" -Os -Wall -Wa,-adhlns="$@.lst" -c -fmessage-length=0 -fsingle-precision-constant -MMD -MP -MF"$(@:%.o=%.d)" -MT"$(@:%.o=%.d)" -mcpu=cortex-m3 -mthumb -g3 -gdwarf-2 -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '

src/project_src/IMU_algorithm.o: /home/user/Desktop/STM32-Projekte/STM32F3_Multicopter_devel/src/project_src/IMU_algorithm.c
	@echo 'Building file: $<'
	@echo 'Invoking: ARM Summon Linux GCC C Compiler'
	arm-none-eabi-gcc -DUSE_STDPERIPH_DRIVER -DHSE_VALUE=8000000 -DSTM32F30X -I"/home/user/Desktop/STM32-Projekte/STM32F3_Multicopter_devel/src/CMSIS/Device/ST/STM32F30x/Include" -I"/home/user/Desktop/STM32-Projekte/STM32F3_Multicopter_devel/src/FreeRTOS/Source/include" -I"/home/user/Desktop/STM32-Projekte/STM32F3_Multicopter_devel/src/FreeRTOS/Source/portable/GCC/ARM_CM4F" -I"/home/user/Desktop/STM32-Projekte/STM32F3_Multicopter_devel/src/STM32F30x_StdPeriph_Driver/inc" -I"/home/user/Desktop/STM32-Projekte/STM32F3_Multicopter_devel/src/project_inc" -I"/home/user/Desktop/STM32-Projekte/STM32F3_Multicopter_devel/src/CMSIS/Include" -Os -Wall -Wa,-adhlns="$@.lst" -c -fmessage-length=0 -fsingle-precision-constant -MMD -MP -MF"$(@:%.o=%.d)" -MT"$(@:%.o=%.d)" -mcpu=cortex-m3 -mthumb -g3 -gdwarf-2 -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '

src/project_src/L3GD20_Lib_SPI.o: /home/user/Desktop/STM32-Projekte/STM32F3_Multicopter_devel/src/project_src/L3GD20_Lib_SPI.c
	@echo 'Building file: $<'
	@echo 'Invoking: ARM Summon Linux GCC C Compiler'
	arm-none-eabi-gcc -DUSE_STDPERIPH_DRIVER -DHSE_VALUE=8000000 -DSTM32F30X -I"/home/user/Desktop/STM32-Projekte/STM32F3_Multicopter_devel/src/CMSIS/Device/ST/STM32F30x/Include" -I"/home/user/Desktop/STM32-Projekte/STM32F3_Multicopter_devel/src/FreeRTOS/Source/include" -I"/home/user/Desktop/STM32-Projekte/STM32F3_Multicopter_devel/src/FreeRTOS/Source/portable/GCC/ARM_CM4F" -I"/home/user/Desktop/STM32-Projekte/STM32F3_Multicopter_devel/src/STM32F30x_StdPeriph_Driver/inc" -I"/home/user/Desktop/STM32-Projekte/STM32F3_Multicopter_devel/src/project_inc" -I"/home/user/Desktop/STM32-Projekte/STM32F3_Multicopter_devel/src/CMSIS/Include" -Os -Wall -Wa,-adhlns="$@.lst" -c -fmessage-length=0 -fsingle-precision-constant -MMD -MP -MF"$(@:%.o=%.d)" -MT"$(@:%.o=%.d)" -mcpu=cortex-m3 -mthumb -g3 -gdwarf-2 -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '

src/project_src/LSM303DLHC_Lib_I2C.o: /home/user/Desktop/STM32-Projekte/STM32F3_Multicopter_devel/src/project_src/LSM303DLHC_Lib_I2C.c
	@echo 'Building file: $<'
	@echo 'Invoking: ARM Summon Linux GCC C Compiler'
	arm-none-eabi-gcc -DUSE_STDPERIPH_DRIVER -DHSE_VALUE=8000000 -DSTM32F30X -I"/home/user/Desktop/STM32-Projekte/STM32F3_Multicopter_devel/src/CMSIS/Device/ST/STM32F30x/Include" -I"/home/user/Desktop/STM32-Projekte/STM32F3_Multicopter_devel/src/FreeRTOS/Source/include" -I"/home/user/Desktop/STM32-Projekte/STM32F3_Multicopter_devel/src/FreeRTOS/Source/portable/GCC/ARM_CM4F" -I"/home/user/Desktop/STM32-Projekte/STM32F3_Multicopter_devel/src/STM32F30x_StdPeriph_Driver/inc" -I"/home/user/Desktop/STM32-Projekte/STM32F3_Multicopter_devel/src/project_inc" -I"/home/user/Desktop/STM32-Projekte/STM32F3_Multicopter_devel/src/CMSIS/Include" -Os -Wall -Wa,-adhlns="$@.lst" -c -fmessage-length=0 -fsingle-precision-constant -MMD -MP -MF"$(@:%.o=%.d)" -MT"$(@:%.o=%.d)" -mcpu=cortex-m3 -mthumb -g3 -gdwarf-2 -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '

src/project_src/MotorPWM.o: /home/user/Desktop/STM32-Projekte/STM32F3_Multicopter_devel/src/project_src/MotorPWM.c
	@echo 'Building file: $<'
	@echo 'Invoking: ARM Summon Linux GCC C Compiler'
	arm-none-eabi-gcc -DUSE_STDPERIPH_DRIVER -DHSE_VALUE=8000000 -DSTM32F30X -I"/home/user/Desktop/STM32-Projekte/STM32F3_Multicopter_devel/src/CMSIS/Device/ST/STM32F30x/Include" -I"/home/user/Desktop/STM32-Projekte/STM32F3_Multicopter_devel/src/FreeRTOS/Source/include" -I"/home/user/Desktop/STM32-Projekte/STM32F3_Multicopter_devel/src/FreeRTOS/Source/portable/GCC/ARM_CM4F" -I"/home/user/Desktop/STM32-Projekte/STM32F3_Multicopter_devel/src/STM32F30x_StdPeriph_Driver/inc" -I"/home/user/Desktop/STM32-Projekte/STM32F3_Multicopter_devel/src/project_inc" -I"/home/user/Desktop/STM32-Projekte/STM32F3_Multicopter_devel/src/CMSIS/Include" -Os -Wall -Wa,-adhlns="$@.lst" -c -fmessage-length=0 -fsingle-precision-constant -MMD -MP -MF"$(@:%.o=%.d)" -MT"$(@:%.o=%.d)" -mcpu=cortex-m3 -mthumb -g3 -gdwarf-2 -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '

src/project_src/PID_control.o: /home/user/Desktop/STM32-Projekte/STM32F3_Multicopter_devel/src/project_src/PID_control.c
	@echo 'Building file: $<'
	@echo 'Invoking: ARM Summon Linux GCC C Compiler'
	arm-none-eabi-gcc -DUSE_STDPERIPH_DRIVER -DHSE_VALUE=8000000 -DSTM32F30X -I"/home/user/Desktop/STM32-Projekte/STM32F3_Multicopter_devel/src/CMSIS/Device/ST/STM32F30x/Include" -I"/home/user/Desktop/STM32-Projekte/STM32F3_Multicopter_devel/src/FreeRTOS/Source/include" -I"/home/user/Desktop/STM32-Projekte/STM32F3_Multicopter_devel/src/FreeRTOS/Source/portable/GCC/ARM_CM4F" -I"/home/user/Desktop/STM32-Projekte/STM32F3_Multicopter_devel/src/STM32F30x_StdPeriph_Driver/inc" -I"/home/user/Desktop/STM32-Projekte/STM32F3_Multicopter_devel/src/project_inc" -I"/home/user/Desktop/STM32-Projekte/STM32F3_Multicopter_devel/src/CMSIS/Include" -Os -Wall -Wa,-adhlns="$@.lst" -c -fmessage-length=0 -fsingle-precision-constant -MMD -MP -MF"$(@:%.o=%.d)" -MT"$(@:%.o=%.d)" -mcpu=cortex-m3 -mthumb -g3 -gdwarf-2 -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '

src/project_src/Vector_math.o: /home/user/Desktop/STM32-Projekte/STM32F3_Multicopter_devel/src/project_src/Vector_math.c
	@echo 'Building file: $<'
	@echo 'Invoking: ARM Summon Linux GCC C Compiler'
	arm-none-eabi-gcc -DUSE_STDPERIPH_DRIVER -DHSE_VALUE=8000000 -DSTM32F30X -I"/home/user/Desktop/STM32-Projekte/STM32F3_Multicopter_devel/src/CMSIS/Device/ST/STM32F30x/Include" -I"/home/user/Desktop/STM32-Projekte/STM32F3_Multicopter_devel/src/FreeRTOS/Source/include" -I"/home/user/Desktop/STM32-Projekte/STM32F3_Multicopter_devel/src/FreeRTOS/Source/portable/GCC/ARM_CM4F" -I"/home/user/Desktop/STM32-Projekte/STM32F3_Multicopter_devel/src/STM32F30x_StdPeriph_Driver/inc" -I"/home/user/Desktop/STM32-Projekte/STM32F3_Multicopter_devel/src/project_inc" -I"/home/user/Desktop/STM32-Projekte/STM32F3_Multicopter_devel/src/CMSIS/Include" -Os -Wall -Wa,-adhlns="$@.lst" -c -fmessage-length=0 -fsingle-precision-constant -MMD -MP -MF"$(@:%.o=%.d)" -MT"$(@:%.o=%.d)" -mcpu=cortex-m3 -mthumb -g3 -gdwarf-2 -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '

src/project_src/filter_Lib.o: /home/user/Desktop/STM32-Projekte/STM32F3_Multicopter_devel/src/project_src/filter_Lib.c
	@echo 'Building file: $<'
	@echo 'Invoking: ARM Summon Linux GCC C Compiler'
	arm-none-eabi-gcc -DUSE_STDPERIPH_DRIVER -DHSE_VALUE=8000000 -DSTM32F30X -I"/home/user/Desktop/STM32-Projekte/STM32F3_Multicopter_devel/src/CMSIS/Device/ST/STM32F30x/Include" -I"/home/user/Desktop/STM32-Projekte/STM32F3_Multicopter_devel/src/FreeRTOS/Source/include" -I"/home/user/Desktop/STM32-Projekte/STM32F3_Multicopter_devel/src/FreeRTOS/Source/portable/GCC/ARM_CM4F" -I"/home/user/Desktop/STM32-Projekte/STM32F3_Multicopter_devel/src/STM32F30x_StdPeriph_Driver/inc" -I"/home/user/Desktop/STM32-Projekte/STM32F3_Multicopter_devel/src/project_inc" -I"/home/user/Desktop/STM32-Projekte/STM32F3_Multicopter_devel/src/CMSIS/Include" -Os -Wall -Wa,-adhlns="$@.lst" -c -fmessage-length=0 -fsingle-precision-constant -MMD -MP -MF"$(@:%.o=%.d)" -MT"$(@:%.o=%.d)" -mcpu=cortex-m3 -mthumb -g3 -gdwarf-2 -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '

src/project_src/helperFunctions.o: /home/user/Desktop/STM32-Projekte/STM32F3_Multicopter_devel/src/project_src/helperFunctions.c
	@echo 'Building file: $<'
	@echo 'Invoking: ARM Summon Linux GCC C Compiler'
	arm-none-eabi-gcc -DUSE_STDPERIPH_DRIVER -DHSE_VALUE=8000000 -DSTM32F30X -I"/home/user/Desktop/STM32-Projekte/STM32F3_Multicopter_devel/src/CMSIS/Device/ST/STM32F30x/Include" -I"/home/user/Desktop/STM32-Projekte/STM32F3_Multicopter_devel/src/FreeRTOS/Source/include" -I"/home/user/Desktop/STM32-Projekte/STM32F3_Multicopter_devel/src/FreeRTOS/Source/portable/GCC/ARM_CM4F" -I"/home/user/Desktop/STM32-Projekte/STM32F3_Multicopter_devel/src/STM32F30x_StdPeriph_Driver/inc" -I"/home/user/Desktop/STM32-Projekte/STM32F3_Multicopter_devel/src/project_inc" -I"/home/user/Desktop/STM32-Projekte/STM32F3_Multicopter_devel/src/CMSIS/Include" -Os -Wall -Wa,-adhlns="$@.lst" -c -fmessage-length=0 -fsingle-precision-constant -MMD -MP -MF"$(@:%.o=%.d)" -MT"$(@:%.o=%.d)" -mcpu=cortex-m3 -mthumb -g3 -gdwarf-2 -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '

src/project_src/main.o: /home/user/Desktop/STM32-Projekte/STM32F3_Multicopter_devel/src/project_src/main.c
	@echo 'Building file: $<'
	@echo 'Invoking: ARM Summon Linux GCC C Compiler'
	arm-none-eabi-gcc -DUSE_STDPERIPH_DRIVER -DHSE_VALUE=8000000 -DSTM32F30X -I"/home/user/Desktop/STM32-Projekte/STM32F3_Multicopter_devel/src/CMSIS/Device/ST/STM32F30x/Include" -I"/home/user/Desktop/STM32-Projekte/STM32F3_Multicopter_devel/src/FreeRTOS/Source/include" -I"/home/user/Desktop/STM32-Projekte/STM32F3_Multicopter_devel/src/FreeRTOS/Source/portable/GCC/ARM_CM4F" -I"/home/user/Desktop/STM32-Projekte/STM32F3_Multicopter_devel/src/STM32F30x_StdPeriph_Driver/inc" -I"/home/user/Desktop/STM32-Projekte/STM32F3_Multicopter_devel/src/project_inc" -I"/home/user/Desktop/STM32-Projekte/STM32F3_Multicopter_devel/src/CMSIS/Include" -Os -Wall -Wa,-adhlns="$@.lst" -c -fmessage-length=0 -fsingle-precision-constant -MMD -MP -MF"$(@:%.o=%.d)" -MT"$(@:%.o=%.d)" -mcpu=cortex-m3 -mthumb -g3 -gdwarf-2 -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '

src/project_src/multicopter_settings.o: /home/user/Desktop/STM32-Projekte/STM32F3_Multicopter_devel/src/project_src/multicopter_settings.c
	@echo 'Building file: $<'
	@echo 'Invoking: ARM Summon Linux GCC C Compiler'
	arm-none-eabi-gcc -DUSE_STDPERIPH_DRIVER -DHSE_VALUE=8000000 -DSTM32F30X -I"/home/user/Desktop/STM32-Projekte/STM32F3_Multicopter_devel/src/CMSIS/Device/ST/STM32F30x/Include" -I"/home/user/Desktop/STM32-Projekte/STM32F3_Multicopter_devel/src/FreeRTOS/Source/include" -I"/home/user/Desktop/STM32-Projekte/STM32F3_Multicopter_devel/src/FreeRTOS/Source/portable/GCC/ARM_CM4F" -I"/home/user/Desktop/STM32-Projekte/STM32F3_Multicopter_devel/src/STM32F30x_StdPeriph_Driver/inc" -I"/home/user/Desktop/STM32-Projekte/STM32F3_Multicopter_devel/src/project_inc" -I"/home/user/Desktop/STM32-Projekte/STM32F3_Multicopter_devel/src/CMSIS/Include" -Os -Wall -Wa,-adhlns="$@.lst" -c -fmessage-length=0 -fsingle-precision-constant -MMD -MP -MF"$(@:%.o=%.d)" -MT"$(@:%.o=%.d)" -mcpu=cortex-m3 -mthumb -g3 -gdwarf-2 -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '

src/project_src/printf_stdarg.o: /home/user/Desktop/STM32-Projekte/STM32F3_Multicopter_devel/src/project_src/printf_stdarg.c
	@echo 'Building file: $<'
	@echo 'Invoking: ARM Summon Linux GCC C Compiler'
	arm-none-eabi-gcc -DUSE_STDPERIPH_DRIVER -DHSE_VALUE=8000000 -DSTM32F30X -I"/home/user/Desktop/STM32-Projekte/STM32F3_Multicopter_devel/src/CMSIS/Device/ST/STM32F30x/Include" -I"/home/user/Desktop/STM32-Projekte/STM32F3_Multicopter_devel/src/FreeRTOS/Source/include" -I"/home/user/Desktop/STM32-Projekte/STM32F3_Multicopter_devel/src/FreeRTOS/Source/portable/GCC/ARM_CM4F" -I"/home/user/Desktop/STM32-Projekte/STM32F3_Multicopter_devel/src/STM32F30x_StdPeriph_Driver/inc" -I"/home/user/Desktop/STM32-Projekte/STM32F3_Multicopter_devel/src/project_inc" -I"/home/user/Desktop/STM32-Projekte/STM32F3_Multicopter_devel/src/CMSIS/Include" -Os -Wall -Wa,-adhlns="$@.lst" -c -fmessage-length=0 -fsingle-precision-constant -MMD -MP -MF"$(@:%.o=%.d)" -MT"$(@:%.o=%.d)" -mcpu=cortex-m3 -mthumb -g3 -gdwarf-2 -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '

src/project_src/remote_control.o: /home/user/Desktop/STM32-Projekte/STM32F3_Multicopter_devel/src/project_src/remote_control.c
	@echo 'Building file: $<'
	@echo 'Invoking: ARM Summon Linux GCC C Compiler'
	arm-none-eabi-gcc -DUSE_STDPERIPH_DRIVER -DHSE_VALUE=8000000 -DSTM32F30X -I"/home/user/Desktop/STM32-Projekte/STM32F3_Multicopter_devel/src/CMSIS/Device/ST/STM32F30x/Include" -I"/home/user/Desktop/STM32-Projekte/STM32F3_Multicopter_devel/src/FreeRTOS/Source/include" -I"/home/user/Desktop/STM32-Projekte/STM32F3_Multicopter_devel/src/FreeRTOS/Source/portable/GCC/ARM_CM4F" -I"/home/user/Desktop/STM32-Projekte/STM32F3_Multicopter_devel/src/STM32F30x_StdPeriph_Driver/inc" -I"/home/user/Desktop/STM32-Projekte/STM32F3_Multicopter_devel/src/project_inc" -I"/home/user/Desktop/STM32-Projekte/STM32F3_Multicopter_devel/src/CMSIS/Include" -Os -Wall -Wa,-adhlns="$@.lst" -c -fmessage-length=0 -fsingle-precision-constant -MMD -MP -MF"$(@:%.o=%.d)" -MT"$(@:%.o=%.d)" -mcpu=cortex-m3 -mthumb -g3 -gdwarf-2 -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '

src/project_src/stm32_configuration.o: /home/user/Desktop/STM32-Projekte/STM32F3_Multicopter_devel/src/project_src/stm32_configuration.c
	@echo 'Building file: $<'
	@echo 'Invoking: ARM Summon Linux GCC C Compiler'
	arm-none-eabi-gcc -DUSE_STDPERIPH_DRIVER -DHSE_VALUE=8000000 -DSTM32F30X -I"/home/user/Desktop/STM32-Projekte/STM32F3_Multicopter_devel/src/CMSIS/Device/ST/STM32F30x/Include" -I"/home/user/Desktop/STM32-Projekte/STM32F3_Multicopter_devel/src/FreeRTOS/Source/include" -I"/home/user/Desktop/STM32-Projekte/STM32F3_Multicopter_devel/src/FreeRTOS/Source/portable/GCC/ARM_CM4F" -I"/home/user/Desktop/STM32-Projekte/STM32F3_Multicopter_devel/src/STM32F30x_StdPeriph_Driver/inc" -I"/home/user/Desktop/STM32-Projekte/STM32F3_Multicopter_devel/src/project_inc" -I"/home/user/Desktop/STM32-Projekte/STM32F3_Multicopter_devel/src/CMSIS/Include" -Os -Wall -Wa,-adhlns="$@.lst" -c -fmessage-length=0 -fsingle-precision-constant -MMD -MP -MF"$(@:%.o=%.d)" -MT"$(@:%.o=%.d)" -mcpu=cortex-m3 -mthumb -g3 -gdwarf-2 -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '

src/project_src/stm32f30x_it.o: /home/user/Desktop/STM32-Projekte/STM32F3_Multicopter_devel/src/project_src/stm32f30x_it.c
	@echo 'Building file: $<'
	@echo 'Invoking: ARM Summon Linux GCC C Compiler'
	arm-none-eabi-gcc -DUSE_STDPERIPH_DRIVER -DHSE_VALUE=8000000 -DSTM32F30X -I"/home/user/Desktop/STM32-Projekte/STM32F3_Multicopter_devel/src/CMSIS/Device/ST/STM32F30x/Include" -I"/home/user/Desktop/STM32-Projekte/STM32F3_Multicopter_devel/src/FreeRTOS/Source/include" -I"/home/user/Desktop/STM32-Projekte/STM32F3_Multicopter_devel/src/FreeRTOS/Source/portable/GCC/ARM_CM4F" -I"/home/user/Desktop/STM32-Projekte/STM32F3_Multicopter_devel/src/STM32F30x_StdPeriph_Driver/inc" -I"/home/user/Desktop/STM32-Projekte/STM32F3_Multicopter_devel/src/project_inc" -I"/home/user/Desktop/STM32-Projekte/STM32F3_Multicopter_devel/src/CMSIS/Include" -Os -Wall -Wa,-adhlns="$@.lst" -c -fmessage-length=0 -fsingle-precision-constant -MMD -MP -MF"$(@:%.o=%.d)" -MT"$(@:%.o=%.d)" -mcpu=cortex-m3 -mthumb -g3 -gdwarf-2 -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '

src/project_src/usart.o: /home/user/Desktop/STM32-Projekte/STM32F3_Multicopter_devel/src/project_src/usart.c
	@echo 'Building file: $<'
	@echo 'Invoking: ARM Summon Linux GCC C Compiler'
	arm-none-eabi-gcc -DUSE_STDPERIPH_DRIVER -DHSE_VALUE=8000000 -DSTM32F30X -I"/home/user/Desktop/STM32-Projekte/STM32F3_Multicopter_devel/src/CMSIS/Device/ST/STM32F30x/Include" -I"/home/user/Desktop/STM32-Projekte/STM32F3_Multicopter_devel/src/FreeRTOS/Source/include" -I"/home/user/Desktop/STM32-Projekte/STM32F3_Multicopter_devel/src/FreeRTOS/Source/portable/GCC/ARM_CM4F" -I"/home/user/Desktop/STM32-Projekte/STM32F3_Multicopter_devel/src/STM32F30x_StdPeriph_Driver/inc" -I"/home/user/Desktop/STM32-Projekte/STM32F3_Multicopter_devel/src/project_inc" -I"/home/user/Desktop/STM32-Projekte/STM32F3_Multicopter_devel/src/CMSIS/Include" -Os -Wall -Wa,-adhlns="$@.lst" -c -fmessage-length=0 -fsingle-precision-constant -MMD -MP -MF"$(@:%.o=%.d)" -MT"$(@:%.o=%.d)" -mcpu=cortex-m3 -mthumb -g3 -gdwarf-2 -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '

src/project_src/xBee_Lib.o: /home/user/Desktop/STM32-Projekte/STM32F3_Multicopter_devel/src/project_src/xBee_Lib.c
	@echo 'Building file: $<'
	@echo 'Invoking: ARM Summon Linux GCC C Compiler'
	arm-none-eabi-gcc -DUSE_STDPERIPH_DRIVER -DHSE_VALUE=8000000 -DSTM32F30X -I"/home/user/Desktop/STM32-Projekte/STM32F3_Multicopter_devel/src/CMSIS/Device/ST/STM32F30x/Include" -I"/home/user/Desktop/STM32-Projekte/STM32F3_Multicopter_devel/src/FreeRTOS/Source/include" -I"/home/user/Desktop/STM32-Projekte/STM32F3_Multicopter_devel/src/FreeRTOS/Source/portable/GCC/ARM_CM4F" -I"/home/user/Desktop/STM32-Projekte/STM32F3_Multicopter_devel/src/STM32F30x_StdPeriph_Driver/inc" -I"/home/user/Desktop/STM32-Projekte/STM32F3_Multicopter_devel/src/project_inc" -I"/home/user/Desktop/STM32-Projekte/STM32F3_Multicopter_devel/src/CMSIS/Include" -Os -Wall -Wa,-adhlns="$@.lst" -c -fmessage-length=0 -fsingle-precision-constant -MMD -MP -MF"$(@:%.o=%.d)" -MT"$(@:%.o=%.d)" -mcpu=cortex-m3 -mthumb -g3 -gdwarf-2 -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '


