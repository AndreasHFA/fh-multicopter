################################################################################
# Automatically-generated file. Do not edit!
################################################################################

-include ../makefile.init

RM := rm -rf

# All of the sources participating in the build are defined here
-include sources.mk
-include src/project_src/subdir.mk
-include src/STM32F30x_StdPeriph_Driver/src/subdir.mk
-include src/FreeRTOS/Source/portable/MemMang/subdir.mk
-include src/FreeRTOS/Source/portable/GCC/ARM_CM4F/subdir.mk
-include src/FreeRTOS/Source/subdir.mk
-include src/CMSIS/Device/ST/STM32F30x/Source/Templates/gcc_ride7/subdir.mk
-include src/CMSIS/Device/ST/STM32F30x/Source/Templates/subdir.mk
-include subdir.mk
-include objects.mk

ifneq ($(MAKECMDGOALS),clean)
ifneq ($(strip $(C_DEPS)),)
-include $(C_DEPS)
endif
ifneq ($(strip $(ASM_DEPS)),)
-include $(ASM_DEPS)
endif
ifneq ($(strip $(S_UPPER_DEPS)),)
-include $(S_UPPER_DEPS)
endif
endif

-include ../makefile.defs

# Add inputs and outputs from these tool invocations to the build variables 
SECONDARY_FLASH += \
STM32F3_Multicopter.hex \

SECONDARY_LIST += \
STM32F3_Multicopter.lst \

SECONDARY_SIZE += \
STM32F3_Multicopter.siz \


# All Target
all: STM32F3_Multicopter.elf secondary-outputs

# Tool invocations
STM32F3_Multicopter.elf: $(OBJS) $(USER_OBJS)
	@echo 'Building target: $@'
	@echo 'Invoking: ARM Summon Linux GCC C Linker'
	arm-none-eabi-gcc -T""/home/user/Desktop/STM32-Projekte/STM32F3_Multicopter_devel/src/LinkerScript/STM32F303VC_FLASH.ld"" -nostartfiles -Xlinker --gc-sections -Wl,-Map,STM32F3_Multicopter.map -mcpu=cortex-m3 -mthumb -g3 -gdwarf-2 -o "STM32F3_Multicopter.elf" $(OBJS) $(USER_OBJS) $(LIBS)
	@echo 'Finished building target: $@'
	@echo ' '

STM32F3_Multicopter.hex: STM32F3_Multicopter.elf
	@echo 'Invoking: ARM Summon Linux GNU Create Flash Image'
	arm-none-eabi-objcopy -O binary STM32F3_Multicopter.elf  "STM32F3_Multicopter.hex"
	@echo 'Finished building: $@'
	@echo ' '

STM32F3_Multicopter.lst: STM32F3_Multicopter.elf
	@echo 'Invoking: ARM Summon Linux GNU Create Listing'
	arm-none-eabi-objdump -h -S STM32F3_Multicopter.elf > "STM32F3_Multicopter.lst"
	@echo 'Finished building: $@'
	@echo ' '

STM32F3_Multicopter.siz: STM32F3_Multicopter.elf
	@echo 'Invoking: ARM Summon Linux GNU Print Size'
	arm-none-eabi-size  --format=berkeley STM32F3_Multicopter.elf
	@echo 'Finished building: $@'
	@echo ' '

# Other Targets
clean:
	-$(RM) $(SECONDARY_SIZE)$(OBJS)$(C_DEPS)$(ASM_DEPS)$(SECONDARY_FLASH)$(EXECUTABLES)$(SECONDARY_LIST)$(S_UPPER_DEPS) STM32F3_Multicopter.elf
	-@echo ' '

secondary-outputs: $(SECONDARY_FLASH) $(SECONDARY_LIST) $(SECONDARY_SIZE)

.PHONY: all clean dependents
.SECONDARY:

-include ../makefile.targets
