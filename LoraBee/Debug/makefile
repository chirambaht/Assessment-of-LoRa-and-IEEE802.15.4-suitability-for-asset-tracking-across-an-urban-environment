################################################################################
# Automatically-generated file. Do not edit!
################################################################################

-include ../makefile.init

RM := rm -rf

# All of the sources participating in the build are defined here
-include sources.mk
-include Drivers/STM32F0xx_HAL_Driver/Src/subdir.mk
-include Core/Startup/subdir.mk
-include Core/Src/subdir.mk
-include subdir.mk
-include objects.mk

ifneq ($(MAKECMDGOALS),clean)
ifneq ($(strip $(S_DEPS)),)
-include $(S_DEPS)
endif
ifneq ($(strip $(S_UPPER_DEPS)),)
-include $(S_UPPER_DEPS)
endif
ifneq ($(strip $(C_DEPS)),)
-include $(C_DEPS)
endif
endif

-include ../makefile.defs

# Add inputs and outputs from these tool invocations to the build variables 
EXECUTABLES += \
LoraBee.elf \

SIZE_OUTPUT += \
default.size.stdout \

OBJDUMP_LIST += \
LoraBee.list \

OBJCOPY_BIN += \
LoraBee.bin \


# All Target
all: LoraBee.elf secondary-outputs

# Tool invocations
LoraBee.elf: $(OBJS) $(USER_OBJS) /home/ramzy/Documents/Project/Assessment\ of\ LoRa\ and\ IEEE802.15.4\ suitability\ for\ asset\ tracking\ across\ an\ urban\ environment/LoraBee/STM32F031K6TX_FLASH.ld
	arm-none-eabi-gcc -o "LoraBee.elf" @"objects.list" $(USER_OBJS) $(LIBS) -mcpu=cortex-m0 -T"/home/ramzy/Documents/Project/Assessment of LoRa and IEEE802.15.4 suitability for asset tracking across an urban environment/LoraBee/STM32F031K6TX_FLASH.ld" --specs=nosys.specs -Wl,-Map="LoraBee.map" -Wl,--gc-sections -static --specs=nano.specs -mfloat-abi=soft -mthumb -Wl,--start-group -lc -lm -Wl,--end-group
	@echo 'Finished building target: $@'
	@echo ' '

default.size.stdout: $(EXECUTABLES)
	arm-none-eabi-size  $(EXECUTABLES)
	@echo 'Finished building: $@'
	@echo ' '

LoraBee.list: $(EXECUTABLES)
	arm-none-eabi-objdump -h -S $(EXECUTABLES) > "LoraBee.list"
	@echo 'Finished building: $@'
	@echo ' '

LoraBee.bin: $(EXECUTABLES)
	arm-none-eabi-objcopy  -O binary $(EXECUTABLES) "LoraBee.bin"
	@echo 'Finished building: $@'
	@echo ' '

# Other Targets
clean:
	-$(RM) *
	-@echo ' '

secondary-outputs: $(SIZE_OUTPUT) $(OBJDUMP_LIST) $(OBJCOPY_BIN)

.PHONY: all clean dependents
.SECONDARY:

-include ../makefile.targets
