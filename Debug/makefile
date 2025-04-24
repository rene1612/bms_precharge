################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (13.3.rel1)
################################################################################

-include ../makefile.init

RM := rm -rf

# All of the sources participating in the build are defined here
-include sources.mk
-include Drivers/STM32F1xx_HAL_Driver/Src/subdir.mk
-include Core/Startup/subdir.mk
-include Core/Src/subdir.mk
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

OPTIONAL_TOOL_DEPS := \
$(wildcard ../makefile.defs) \
$(wildcard ../makefile.init) \
$(wildcard ../makefile.targets) \


BUILD_ARTIFACT_NAME := bms_precharge
BUILD_ARTIFACT_EXTENSION := elf
BUILD_ARTIFACT_PREFIX :=
BUILD_ARTIFACT := $(BUILD_ARTIFACT_PREFIX)$(BUILD_ARTIFACT_NAME)$(if $(BUILD_ARTIFACT_EXTENSION),.$(BUILD_ARTIFACT_EXTENSION),)

# Add inputs and outputs from these tool invocations to the build variables 
EXECUTABLES += \
bms_precharge.elf \

MAP_FILES += \
bms_precharge.map \

SIZE_OUTPUT += \
default.size.stdout \

OBJDUMP_LIST += \
bms_precharge.list \

OBJCOPY_HEX += \
bms_precharge.hex \

OBJCOPY_BIN += \
bms_precharge.bin \


# All Target
all:
	+@$(MAKE) --no-print-directory pre-build && $(MAKE) --no-print-directory main-build && $(MAKE) --no-print-directory post-build

# Main-build Target
main-build: bms_precharge.elf secondary-outputs

# Tool invocations
bms_precharge.elf bms_precharge.map: $(OBJS) $(USER_OBJS) C:\Users\res\STM32CubeIDE\workspace_1.12.0\bms_precharge\STM32F103CBTX_FLASH_debug.ld makefile objects.list $(OPTIONAL_TOOL_DEPS)
	arm-none-eabi-gcc -o "bms_precharge.elf" @"objects.list" $(USER_OBJS) $(LIBS) -mcpu=cortex-m3 -T"C:\Users\res\STM32CubeIDE\workspace_1.12.0\bms_precharge\STM32F103CBTX_FLASH_debug.ld" --specs=nosys.specs -Wl,-Map="bms_precharge.map" -Wl,--gc-sections -static --specs=nano.specs -mfloat-abi=soft -mthumb -Wl,--start-group -lc -lm -Wl,--end-group
	@echo 'Finished building target: $@'
	@echo ' '

default.size.stdout: $(EXECUTABLES) makefile objects.list $(OPTIONAL_TOOL_DEPS)
	arm-none-eabi-size  $(EXECUTABLES)
	@echo 'Finished building: $@'
	@echo ' '

bms_precharge.list: $(EXECUTABLES) makefile objects.list $(OPTIONAL_TOOL_DEPS)
	arm-none-eabi-objdump -h -S $(EXECUTABLES) > "bms_precharge.list"
	@echo 'Finished building: $@'
	@echo ' '

bms_precharge.hex: $(EXECUTABLES) makefile objects.list $(OPTIONAL_TOOL_DEPS)
	arm-none-eabi-objcopy  -O ihex $(EXECUTABLES) "bms_precharge.hex"
	@echo 'Finished building: $@'
	@echo ' '

bms_precharge.bin: $(EXECUTABLES) makefile objects.list $(OPTIONAL_TOOL_DEPS)
	arm-none-eabi-objcopy  -O binary $(EXECUTABLES) "bms_precharge.bin"
	@echo 'Finished building: $@'
	@echo ' '

# Other Targets
clean:
	-$(RM) bms_precharge.bin bms_precharge.elf bms_precharge.hex bms_precharge.list bms_precharge.map default.size.stdout
	-@echo ' '

pre-build:
	git log --pretty=format:'#define GIT_INFO_PRESENT%n#define __GIT_SHORT_HASH__ 0x%h%n#define __GIT_BRANCH__ "%D"%n#define __GIT_DATE_STR__ "%cs"%n#define __GIT_DATE_UT__ %at' -n 1 > ../Core/Inc/gitcommit.h
	-@echo ' '

post-build:
	arm-none-eabi-objcopy -O ihex bms_precharge.elf bms_precharge.hex
	-@echo ' '

secondary-outputs: $(SIZE_OUTPUT) $(OBJDUMP_LIST) $(OBJCOPY_HEX) $(OBJCOPY_BIN)

fail-specified-linker-script-missing:
	@echo 'Error: Cannot find the specified linker script. Check the linker settings in the build configuration.'
	@exit 2

warn-no-linker-script-specified:
	@echo 'Warning: No linker script specified. Check the linker settings in the build configuration.'

.PHONY: all clean dependents main-build fail-specified-linker-script-missing warn-no-linker-script-specified pre-build post-build

-include ../makefile.targets
