################################################################################
# Automatically-generated file. Do not edit!
################################################################################

-include ..\makefile.init

RM := del 

# All of the sources participating in the build are defined here
-include sources.mk
-include subdir.mk
-include libraries\encoder\subdir.mk
-include libraries\Wire\subdir.mk
-include libraries\SPI\src\subdir.mk
-include libraries\PID_v1\subdir.mk
-include libraries\Menu\subdir.mk
-include libraries\MAX6675-library\src\subdir.mk
-include libraries\EEPROM\subdir.mk
-include libraries\Adafruit-GFX-Library-master\subdir.mk
-include extEEPROM\examples\eepromTest\subdir.mk
-include extEEPROM\examples\eepromReadWrite\subdir.mk
-include extEEPROM\subdir.mk
-include core\variant\wirish\subdir.mk
-include core\variant\subdir.mk
-include core\core\stm32f1\subdir.mk
-include core\core\libmaple\usb\usb_lib\subdir.mk
-include core\core\libmaple\usb\stm32f1\subdir.mk
-include core\core\libmaple\stm32f1\performance\subdir.mk
-include core\core\libmaple\subdir.mk
-include core\core\avr\subdir.mk
-include core\core\subdir.mk
-include TFT_ILI9163C\subdir.mk
-include objects.mk

ifneq ($(MAKECMDGOALS),clean)
ifneq ($(strip $(CC_DEPS)),)
-include $(CC_DEPS)
endif
ifneq ($(strip $(C++_DEPS)),)
-include $(C++_DEPS)
endif
ifneq ($(strip $(C_UPPER_DEPS)),)
-include $(C_UPPER_DEPS)
endif
ifneq ($(strip $(CXX_DEPS)),)
-include $(CXX_DEPS)
endif
ifneq ($(strip $(ASM_DEPS)),)
-include $(ASM_DEPS)
endif
ifneq ($(strip $(INO_DEPS)),)
-include $(INO_DEPS)
endif
ifneq ($(strip $(PDE_DEPS)),)
-include $(PDE_DEPS)
endif
ifneq ($(strip $(S_UPPER_DEPS)),)
-include $(S_UPPER_DEPS)
endif
ifneq ($(strip $(CPP_DEPS)),)
-include $(CPP_DEPS)
endif
ifneq ($(strip $(C_DEPS)),)
-include $(C_DEPS)
endif
endif

-include ..\makefile.defs

# Add inputs and outputs from these tool invocations to the build variables 
AR += \
C:\Users\Victor\git\reflowOven\bluePillReflow\Release\arduino.ar \

ELF += \
bluePillReflow.elf \

HEX += \
bluePillReflow.hex \


# All Target
all: bluePillReflow

# Tool invocations
bluePillReflow: $(HEX)
	@echo 'Building target: $@'
	@echo 'Printing size:'
	"C:\sloeber\/arduinoPlugin/packages/arduino/tools/arm-none-eabi-gcc/4.8.3-2014q1/bin/arm-none-eabi-size" -A "C:/Users/Victor/git/reflowOven/bluePillReflow/Release/bluePillReflow.elf"
	@echo 'Finished building target: $@'
	@echo ' '

C:\Users\Victor\git\reflowOven\bluePillReflow\Release\arduino.ar: $(AR_OBJ)
	@echo 'Starting archiver'
	"C:\sloeber\/arduinoPlugin/packages/arduino/tools/arm-none-eabi-gcc/4.8.3-2014q1/bin/arm-none-eabi-ar" rcs    C:/Users/Victor/git/reflowOven/bluePillReflow/Release/arduino.ar $(AR_OBJ)
	@echo 'Finished building: $@'
	@echo ' '

bluePillReflow.elf: $(LINK_OBJ) $(AR)
	@echo 'Starting combiner'
	"C:\sloeber\/arduinoPlugin/packages/arduino/tools/arm-none-eabi-gcc/4.8.3-2014q1/bin/arm-none-eabi-g++" -Os -Wl,--gc-sections -mcpu=cortex-m3 "-TC:/Users/Victor/Documents/Arduino/Hardware/Arduino_STM32/STM32F1/variants/generic_stm32f103c/ld/bootloader_20.ld" "-Wl,-Map,C:\Users\Victor\git\reflowOven\bluePillReflow/Release/bluePillReflow.map" "-LC:/Users/Victor/Documents/Arduino/Hardware/Arduino_STM32/STM32F1/variants/generic_stm32f103c/ld" -o "C:\Users\Victor\git\reflowOven\bluePillReflow/Release/bluePillReflow.elf" "-LC:\Users\Victor\git\reflowOven\bluePillReflow/Release" -lm -lgcc -mthumb -Wl,--cref -Wl,--check-sections -Wl,--gc-sections -Wl,--unresolved-symbols=report-all -Wl,--warn-common -Wl,--warn-section-align -Wl,--warn-unresolved-symbols -Wl,--start-group -u _sbrk -u link -u _close -u _fstat -u _isatty -u _lseek -u _read -u _write -u _exit -u kill -u _getpid   $(LINK_OBJ)   C:/Users/Victor/git/reflowOven/bluePillReflow/Release/arduino.ar  -Wl,--end-group
	@echo 'Finished building: $@'
	@echo ' '

bluePillReflow.hex: $(ELF)
	@echo 'Create eeprom image'
	"C:\sloeber\/arduinoPlugin/packages/arduino/tools/arm-none-eabi-gcc/4.8.3-2014q1/bin/arm-none-eabi-objcopy" -O binary  "C:/Users/Victor/git/reflowOven/bluePillReflow/Release/bluePillReflow.elf" "C:/Users/Victor/git/reflowOven/bluePillReflow/Release/bluePillReflow.bin"
	@echo 'Finished building: $@'
	@echo ' '

# Other Targets
clean:
	-$(RM) $(C_UPPER_DEPS)$(PDE_DEPS)$(C_DEPS)$(AR)$(CC_DEPS)$(AR_OBJ)$(C++_DEPS)$(LINK_OBJ)$(CXX_DEPS)$(ASM_DEPS)$(HEX)$(INO_DEPS)$(SIZEDUMMY)$(S_UPPER_DEPS)$(ELF)$(CPP_DEPS)
	-@echo ' '

.PHONY: all clean dependents
.SECONDARY:

-include ..\makefile.targets
