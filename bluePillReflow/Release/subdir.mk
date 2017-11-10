################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
INO_SRCS += \
..\bluePillReflow.ino 

CPP_SRCS += \
..\sloeber.ino.cpp 

LINK_OBJ += \
.\sloeber.ino.cpp.o 

INO_DEPS += \
.\bluePillReflow.ino.d 

CPP_DEPS += \
.\sloeber.ino.cpp.d 


# Each subdirectory must supply rules for building sources it contributes
bluePillReflow.o: ..\bluePillReflow.ino
	@echo 'Building file: $<'
	@echo 'Starting C++ compile'
	"C:\sloeber\/arduinoPlugin/packages/arduino/tools/arm-none-eabi-gcc/4.8.3-2014q1/bin/arm-none-eabi-g++" -c -g -Os -Wall -Wextra -DDEBUG_LEVEL=DEBUG_ALL -std=gnu++11 -ffunction-sections -fdata-sections -nostdlib --param max-inline-insns-single=500 -fno-rtti -fno-exceptions -DBOARD_generic_stm32f103c -DVECT_TAB_ADDR=0x8002000 -DERROR_LED_PORT=GPIOB -DERROR_LED_PIN=1 -mcpu=cortex-m3 -DF_CPU=72000000L -DARDUINO=10802 -DARDUINO_GENERIC_STM32F103C -DARDUINO_ARCH_STM32F1 -DSERIAL_USB -DGENERIC_BOOTLOADER -DMCU_STM32F103CB -mthumb -march=armv7-m -D__STM32F1__ -DMCU_STM32F103CB -mthumb -march=armv7-m -D__STM32F1__ "-IC:/Users/Victor/Documents/Arduino/Hardware/Arduino_STM32/STM32F1/system/libmaple" "-IC:/Users/Victor/Documents/Arduino/Hardware/Arduino_STM32/STM32F1/system/libmaple/include" "-IC:/Users/Victor/Documents/Arduino/Hardware/Arduino_STM32/STM32F1/system/libmaple/stm32f1/include"                              "-IC:/Users/Victor/Documents/Arduino/Hardware/Arduino_STM32/STM32F1/system/libmaple/usb/stm32f1" "-IC:/Users/Victor/Documents/Arduino/Hardware/Arduino_STM32/STM32F1/system/libmaple/usb/usb_lib"  -I"C:\Users\Victor\Documents\Arduino\Hardware\Arduino_STM32\STM32F1\cores\maple" -I"C:\Users\Victor\Documents\Arduino\Hardware\Arduino_STM32\STM32F1\variants\generic_stm32f103c" -I"C:\Users\Victor\Documents\Arduino\Hardware\Arduino_STM32\STM32F1\libraries\EEPROM" -I"C:\Users\Victor\Documents\Arduino\Hardware\Arduino_STM32\STM32F1\libraries\SPI" -I"C:\Users\Victor\Documents\Arduino\Hardware\Arduino_STM32\STM32F1\libraries\SPI\src" -I"C:\Users\Victor\Documents\Arduino\libraries\Menu" -I"C:\Users\Victor\Documents\Arduino\libraries\PID_v1" -I"C:\Users\Victor\Documents\Arduino\libraries\MAX6675-library" -I"C:\Users\Victor\Documents\Arduino\libraries\encoder" -I"C:\Users\Victor\Documents\Arduino\Hardware\Arduino_STM32\STM32F1\libraries\Wire" -I"C:\Users\Victor\Documents\Arduino\libraries\MAX6675-library\src" -I"C:\Users\Victor\Documents\Arduino\libraries\Adafruit-GFX-Library-master" -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" -D__IN_ECLIPSE__=1 -x c++ "$<"  -o  "$@"
	@echo 'Finished building: $<'
	@echo ' '

sloeber.ino.cpp.o: ..\sloeber.ino.cpp
	@echo 'Building file: $<'
	@echo 'Starting C++ compile'
	"C:\sloeber\/arduinoPlugin/packages/arduino/tools/arm-none-eabi-gcc/4.8.3-2014q1/bin/arm-none-eabi-g++" -c -g -Os -Wall -Wextra -DDEBUG_LEVEL=DEBUG_ALL -std=gnu++11 -ffunction-sections -fdata-sections -nostdlib --param max-inline-insns-single=500 -fno-rtti -fno-exceptions -DBOARD_generic_stm32f103c -DVECT_TAB_ADDR=0x8002000 -DERROR_LED_PORT=GPIOB -DERROR_LED_PIN=1 -mcpu=cortex-m3 -DF_CPU=72000000L -DARDUINO=10802 -DARDUINO_GENERIC_STM32F103C -DARDUINO_ARCH_STM32F1 -DSERIAL_USB -DGENERIC_BOOTLOADER -DMCU_STM32F103CB -mthumb -march=armv7-m -D__STM32F1__ -DMCU_STM32F103CB -mthumb -march=armv7-m -D__STM32F1__ "-IC:/Users/Victor/Documents/Arduino/Hardware/Arduino_STM32/STM32F1/system/libmaple" "-IC:/Users/Victor/Documents/Arduino/Hardware/Arduino_STM32/STM32F1/system/libmaple/include" "-IC:/Users/Victor/Documents/Arduino/Hardware/Arduino_STM32/STM32F1/system/libmaple/stm32f1/include"                              "-IC:/Users/Victor/Documents/Arduino/Hardware/Arduino_STM32/STM32F1/system/libmaple/usb/stm32f1" "-IC:/Users/Victor/Documents/Arduino/Hardware/Arduino_STM32/STM32F1/system/libmaple/usb/usb_lib"  -I"C:\Users\Victor\Documents\Arduino\Hardware\Arduino_STM32\STM32F1\cores\maple" -I"C:\Users\Victor\Documents\Arduino\Hardware\Arduino_STM32\STM32F1\variants\generic_stm32f103c" -I"C:\Users\Victor\Documents\Arduino\Hardware\Arduino_STM32\STM32F1\libraries\EEPROM" -I"C:\Users\Victor\Documents\Arduino\Hardware\Arduino_STM32\STM32F1\libraries\SPI" -I"C:\Users\Victor\Documents\Arduino\Hardware\Arduino_STM32\STM32F1\libraries\SPI\src" -I"C:\Users\Victor\Documents\Arduino\libraries\Menu" -I"C:\Users\Victor\Documents\Arduino\libraries\PID_v1" -I"C:\Users\Victor\Documents\Arduino\libraries\MAX6675-library" -I"C:\Users\Victor\Documents\Arduino\libraries\encoder" -I"C:\Users\Victor\Documents\Arduino\Hardware\Arduino_STM32\STM32F1\libraries\Wire" -I"C:\Users\Victor\Documents\Arduino\libraries\MAX6675-library\src" -I"C:\Users\Victor\Documents\Arduino\libraries\Adafruit-GFX-Library-master" -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" -D__IN_ECLIPSE__=1 -x c++ "$<"  -o  "$@"
	@echo 'Finished building: $<'
	@echo ' '


