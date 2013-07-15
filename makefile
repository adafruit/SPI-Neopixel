PROJECT = spineopixel
MCU = attiny85
F_CPU = 8000000
BURNMCU = attiny85
BURNPROGRAMMER = usbtiny
TARGET = ./$(PROJECT).elf
CC = avr-gcc
CCXX = avr-g++

## Project specific options here
INPUT_MODE = 2
BAUDRATE = 38400
I2C_ADDR = 0xAA

## Flags common to C, ASM, and Linker
COMMON = -mmcu=$(MCU)

## Flags common to C only
CFLAGS = $(COMMON)
CONLYFLAGS = -std=gnu99
CFLAGS += -Wall -gdwarf-2 -Os -funsigned-char -funsigned-bitfields -fpack-struct -fshort-enums -ffunction-sections -fdata-sections
CFLAGS += -DF_CPU=$(F_CPU) -DINPUT_MODE=$(INPUT_MODE) -DBAUDRATE=$(BAUDRATE) -DI2C_ADDR=$(I2C_ADDR)
#CFLAGS += -MD -MP -MT $(*F).o

## Flags common to ASM only
ASMFLAGS = $(COMMON)
ASMFLAGS += $(CFLAGS)
ASMFLAGS += -x assembler-with-cpp -Wa,-gdwarf2
ASMFLAGS += 

## Flags common to CPP/CXX only
CXXFLAGS = $(COMMON)
CXXFLAGS += $(CFLAGS)
CXXFLAGS += -std=c99

## Flags common to Linker only
LDFLAGS = $(COMMON)
LDFLAGS += -Wl,-Map=./$(PROJECT).map
LDFLAGS += -Wl,--gc-sections

## Flags for Intel HEX file production
HEX_FLASH_FLAGS = -R .eeprom -R .fuse -R .lock -R .signature

HEX_EEPROM_FLAGS = -j .eeprom
HEX_EEPROM_FLAGS += --set-section-flags=.eeprom="alloc,load"
HEX_EEPROM_FLAGS += --change-section-lma .eeprom=0 --no-change-warnings

## Include Directories
INCLUDES = -I"."

## Libraries
LIBS = -lm -lc

## Link these object files to be made
OBJECTS = spineopixel.o

## Link objects specified by users
LINKONLYOBJECTS = 

## Compile

all: $(TARGET)

spineopixel.o: ./spineopixel.c
	 $(CC) $(INCLUDES) $(CFLAGS) $(CONLYFLAGS) -c  $<

## Link
$(TARGET): $(OBJECTS)
	-rm -rf $(TARGET) ./$(PROJECT).map
	$(CC) $(LDFLAGS) $(OBJECTS) $(LINKONLYOBJECTS) $(LIBDIRS) $(LIBS) -o $(TARGET)
	-rm -rf $(OBJECTS) spineopixel.d
	-rm -rf ./$(PROJECT).hex ./$(PROJECT).eep ./$(PROJECT).lss
	avr-objcopy -O ihex $(HEX_FLASH_FLAGS) $(TARGET) ./$(PROJECT).hex
	avr-objcopy $(HEX_FLASH_FLAGS) -O ihex $(TARGET) ./$(PROJECT).eep || exit 0
	avr-objdump -h -S $(TARGET) >> ./$(PROJECT).lss
	@avr-size -C --mcu=${MCU} ${TARGET}

## Program
burn:
	avrdude -p $(BURNMCU) -c $(BURNPROGRAMMER)  -U flash:w:./$(PROJECT).hex:a 

burnfuses:
	avrdude -p $(BURNMCU) -c $(BURNPROGRAMMER)  -U lfuse:w:0xE2:m -U hfuse:w:0xD7:m

## Clean target
.PHONY: clean
clean:
	-rm -rf $(OBJECTS) ./$(PROJECT).elf ./$(PROJECT).map ./$(PROJECT).lss ./$(PROJECT).hex ./$(PROJECT).eep
