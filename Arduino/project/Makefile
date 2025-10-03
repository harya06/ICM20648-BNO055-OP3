# ======== Config ========
MCU = atmega2560
F_CPU = 16000000UL
PORT = /dev/ttyUSB1 #PORTTTT
BAUD = 115200
TARGET = build/main

CC = avr-g++
C = avr-gcc
OBJCOPY = avr-objcopy
AVRDUDE = avrdude
PROGRAMMER = wiring

# ======== Include paths ========
INCLUDES = \
  -Ilib/ArduinoCore-avr/cores/arduino \
  -Ilib/ArduinoCore-avr/variants/mega \
  -Ilib/ArduinoCore-avr/libraries/Wire/src \
  -Ilib/ArduinoCore-avr/libraries/SPI/src \
  -Ilib/Adafruit_BNO055 \
  -Ilib/Adafruit_Sensor \
  -Ilib/Adafruit_BusIO \
  -Ilib/Adafruit_BNO055/utility \
  -Ilib/imumaths

# ======== Source files ========
SRC = \
  main.cpp \
  lib/Adafruit_Sensor/Adafruit_Sensor.cpp \
  lib/imumaths/imumaths.cpp \
  lib/Adafruit_BNO055/Adafruit_BNO055.cpp \
  lib/Adafruit_BusIO/Adafruit_I2CDevice.cpp \
  lib/Adafruit_BusIO/Adafruit_SPIDevice.cpp \
  lib/Adafruit_BusIO/Adafruit_BusIO_Register.cpp \
  lib/Adafruit_BusIO/Adafruit_GenericDevice.cpp \
  lib/ArduinoCore-avr/libraries/Wire/src/Wire.cpp \
  lib/ArduinoCore-avr/libraries/SPI/src/SPI.cpp \
  lib/ArduinoCore-avr/libraries/Wire/src/utility/twi.c \
  lib/ArduinoCore-avr/cores/arduino/wiring_pulse.S

# Tambahkan semua file .c dan .cpp dari core Arduino
SRC += $(wildcard lib/ArduinoCore-avr/cores/arduino/*.c)
SRC += $(wildcard lib/ArduinoCore-avr/cores/arduino/*.cpp)
SRC += $(wildcard lib/ArduinoCore-avr/cores/arduino/*.S)


# Object files 
OBJ = $(SRC:.cpp=.o)
OBJ := $(OBJ:.c=.o)
OBJ := $(OBJ:.S=.o)

# Flags
CXXFLAGS = -mmcu=$(MCU) -DF_CPU=$(F_CPU) -Os -Wall -std=c++11 $(INCLUDES)
CFLAGS   = -mmcu=$(MCU) -DF_CPU=$(F_CPU) -Os -Wall $(INCLUDES)
ASFLAGS  = -mmcu=$(MCU) -DF_CPU=$(F_CPU) -x assembler-with-cpp -Os -Wall

# Build Rules
.PHONY: all clean upload

all: $(TARGET).hex
all: print_port $(TARGET).hex

print_port:
	@echo "Detected PORT = $(PORT)"


%.o: %.cpp
	$(CC) $(CXXFLAGS) -c $< -o $@

%.o: %.c
	$(C) $(CFLAGS) -c $< -o $@

%.o: %.S
	avr-gcc -mmcu=$(MCU) -DF_CPU=$(F_CPU) -Os -Wall $(INCLUDES) -c $< -o $@

%.o: %.S
	$(C) $(ASFLAGS) -c $< -o $@

$(TARGET).elf: $(OBJ)
	mkdir -p build
	$(CC) $(CXXFLAGS) $^ -o $@

$(TARGET).hex: $(TARGET).elf
	$(OBJCOPY) -O ihex -R .eeprom $< $@

upload: $(TARGET).hex
	$(AVRDUDE) -c $(PROGRAMMER) -p $(MCU) -P $(PORT) -b $(BAUD) -D -U flash:w:$<:i

clean:
	rm -f $(OBJ) $(TARGET).elf $(TARGET).hex
