#! /bin/bash

##1st argument is the C file, 2nd is hex file name

clear
avr-gcc -Os -DF_CPU=16000000UL -mmcu=atmega328p -c -o $2.o $1
avr-gcc -mmcu=atmega328p $2.o -o $2
avr-objcopy -O ihex -R .eeprom $2 $2.hex
sudo avrdude -c arduino -p m328p -P /dev/ttyACM0 -b 115200 -U flash:w:$2.hex
