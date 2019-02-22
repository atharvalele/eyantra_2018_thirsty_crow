#!/bin/bash

avr-gcc $1 -mmcu=atmega2560 -Os -c -o $1.o
avr-gcc -mmcu=atmega2560 -o $1.elf $1.o
avr-objcopy -j .text -j .data -O ihex $1.elf $1.hex
