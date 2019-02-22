#!/bin/bash
sudo avrdude -v -p m2560 -b 9600 -P /dev/ttyACM0 -c stk500v2 -e -U flash:w:$1
