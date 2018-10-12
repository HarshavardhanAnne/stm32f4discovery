#! /bin/bash

elfname="${PWD##*/}.elf"
if [ "$1" != "" ]; then
	elfname=$1
fi

openocd -f board/stm32f4discovery.cfg -f my_stm32f4.cfg -c "myFlash $elfname"
