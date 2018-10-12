#! /bin/bash

###HOW TO###
# sudo chmod +x program.sh

# Method 1:
#	./program.sh
#	This method uses the default directory name you are currently in.

# Method 2:
#	./program.sh filename.elf
#	This method uses a specified filename from the user.
############

elfname="${PWD##*/}.elf"
if [ "$1" != "" ]; then
	elfname=$1
fi

openocd -f board/stm32f4discovery.cfg -f my_stm32f4.cfg -c "myFlash $elfname"
