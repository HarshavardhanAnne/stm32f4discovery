#! /bin/bash

openocd -f board/stm32f4discovery.cfg -f my_stm32f4.cfg -c "myFlash stm32f401c_disco.elf"
