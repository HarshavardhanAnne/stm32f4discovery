#! /bin/bash

openocd -f board/stm32f4discovery.cfg -f gdb.cfg -c "myFlash ()"
