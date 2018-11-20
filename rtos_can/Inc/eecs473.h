//EECS 473 - Advanced Embedded Systems 
//UM Solar Car

#include <stdlib.h>
#include <stdint.h>
#include "stm32f4xx_hal.h"

struct eecs_GPIO {
	void (*init)(GPIO_TypeDef*,uint16_t,uint32_t,uint32_t,uint32_t);// = &eecs_GPIO_Init;
	void (*write)(GPIO_TypeDef*,uint16_t,uint32_t,uint32_t,uint32_t);// = eecs_GPIO_Write;
	void (*toggle)(GPIO_TypeDef*,uint16_t);// = eecs_GPIO_Toggle;
	int clock_enable;
};

struct eecs_UART {
	void (*init)(void);// = eecs_UART_Init;
	void (*print)(uint8_t*,uint8_t);// = eecs_UART_Print;
	void (*test)(void const *);// = eecs_UART_Test;
};

struct eecs_SPI {
	void (*init)(void);
	//void ()
};

void eecs_Error_Handler();
void eecs_GPIO_Init(GPIO_TypeDef*,uint16_t,uint32_t,uint32_t,uint32_t);
void eecs_GPIO_Write(GPIO_TypeDef*,uint16_t,uint8_t);
void eecs_GPIO_Toggle(GPIO_TypeDef*,uint16_t);

void eecs_UART_Init(void);
void eecs_UART_Print(uint8_t*,uint8_t);
void eecs_UART_Test(void const *);

extern struct eecs_GPIO eecsGPIO;
extern struct eecs_UART eecsUART;
extern struct eecs_SPI eecsSPI;
