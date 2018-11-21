//EECS 473 - Advanced Embedded Systems 
//UM Solar Car

#include <stdlib.h>
#include <stdint.h>
#include "stm32f4xx_hal.h"

int CLOCK_ENABLED = 0;

const uint16_t pins[] = {GPIO_PIN_0,
				 GPIO_PIN_1,
				 GPIO_PIN_2,
				 GPIO_PIN_3,
				 GPIO_PIN_4,
				 GPIO_PIN_5,
				 GPIO_PIN_6,
				 GPIO_PIN_7,
				 GPIO_PIN_8,
				 GPIO_PIN_9,
				 GPIO_PIN_10,
				 GPIO_PIN_11,
				 GPIO_PIN_12,
				 GPIO_PIN_13,
				 GPIO_PIN_14,
				 GPIO_PIN_15};

/*#define GREEN_LED pins[12];
#define ORANGE_LED pins[13];
#define RED_LED pins[14];
#define BLUE_LED pins[15];*/

const uint32_t modes[] = {GPIO_MODE_INPUT,
				  GPIO_MODE_OUTPUT_PP,
				  GPIO_MODE_OUTPUT_OD,
				  GPIO_MODE_AF_PP,
				  GPIO_MODE_AF_OD,
				  GPIO_MODE_ANALOG,
				  GPIO_MODE_IT_RISING,
				  GPIO_MODE_IT_FALLING,
				  GPIO_MODE_IT_RISING_FALLING,
				  GPIO_MODE_EVT_RISING,
				  GPIO_MODE_EVT_FALLING,
				  GPIO_MODE_EVT_RISING_FALLING};

const uint32_t speeds[] = {GPIO_SPEED_FREQ_LOW,
				   GPIO_SPEED_FREQ_MEDIUM,
				   GPIO_SPEED_FREQ_HIGH,
				   GPIO_SPEED_FREQ_VERY_HIGH};

const uint32_t pulls[] = {GPIO_NOPULL,
				  GPIO_PULLUP,
				  GPIO_PULLDOWN};

GPIO_PinState HIGH = GPIO_PIN_SET;
GPIO_PinState LOW = GPIO_PIN_RESET;


UART_HandleTypeDef huart;
SPI_HandleTypeDef hspi2;
SPI_HandleTypeDef hspi3;
I2C_HandleTypeDef hi2c;
CAN_HandleTypeDef hcan;


void eecs_Error_Handler();
void eecs_GPIO_Init(GPIO_TypeDef*,uint16_t,uint32_t,uint32_t,uint32_t);
void eecs_GPIO_Write(GPIO_TypeDef*,uint16_t,uint8_t);
void eecs_GPIO_Toggle(GPIO_TypeDef*,uint16_t);

void eecs_UART_Init(void);
void eecs_UART_Print(uint8_t*,uint8_t);
void eecs_UART_Test(void const *);

void eecs_SPI_Init(int);
void eecs_GPIO_Clock_Init(void) {
  if (!CLOCK_ENABLED) {
    /* GPIO Ports Clock Enable */
    __HAL_RCC_GPIOA_CLK_ENABLE();
    __HAL_RCC_GPIOD_CLK_ENABLE();
    __HAL_RCC_GPIOB_CLK_ENABLE();
    CLOCK_ENABLED = 1;
  }
}

void eecs_GPIO_Init(GPIO_TypeDef *GPIOx,uint16_t Pins,uint32_t Mode,uint32_t Pull,uint32_t Speed) {
  GPIO_InitTypeDef GPIO_InitStruct;
  GPIO_InitStruct.Pin = Pins;
  GPIO_InitStruct.Mode = Mode;
  GPIO_InitStruct.Pull = Pull;
  GPIO_InitStruct.Speed = Speed;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);
}

void eecs_GPIO_Write(GPIO_TypeDef *GPIOx,uint16_t Pin, GPIO_PinState PinState) {
  HAL_GPIO_WritePin(GPIOx,Pin,PinState);
}

void eecs_GPIO_Toggle(GPIO_TypeDef *GPIOx, uint16_t Pin) {
  HAL_GPIO_TogglePin(GPIOx,Pin);
}

void eecs_UART_Init(void) {
  huart.Instance = UART4;
  huart.Init.BaudRate = 57600;
  huart.Init.WordLength = UART_WORDLENGTH_8B;
  huart.Init.StopBits = UART_STOPBITS_1;
  huart.Init.Parity = UART_PARITY_NONE;
  huart.Init.Mode = UART_MODE_TX_RX;
  huart.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart) != HAL_OK) {
  	eecs_Error_Handler();
  }
}

void eecs_UART_Write(uint8_t* arr, uint8_t buffsize) {
	HAL_StatusTypeDef status = HAL_UART_Transmit(&huart,arr,buffsize,HAL_MAX_DELAY);
	if (status != HAL_OK) {

	}
}

void eecs_UART_Test(void const *argument) {
  char arr[] = "TESTING UART FUNCTION";
  while (1) {
    HAL_GPIO_TogglePin(GPIOD, GPIO_PIN_12);
    //HAL_UART_Transmit(&huart4,ptr,size_,HAL_MAX_DELAY);
    uart_debug(arr, sizeof(arr));
    osDelay(100);
  }
}

void eecs_SPI_Init(int spi_bus) {
  SPI_HandleTypeDef* spiptr = (spi_bus==2) ? &hspi2:&hspi3;
  spiptr->Instance = (spi_bus == 2) ? SPI2:SPI3;
  spiptr->Init.Mode = SPI_MODE_MASTER;
  spiptr->Init.Direction = SPI_DIRECTION_2LINES;
  spiptr->Init.DataSize = SPI_DATASIZE_8BIT;
  spiptr->Init.CLKPolarity = SPI_POLARITY_LOW;
  spiptr->Init.CLKPhase = SPI_PHASE_1EDGE;
  spiptr->Init.NSS = SPI_NSS_SOFT;
  spiptr->Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_2;
  spiptr->Init.FirstBit = SPI_FIRSTBIT_MSB;
  spiptr->Init.TIMode = SPI_TIMODE_DISABLE;
  spiptr->Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  spiptr->Init.CRCPolynomial = 10;

  /*hspi1.Instance = (spi_bus == 2) ? SPI2:SPI3;
  hspi1.Init.Mode = SPI_MODE_MASTER;
  hspi1.Init.Direction = SPI_DIRECTION_2LINES;
  hspi1.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi1.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi1.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi1.Init.NSS = SPI_NSS_SOFT;
  hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_2;
  hspi1.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi1.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi1.Init.CRCPolynomial = 10;*/

  if (HAL_SPI_Init(spiptr) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }
}




void eecs_Error_Handler() {
	while (1) {
		eecs_GPIO_Toggle(GPIOD, pins[15]);
		HAL_DELAY(1000);
	}
}