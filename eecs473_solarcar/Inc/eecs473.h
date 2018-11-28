//EECS 473 - Advanced Embedded Systems 
//UM Solar Car

#include <stdlib.h>
#include <stdint.h>
//#include "stm32f4xx_hal.h"

UART_HandleTypeDef huart;
SPI_HandleTypeDef hspi2;
SPI_HandleTypeDef hspi3;
I2C_HandleTypeDef hi2c;
CAN_HandleTypeDef hcan;

const uint16_t pins[16] = {GPIO_PIN_0,
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

struct pin_pair {
	GPIO_TypeDef* GPIOx;
	uint16_t pin;
};

struct SPI {
  uint8_t address[3];// = {0b00111000,0b00111001,0b00000000};
  uint8_t rxbuffer[8];
  struct pin_pair cs[4];
  SPI_HandleTypeDef *hspi;
};
struct I2C {
	uint8_t address_accel[6];
	uint8_t address_gyro[6];
	uint8_t data_accel[6];
	uint8_t data_gyro[6];
};

struct pin_pair cs1a = {GPIOA,GPIO_PIN_4};
struct pin_pair cs1b = {GPIOA,GPIO_PIN_3};
struct pin_pair cs2a = {GPIOB,GPIO_PIN_1};
struct pin_pair cs2b = {GPIOA,GPIO_PIN_2};
struct pin_pair cs3a = {GPIOC,GPIO_PIN_7};
struct pin_pair cs3b = {GPIOC,GPIO_PIN_6};
struct pin_pair cs4a = {GPIOC,GPIO_PIN_9};
struct pin_pair cs4b = {GPIOC,GPIO_PIN_8};

struct SPI* spiA;
struct SPI* spiB;
struct I2C i2c;

GPIO_PinState HIGH = GPIO_PIN_SET;
GPIO_PinState LOW = GPIO_PIN_RESET;

/*define GREEN_LED pins[12];
define ORANGE_LED pins[13];
define RED_LED pins[14];
define BLUE_LED pins[15];*/

int CLOCK_ENABLED = 0;

void eecs_Error_Handler();
void eecs_GPIO_Init(GPIO_TypeDef*,uint16_t,uint32_t,uint32_t,uint32_t);
void eecs_GPIO_Write(GPIO_TypeDef*,uint16_t,uint8_t);
void eecs_GPIO_Toggle(GPIO_TypeDef*,uint16_t);

void eecs_UART_Init(void);
void eecs_UART_Print(uint8_t*,uint8_t);
void eecs_UART_Test(void const *);

void eecs_I2C_Init(void);

void eecs_SPI_Init(int);
void eecs_SPI_Read(struct SPI*,uint8_t,uint8_t);

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
  SPI_HandleTypeDef* spi_handle_ptr = (spi_bus==2) ? &hspi2:&hspi3;
  spi_handle_ptr->Instance = (spi_bus == 2) ? SPI2:SPI3;
  spi_handle_ptr->Init.Mode = SPI_MODE_MASTER;
  spi_handle_ptr->Init.Direction = SPI_DIRECTION_2LINES;
  spi_handle_ptr->Init.DataSize = SPI_DATASIZE_8BIT;
  spi_handle_ptr->Init.CLKPolarity = SPI_POLARITY_LOW;
  spi_handle_ptr->Init.CLKPhase = SPI_PHASE_1EDGE;
  spi_handle_ptr->Init.NSS = SPI_NSS_SOFT;
  spi_handle_ptr->Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_2;
  spi_handle_ptr->Init.FirstBit = SPI_FIRSTBIT_MSB;
  spi_handle_ptr->Init.TIMode = SPI_TIMODE_DISABLE;
  spi_handle_ptr->Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  spi_handle_ptr->Init.CRCPolynomial = 10;

  if (HAL_SPI_Init(spi_handle_ptr) != HAL_OK)
  {
    eecs_Error_Handler();
  }

  //SPI address initialization
  struct SPI* spiptr = (spi_bus==2) ? spiA:spiB;
  spiptr->hspi = (spi_bus==2) ? &hspi2:&hspi3;

  spiptr->address[0] = 0b00111000; //MAX1415 address CH0
  spiptr->address[1] = 0b00000000; //address padding
  spiptr->address[2] = 0b00111001; //MAX1415 address CH1
  spiptr->address[3] = 0b00000000;

  spiptr->cs[0] = (spi_bus==2) ? cs1a:cs3a;
  spiptr->cs[1] = (spi_bus==2) ? cs1b:cs3b;
  spiptr->cs[2] = (spi_bus==2) ? cs2a:cs4a;
  spiptr->cs[3] = (spi_bus==2) ? cs2b:cs4b;
}

void eecs_SPI_Read(struct SPI* spi,uint8_t csPin,uint8_t channel) {
	uint8_t rxoffset = 2 * csPin;
	HAL_StatusTypeDef status;
	if (csPin < 0 || csPin > 3) {
		//uart error msg
		return;
	}
	if (channel < 0 || channel > 1) {
		//uart error msg
		return;
	}
	HAL_GPIO_WritePin(spi->cs[csPin].GPIOx,spi->cs[csPin].pin,GPIO_PIN_RESET);
	//HAL_Delay(1);
	osDelay(1);
	status = HAL_SPI_TransmitReceive(spi->hspi,spi->address+channel,spi->rxbuffer+rxoffset,1,HAL_MAX_DELAY);
	if (status != HAL_OK) {
		//do something
	}
	status = HAL_SPI_TransmitReceive(spi->hspi,spi->address+2,spi->rxbuffer+rxoffset+1,1,HAL_MAX_DELAY);
	if (status != HAL_OK) {
		//uart error msg
	}
	osDelay(1); //delay before pulling CS pin high
	HAL_GPIO_WritePin(spi->cs[csPin].GPIOx,spi->cs[csPin].pin,GPIO_PIN_SET);

	return;
}

void eecs_I2C_Init(void) {
  hi2c.Instance = I2C2;
  hi2c.Init.ClockSpeed = 100000;
  hi2c.Init.DutyCycle = I2C_DUTYCYCLE_2;
  hi2c.Init.OwnAddress1 = 0;
  hi2c.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c.Init.OwnAddress2 = 0;
  hi2c.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c) != HAL_OK)
  {
    eecs_Error_Handler();
  }
}



void eecs_Error_Handler() {
	while (1) {
		//eecs_GPIO_Toggle(GPIOD, GPIO_PIN_15);
		HAL_Delay(1000);
	}
}