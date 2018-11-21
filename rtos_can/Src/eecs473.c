#include "eecs473.h"

//Author - Harshavardhan Anne

//GPIO

void eecs_GPIO_Start_Clocks(void) {
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

  /*Configure GPIO pins : PD12 PD13 PD14 PD15 */
  GPIO_InitStruct.Pin = Pins;
  GPIO_InitStruct.Mode = Mode;
  GPIO_InitStruct.Pull = Pull;
  GPIO_InitStruct.Speed = Speed;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);


  /*GPIO_InitStruct.Pin = GPIO_PIN_8 | GPIO_PIN_9;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  GPIO_InitStruct.Pin = GPIO_PIN_4;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_SET);*/
}

void eecs_GPIO_Write(GPIO_TypeDef *GPIOx,uint16_t Pin, GPIO_PinState PinState) {
  HAL_GPIO_WritePin(GPIOx,Pin,PinState);
}

void eecs_GPIO_Toggle(GPIO_TypeDef *GPIOx, uint16_t Pin) {
  HAL_GPIO_TogglePin(GPIOx,Pin);
}


void eecs_Error_Handler() {
	while (1) {

	}
}