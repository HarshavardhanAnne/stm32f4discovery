
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * This notice applies to any and all portions of this file
  * that are not between comment pairs USER CODE BEGIN and
  * USER CODE END. Other portions of this file, whether 
  * inserted by the user or by software development tools
  * are owned by their respective copyright owners.
  *
  * Copyright (c) 2018 STMicroelectronics International N.V. 
  * All rights reserved.
  *
  * Redistribution and use in source and binary forms, with or without 
  * modification, are permitted, provided that the following conditions are met:
  *
  * 1. Redistribution of source code must retain the above copyright notice, 
  *    this list of conditions and the following disclaimer.
  * 2. Redistributions in binary form must reproduce the above copyright notice,
  *    this list of conditions and the following disclaimer in the documentation
  *    and/or other materials provided with the distribution.
  * 3. Neither the name of STMicroelectronics nor the names of other 
  *    contributors to this software may be used to endorse or promote products 
  *    derived from this software without specific written permission.
  * 4. This software, including modifications and/or derivative works of this 
  *    software, must execute solely and exclusively on microcontroller or
  *    microprocessor devices manufactured by or for STMicroelectronics.
  * 5. Redistribution and use of this software other than as permitted under 
  *    this license is void and will automatically terminate your rights under 
  *    this license. 
  *
  * THIS SOFTWARE IS PROVIDED BY STMICROELECTRONICS AND CONTRIBUTORS "AS IS" 
  * AND ANY EXPRESS, IMPLIED OR STATUTORY WARRANTIES, INCLUDING, BUT NOT 
  * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY, FITNESS FOR A 
  * PARTICULAR PURPOSE AND NON-INFRINGEMENT OF THIRD PARTY INTELLECTUAL PROPERTY
  * RIGHTS ARE DISCLAIMED TO THE FULLEST EXTENT PERMITTED BY LAW. IN NO EVENT 
  * SHALL STMICROELECTRONICS OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
  * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
  * LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, 
  * OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF 
  * LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING 
  * NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
  * EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
  *
  ******************************************************************************
  */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "stm32f4xx_hal.h"
#include "cmsis_os.h"
#include "task.h"

/* USER CODE BEGIN Includes */
#include "eecs473.h"
/* USER CODE END Includes */

/* Private variables ---------------------------------------------------------*/
CAN_HandleTypeDef hcan1;

I2C_HandleTypeDef hi2c1;

SPI_HandleTypeDef hspi1;

UART_HandleTypeDef huart4;

osThreadId defaultTaskHandle;

/* USER CODE BEGIN PV */
/* Private variables ---------------------------------------------------------*/
osThreadId putMessageQueueI2CHandle;
osThreadId putMessageQueueADCHandle;
osThreadId putMessageQueueSPIHandle;
osThreadId putMessageQueueHandle;
osThreadId uartTaskHandle;
osThreadId getMessageQueueHandle;
osThreadId spiATaskHandle;
osThreadId spiBTaskHandle;
osThreadId adcTaskHandle;
ADC_HandleTypeDef g_AdcHandle;
osSemaphoreId spiSemaphore;
osSemaphoreId spiSemaphore2;
osSemaphoreId adcSemaphore;
osSemaphoreId i2cSemaphore;
osSemaphoreDef (spiSemaphore);
osSemaphoreDef (spiSemaphore2);
osSemaphoreDef (adcSemaphore);
osSemaphoreDef (i2cSemaphore);

typedef struct {
  uint32_t StdId;
  uint32_t DLC;
  uint16_t data[4];
} eecsMessage;

osPoolDef(mpool,128,eecsMessage);
osPoolId mpool;
osMessageQDef(MsgBox,128,eecsMessage);
osMessageQId MsgBox;

//osThreadId ledTaskHandle;
#define MY_I2C_SPEED 400000
#define I2C_ADDRESS_IMU (uint8_t)(0b1101000 << 1)

uint8_t i2c_rx_buff_accel[6];
uint8_t i2c_tx_buff_accel[6];
uint8_t i2c_tx_buff_gyro[6];
uint8_t i2c_rx_buff_gyro[6];
uint16_t i2c_accel[4];
uint8_t spi_address[2] = {0b10000100,0b00000000};
uint8_t spi_rx_buff[2];
uint8_t max_data_addr[2] = {0b00111000,0b00000000};
uint8_t max_rx_buff[2];
uint8_t arr[9];
uint16_t random[4] = {0xCD,0xAB,0x20,0x04};
volatile uint32_t g_ADCValue;

uint8_t MAILBOX_INDEX_0 = 0;
uint8_t MAILBOX_INDEX_1 = 0;
uint8_t MAILBOX_INDEX_2 = 0;

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);

void StartDefaultTask(void const * argument);

/* USER CODE BEGIN PFP */
/* Private function prototypes -----------------------------------------------*/
void write_i2c(void const *argument);
void uart_debug(uint8_t* arr, uint8_t buffsize);
void canTest(void const *argument);
void uartTest(void const *argument);
void ConfigureADC();
void adcTest(void const *argument);

void putMessageQueue(void const *argument) {
  TickType_t tick = osKernelSysTick();
  HAL_GPIO_WritePin(GPIOB,GPIO_PIN_4,GPIO_PIN_SET);
  HAL_GPIO_WritePin(GPIOB,GPIO_PIN_4,GPIO_PIN_RESET);
  eecsMessage *mptr;
  while (1) {
    osDelayUntil(&tick,100);
    HAL_GPIO_WritePin(GPIOB,GPIO_PIN_4,GPIO_PIN_SET);
    eecs_I2C_Read();
    //osSemaphoreWait(i2cSemaphore,osWaitForever);
    //osSemaphoreRelease(i2cSemaphore);
    mptr = (eecsMessage*)osPoolAlloc(mpool);
    mptr->StdId = 0x6FF;
    mptr->DLC = sizeof(i2c.data);
    mptr->data[0] = i2c.data[0];
    mptr->data[1] = i2c.data[1];
    mptr->data[2] = i2c.data[2];
    mptr->data[3] = i2c.data[3];
    osMessagePut(MsgBox,(uint32_t)mptr,osWaitForever);
    //HAL_GPIO_WritePin(GPIOB,GPIO_PIN_4,GPIO_PIN_RESET);

    mptr = NULL;
    mptr = (eecsMessage*)osPoolAlloc(mpool);
    mptr->StdId = 0x6FB;
    mptr->DLC = sizeof(adc.data);
    mptr->data[0] = adc.data[0];
    mptr->data[1] = adc.data[1];
    mptr->data[2] = adc.data[2];
    mptr->data[3] = adc.data[3];
    osMessagePut(MsgBox,(uint32_t)mptr,osWaitForever);

    mptr = NULL;
    mptr = (eecsMessage*)osPoolAlloc(mpool);
    mptr->StdId = 0x6F7;
    mptr->DLC = sizeof(spiA.candata);
    mptr->data[0] = spiA.candata[0];
    mptr->data[1] = spiA.candata[1];
    mptr->data[2] = spiA.candata[2];
    mptr->data[3] = spiA.candata[3];
    osMessagePut(MsgBox,(uint32_t)mptr,osWaitForever);

    mptr = NULL;
    mptr = (eecsMessage*)osPoolAlloc(mpool);
    mptr->StdId = 0x6F8;
    mptr->DLC = sizeof(spiA.candata2);
    mptr->data[0] = spiA.candata2[0];
    mptr->data[1] = spiA.candata2[1];
    mptr->data[2] = spiA.candata2[2];
    mptr->data[3] = spiA.candata2[3];
    osMessagePut(MsgBox,(uint32_t)mptr,osWaitForever);
    HAL_GPIO_WritePin(GPIOB,GPIO_PIN_4,GPIO_PIN_RESET);
  }
}

void putMessageQueueI2C(void const *argument) {
  TickType_t tick = osKernelSysTick();
  HAL_GPIO_WritePin(GPIOB,GPIO_PIN_4,GPIO_PIN_SET);
  HAL_GPIO_WritePin(GPIOB,GPIO_PIN_4,GPIO_PIN_RESET);
  eecsMessage *mptr;
  while (1) {
    osDelayUntil(&tick,50);
    HAL_GPIO_WritePin(GPIOB,GPIO_PIN_4,GPIO_PIN_SET);
    eecs_I2C_Read();
    //osSemaphoreWait(i2cSemaphore,osWaitForever);
    //osSemaphoreRelease(i2cSemaphore);
    /*mptr = (eecsMessage*)osPoolAlloc(mpool);
    mptr->StdId = 0x6FF;
    mptr->DLC = sizeof(i2c.data);
    mptr->data[0] = i2c.data[0];
    mptr->data[1] = i2c.data[1];
    mptr->data[2] = i2c.data[2];
    mptr->data[3] = i2c.data[3];
    osMessagePut(MsgBox,(uint32_t)mptr,osWaitForever);*/
    HAL_GPIO_WritePin(GPIOB,GPIO_PIN_4,GPIO_PIN_RESET);
  }
}

void putMessageQueueADC(void const *argument) {
  TickType_t tick = osKernelSysTick();
  eecsMessage *mptr;
  while (1) {
    osDelayUntil(&tick,50);
    mptr = NULL;
    mptr = (eecsMessage*)osPoolAlloc(mpool);
    mptr->StdId = 0x6FB;
    mptr->DLC = sizeof(adc.data);
    mptr->data[0] = adc.data[0];
    mptr->data[1] = adc.data[1];
    mptr->data[2] = adc.data[2];
    mptr->data[3] = adc.data[3];
    osMessagePut(MsgBox,(uint32_t)mptr,osWaitForever);
  }
}

void putMessageQueueSPI(void const *argument) {
  TickType_t tick = osKernelSysTick();
  eecsMessage *mptr;
  while (1) {
    osDelayUntil(&tick,50);
    mptr = NULL;
    mptr = (eecsMessage*)osPoolAlloc(mpool);
    mptr->StdId = 0x6F7;
    mptr->DLC = sizeof(spiA.candata);
    mptr->data[0] = spiA.candata[0];
    mptr->data[1] = spiA.candata[1];
    mptr->data[2] = spiA.candata[2];
    mptr->data[3] = spiA.candata[3];
    osMessagePut(MsgBox,(uint32_t)mptr,osWaitForever);

    mptr = NULL;
    mptr = (eecsMessage*)osPoolAlloc(mpool);
    mptr->StdId = 0x6F8;
    mptr->DLC = sizeof(spiA.candata2);
    mptr->data[0] = spiA.candata2[0];
    mptr->data[1] = spiA.candata2[1];
    mptr->data[2] = spiA.candata2[2];
    mptr->data[3] = spiA.candata2[3];
    osMessagePut(MsgBox,(uint32_t)mptr,osWaitForever);
  }
}

/*void uartTest(void const *argument) {
  int i = 0;
  for (i = 0; i < 9; i++) {
    arr[i] = i;
  }
  while (1) {
    HAL_GPIO_TogglePin(GPIOD, GPIO_PIN_12);
    //HAL_UART_Transmit(&huart4,ptr,size_,HAL_MAX_DELAY);
    uart_debug(arr, sizeof(arr));
    osDelay(100);
  }
}*/

void readSPIA(void const *argument) {
  TickType_t tick = osKernelSysTick();

  while (1) {
    osDelayUntil(&tick,1);
    //HAL_GPIO_WritePin(GPIOB,GPIO_PIN_4,GPIO_PIN_SET);
    eecs_SPI_Read(&spiA,0,0);
    eecs_SPI_Read(&spiA,1,0);
    eecs_SPI_Read(&spiA,0,1);
    eecs_SPI_Read(&spiA,1,1);
    eecs_SPI_Read(&spiA,2,0);
    eecs_SPI_Read(&spiA,3,0);
    eecs_SPI_Read(&spiA,2,1);
    eecs_SPI_Read(&spiA,3,1);
    //HAL_GPIO_WritePin(GPIOB,GPIO_PIN_4,GPIO_PIN_RESET);
  }
}

void readSPIB(void const *argument) {
  TickType_t tick = osKernelSysTick();

  while (1) {
    osDelayUntil(&tick,1);
    eecs_SPI_Read(&spiB,0,0);
    eecs_SPI_Read(&spiB,1,0);
    eecs_SPI_Read(&spiB,0,1);
    eecs_SPI_Read(&spiB,1,1);
    eecs_SPI_Read(&spiB,2,0);
    eecs_SPI_Read(&spiB,3,0);
    eecs_SPI_Read(&spiB,2,1);
    eecs_SPI_Read(&spiB,3,1);
  }
}

void getMessageQueue(void const *argument) {
  uint8_t* data_ptr;
  data_ptr = adc.data;
  HAL_CAN_Start(&hcan);
  HAL_CAN_WakeUp(&hcan);
  CAN_TxHeaderTypeDef tx_buffer;
  CAN_TxHeaderTypeDef* tx_buffer_ptr = &tx_buffer;
  tx_buffer.IDE = CAN_ID_STD;
  tx_buffer.RTR = CAN_RTR_DATA;
  tx_buffer.StdId = 0x6FF;
  tx_buffer.DLC = sizeof(adc.data);
  HAL_StatusTypeDef status = HAL_OK;

  eecsMessage *rptr;
  osEvent evt;


  TickType_t tick = osKernelSysTick();
  while (1) {
    //osDelay(50); //20Hz 
    //osDelay(1); //1 kHz works for 8 bytes of data !THIS SOMETIMES FAILS
    //osDelay(4); //250 Hz , this works with 8 bytes
    //osDelay(25); //40Hz
    osDelayUntil(&tick,25);
    HAL_GPIO_WritePin(GPIOB,GPIO_PIN_4,GPIO_PIN_RESET);

    if (!eecs_CAN_Mail_Ready(CAN_TX_MAILBOX0)) {
      switch (MAILBOX_INDEX_0) {
        case (0):
          eecs_CAN_Set_Params(0x6F7,sizeof(spiA.candata),spiA.candata);
          /*data_ptr = spiA.candata;
          tx_buffer.StdId = 0x6F7;
          tx_buffer.DLC = sizeof(spiA.candata);*/
          //status = HAL_CAN_AddTxMessage(&hcan,tx_buffer_ptr,data_ptr,(uint32_t *)CAN_TX_MAILBOX0);
          eecs_CAN_Send(CAN_TX_MAILBOX0);
          MAILBOX_INDEX_0++;
          break;
        case (1):
          eecs_CAN_Set_Params(0x6F8,sizeof(spiA.candata2),spiA.candata2);
          eecs_CAN_Send(CAN_TX_MAILBOX0);
          /*data_ptr = spiA.candata2;
          tx_buffer.StdId = 0x6F8;
          tx_buffer.DLC = sizeof(spiA.candata2);
          status = HAL_CAN_AddTxMessage(&hcan,tx_buffer_ptr,data_ptr,(uint32_t *)CAN_TX_MAILBOX0);
          */
          MAILBOX_INDEX_0--;
          break;
      }
    }
    if (!eecs_CAN_Mail_Ready(CAN_TX_MAILBOX1)) {
      switch (MAILBOX_INDEX_1) {
        case (0):
          eecs_CAN_Set_Params(0x6F9,sizeof(spiB.candata),spiB.candata);
          eecs_CAN_Send(CAN_TX_MAILBOX1);
          /*data_ptr = random;
          tx_buffer.StdId = 0x6F9;
          tx_buffer.DLC = sizeof(random);
          status = HAL_CAN_AddTxMessage(&hcan,tx_buffer_ptr,data_ptr,(uint32_t *)CAN_TX_MAILBOX1);
          */
          MAILBOX_INDEX_1++;
          break;
        case (1):
          eecs_CAN_Set_Params(0x6FA,sizeof(spiB.candata2),spiB.candata2);
          eecs_CAN_Send(CAN_TX_MAILBOX1);
          /*data_ptr = random;
          tx_buffer.StdId = 0x6FA;
          tx_buffer.DLC = sizeof(random);
          status = HAL_CAN_AddTxMessage(&hcan,tx_buffer_ptr,data_ptr,(uint32_t *)CAN_TX_MAILBOX1);*/
          MAILBOX_INDEX_1--;
          break;
      }
    }
    if (!eecs_CAN_Mail_Ready(CAN_TX_MAILBOX2)) {
      switch (MAILBOX_INDEX_2) {
        case (0):
          eecs_CAN_Set_Params(0x6FB,sizeof(adc.data),adc.data);
          eecs_CAN_Send(CAN_TX_MAILBOX2);
          /*data_ptr = adc.data;
          tx_buffer.StdId = 0x6FB;
          tx_buffer.DLC = sizeof(adc.data);
          status = HAL_CAN_AddTxMessage(&hcan,tx_buffer_ptr,data_ptr,(uint32_t *)CAN_TX_MAILBOX2);
          //MAILBOX_INDEX_0++;*/
          break;
        /*case (1):
          data_ptr = spiA.candata2;
          tx_buffer.StdId = 0x6F8;
          tx_buffer.DLC = sizeof(spiA.candata2);
          status = HAL_CAN_AddTxMessage(&hcan1,tx_buffer_ptr,data_ptr,(uint32_t *)CAN_TX_MAILBOX0);
          MAILBOX_INDEX_0--;
          break;*/
      }
    }
  }

}

void adcTest(void const *argument) {
  //eecs_ADC_Begin();
  TickType_t lastWakeTime = osKernelSysTick();
  while (1) {

    osDelayUntil(&lastWakeTime,10);
    //vTaskDelayUntil(&lastWakeTime,(10/portTICK_PERIOD_MS));
    HAL_GPIO_TogglePin(GPIOB,GPIO_PIN_4);
    //eecs_ADC_Read();
    eecsMessage *mptr;
    mptr = osPoolAlloc(mpool);
    mptr->StdId = 0x6FB;
    mptr->DLC = sizeof(adc.data);
    mptr->data[0] = adc.data[0];
    mptr->data[1] = adc.data[1];
    mptr->data[2] = adc.data[2];
    mptr->data[3] = adc.data[3];
    osMessagePut(MsgBox,(uint32_t)mptr,osWaitForever);
  }

}

int main(void)
{
  spiSemaphore = osSemaphoreCreate(osSemaphore(spiSemaphore),1);
  spiSemaphore2 = osSemaphoreCreate(osSemaphore(spiSemaphore2),1);
  adcSemaphore = osSemaphoreCreate(osSemaphore(adcSemaphore),1);
  i2cSemaphore = osSemaphoreCreate(osSemaphore(i2cSemaphore),1);
  
  //mpool = osPoolCreate(osPool(mpool));
  //MsgBox = osMessageCreate(osMessageQ(MsgBox),NULL);
  HAL_Init();
  HAL_Delay(5);
  SystemClock_Config();
  HAL_Delay(5);
  eecs_GPIO_Clock_Init();
  HAL_Delay(5);
  eecs_I2C_Init();
  HAL_Delay(5);
  eecs_CAN_Init();
  HAL_Delay(5);
  eecs_ADC_Init();
  HAL_Delay(5);
  eecs_ADC_Begin();
  HAL_Delay(5);

  eecs_SPI_Init(2);
  HAL_Delay(10);
  eecs_SPI_Init(3);
  HAL_Delay(10);

  eecs_SPI_Begin(&spiA,0);
  HAL_Delay(1);
  eecs_SPI_Begin(&spiA,1);
  HAL_Delay(1);
  eecs_SPI_Begin(&spiA,2);
  HAL_Delay(1);
  eecs_SPI_Begin(&spiA,3);
  HAL_Delay(1);

  eecs_SPI_Begin(&spiB,0);
  HAL_Delay(1);
  eecs_SPI_Begin(&spiB,1);
  HAL_Delay(1);
  eecs_SPI_Begin(&spiB,2);
  HAL_Delay(1);
  eecs_SPI_Begin(&spiB,3);
  HAL_Delay(1);

    //debug led
  GPIO_InitTypeDef GPIO_InitStruct;
  GPIO_InitStruct.Pin = GPIO_PIN_4;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  osThreadDef(defaultTask, StartDefaultTask, osPriorityNormal, 1, 2);
  defaultTaskHandle = osThreadCreate(osThread(defaultTask), NULL);

  //osThreadDef(uartTask, uartTest, osPriorityAboveNormal, 1, 128);
  //uartTaskHandle = osThreadCreate(osThread(uartTask), NULL);
  osThreadDef(getMessageQueue, getMessageQueue, osPriorityRealtime, 1, 128);
  getMessageQueueHandle = osThreadCreate(osThread(getMessageQueue),NULL);
  osThreadDef(putMessageQueueI2C, putMessageQueueI2C, osPriorityNormal,1,32);
  putMessageQueueI2CHandle = osThreadCreate(osThread(putMessageQueueI2C),NULL);
  osThreadDef(readSPIB,readSPIB,osPriorityAboveNormal,1,128);
  spiBTaskHandle = osThreadCreate(osThread(readSPIB),NULL);
  osThreadDef(readSPIA,readSPIA,osPriorityAboveNormal,1,128);
  spiATaskHandle = osThreadCreate(osThread(readSPIA),NULL);
  //osThreadDef(putMessageQueueADC, putMessageQueueADC, osPriorityAboveNormal,1,32);
  //putMessageQueueADCHandle = osThreadCreate(osThread(putMessageQueueADC),NULL);
  //osThreadDef(putMessageQueueSPI, putMessageQueueSPI, osPriorityAboveNormal,1,128);
  //putMessageQueueSPIHandle = osThreadCreate(osThread(putMessageQueueSPI),NULL);
  //osThreadDef(adcTask, adcTest, osPriorityAboveNormal,1,128);
  //adcTaskHandle = osThreadCreate(osThread(adcTask),NULL);
  //osThreadDef(putMessageQueue,putMessageQueue,osPriorityRealtime,1,128);
  //putMessageQueueHandle = osThreadCreate(osThread(putMessageQueue),NULL);

  /* Start scheduler */
  osKernelStart();
  
  while (1)
  {

  }
}

void SystemClock_Config(void)
{

  RCC_OscInitTypeDef RCC_OscInitStruct;
  RCC_ClkInitTypeDef RCC_ClkInitStruct;

    /**Configure the main internal regulator output voltage 
    */
  __HAL_RCC_PWR_CLK_ENABLE();

  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

    /**Initializes the CPU, AHB and APB busses clocks 
    */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = 16;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Initializes the CPU, AHB and APB busses clocks 
    */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Configure the Systick interrupt time 
    */
  HAL_SYSTICK_Config(HAL_RCC_GetHCLKFreq()/1000);

    /**Configure the Systick 
    */
  HAL_SYSTICK_CLKSourceConfig(SYSTICK_CLKSOURCE_HCLK);

  /* SysTick_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(SysTick_IRQn, 15, 0);

}

void StartDefaultTask(void const * argument)
{

  for(;;)
  {
  }
}

/**
  * @brief  Period elapsed callback in non blocking mode
  * @note   This function is called  when TIM1 interrupt took place, inside
  * HAL_TIM_IRQHandler(). It makes a direct call to HAL_IncTick() to increment
  * a global variable "uwTick" used as application time base.
  * @param  htim : TIM handle
  * @retval None
  */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  if (htim->Instance == TIM1) {
    HAL_IncTick();
  }
}

void _Error_Handler(char *file, int line)
{
  GPIO_InitTypeDef GPIO_InitStruct;
  GPIO_InitStruct.Pin = GPIO_PIN_8;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);
  while(1)
  {
    HAL_GPIO_TogglePin(GPIOB, GPIO_PIN_8);
    HAL_Delay(2000);
  }
}
