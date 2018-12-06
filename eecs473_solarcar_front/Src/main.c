#include "main.h"
#include "stm32f4xx_hal.h"
#include "cmsis_os.h"
#include "task.h"
#include "eecs473.h"


osThreadId defaultTaskHandle;
osThreadId getMessageQueueHandle;
osThreadId spiATaskHandle;
osThreadId spiBTaskHandle;

uint8_t MAILBOX_INDEX_0 = 0;
uint8_t MAILBOX_INDEX_1 = 0;
uint8_t MAILBOX_INDEX_2 = 0;

void SystemClock_Config(void);
void StartDefaultTask(void const * argument);

void putMessageQueueI2C(void const *argument) {
  TickType_t tick = osKernelSysTick();
  HAL_GPIO_WritePin(GPIOB,GPIO_PIN_4,GPIO_PIN_SET);
  HAL_GPIO_WritePin(GPIOB,GPIO_PIN_4,GPIO_PIN_RESET);
  while (1) {
    osDelayUntil(&tick,20);
    HAL_GPIO_WritePin(GPIOB,GPIO_PIN_4,GPIO_PIN_SET);
    eecs_I2C_Read();
    HAL_GPIO_WritePin(GPIOB,GPIO_PIN_4,GPIO_PIN_RESET);
  }
}

void readSPIA(void const *argument) {
  TickType_t tick = osKernelSysTick();

  while (1) {
    osDelayUntil(&tick,2);
    eecs_SPI_Read(&spiA,0,0);
    eecs_SPI_Read(&spiA,1,0);
    eecs_SPI_Read(&spiA,0,1);
    eecs_SPI_Read(&spiA,1,1);
    eecs_SPI_Read(&spiA,2,0);
    eecs_SPI_Read(&spiA,3,0);
    eecs_SPI_Read(&spiA,2,1);
    eecs_SPI_Read(&spiA,3,1);
  }
}

void readSPIB(void const *argument) {
  TickType_t tick = osKernelSysTick();

  while (1) {
    osDelayUntil(&tick,2);
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
          eecs_CAN_Set_Params(0x6F7,sizeof(spiA.candata),(uint8_t*)spiA.candata);
          eecs_CAN_Send(CAN_TX_MAILBOX0);
          MAILBOX_INDEX_0++;
          break;
        case (1):
          eecs_CAN_Set_Params(0x6F8,sizeof(spiA.candata2),(uint8_t*)spiA.candata2);
          eecs_CAN_Send(CAN_TX_MAILBOX0);
          MAILBOX_INDEX_0--;
          break;
      }
    }
    if (!eecs_CAN_Mail_Ready(CAN_TX_MAILBOX1)) {
      switch (MAILBOX_INDEX_1) {
        case (0):
          eecs_CAN_Set_Params(0x6F9,sizeof(spiB.candata),spiB.candata);
          eecs_CAN_Send(CAN_TX_MAILBOX1);
          MAILBOX_INDEX_1++;
          break;
        case (1):
          eecs_CAN_Set_Params(0x6FA,sizeof(spiB.candata2),spiB.candata2);
          eecs_CAN_Send(CAN_TX_MAILBOX1);
          MAILBOX_INDEX_1--;
          break;
      }
    }
    if (!eecs_CAN_Mail_Ready(CAN_TX_MAILBOX2)) {
      switch (MAILBOX_INDEX_2) {
        case (0):
          eecs_CAN_Set_Params(0x6FB,sizeof(adc.data),(uint8_t*)adc.data);
          eecs_CAN_Send(CAN_TX_MAILBOX2);
          break;
      }
    }
    HAL_GPIO_WritePin(GPIOB,GPIO_PIN_4,GPIO_PIN_SET);
  }

}

int main(void)
{ 
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

  osThreadDef(defaultTask, StartDefaultTask, osPriorityNormal, 1, 128);
  defaultTaskHandle = osThreadCreate(osThread(defaultTask), NULL);

  osThreadDef(getMessageQueue, getMessageQueue, osPriorityRealtime, 1, 128);
  getMessageQueueHandle = osThreadCreate(osThread(getMessageQueue),NULL);
  //osThreadDef(putMessageQueueI2C, putMessageQueueI2C, osPriorityAboveNormal,1,128);
  //putMessageQueueI2CHandle = osThreadCreate(osThread(putMessageQueueI2C),NULL);
  osThreadDef(readSPIB,readSPIB,osPriorityAboveNormal,1,128);
  spiBTaskHandle = osThreadCreate(osThread(readSPIB),NULL);
  osThreadDef(readSPIA,readSPIA,osPriorityAboveNormal,1,128);
  spiATaskHandle = osThreadCreate(osThread(readSPIA),NULL);

  /* Start scheduler */
  osKernelStart();
  
  while (1)
  {

  }
}

void StartDefaultTask(void const *argument) {
  TickType_t tick = osKernelSysTick();
  while (1) {

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
