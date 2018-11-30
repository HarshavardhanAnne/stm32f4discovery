
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
osThreadId i2cTaskHandle;
osThreadId uartTaskHandle;
osThreadId canTaskHandle;
osThreadId spiTaskHandle;
osThreadId adcTaskHandle;
ADC_HandleTypeDef g_AdcHandle;
//osThreadId ledTaskHandle;
#define MY_I2C_SPEED 400000
#define I2C_ADDRESS_IMU (uint8_t)(0b1101000 << 1)
#define RED_LED GPIO_PIN_14
#define GREEN_LED GPIO_PIN_12
#define BLUE_LED GPIO_PIN_15
#define ORANGE_LED GPIO_PIN_13
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
volatile uint32_t g_ADCValue;

//CANTX - PB9
//CANRX - PB8
//I2CSDA - PB7
//I2CSCL - PB6
//UARTtx - PA0
//UARTrx - PA1
//SPISCK - PA5
//SPIMISO - PA6
//SPIMOSI - PA7
//SPICS - PA4 //This is NSS pin
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_UART4_Init(void);
static void MX_SPI1_Init(void);
static void MX_I2C1_Init(void);
static void MX_CAN1_Init(void);
void StartDefaultTask(void const * argument);

/* USER CODE BEGIN PFP */
/* Private function prototypes -----------------------------------------------*/
void write_i2c(void const *argument);
void uart_debug(uint8_t* arr, uint8_t buffsize);
void canTest(void const *argument);
void uartTest(void const *argument);
void ConfigureADC();
void adcTest(void const *argument);
/* USER CODE END PFP */

/* USER CODE BEGIN 0 */
void writei2c(void const *argument) {
  //HAL_GPIO_WritePin(GPIOD, GPIO_PIN_13, GPIO_PIN_SET);
  HAL_StatusTypeDef status = HAL_OK;
  uint32_t prevWakeTime;
  uint32_t nextWakeTime;
  uint8_t addr = 59;
  int i;
  for (i = 0; i < 6; i++) {
    i2c_tx_buff_accel[i] = addr++;
    //i2c_rx_buff_accel[i] = i;
  }
  //i2c_rx_buff_accel[0] = 0;
  //i2c_rx_buff_accel[1] = 0;
  //ACCEL_XOUT_H , ACCEL_XOUT_L
  while (1) {
    //prevWakeTime = osKernelSysTick();
    //nextWakeTime += (1000 * osKernelSysTick())
    //osDelayUntil(&prevWakeTime,5000);
    osDelay(20);
    status = HAL_OK;
    for (i = 0; i < 6; i++) {
      status = HAL_I2C_Master_Transmit(&hi2c1,I2C_ADDRESS_IMU,i2c_tx_buff_accel+i,sizeof(uint8_t),10);

      if (status != HAL_OK) {
        HAL_GPIO_WritePin(GPIOD, GPIO_PIN_15, GPIO_PIN_SET);
      }
      else {
        //HAL_GPIO_WritePin(GPIOD,GPIO_PIN_15, GPIO_PIN_RESET);
      }
      status = HAL_I2C_Master_Receive(&hi2c1,I2C_ADDRESS_IMU,i2c_rx_buff_accel+i,sizeof(uint8_t),10);
    }
    //itoa
    //debug();
    i2c_accel[0] = 0;
    i2c_accel[1] = (i2c_rx_buff_accel[0] << 8) + i2c_rx_buff_accel[1];
    i2c_accel[2] = (i2c_rx_buff_accel[2] << 8) + i2c_rx_buff_accel[3];
    i2c_accel[3] = (i2c_rx_buff_accel[4] << 8) + i2c_rx_buff_accel[5];
    //uart_debug(i2c_accel,sizeof(i2c_accel));
    //HAL_GPIO_WritePin(GPIOD, GPIO_PIN_15, GPIO_PIN_SET);
    //HAL_GPIO_TogglePin(GPIOD,GPIO_PIN_12); //GPIOD12 is green
  }
}

void uartTest(void const *argument) {
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
}

void spiTest(void const *argument) {
  volatile uint16_t val1;
  volatile uint16_t val2;
  volatile uint16_t val3;
  volatile uint16_t val4;

  eecs_SPI_Init(2);
  eecs_SPI_Init(3);

  //debug led
  GPIO_InitTypeDef GPIO_InitStruct;
  GPIO_InitStruct.Pin = GPIO_PIN_4;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  eecs_SPI_Begin(&spiA,0);

  while (1) {
    osDelay(10);
    eecs_SPI_Read(&spiA,0,0);
    //eecs_SPI_Read(&spiA,0,1);
    //eecs_SPI_Read(&spiA,1,0);
    //eecs_SPI_Read(&spiA,1,1);

    //val1 = (spiA.rxbuffer[0] << 8) + spiA.rxbuffer[1];
    //val2 = (spiA.rxbuffer[2] << 8) + spiA.rxbuffer[3];
    //val3 = (spiA.rxbuffer[4] << 8) + spiA.rxbuffer[5];
    //val4 = (spiA.rxbuffer[6] << 8) + spiA.rxbuffer[7];

    HAL_GPIO_TogglePin(GPIOB,GPIO_PIN_4);
    //HAL_GPIO_WritePin(GPIOA,GPIO_PIN_3, GPIO_PIN_RESET);
  }
  /*HAL_StatusTypeDef status;
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_SET);
  while (1) {
    osDelay(10);
    status = HAL_OK;
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4,GPIO_PIN_RESET);
    HAL_Delay(1);
    //status = HAL_SPI_Transmit(&hspi1, spi_address,1,HAL_MAX_DELAY);
    //status = HAL_SPI_Receive(&hspi1, spi_rx_buff,1,HAL_MAX_DELAY);
    //status = HAL_SPI_Transmit(&hspi1, spi_address+1,1,HAL_MAX_DELAY);
    //status = HAL_SPI_Receive(&hspi1, spi_rx_buff+1,1,HAL_MAX_DELAY);
    status = HAL_SPI_TransmitReceive(&hspi1,max_data_addr,max_rx_buff,1,HAL_MAX_DELAY);
    //HAL_Delay(1);
    if (status != HAL_OK) {
      //HAL_GPIO_WritePin(GPIOD, GREEN_LED, GPIO_PIN_SET);
    }
    else {
      //HAL_GPIO_WritePin(GPIOD, GREEN_LED, GPIO_PIN_RESET);
    }

    status = HAL_SPI_TransmitReceive(&hspi1,max_data_addr+1,max_rx_buff+1,1,HAL_MAX_DELAY);
    HAL_Delay(1);
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_SET);
    //spi_rx_buff[0] = 'A';
    //spi_rx_buff[1] = 'F';
    //uart_debug(max_rx_buff, sizeof(max_rx_buff));
  }*/
}

//CANTX - PB9
//CANRX - PB8
void canTest(void const *argument) {
  uint8_t* data_ptr;
  data_ptr = spiA.rxbuffer;
  HAL_CAN_Start(&hcan1);
  HAL_CAN_WakeUp(&hcan1);
  uint8_t data[8] = {0,0,1,0,2,0,3,0};
  uint8_t data2[8] = {0xAA,0xAA,0xAA,0xAA,0xAA,0xAA,0xAA,0xAA};
  uint8_t data_sel = 0;
  CAN_TxHeaderTypeDef tx_buffer;
  CAN_TxHeaderTypeDef* tx_buffer_ptr = &tx_buffer;
  tx_buffer.StdId = 0x6FB;
  tx_buffer.ExtId = 0xF00;
  tx_buffer.IDE = CAN_ID_STD;
  tx_buffer.RTR = CAN_RTR_DATA;
  tx_buffer.DLC = sizeof(spiA.rxbuffer);
  HAL_StatusTypeDef status = HAL_OK;

  //data_ptr = &i2c_accel;//spiA->rxbuffer;//&i2c_accel;
  while (1) {
    //osDelay(50); //20Hz 
    //osDelay(1); //1 kHz works for 8 bytes of data !THIS SOMETIMES FAILS
    //osDelay(4); //250 Hz , this works with 8 bytes
    //osDelay(25); //40Hz
    osDelay(10);
    status = HAL_OK;
    //data_ptr = (data_sel) ? data : data2;
    //data_sel ^= 0b1;
    while (HAL_CAN_IsTxMessagePending(&hcan1, (uint32_t)CAN_TX_MAILBOX0)) {
      //HAL_GPIO_WritePin(GPIOD, ORANGE_LED, GPIO_PIN_SET);
      //osDelay(100);
      //HAL_GPIO_WritePin(GPIOD, ORANGE_LED, GPIO_PIN_RESET);
    }
    status = HAL_CAN_AddTxMessage(&hcan1, tx_buffer_ptr, data_ptr, (uint32_t *)CAN_TX_MAILBOX0);

    if (status == HAL_OK) {
      //HAL_GPIO_WritePin(GPIOD, GREEN_LED, GPIO_PIN_SET);
    }
    else {
      //HAL_GPIO_WritePin(GPIOD, GREEN_LED, GPIO_PIN_RESET);
      //HAL_GPIO_TogglePin(GPIOD, RED_LED);
      //unsigned char temparr[] = {"NOT WORKING"};
      //uart_debug(temparr, sizeof(temparr));
    }

  }

}

void uart_debug(uint8_t* arr, uint8_t buffsize) {
  HAL_StatusTypeDef status = HAL_OK;
  
  status = HAL_UART_Transmit(&huart4, arr, buffsize, HAL_MAX_DELAY);
  if (status != HAL_OK) {
    //HAL_GPIO_WritePin(GPIOD, GPIO_PIN_15, GPIO_PIN_SET);
  }
  else {
    HAL_GPIO_TogglePin(GPIOD,GPIO_PIN_15);
    //HAL_GPIO_WritePin(GPIOD, GPIO_PIN_15, GPIO_PIN_RESET);
  }
}

void Leds(void const *argument) {

  /*GPIO_InitTypeDef GPIO_InitStruct;

  __HAL_RCC_GPIOD_CLK_ENABLE();
  //HAL_GPIO_WritePin(CS_I2C_SPI_GPIO_Port, CS_I2C_SPI_Pin, GPIO_PIN_RESET);

  //Configure GPIO pin Output Level 
  //HAL_GPIO_WritePin(OTG_FS_PowerSwitchOn_GPIO_Port, OTG_FS_PowerSwitchOn_Pin, GPIO_PIN_SET);

  //Configure GPIO pin Output Level
  HAL_GPIO_WritePin(GPIOD, GPIO_PIN_15|GPIO_PIN_14|GPIO_PIN_13|GPIO_PIN_12, GPIO_PIN_RESET);

  GPIO_InitStruct.Pin = GPIO_PIN_15|GPIO_PIN_14|GPIO_PIN_13|GPIO_PIN_12;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);*/
  uint32_t prevWakeTime;
  while (1) {
    //HAL_GPIO_TogglePin(GPIOD, GPIO_PIN_15|GPIO_PIN_14|GPIO_PIN_13|GPIO_PIN_12);
    HAL_GPIO_WritePin(GPIOD, GPIO_PIN_15, GPIO_PIN_SET);
    osDelay(100);
    HAL_GPIO_WritePin(GPIOD, GPIO_PIN_15, GPIO_PIN_RESET);
    osDelay(100);
    HAL_GPIO_WritePin(GPIOD, GPIO_PIN_12, GPIO_PIN_SET);
    osDelay(100);
    HAL_GPIO_WritePin(GPIOD, GPIO_PIN_12, GPIO_PIN_RESET);
    osDelay(100);
    HAL_GPIO_WritePin(GPIOD, GPIO_PIN_13, GPIO_PIN_SET);
    osDelay(100);
    HAL_GPIO_WritePin(GPIOD, GPIO_PIN_13, GPIO_PIN_RESET);
    osDelay(100);
    HAL_GPIO_WritePin(GPIOD, GPIO_PIN_14, GPIO_PIN_SET);
    osDelay(100);
    HAL_GPIO_WritePin(GPIOD, GPIO_PIN_14, GPIO_PIN_RESET);

    prevWakeTime = osKernelSysTick();
    osDelayUntil(&prevWakeTime, 50);
  }
}
void ConfigureADC() {
    GPIO_InitTypeDef gpioInit;
    __ADC1_CLK_ENABLE();
 
    gpioInit.Pin = GPIO_PIN_4;
    gpioInit.Mode = GPIO_MODE_ANALOG;
    gpioInit.Pull = GPIO_NOPULL;
    HAL_GPIO_Init(GPIOC, &gpioInit);
 
    HAL_NVIC_SetPriority(ADC_IRQn, 0, 0);
    HAL_NVIC_EnableIRQ(ADC_IRQn);
 
    ADC_ChannelConfTypeDef adcChannel;
 
    g_AdcHandle.Instance = ADC1;
 
    g_AdcHandle.Init.ClockPrescaler = ADC_CLOCKPRESCALER_PCLK_DIV2;
    g_AdcHandle.Init.Resolution = ADC_RESOLUTION_12B;
    g_AdcHandle.Init.ScanConvMode = DISABLE;
    g_AdcHandle.Init.ContinuousConvMode = ENABLE;
    g_AdcHandle.Init.DiscontinuousConvMode = DISABLE;
    g_AdcHandle.Init.NbrOfDiscConversion = 0;
    g_AdcHandle.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
    g_AdcHandle.Init.ExternalTrigConv = ADC_EXTERNALTRIGCONV_T1_CC1;
    g_AdcHandle.Init.DataAlign = ADC_DATAALIGN_RIGHT;
    g_AdcHandle.Init.NbrOfConversion = 1;
    g_AdcHandle.Init.DMAContinuousRequests = ENABLE;
    g_AdcHandle.Init.EOCSelection = DISABLE;
 
    HAL_ADC_Init(&g_AdcHandle);
 
    adcChannel.Channel = ADC_CHANNEL_11;
    adcChannel.Rank = 1;
    adcChannel.SamplingTime = ADC_SAMPLETIME_480CYCLES;
    adcChannel.Offset = 0;
 
    if (HAL_ADC_ConfigChannel(&g_AdcHandle, &adcChannel) != HAL_OK)
    {
        /*while (1) {
          //HAL_GPIO_TogglePin(GPIOD,BLUE_LED);
          //HAL_Delay(100);
        }*/
    }
}

void adcTest(void const *argument) {
/*  ConfigureADC();
  HAL_ADC_Start(&g_AdcHandle);
  int g_MeasurementNumber;
  while (1) {
    osDelay(5);
    if (HAL_ADC_PollForConversion(&g_AdcHandle,1000000) == HAL_OK) {
      g_ADCValue = HAL_ADC_GetValue(&g_AdcHandle);
      g_MeasurementNumber++;
    }
  }*/
  //volatile int temp;
  eecs_ADC_Init();
  eecs_ADC_Begin();

  while (1) {
    osDelay(100);
    HAL_GPIO_TogglePin(GPIOB,GPIO_PIN_4);
  }

}

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  *
  * @retval None
  */
int main(void)
{
  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration----------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  eecs_GPIO_Clock_Init();
  //MX_GPIO_Init();
  //MX_UART4_Init();
  //MX_SPI1_Init();
  MX_I2C1_Init();
  MX_CAN1_Init();
  /* USER CODE BEGIN 2 */

  /* USER CODE END 2 */

  /* USER CODE BEGIN RTOS_MUTEX */
  /* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* add semaphores, ... */
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* Create the thread(s) */
  /* definition and creation of defaultTask */
  osThreadDef(defaultTask, StartDefaultTask, osPriorityNormal, 0, 128);
  defaultTaskHandle = osThreadCreate(osThread(defaultTask), NULL);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  //osThreadDef(uartTask, uartTest, osPriorityAboveNormal, 1, 128);
  //uartTaskHandle = osThreadCreate(osThread(uartTask), NULL);
  //osThreadDef(i2cTask, writei2c, osPriorityAboveNormal,1,256);
  //i2cTaskHandle = osThreadCreate(osThread(i2cTask),NULL);
  //osThreadDef(adcTask, adcTest, osPriorityAboveNormal,1,128);
  //adcTaskHandle = osThreadCreate(osThread(adcTask),NULL);
  osThreadDef(spiTask,spiTest,osPriorityAboveNormal,1,128);
  spiTaskHandle = osThreadCreate(osThread(spiTask),NULL);
  //osThreadDef(canTask, canTest, osPriorityAboveNormal, 1, 128);
  //canTaskHandle = osThreadCreate(osThread(canTask),NULL);

  
  //osThreadDef(ledTask, Leds, osPriorityAboveNormal, 1, 128);
  //ledTaskHandle = osThreadCreate(osThread(ledTask),NULL);

  /* USER CODE END RTOS_THREADS */

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */
 

  /* Start scheduler */
  osKernelStart();
  
  /* We should never get here as control is now taken by the scheduler */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {

  /* USER CODE END WHILE */

  /* USER CODE BEGIN 3 */

  }
  /* USER CODE END 3 */

}

/**
  * @brief System Clock Configuration
  * @retval None
  */
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



 /* RCC_PeriphCLKInitTypeDef periphClockConfig;
  HAL_RCCEx_GetPeriphCLKConfig(&periphClockConfig);
  periphClockConfig.Adc12ClockSelection = RCC_ADC12PLLCLK_DIV1;
  HAL_RCCEx_PeriphCLKConfig(&periphClockConfig);*/
}

/* CAN1 init function */
static void MX_CAN1_Init(void)
{

  hcan1.Instance = CAN1;
  hcan1.Init.Prescaler = 2;
  hcan1.Init.Mode = CAN_MODE_NORMAL;
  hcan1.Init.SyncJumpWidth = CAN_SJW_4TQ;
  hcan1.Init.TimeSeg1 = CAN_BS1_9TQ;
  hcan1.Init.TimeSeg2 = CAN_BS2_6TQ;
  hcan1.Init.TimeTriggeredMode = DISABLE;
  hcan1.Init.AutoBusOff = ENABLE;
  hcan1.Init.AutoWakeUp = DISABLE;
  hcan1.Init.AutoRetransmission = DISABLE;
  hcan1.Init.ReceiveFifoLocked = ENABLE;
  hcan1.Init.TransmitFifoPriority = ENABLE;
  if (HAL_CAN_Init(&hcan1) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

}

/* I2C1 init function */
static void MX_I2C1_Init(void)
{

  hi2c1.Instance = I2C2;
  hi2c1.Init.ClockSpeed = 100000;
  hi2c1.Init.DutyCycle = I2C_DUTYCYCLE_2;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

}

/* SPI1 init function */
static void MX_SPI1_Init(void)
{

  /* SPI1 parameter configuration*/
  hspi1.Instance = SPI1;
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
  hspi1.Init.CRCPolynomial = 10;
  if (HAL_SPI_Init(&hspi1) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }
  
}

/* UART4 init function */
static void MX_UART4_Init(void)
{

  huart4.Instance = UART4;
  huart4.Init.BaudRate = 57600;
  huart4.Init.WordLength = UART_WORDLENGTH_8B;
  huart4.Init.StopBits = UART_STOPBITS_1;
  huart4.Init.Parity = UART_PARITY_NONE;
  huart4.Init.Mode = UART_MODE_TX_RX;
  huart4.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart4.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart4) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

}

/** Configure pins as 
        * Analog 
        * Input 
        * Output
        * EVENT_OUT
        * EXTI
*/
static void MX_GPIO_Init(void)
{

  GPIO_InitTypeDef GPIO_InitStruct;

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOD, GPIO_PIN_12|GPIO_PIN_13|GPIO_PIN_14|GPIO_PIN_15, GPIO_PIN_RESET);

  /*Configure GPIO pins : PD12 PD13 PD14 PD15 */
  GPIO_InitStruct.Pin = GPIO_PIN_12|GPIO_PIN_13|GPIO_PIN_14|GPIO_PIN_15;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);


  GPIO_InitStruct.Pin = GPIO_PIN_8 | GPIO_PIN_9;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  GPIO_InitStruct.Pin = GPIO_PIN_4;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_SET);

}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/* USER CODE BEGIN Header_StartDefaultTask */
/**
  * @brief  Function implementing the defaultTask thread.
  * @param  argument: Not used 
  * @retval None
  */
/* USER CODE END Header_StartDefaultTask */
void StartDefaultTask(void const * argument)
{

  /* USER CODE BEGIN 5 */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END 5 */ 
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
  /* USER CODE BEGIN Callback 0 */

  /* USER CODE END Callback 0 */
  if (htim->Instance == TIM1) {
    HAL_IncTick();
  }
  /* USER CODE BEGIN Callback 1 */

  /* USER CODE END Callback 1 */
}

/**
  * @brief  This function is executed in case of error occurrence.
  * @param  file: The file name as string.
  * @param  line: The line in file as a number.
  * @retval None
  */
void _Error_Handler(char *file, int line)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  volatile int i;
  while(1)
  {
    HAL_GPIO_WritePin(GPIOD, BLUE_LED, GPIO_PIN_SET);
  }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t* file, uint32_t line)
{ 
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/**
  * @}
  */

/**
  * @}
  */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
