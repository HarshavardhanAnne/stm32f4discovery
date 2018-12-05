//EECS 473 - Advanced Embedded Systems 
//UM Solar Car

#include <stdlib.h>
#include <stdint.h>

#define ADC_BUFFER_LENGTH 4

UART_HandleTypeDef huart;
SPI_HandleTypeDef hspi2;
SPI_HandleTypeDef hspi3;
I2C_HandleTypeDef hi2c;
CAN_HandleTypeDef hcan;
ADC_HandleTypeDef hadc1;
DMA_HandleTypeDef hdma;

uint32_t adcbuffer[ADC_BUFFER_LENGTH];
uint16_t spiBuffer[500] = {0};
uint16_t spiBufferCount = 0;

struct pin_pair {
  GPIO_TypeDef* const gpiox;
  const uint16_t pin;
};

struct eecsSPI {
  SPI_HandleTypeDef* hspi;
  uint8_t csindex[4];
  uint16_t candata[4];
  uint16_t candata2[4];
  uint8_t drdy;
};

struct eecsADC {
  uint16_t data[4];
};

struct eecsCAN {
  uint8_t* data_ptr;
  CAN_TxHeaderTypeDef tx_buffer;
};

struct eecsI2C {
  uint8_t address;
  uint8_t tx_buff[6];
  uint8_t rx_buff[6];
  uint16_t data[4];
};

struct pin_pair cs1a = {GPIOA,GPIO_PIN_4};
struct pin_pair cs1b = {GPIOA,GPIO_PIN_3};
struct pin_pair cs2a = {GPIOB,GPIO_PIN_1};
struct pin_pair cs2b = {GPIOA,GPIO_PIN_2};
struct pin_pair cs3a = {GPIOC,GPIO_PIN_7};
struct pin_pair cs3b = {GPIOC,GPIO_PIN_6};
struct pin_pair cs4a = {GPIOC,GPIO_PIN_9};
struct pin_pair cs4b = {GPIOC,GPIO_PIN_8};

struct pin_pair* cs[8] = {&cs1a,&cs1b,&cs2a,&cs2b,&cs3a,&cs3b,&cs4a,&cs4b};

struct eecsSPI spiA;
struct eecsSPI spiB;
struct eecsI2C i2c;
struct eecsADC adc;
struct eecsI2C i2c;
struct eecsCAN can;

GPIO_PinState HIGH = GPIO_PIN_SET;
GPIO_PinState LOW = GPIO_PIN_RESET;

int GPIO_CLOCK_ENABLED = 0;

void eecs_Error_Handler();
void eecs_GPIO_Init(GPIO_TypeDef*,uint16_t,uint32_t,uint32_t,uint32_t);
void eecs_GPIO_Write(GPIO_TypeDef*,uint16_t,uint8_t);
void eecs_GPIO_Toggle(GPIO_TypeDef*,uint16_t);

void eecs_UART_Init(void);
void eecs_UART_Print(uint8_t*,uint8_t);
void eecs_UART_Debug(uint8_t*,uint8_t);

void eecs_I2C_Init(void);

void eecs_SPI_Init(int);
void eecs_SPI_ReadSetupReg(struct eecsSPI* , GPIO_TypeDef* , uint16_t );
void eecs_SPI_Begin(struct eecsSPI* ,uint8_t );
void eecs_SPI_Wait(struct eecsSPI* ,GPIO_TypeDef* ,uint16_t );
void eecs_SPI_Read(struct eecsSPI* ,uint8_t ,uint8_t );
uint16_t average(uint16_t data[]);


void eecs_ADC_Init();
void eecs_ADC_ConfigureDMA();
void eecs_ADC_Begin();
void eecs_ADC_Read();

void eecs_CAN_Init();
uint32_t eecs_CAN_Mail_Ready(uint32_t);
void eecs_CAN_Set_Params(uint32_t,uint32_t,uint16_t *);
void eecs_CAN_Send();

void eecs_GPIO_Clock_Init(void) {
  if (!GPIO_CLOCK_ENABLED) {
    /* GPIO Ports Clock Enable */
    __HAL_RCC_GPIOA_CLK_ENABLE();
    __HAL_RCC_GPIOD_CLK_ENABLE();
    __HAL_RCC_GPIOB_CLK_ENABLE();
    __HAL_RCC_GPIOC_CLK_ENABLE();
    GPIO_CLOCK_ENABLED = 1;
  }
}

void eecs_GPIO_Init(GPIO_TypeDef *GPIOx,uint16_t Pins,uint32_t Mode,uint32_t Pull,uint32_t Speed) {
  GPIO_InitTypeDef GPIO_InitStruct;
  GPIO_InitStruct.Pin = Pins;
  GPIO_InitStruct.Mode = Mode;
  GPIO_InitStruct.Pull = Pull;
  GPIO_InitStruct.Speed = Speed;
  HAL_GPIO_Init(GPIOx, &GPIO_InitStruct);
}

void eecs_GPIO_Write(GPIO_TypeDef *GPIOx,uint16_t Pin, GPIO_PinState PinState) {
  HAL_GPIO_WritePin(GPIOx,Pin,PinState);
}

void eecs_GPIO_Toggle(GPIO_TypeDef *GPIOx, uint16_t Pin) {
  HAL_GPIO_TogglePin(GPIOx,Pin);
}

void eecs_UART_Init(void) {
  huart.Instance = UART4;
  huart.Init.BaudRate = 115200;
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

void eecs_UART_Print(uint8_t* arr, uint8_t buffsize) {
  HAL_StatusTypeDef status = HAL_UART_Transmit(&huart,arr,buffsize,HAL_MAX_DELAY);
}

void eecs_SPI_Init(int spi_bus) {
  if (spi_bus == 2) {
    hspi2.Instance = SPI2;
    hspi2.Init.Mode = SPI_MODE_MASTER;
    hspi2.Init.Direction = SPI_DIRECTION_2LINES;
    hspi2.Init.DataSize = SPI_DATASIZE_8BIT;
    hspi2.Init.CLKPolarity = SPI_POLARITY_HIGH;
    hspi2.Init.CLKPhase = SPI_PHASE_2EDGE;
    hspi2.Init.NSS = SPI_NSS_SOFT;
    hspi2.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_2;
    hspi2.Init.FirstBit = SPI_FIRSTBIT_MSB;
    hspi2.Init.TIMode = SPI_TIMODE_DISABLE;
    hspi2.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
    hspi2.Init.CRCPolynomial = 10;

    //spiB.hspi = &hspi2;
    if (HAL_SPI_Init(&hspi2) != HAL_OK)
    {
      eecs_Error_Handler();
    }

    spiB.hspi = &hspi2;
    spiB.csindex[0] = 4;
    spiB.csindex[1] = 5;
    spiB.csindex[2] = 6;
    spiB.csindex[3] = 7;
    spiB.candata[0] = 0xAAAA;
    spiB.candata[1] = 0xAAAA;
    spiB.candata[2] = 0xAAAA;
    spiB.candata[3] = 0xAAAA;
    spiB.candata2[0] = 0xAAAA;
    spiB.candata2[1] = 0xAAAA;
    spiB.candata2[2] = 0xAAAA;
    spiB.candata2[3] = 0xAAAA;
  }
  else if (spi_bus == 3) {
    hspi3.Instance = SPI3;
    hspi3.Init.Mode = SPI_MODE_MASTER;
    hspi3.Init.Direction = SPI_DIRECTION_2LINES;
    hspi3.Init.DataSize = SPI_DATASIZE_8BIT;
    hspi3.Init.CLKPolarity = SPI_POLARITY_HIGH;
    hspi3.Init.CLKPhase = SPI_PHASE_2EDGE;
    hspi3.Init.NSS = SPI_NSS_SOFT;
    hspi3.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_2;
    hspi3.Init.FirstBit = SPI_FIRSTBIT_MSB;
    hspi3.Init.TIMode = SPI_TIMODE_DISABLE;
    hspi3.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
    hspi3.Init.CRCPolynomial = 10;

    //spiA.hspi = &hspi3;
    if (HAL_SPI_Init(&hspi3) != HAL_OK)
    {
      eecs_Error_Handler();
    }

    spiA.hspi = &hspi3;
    spiA.csindex[0] = 0;
    spiA.csindex[1] = 1;
    spiA.csindex[2] = 2;
    spiA.csindex[3] = 3;
    spiA.candata[0] = 0xAAAA;
    spiA.candata[1] = 0xAAAA;
    spiA.candata[2] = 0xAAAA;
    spiA.candata[3] = 0xAAAA;
    spiA.candata2[0] = 0xAAAA;
    spiA.candata2[1] = 0xAAAA;
    spiA.candata2[2] = 0xAAAA;
    spiA.candata2[3] = 0xAAAA;
  }
}

void eecs_SPI_ReadSetupReg(struct eecsSPI* spi, GPIO_TypeDef* gpiox, uint16_t pin) {
  uint8_t temp;
  uint8_t tx = 0x18;

  HAL_GPIO_WritePin(gpiox,pin,GPIO_PIN_RESET);

  if (HAL_SPI_TransmitReceive(spi->hspi,&tx,&temp,1,HAL_MAX_DELAY)!=HAL_OK) {
    //do something
  }
  tx = 0x00;
  if (HAL_SPI_TransmitReceive(spi->hspi,&tx,&temp,1,HAL_MAX_DELAY)!=HAL_OK) {
    //do something
  }
  HAL_GPIO_WritePin(gpiox,pin,GPIO_PIN_SET);
}

void eecs_SPI_Begin(struct eecsSPI* spi,uint8_t csPin) {
  uint8_t rxbuff[2];
  uint8_t txbuff[4] = {0x20,0xA7,0x10,0x44};

  uint8_t txoffset = 0;
  volatile HAL_StatusTypeDef status;
  if (csPin < 0 || csPin > 3) {
    //uart error msg
    return;
  }
  uint8_t csidx = spi->csindex[csPin];

  GPIO_TypeDef* temp_gpiox = (cs[csidx])->gpiox;
  uint16_t pinnum = cs[csidx]->pin;

  HAL_GPIO_WritePin(temp_gpiox,pinnum,GPIO_PIN_RESET);

  status = HAL_SPI_TransmitReceive(spi->hspi,txbuff+txoffset,rxbuff,1,HAL_MAX_DELAY);
  HAL_Delay(100);
  if (status != HAL_OK) {
    //do something
  }
  txoffset++;

  status = HAL_SPI_TransmitReceive(spi->hspi,txbuff+txoffset,rxbuff,1,HAL_MAX_DELAY);
  HAL_Delay(100);
  if (status != HAL_OK) {
    //do something
  }
  txoffset++;

  status = HAL_SPI_TransmitReceive(spi->hspi,txbuff+txoffset,rxbuff,1,HAL_MAX_DELAY);
  HAL_Delay(100);
  if (status != HAL_OK) {
    //do something
  }
  txoffset++;

  status = HAL_SPI_TransmitReceive(spi->hspi,txbuff+txoffset,rxbuff,1,HAL_MAX_DELAY);
  if (status != HAL_OK) {
    //do something
  }
  HAL_GPIO_WritePin(temp_gpiox,pinnum,GPIO_PIN_SET);

  HAL_Delay(100);
  eecs_SPI_ReadSetupReg(spi, temp_gpiox,pinnum);
  return;
}


void eecs_SPI_Wait(struct eecsSPI* spi,GPIO_TypeDef* gpiox,uint16_t pin) {
  uint8_t rxbuff;
  uint8_t txbuff[2] = {0x08,0x00};
  uint8_t MAXRETRIES = 0;
  spi->drdy = 0x80;

  HAL_GPIO_WritePin(gpiox,pin,GPIO_PIN_RESET);

  while (spi->drdy && (MAXRETRIES < 5)) {
    if (HAL_SPI_TransmitReceive(spi->hspi,txbuff,&rxbuff,1,2) != HAL_OK) {
      //do something
    }
    if (HAL_SPI_TransmitReceive(spi->hspi,txbuff+1,&(spi->drdy),1,2) != HAL_OK) {
      //do something
    }
    spi->drdy &= 0x80;
    MAXRETRIES++;
  }
  HAL_GPIO_WritePin(gpiox,pin,GPIO_PIN_SET);
}

void eecs_SPI_Read(struct eecsSPI* spi,uint8_t csPin,uint8_t channel) {
  uint16_t i = 0;
  uint16_t sum = 0;
  uint8_t txbuff[2] = {0x38,0x00};
  uint8_t rxbuff[2];
  uint16_t oldValue;
  uint16_t newValue;
  txbuff[0] += channel;
  uint8_t csidx = spi->csindex[csPin];
  GPIO_TypeDef* temp_gpiox = (cs[csidx])->gpiox;
  uint16_t temppin = cs[csidx]->pin;
  //uint8_t rxoffset = (4 * csPin + 2 * channel)*sizeof(uint8_t);
  uint8_t rxoffset = (2 * (csPin%2) + channel)*sizeof(uint8_t);
  volatile HAL_StatusTypeDef status;

  eecs_SPI_Wait(spi, temp_gpiox, temppin);

  HAL_GPIO_WritePin(temp_gpiox,temppin,GPIO_PIN_RESET);

  status = HAL_SPI_TransmitReceive(spi->hspi,txbuff,rxbuff,1,2);
  if (status!=HAL_OK) {
    //do something
  }
  status = HAL_SPI_TransmitReceive(spi->hspi,txbuff+1,rxbuff+1,1,2);
  if (status!=HAL_OK) {
    //do something
  }
  status = HAL_SPI_TransmitReceive(spi->hspi,txbuff+1,rxbuff,1,2);
  if (status!=HAL_OK) {
    //do something
  }

  HAL_GPIO_WritePin(temp_gpiox,temppin,GPIO_PIN_SET);

  newValue = (rxbuff[0] << 8) + rxbuff[1];
  //FEEDBACK FILTER
  if (csPin < 2) {
    oldValue = *(spi->candata+rxoffset);
    oldValue += (newValue - oldValue) / 200; //Tune N=500 for better sampling

    *(spi->candata+rxoffset) = oldValue;
  }
  else {
    oldValue = *(spi->candata2+rxoffset);
    oldValue += (newValue - oldValue) / 200;
    *(spi->candata2+rxoffset) = oldValue;
  }
}

uint16_t average(uint16_t data[])
{
  uint32_t sum = 0;
  uint16_t avg = 0;

  uint16_t i = 0;
  for (i = 0; i < 500; ++i) {
    sum += data[i];
  }
  avg = sum / 500;
  return avg;
}

void eecs_I2C_Init(void) {
  uint8_t i;
  uint8_t addr = 59;
  hi2c.Instance = I2C2;
  hi2c.Init.ClockSpeed = 400000;
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
  for (i = 0; i < 6; i++) {
    i2c.tx_buff[i] = addr++;
  }
  i2c.address = (uint8_t)(0b1101000 << 1);
}

void eecs_I2C_Read() {
  int i;
  for (i = 0; i < 6; i++) {
    HAL_I2C_Master_Transmit(&hi2c,i2c.address,i2c.tx_buff+i,sizeof(uint8_t),10);
    HAL_I2C_Master_Receive(&hi2c,i2c.address,i2c.rx_buff+i,sizeof(uint8_t),10);
  }
  i2c.data[0] = 0;
  i2c.data[1] = (i2c.rx_buff[0] << 8) + i2c.rx_buff[1];
  i2c.data[2] = (i2c.rx_buff[2] << 8) + i2c.rx_buff[3];
  i2c.data[3] = (i2c.rx_buff[4] << 8) + i2c.rx_buff[5];
}

void eecs_ADC_Init(void) {
  __ADC1_CLK_ENABLE();

  HAL_NVIC_SetPriority(ADC_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(ADC_IRQn);
 
  ADC_ChannelConfTypeDef adcChannel;

  hadc1.Instance = ADC1;
  hadc1.Init.ClockPrescaler = ADC_CLOCKPRESCALER_PCLK_DIV2;
  hadc1.Init.Resolution = ADC_RESOLUTION_12B;
  hadc1.Init.ScanConvMode = ENABLE;
  hadc1.Init.ContinuousConvMode = ENABLE;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.NbrOfDiscConversion = 0;
  hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc1.Init.ExternalTrigConv = ADC_EXTERNALTRIGCONV_T1_CC1;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.NbrOfConversion = 4;
  hadc1.Init.DMAContinuousRequests = ENABLE;
  hadc1.Init.EOCSelection = DISABLE;

  HAL_ADC_Init(&hadc1);

  //steering sensor should have rank=1;
  adcChannel.Channel = ADC_CHANNEL_13; //steering sensor channel
  adcChannel.Rank = 4;
  adcChannel.SamplingTime = ADC_SAMPLETIME_480CYCLES;
  adcChannel.Offset = 0; 
  if (HAL_ADC_ConfigChannel(&hadc1,&adcChannel) != HAL_OK) {
    //print uart debug message
  }

  adcChannel.Channel = ADC_CHANNEL_14;
  adcChannel.Rank = 1;
  adcChannel.SamplingTime = ADC_SAMPLETIME_480CYCLES;
  adcChannel.Offset = 0; 
  if (HAL_ADC_ConfigChannel(&hadc1,&adcChannel) != HAL_OK) {
    //print uart debug message
  }
  adcChannel.Channel = ADC_CHANNEL_15;
  adcChannel.Rank = 2;
  adcChannel.SamplingTime = ADC_SAMPLETIME_480CYCLES;
  adcChannel.Offset = 0; 
  if (HAL_ADC_ConfigChannel(&hadc1,&adcChannel) != HAL_OK) {
    //print uart debug message
  }
  adcChannel.Channel = ADC_CHANNEL_8;
  adcChannel.Rank = 3;
  adcChannel.SamplingTime = ADC_SAMPLETIME_480CYCLES;
  adcChannel.Offset = 0; 
  if (HAL_ADC_ConfigChannel(&hadc1,&adcChannel) != HAL_OK) {
    //print uart debug message
  }


  //eecs_ADC_ConfigureDMA();
}

void eecs_ADC_ConfigureDMA(void) {
  __DMA2_CLK_ENABLE();
  hdma.Instance = DMA2_Stream4;
  hdma.Init.Channel = DMA_CHANNEL_0;
  hdma.Init.Direction = DMA_PERIPH_TO_MEMORY;
  hdma.Init.MemInc = DMA_MINC_ENABLE;
  hdma.Init.PeriphDataAlignment = DMA_PDATAALIGN_WORD;
  hdma.Init.MemDataAlignment = DMA_MDATAALIGN_WORD;
  hdma.Init.Mode = DMA_CIRCULAR;
  hdma.Init.Priority = DMA_PRIORITY_HIGH;
  hdma.Init.FIFOMode = DMA_FIFOMODE_DISABLE;
  hdma.Init.FIFOThreshold = DMA_FIFO_THRESHOLD_HALFFULL;
  hdma.Init.MemBurst = DMA_MBURST_SINGLE;
  hdma.Init.PeriphBurst = DMA_PBURST_SINGLE;

  HAL_DMA_Init(&hdma);
  __HAL_LINKDMA(&hadc1,DMA_Handle,hdma);
  HAL_NVIC_SetPriority(DMA2_Stream4_IRQn,0,0);
  HAL_NVIC_EnableIRQ(DMA2_Stream4_IRQn);
}

void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef* AdcHandle) {
  int i;
  //osSemaphoreWait(adcSemaphore,osWaitForever);
  //HAL_GPIO_WritePin(GPIOB,GPIO_PIN_4,GPIO_PIN_SET);
  for (i = 0; i < ADC_BUFFER_LENGTH; i++) {
    adc.data[i] = (uint16_t)(adcbuffer[i]);
  }
  //HAL_GPIO_WritePin(GPIOB,GPIO_PIN_4,GPIO_PIN_RESET);
  //osSemaphoreRelease(adcSemaphore);
}

void HAL_ADCEx_InjectedConvCpltCallback(ADC_HandleTypeDef* AdcHandle) {
  //HAL_GPIO_TogglePin(GPIOB,GPIO_PIN_4);
}

void DMA2_Stream4_IRQHandler() {
  HAL_DMA_IRQHandler(&hdma);
}
void ADC_IRQHandler() {
  HAL_ADC_IRQHandler(&hadc1);
}

void eecs_ADC_Begin(void) {
  eecs_ADC_ConfigureDMA();
  HAL_ADC_Start_DMA(&hadc1,adcbuffer,ADC_BUFFER_LENGTH);
}

void eecs_CAN_Init() {
  hcan.Instance = CAN1;
  hcan.Init.Prescaler = 2;
  hcan.Init.Mode = CAN_MODE_NORMAL;
  hcan.Init.SyncJumpWidth = CAN_SJW_4TQ;
  hcan.Init.TimeSeg1 = CAN_BS1_9TQ;
  hcan.Init.TimeSeg2 = CAN_BS2_6TQ;
  hcan.Init.TimeTriggeredMode = DISABLE;
  hcan.Init.AutoBusOff = ENABLE;
  hcan.Init.AutoWakeUp = DISABLE;
  hcan.Init.AutoRetransmission = DISABLE;
  hcan.Init.ReceiveFifoLocked = ENABLE;
  hcan.Init.TransmitFifoPriority = ENABLE;
  if (HAL_CAN_Init(&hcan) != HAL_OK)
  {
    eecs_Error_Handler();
  }
}

void eecs_CAN_Set_Params(uint32_t id,uint32_t size,uint16_t *data) {
  can.tx_buffer.StdId = id;
  can.tx_buffer.DLC = size;
  can.data_ptr = data;
}

uint32_t eecs_CAN_Mail_Ready(uint32_t mailbox) {
  return HAL_CAN_IsTxMessagePending(&hcan,(uint32_t)mailbox);
}
void eecs_CAN_Send(uint32_t mailbox) {
  HAL_CAN_AddTxMessage(&hcan,&(can.tx_buffer),can.data_ptr,(uint32_t *)mailbox);
}

void eecs_Error_Handler() {
  while (1) {
    //eecs_GPIO_Toggle(GPIOD, GPIO_PIN_15);
    HAL_Delay(1000);
  }
}