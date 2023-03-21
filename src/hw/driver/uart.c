/*
 * uart.c
 *
 *  Created on: 2020. 12. 8.
 *      Author: baram
 */


#include "uart.h"
#include "ring_buffer.h"
#include "usbd_cdc_if.h"


static bool is_open[UART_MAX_CH];

static Queue_t ring_buffer[UART_MAX_CH];
//static uint8_t u1_rx_buf[MAX_SIZE];
static uint8_t u2_rx_buf[MAX_SIZE];


UART_HandleTypeDef huart2;
DMA_HandleTypeDef hdma_usart2_rx;

bool uartInit(void)
{
  for (int i=0; i<UART_MAX_CH; i++)
  {
    is_open[i] = false;
  }

  return true;
}

bool uartOpen(uint8_t ch, uint32_t baud)
{
  bool ret = false;

  switch(ch)
  {
    case _DEF_USB:
      is_open[ch] = true;
      ret = true;
      break;

    case _DEF_UART2:
    	huart2.Instance = USART2;
    	huart2.Init.BaudRate = baud;
    	huart2.Init.WordLength = UART_WORDLENGTH_8B;
      huart2.Init.StopBits = UART_STOPBITS_1;
    	huart2.Init.Parity = UART_PARITY_NONE;
    	huart2.Init.Mode = UART_MODE_TX_RX;
    	huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
    	huart2.Init.OverSampling = UART_OVERSAMPLING_16;

    	QueueCreate(&ring_buffer[ch], &u2_rx_buf[0], MAX_SIZE);

    	if (HAL_UART_Init(&huart2) != HAL_OK)
    	{
    	  Error_Handler();
    	}
    	else
    	{
    		ret = true;
        is_open[ch] = true;
        if(HAL_UART_Receive_DMA(&huart2, (uint8_t *)&u2_rx_buf[0], MAX_SIZE) != HAL_OK)
        {
          ret = false;
        }
        ring_buffer[ch].head  = ring_buffer[ch].size - hdma_usart2_rx.Instance->NDTR;
        ring_buffer[ch].tail = ring_buffer[ch].head;
    	}
      break;
  }

  return ret;
}

uint32_t uartAvailable(uint8_t ch)
{
  uint32_t ret = 0;

  switch(ch)
  {

    case _DEF_USB:
      ret = cdcAvailable();
      break;

    case _DEF_UART2:
    	ring_buffer[ch].head = (ring_buffer[ch].size - hdma_usart2_rx.Instance->NDTR);
      ret = QueueAvailable(&ring_buffer[ch]);
      break;
  }
  return ret;
}

uint8_t uartRead(uint8_t ch)
{
  uint8_t ret = 0;

  switch(ch)
  {
    case _DEF_UART2:
    	Q_read(&ring_buffer[_DEF_UART2], &ret, 1);
      break;

    case _DEF_USB:
      ret = cdcRead();
      break;
  }

  return ret;
}

uint32_t uartWrite(uint8_t ch, uint8_t *p_data, uint32_t length)
{
  uint32_t ret = 0;
  HAL_StatusTypeDef status;

  switch(ch)
  {
    case _DEF_UART2:
      status = HAL_UART_Transmit(&huart2, p_data, length, 100);
      if (status == HAL_OK)
      {
        ret = length;
      }
      break;

    case _DEF_USB:
      ret = cdcWrite(p_data, length);
      break;
  }

  return ret;
}

uint32_t uartWriteIT(uint8_t ch, uint8_t *p_data, uint32_t length)
{
  uint32_t ret = 0;
  HAL_StatusTypeDef status;

  switch(ch)
  {

    case _DEF_UART2:
      status = HAL_UART_Transmit_IT(&huart2, p_data, length);
      if (status == HAL_OK)
      {
        ret = length;
      }
      break;

    case _DEF_USB:
      status = HAL_UART_Transmit_IT(&huart2, p_data, length);
      if (status == HAL_OK)
      {
        ret = length;
      }
      break;
  }

  return ret;
}

uint32_t uartPrintf(uint8_t ch, char *fmt, ...)
{
  char buf[MAX_SIZE];
  va_list args;
  int len;
  uint32_t ret;

  va_start(args, fmt);
  len = vsnprintf(buf, MAX_SIZE, fmt, args);

  ret = uartWrite(ch, (uint8_t *)buf, len);

  va_end(args);


  return ret;
}

uint32_t uartPrintf_IT(uint8_t ch, char *fmt, ...)
{
  char buf[MAX_SIZE];
  va_list args;
  int len;
  uint32_t ret;

  va_start(args, fmt);
  len = vsnprintf(buf, MAX_SIZE, fmt, args);

  ret = uartWriteIT(ch, (uint8_t *)buf, len);

  va_end(args);


  return ret;
}

uint32_t uartGetBaud(uint8_t ch)
{
  uint32_t ret = 0;


  switch(ch)
  {
    case _DEF_USB:
      ret = cdcGetBaud();
      break;

    case _DEF_UART2:
      ret = huart2.Init.BaudRate;
      break;
  }

  return ret;
}

bool uartSetBaud(uint8_t ch, uint32_t baud)
{
	bool ret = false;

	switch(ch)
	{
    case _DEF_USB:
			huart2.Init.BaudRate = baud;
    	if (HAL_UART_Init(&huart2) != HAL_OK)
    	{
    	  Error_Handler();
    	}else
    	{
    		ret = true;
    	}
			break;

		case _DEF_UART2:
			huart2.Init.BaudRate = baud;
    	if (HAL_UART_Init(&huart2) != HAL_OK)
    	{
    	  Error_Handler();
    	}else
    	{
    		ret = true;
    	}
			break;
	}

	return ret;
}




void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart)
{
  if (huart->Instance == USART2)
  {
  }

  if(huart->ErrorCode == HAL_UART_ERROR_FE) //current USART
  {
    HAL_UART_Receive_DMA(&huart2, (uint8_t*)&u2_rx_buf[0], 1);
  }
  if(huart->ErrorCode == HAL_UART_ERROR_NE) //current USART
  {
    HAL_UART_Receive_DMA(&huart2, (uint8_t*)&u2_rx_buf[0], 1);
  }
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
  //  if (huart->Instance == USART1)
  //  {
  //  	Q_write(&ring_buffer[_DEF_UART1], &rx_data[_DEF_UART1], 1);
  //  }
  // if(huart->Instance == USART2)
  // {
  //   Q_write(&ring_buffer[_DEF_UART2], &u2_rx_buf[0], 1);
  //   cliPrintf("%d",u2_rx_buf[0]);
  // }
}

void HAL_UART_MspInit(UART_HandleTypeDef* uartHandle)
{
 GPIO_InitTypeDef GPIO_InitStruct = {0};
  if(uartHandle->Instance==USART2)
  {
  /* USER CODE BEGIN USART2_MspInit 0 */

  /* USER CODE END USART2_MspInit 0 */
    /* USART2 clock enable */
    __HAL_RCC_USART2_CLK_ENABLE();

    __HAL_RCC_GPIOA_CLK_ENABLE();
    /**USART2 GPIO Configuration
    PA2     ------> USART2_TX
    PA3     ------> USART2_RX
    */
    GPIO_InitStruct.Pin = GPIO_PIN_2|GPIO_PIN_3;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    GPIO_InitStruct.Alternate = GPIO_AF7_USART2;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

    /* USART2 DMA Init */
    /* USART2_RX Init */
    hdma_usart2_rx.Instance = DMA1_Stream5;
    hdma_usart2_rx.Init.Channel = DMA_CHANNEL_4;
    hdma_usart2_rx.Init.Direction = DMA_PERIPH_TO_MEMORY;
    hdma_usart2_rx.Init.PeriphInc = DMA_PINC_DISABLE;
    hdma_usart2_rx.Init.MemInc = DMA_MINC_ENABLE;
    hdma_usart2_rx.Init.PeriphDataAlignment = DMA_PDATAALIGN_BYTE;
    hdma_usart2_rx.Init.MemDataAlignment = DMA_MDATAALIGN_BYTE;
    hdma_usart2_rx.Init.Mode = DMA_NORMAL;
    hdma_usart2_rx.Init.Priority = DMA_PRIORITY_LOW;
    hdma_usart2_rx.Init.FIFOMode = DMA_FIFOMODE_DISABLE;
    if (HAL_DMA_Init(&hdma_usart2_rx) != HAL_OK)
    {
      Error_Handler();
    }

    __HAL_LINKDMA(uartHandle,hdmarx,hdma_usart2_rx);

    /* USART2 interrupt Init */
    HAL_NVIC_SetPriority(USART2_IRQn, 0, 0);
    HAL_NVIC_EnableIRQ(USART2_IRQn);
  /* USER CODE BEGIN USART2_MspInit 1 */

  /* USER CODE END USART2_MspInit 1 */
  }
}

void HAL_UART_MspDeInit(UART_HandleTypeDef* uartHandle)
{

  if(uartHandle->Instance==USART2)
  {
  /* USER CODE BEGIN USART2_MspDeInit 0 */

  /* USER CODE END USART2_MspDeInit 0 */
    /* Peripheral clock disable */
    __HAL_RCC_USART2_CLK_DISABLE();

    /**USART2 GPIO Configuration
    PA2     ------> USART2_TX
    PA3     ------> USART2_RX
    */
    HAL_GPIO_DeInit(GPIOA, GPIO_PIN_2|GPIO_PIN_3);

    /* USART2 DMA DeInit */
    HAL_DMA_DeInit(uartHandle->hdmarx);

    /* USART2 interrupt Deinit */
    HAL_NVIC_DisableIRQ(USART2_IRQn);
  /* USER CODE BEGIN USART2_MspDeInit 1 */

  /* USER CODE END USART2_MspDeInit 1 */
  }
}
