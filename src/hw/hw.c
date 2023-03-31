/*
 * hw.c
 *
 *  Created on: Dec 6, 2020
 *      Author: baram
 */


#include "hw.h"

static void MX_DMA_Init(void);

typedef struct
{
  uint32_t magic_number;
  char version_str[32];
} version_info_t;

__attribute__((section(".pg_registry"))) version_info_t verstion_info =
{
  .magic_number = 0x5555AAAA,
  .version_str = "V230331R1"
};

const version_info_t *p_verstion_info = (version_info_t *)0x8000400;


void hwInit(void)
{
  bspInit();
  gpioInit();
  ledInit();
  MX_DMA_Init();
  usbInit();
  uartInit();
  cliInit();
  i2cInit();
  spiInit();

	// ledOn(ST1);
  // ledOff(ST2);
  // for (int i = 0; i < 10; i++){
  //   ledToggle(ST1);
  //   ledToggle(ST2);
  //   HAL_Delay(25);
  //   //BEEP_ON;
  //   HAL_Delay(25);
  //   //BEEP_OFF;
  // }
  // ledOff(ST1);
  // ledOff(ST2);
}


/**
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA1_CLK_ENABLE();
  __HAL_RCC_DMA2_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Stream3_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Stream3_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Stream3_IRQn);
  /* DMA1_Stream4_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Stream4_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Stream4_IRQn);
  /* DMA1_Stream5_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Stream5_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Stream5_IRQn);
  /* DMA2_Stream2_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA2_Stream2_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA2_Stream2_IRQn);
  /* DMA2_Stream3_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA2_Stream3_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA2_Stream3_IRQn);

}