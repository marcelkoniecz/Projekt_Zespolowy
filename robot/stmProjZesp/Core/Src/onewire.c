/*
 * onewire.c
 *
 *  Created on: Apr 11, 2022
 *      Author: marcel
 */

#include "usart.h"
#include "onewire.h"

static void setBaudrate(uint32_t baudrate){
	  huart1.Instance = USART1;
	  huart1.Init.BaudRate = baudrate;
	  huart1.Init.WordLength = UART_WORDLENGTH_8B;
	  huart1.Init.StopBits = UART_STOPBITS_1;
	  huart1.Init.Parity = UART_PARITY_NONE;
	  huart1.Init.Mode = UART_MODE_TX_RX;
	  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
	  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
	  huart1.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
	  huart1.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_RXOVERRUNDISABLE_INIT;
	  huart1.AdvancedInit.OverrunDisable = UART_ADVFEATURE_OVERRUN_DISABLE;
	  if (HAL_HalfDuplex_Init(&huart1) != HAL_OK)
	  {
	    Error_Handler();
	  }

}

HAL_StatusTypeDef wireReset(void){

	uint8_t dataOut = 0xF0;
	uint8_t dataIn = 0;

	setBaudrate(9600);
	HAL_UART_Transmit(&huart1, &dataOut, 1,HAL_MAX_DELAY);
	HAL_UART_Receive(&huart1, &dataIn, 1, HAL_MAX_DELAY);
	setBaudrate(115200);

	if(dataIn!=0XF0)
		return HAL_OK;
	else
		return HAL_ERROR;
}


static void writeBit(int value){
  if (value) {
      uint8_t dataOut = 0xff;
    HAL_UART_Transmit(&huart1, &dataOut, 1, HAL_MAX_DELAY);
  } else {
      uint8_t dataOut = 0x0;
    HAL_UART_Transmit(&huart1, &dataOut, 1, HAL_MAX_DELAY);
  }
}

static int readBit(void){
  uint8_t dataOut = 0xFF;
  uint8_t dataIn = 0;
  HAL_UART_Transmit(&huart1, &dataOut, 1, HAL_MAX_DELAY);
  HAL_UART_Receive(&huart1, &dataIn, 1, HAL_MAX_DELAY);

  return dataIn & 0x01;
}
