/*
 * wire.h
 *
 *  Created on: May 16, 2022
 *      Author: marcel
 */

#ifndef INC_WIRE_H_
#define INC_WIRE_H_

#pragma once

#include "main.h"
TIM_HandleTypeDef htim6;
HAL_StatusTypeDef wire_init(void);
HAL_StatusTypeDef wire_reset(void);
uint8_t wire_read(void);
void wire_write(uint8_t byte);
uint8_t wire_crc(const uint8_t* data, int len);

#endif /* INC_WIRE_H_ */
