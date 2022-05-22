/*
 * ds18b20.h
 *
 *  Created on: May 16, 2022
 *      Author: marcel
 */

#ifndef INC_DS18B20_H_
#define INC_DS18B20_H_


#pragma once

#include "main.h"

#define DS18B20_ROM_CODE_SIZE		8


HAL_StatusTypeDef ds18b20_init(void);
HAL_StatusTypeDef ds18b20_read_address(uint8_t* rom_code);
HAL_StatusTypeDef ds18b20_start_measure(const uint8_t* rom_code);
float ds18b20_get_temp(const uint8_t* rom_code);

#endif /* INC_DS18B20_H_ */
