/*
 * onewire.h
 *
 *  Created on: Apr 11, 2022
 *      Author: marcel
 */

#ifndef INC_ONEWIRE_H_
#define INC_ONEWIRE_H_

static void setBaudrate(uint32_t baudrate);
HAL_StatusTypeDef wireReset(void);
static void writeBit(int value);
static int readBit(void);

#endif /* INC_ONEWIRE_H_ */
