/*
 * ds18b20.h
 *
 *  Created on: Apr 11, 2022
 *      Author: marcel
 */

#ifndef INC_DS18B20_H_
#define INC_DS18B20_H_


#define DS18B20_SCRATCHPAD_SIZE 9

//Rom commands
#define DS18B20_SEARCH_ROM 			0xF0
#define DS18B20_READ_ROM			0x33
#define DS18B20_MATCH_ROM			0x55
#define DS18B20_SKIP_ROM			0xCC
#define DS18B20_ALARM_SEARCH		0xEC


//Temperature conversion commands
#define DS18B20_CONVERTT 			0X44

//Memory commands
#define DS18B20_READ_SCRATCHPAD		0xBE
#define DS18B20_WRITE_SCRATCHPAD	0x4E
#define DS18B20_COPY_SCRATCHPAD		0x48
#define DS18B20_RECALL_E2			0xB8
#define DS18B20_READ_POW_SUPPLY		0xB4



#endif /* INC_DS18B20_H_ */
