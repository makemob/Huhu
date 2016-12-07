/*
 * comms.h
 *
 *  Created on: 12 Jun 2016
 *      Author: Chris Mock
 */

#ifndef COMMS_H_
#define COMMS_H_

#include "chip.h"

void CommsInit(uint32_t baud);

void CommsSendString(uint8_t* str);

void CommsSendChars(uint8_t* str, uint8_t numChars);

int16_t CommsReadKey(uint8_t *result);

int16_t CommsReadChars(uint8_t *buffer, uint8_t bufferSize);

uint8_t CommsGetNumRxChars(void);

void CommsFlushRxBuffer(void);

void CommsSetTxEnableDelay(uint16_t delay);

void CommsPoll(void);

#endif /* COMMS_H_ */
