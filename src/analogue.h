/*
 * analogue.h
 *
 *  Created on: 12 Jun 2016
 *      Author: Chris Mock
 */

#ifndef ANALOGUE_H_
#define ANALOGUE_H_

void ADCInit(void);

void ADCTakeSamples(void);

uint16_t ADCGetTemperature(void);

uint16_t ADCGetBattVoltage(void);

uint16_t ADCGetBridgeCurrent(void);

uint16_t ADCGetExt1(void);

uint16_t ADCGetExt2(void);

bool ADCIsReady(void);

#endif /* ANALOGUE_H_ */
