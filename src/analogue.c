/*
 * analogue.c
 *
 *  Created on: 12 Jun 2016
 *      Author: Chris Mock
 */


#include "chip.h"
#include "hardware.h"
#include <string.h>  // for memset()

#define ADC_SAMPLE_WIDTH (128)  // Number of samples to average ADC readings over

typedef struct {
	uint8_t sampleNum;
	bool valid;
	uint16_t sample[ADC_SAMPLE_WIDTH];
} ADCSample_t;

typedef enum {
	SAMPLE_ADC_TEMPERATURE = 0,
	SAMPLE_ADC_BATT_V,
	SAMPLE_ADC_CURR_SENSE,
	SAMPLE_ADC_EXT_1,
	SAMPLE_ADC_EXT_2,
	NUM_ADCS_TO_SAMPLE
} ADCSampleIndex_t;

// Map ADC channels to samples, must be in same order as ADCSampleIndex_t
static const ADC_CHANNEL_T channelMap[NUM_ADCS_TO_SAMPLE] = {TEMPERATURE_ADC,
															 BATT_V_ADC,
															 CURR_SENSE_ADC,
															 EXT_1_ADC,
															 EXT_2_ADC};

static volatile ADCSample_t ADCSamples[NUM_ADCS_TO_SAMPLE];

static ADC_CLOCK_SETUP_T ADCSetup;

static uint16_t getAverage(ADCSampleIndex_t index) {
	uint8_t sampleNum;
	uint32_t total = 0;

	if (ADCSamples[index].valid == true) {
		for (sampleNum = 0; sampleNum < ADC_SAMPLE_WIDTH; sampleNum++) {
			total += ADCSamples[index].sample[sampleNum];
		}

		total = total / ADC_SAMPLE_WIDTH;
	} else {
		total = 0xFFFF;  // Sample not ready
	}

	return ((uint16_t)(total & 0xFFFF));
}

void ADCInit(void) {

	memset(ADCSamples, 0, sizeof(ADCSample_t) * NUM_ADCS_TO_SAMPLE);

	Chip_ADC_Init(LPC_ADC, &ADCSetup);
	Chip_ADC_SetResolution(LPC_ADC, &ADCSetup, ADC_10BITS);
	Chip_ADC_EnableChannel(LPC_ADC, TEMPERATURE_ADC, ENABLE);
	Chip_ADC_EnableChannel(LPC_ADC, BATT_V_ADC, ENABLE);
	Chip_ADC_EnableChannel(LPC_ADC, CURR_SENSE_ADC, ENABLE);
	Chip_ADC_EnableChannel(LPC_ADC, EXT_1_ADC, ENABLE);
	Chip_ADC_EnableChannel(LPC_ADC, EXT_2_ADC, ENABLE);
	Chip_ADC_SetBurstCmd(LPC_ADC, ENABLE);

}

// Runs in interrupt time, called from SysTick_Handler
// Only sample here, take averages in main loop time
// Any race during averaging will not affect end result as atomic 16bit reads
void ADCTakeSamples(void) {

	ADCSample_t *thisSample;
	uint8_t adc;
	uint16_t result;

	for (adc = 0; adc < NUM_ADCS_TO_SAMPLE; adc++) {
		if (Chip_ADC_ReadValue(LPC_ADC, channelMap[adc], &result) == SUCCESS) {
			thisSample = &ADCSamples[adc];

			thisSample->sample[thisSample->sampleNum] = result;
			if(++thisSample->sampleNum >= ADC_SAMPLE_WIDTH) {
				thisSample->sampleNum = 0;
				thisSample->valid = true; // Valid once all samples collected
			}
		}
	}

}


uint16_t ADCGetTemperature(void) {

	return(getAverage(SAMPLE_ADC_TEMPERATURE));
}

uint16_t ADCGetBattVoltage(void) {

	return (((uint32_t)getAverage(SAMPLE_ADC_BATT_V) * 10000) / 235);
}

uint16_t ADCGetBridgeCurrent(void) {

	return (getAverage(SAMPLE_ADC_CURR_SENSE) * 20);
}

uint16_t ADCGetExt1(void) {

	return(getAverage(SAMPLE_ADC_EXT_1));
}

uint16_t ADCGetExt2(void) {

	return(getAverage(SAMPLE_ADC_EXT_2));
}

bool ADCIsReady(void) {
	return (ADCSamples[NUM_ADCS_TO_SAMPLE - 1].valid);
}
