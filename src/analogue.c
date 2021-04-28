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
#ifdef ENABLE_EXT_1_ADC
	SAMPLE_ADC_EXT_1,
#endif
#ifdef ENABLE_EXT_2_ADC
	SAMPLE_ADC_EXT_2,
#endif
	NUM_ADCS_TO_SAMPLE
} ADCSampleIndex_t;

// Map ADC channels to samples, must be in same order as ADCSampleIndex_t

static const ADC_CHANNEL_T channelMap[NUM_ADCS_TO_SAMPLE] = {TEMPERATURE_ADC,
															 BATT_V_ADC,
															 CURR_SENSE_ADC
#ifdef ENABLE_EXT_1_ADC
															 , EXT_1_ADC
#endif
#ifdef ENABLE_EXT_2_ADC
															 , EXT_2_ADC
#endif
};

/*
# else
static const ADC_CHANNEL_T channelMap[NUM_ADCS_TO_SAMPLE] = {TEMPERATURE_ADC,
															 BATT_V_ADC,
															 CURR_SENSE_ADC};
#endif
*/


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
#ifdef ENABLE_EXT_1_ADC
	Chip_ADC_EnableChannel(LPC_ADC, EXT_1_ADC, ENABLE);
#endif
#ifdef ENABLE_EXT_2_ADC
	Chip_ADC_EnableChannel(LPC_ADC, EXT_2_ADC, ENABLE);
#endif
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
	/* Using R:

	library(dplyr)
	library(ggplot2)

	r_bias <- 10000

	# Lookup table from Vishay NTCS0805E3103JMT datasheet
	ntc <- data.frame(temp = c(0, 10, 20, 30, 40, 50, 60, 70, 80),
					  resistance = c(28829, 18515, 12205, 8240.3, 5686.6, 4004.2, 2872.3, 2095.9, 1553.8)) %>%
	  mutate(ratio = resistance / (resistance + r_bias),
			 adc_counts = 1024 * ratio,
			 adc_counts2 = adc_counts ^ 2)

	mod <- lm(formula = temp ~ adc_counts + adc_counts2, data = ntc)
	summary(mod)

	ntc$prediction <- predict(mod, newdata = ntc)

	ggplot(ntc) + geom_line(aes(x = temp, y = adc_counts), color = 'blue') +
	  geom_line(aes(x = prediction, y = adc_counts), color = 'red')


	Call:
	lm(formula = temp ~ adc_counts + adc_counts2, data = ntc)

	Residuals:
		Min      1Q  Median      3Q     Max
	-1.9171 -1.7472 -0.3925  1.2336  2.6875

	Coefficients:
				  Estimate Std. Error t value Pr(>|t|)
	(Intercept)  1.035e+02  3.216e+00  32.193 5.97e-08 ***
	adc_counts  -2.028e-01  1.713e-02 -11.840 2.19e-05 ***
	adc_counts2  9.066e-05  1.907e-05   4.754  0.00315 **
	---
	Signif. codes:  0 ‘***’ 0.001 ‘**’ 0.01 ‘*’ 0.05 ‘.’ 0.1 ‘ ’ 1

	Residual standard error: 2.01 on 6 degrees of freedom
	Multiple R-squared:  0.996,	Adjusted R-squared:  0.9946
	F-statistic: 739.7 on 2 and 6 DF,  p-value: 6.59e-08
 	*/

	uint32_t adc = getAverage(SAMPLE_ADC_TEMPERATURE);
	return((uint16_t)((89923 - 123 * adc) / 1000));
}

uint16_t ADCGetBattVoltage(void) {

	/* Using R:

		> library(ggplot2)
		>
		> volts <- data.frame(mV = c(15000, 17000, 19000, 21000, 23000, 25000, 27000, 30000),
		+                     ADC = c(349, 388, 423, 452, 477, 498, 516, 539))
		>
		> volts$ADC_2 <- volts$ADC ^ 2
		>
		> mod <- lm(formula = mV ~ ADC_2 + ADC, data = volts)
		> summary(mod)

		Call:
		lm(formula = mV ~ ADC_2 + ADC, data = volts)

		Residuals:
			  1       2       3       4       5       6       7       8
		-169.89  213.50  155.38   14.89 -146.92 -189.35 -104.61  227.00

		Coefficients:
					  Estimate Std. Error t value Pr(>|t|)
		(Intercept)  3.246e+04  4.303e+03   7.543 0.000649 ***
		ADC_2        2.345e-01  2.201e-02  10.651 0.000126 ***
		ADC         -1.314e+02  1.961e+01  -6.698 0.001122 **
		---
		Signif. codes:  0 ‘***’ 0.001 ‘**’ 0.01 ‘*’ 0.05 ‘.’ 0.1 ‘ ’ 1

		Residual standard error: 209.2 on 5 degrees of freedom
		Multiple R-squared:  0.9988,	Adjusted R-squared:  0.9983
		F-statistic:  2088 on 2 and 5 DF,  p-value: 4.947e-08

		>
		> volts$prediction <- predict(mod, newdata = volts)
		>
		> ggplot(volts) + geom_line(aes(x = mV, y = ADC), color = 'blue') +
		+   geom_line(aes(x = prediction, y = ADC), color = 'red')

	*/

	uint32_t adc = getAverage(SAMPLE_ADC_BATT_V);
	return((uint16_t)(((2345 * adc * adc) / 10000) - 131 * adc + 32460));

}

uint16_t ADCGetBridgeCurrent(void) {

	return (getAverage(SAMPLE_ADC_CURR_SENSE) * 20);
}

uint16_t ADCGetExt1(void) {
#ifdef ENABLE_EXT_1_ADC
	return(getAverage(SAMPLE_ADC_EXT_1));
#else
	return(0xFFFF);
#endif
}

uint16_t ADCGetExt2(void) {
#ifdef ENABLE_EXT_2_ADC
	return(getAverage(SAMPLE_ADC_EXT_2));
#else
	return(0xFFFF);
#endif
}

bool ADCIsReady(void) {
	return (ADCSamples[NUM_ADCS_TO_SAMPLE - 1].valid);
}
