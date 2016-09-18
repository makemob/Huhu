/*
 * timer.c
 *
 *  Created on: 12 Jun 2016
 *      Author: Chris Mock
 *
 *
 */

#include "chip.h"
#include "hardware.h"
#include "timer.h"
#include "analogue.h"

#define TICKRATE_HZ (1000)


typedef struct {
	uint32_t start;
	uint32_t duration;
	bool isExpired;
} Timer;

static volatile uint32_t ticker = 0;

static uint32_t uptimeSeconds = 0;

static Timer timer[MAX_TIMERS];


void SysTick_Handler(void)
{
	ticker--;

	if ((ticker % TICKRATE_HZ) == 0) {
		uptimeSeconds++;
	}

    ADCTakeSamples();
}




/**
 * @brief	Handle interrupt from 32-bit timer
 * @return	Nothing

void TIMER32_0_IRQHandler(void)
{
	if (Chip_TIMER_MatchPending(LPC_TIMER32_0, 1)) {
		Chip_TIMER_ClearMatch(LPC_TIMER32_0, 1);

		ticker--;
	}
}
*/

uint32_t TimerGetTicker(void) {
	// Assume atomic
	return ticker;
}


void TimerInit(void) {
	// uint32_t timerFreq;

	uint8_t i;

	for (i = 0; i < MAX_TIMERS; i++) {
		timer[i].isExpired = TRUE;
	}

	SysTick_Config(SystemCoreClock / TICKRATE_HZ);

	/*** Timer 1 Init

	Chip_TIMER_Init(LPC_TIMER32_0);

	timerFreq = Chip_Clock_GetSystemClockRate();

	Chip_TIMER_Reset(LPC_TIMER32_0);
	Chip_TIMER_MatchEnableInt(LPC_TIMER32_0, 1);
	Chip_TIMER_SetMatch(LPC_TIMER32_0, 1, (timerFreq / TICKRATE_HZ));
	Chip_TIMER_ResetOnMatchEnable(LPC_TIMER32_0, 1);
	Chip_TIMER_Enable(LPC_TIMER32_0);

	NVIC_ClearPendingIRQ(TIMER_32_0_IRQn);
	NVIC_EnableIRQ(TIMER_32_0_IRQn);

	***/
}

bool TimerCheckExpired(TimerNum t) {

	if (!timer[t].isExpired && ((timer[t].start - ticker) > timer[t].duration)) {
		timer[t].isExpired = TRUE;
		return TRUE;
	} else {
		return FALSE;
	}
}

void TimerForceExpiry(TimerNum t) {
	timer[t].isExpired = TRUE;
}

void TimerSetDurationMs(TimerNum t, uint32_t duration) {
	timer[t].duration = duration;
	timer[t].start = ticker;
	timer[t].isExpired = FALSE;
}

uint32_t TimerGetUptimeSeconds(void) {
	return uptimeSeconds;
}
