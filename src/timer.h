/*
 * timer.h
 *
 *  Created on: 12 Jun 2016
 *      Author: chrismock
 */

#ifndef TIMER_H_
#define TIMER_H_

typedef enum {
	TIMER_MOTOR,
	TIMER_TX_ENABLE_DELAY,
	TIMER_MODBUS_INTERCHAR,
	TIMER_MOTOR_LED,
	TIMER_INWARD_ENDSTOP_DEBOUNCE,
	TIMER_OUTWARD_ENDSTOP_DEBOUNCE,
	TIMER_POSITION_ENCODER_A_DEBOUNCE,
	TIMER_POSITION_ENCODER_B_DEBOUNCE,
	TIMER_POSITION_ENCODER_MUX,
	TIMER_POSITION_ENCODER_FAIL,
	TIMER_HEARTBEAT,
	TIMER_TEST,
	MAX_TIMERS,
} TimerNum;

uint32_t TimerGetTicker(void);

void TimerInit(void);

bool TimerCheckExpired(TimerNum t);

void TimerForceExpiry(TimerNum t);

void TimerSetDurationMs(TimerNum t, uint32_t duration);

uint32_t TimerGetUptimeSeconds(void);

#endif /* TIMER_H_ */
