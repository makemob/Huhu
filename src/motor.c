/*
 * motor.c
 *
 *  Created on: 12 Jun 2016
 *      Author: Chris Mock
 */

#include <stdlib.h>
#include "chip.h"
#include "hardware.h"
#include "timer.h"
#include "analogue.h"

#define PWM_HZ (20000)

#define DEFAULT_CURRENT_LIMIT (1500)  // mA
#define MAX_CURRENT_LIMIT (8000)      // mA

#define MAX_BATT_VOLTAGE (40000)     // mV

#define DEFAULT_HEARTBEAT_TIMEOUT (5)  // seconds between heartbeats before motor stops

#define MIN_SPEED (10)    // %, set to ensure interrupt can trigger fast enough to maintain duty cycle
#define MAX_SPEED (90)   // %, set to ensure HSD bootstrap capacitor remains charged

// Acceleration timer set in ms per % speed change
#define MIN_ACCEL (500)         // Minimum (sane) acceleration
#define E_STOP_ACCEL (10)       // Max safe rate to bring motor to a stop
#define ACCEL_PERIOD(percent) ((((100 - (percent)) * (MIN_ACCEL - E_STOP_ACCEL)) / 100) + E_STOP_ACCEL)

// User facing acceleration set in % [0, 100]
#define DEFAULT_ACCEL (100)   // % accel set on powerup
static uint8_t accelPercent = DEFAULT_ACCEL;
static uint16_t accelPeriod = ACCEL_PERIOD(DEFAULT_ACCEL);

static uint32_t PWMPeriod;
static uint32_t PWMDuty;

static int8_t speedSetpoint = 0;  // % [-MAX_SPEED, MAX_SPEED]
static int8_t speedActual = 0;    // % [-MAX_SPEED, MAX_SPEED]

static bool isStopping = false;  // Motor is stopping for important reason (eg. microswitch)
static bool isEStopping = false; // Motor is stopping for critical reason (eg. current limit), will need manual reset
static bool resetEStop = false;  // Set when manual e-stop reset triggered

static uint16_t currentLimitInward = DEFAULT_CURRENT_LIMIT;
static uint16_t currentLimitOutward = DEFAULT_CURRENT_LIMIT;
static uint16_t currentTripsInward = 0;
static uint16_t currentTripsOutward = 0;

static uint16_t battVoltageTrips = 0;

static uint16_t inwardEndstops = 0;
static uint16_t outwardEndstops = 0;

static uint16_t heartbeatExpiries = 0;
static uint16_t heartbeatTimeout = DEFAULT_HEARTBEAT_TIMEOUT;

// Store the high & low side ports for the PWM FETs
static uint8_t bridgeUpperPort;
static uint8_t bridgeUpperPin;
static uint8_t bridgeLowerPort;
static uint8_t bridgeLowerPin;

/**
 * @brief	Handle interrupt from 32-bit timer
 * @return	Nothing
 */
void TIMER32_1_IRQHandler(void)
{
	// Drive motor PWM (MIC4605 deals with deadtime/shoot-thru protection)

	// Sign magnitude drive

	if (Chip_TIMER_MatchPending(LPC_TIMER32_1, 1)) {
		Chip_TIMER_ClearMatch(LPC_TIMER32_1, 1);

		Chip_GPIO_SetPinState(LPC_GPIO, bridgeUpperPort, bridgeUpperPin, FALSE);
		Chip_GPIO_SetPinState(LPC_GPIO, bridgeLowerPort, bridgeLowerPin, TRUE);

	}

	if (Chip_TIMER_MatchPending(LPC_TIMER32_1, 2)) {
		Chip_TIMER_ClearMatch(LPC_TIMER32_1, 2);

		Chip_TIMER_SetMatch(LPC_TIMER32_1, 1, PWMDuty);

		Chip_GPIO_SetPinState(LPC_GPIO, bridgeLowerPort, bridgeLowerPin, FALSE);
		Chip_GPIO_SetPinState(LPC_GPIO, bridgeUpperPort, bridgeUpperPin, TRUE);

	}

	/*
	// Lock anti-phase drive - difficult to get transitions synchronised due to pinout on different ports

	if (Chip_TIMER_MatchPending(LPC_TIMER32_1, 1)) {
		Chip_TIMER_ClearMatch(LPC_TIMER32_1, 1);

		if (speedActual != 0) {
			// Off before on to prevent shoot through

			Chip_GPIO_SetPinState(LPC_GPIO, BRIDGE_3_PORT, BRIDGE_3_PIN, FALSE);
			Chip_GPIO_SetPinState(LPC_GPIO, BRIDGE_2_PORT, BRIDGE_2_PIN, FALSE);


			Chip_GPIO_SetPinState(LPC_GPIO, BRIDGE_4_PORT, BRIDGE_4_PIN, TRUE);
			Chip_GPIO_SetPinState(LPC_GPIO, BRIDGE_1_PORT, BRIDGE_1_PIN, TRUE);
		}

	}

	if (Chip_TIMER_MatchPending(LPC_TIMER32_1, 2)) {
		Chip_TIMER_ClearMatch(LPC_TIMER32_1, 2);

		Chip_TIMER_SetMatch(LPC_TIMER32_1, 1, PWMDuty);

		if (speedActual != 0) {
			// Off before on to prevent shoot through
			Chip_GPIO_SetPinState(LPC_GPIO, BRIDGE_1_PORT, BRIDGE_1_PIN, FALSE);
			Chip_GPIO_SetPinState(LPC_GPIO, BRIDGE_4_PORT, BRIDGE_4_PIN, FALSE);
			Chip_GPIO_SetPinState(LPC_GPIO, BRIDGE_2_PORT, BRIDGE_2_PIN, TRUE);
			Chip_GPIO_SetPinState(LPC_GPIO, BRIDGE_3_PORT, BRIDGE_3_PIN, TRUE);
		} else {
			Chip_GPIO_SetPinState(LPC_GPIO, BRIDGE_1_PORT, BRIDGE_1_PIN, FALSE);
			Chip_GPIO_SetPinState(LPC_GPIO, BRIDGE_2_PORT, BRIDGE_2_PIN, FALSE);
			Chip_GPIO_SetPinState(LPC_GPIO, BRIDGE_3_PORT, BRIDGE_3_PIN, FALSE);
			Chip_GPIO_SetPinState(LPC_GPIO, BRIDGE_4_PORT, BRIDGE_4_PIN, FALSE);
		}

	}
	*/
}

void MotorInit(void) {

	PWMPeriod = Chip_Clock_GetSystemClockRate() / PWM_HZ;
	PWMDuty = 0;   // sign magnitude drive 'off'
	// PWMDuty = PWMPeriod / 2;  // lock antiphase drive midpoint

	Chip_GPIO_SetPinState(LPC_GPIO, BRIDGE_1_PORT, BRIDGE_1_PIN, FALSE);
	Chip_GPIO_SetPinState(LPC_GPIO, BRIDGE_3_PORT, BRIDGE_3_PIN, FALSE);
	Chip_GPIO_SetPinState(LPC_GPIO, BRIDGE_2_PORT, BRIDGE_2_PIN, FALSE);
	Chip_GPIO_SetPinState(LPC_GPIO, BRIDGE_4_PORT, BRIDGE_4_PIN, FALSE);

	// Timer 32_1 Init:
	// Match 1 sets duty cycle
	// Match 2 sets PWM rate

	Chip_TIMER_Init(LPC_TIMER32_1);
	Chip_TIMER_Reset(LPC_TIMER32_1);
	Chip_TIMER_MatchEnableInt(LPC_TIMER32_1, 1);
	Chip_TIMER_MatchEnableInt(LPC_TIMER32_1, 2);
	Chip_TIMER_SetMatch(LPC_TIMER32_1, 1, PWMDuty);
	Chip_TIMER_SetMatch(LPC_TIMER32_1, 2, PWMPeriod);
	Chip_TIMER_ResetOnMatchEnable(LPC_TIMER32_1, 2);
	Chip_TIMER_Enable(LPC_TIMER32_1);

	// Enable timer interrupt
//	NVIC_ClearPendingIRQ(TIMER_32_1_IRQn);
//	NVIC_EnableIRQ(TIMER_32_1_IRQn);

	TimerSetDurationMs(TIMER_MOTOR, accelPeriod);

	TimerSetDurationMs(TIMER_MOTOR_LED, 10 * (MAX_SPEED - speedSetpoint + 1));
}

void MotorStop(void) {
	speedSetpoint = 0;
	accelPeriod = E_STOP_ACCEL;
	TimerSetDurationMs(TIMER_MOTOR, accelPeriod);
	isStopping = true;
}

void MotorEStop(void) {
	MotorStop();
	isEStopping = true;
	resetEStop = false;
}

void MotorPoll(void) {
	int8_t lastSpeedActual;

	if (isEStopping) {
		if (resetEStop && (speedActual == 0)) {
			isEStopping = false;
			resetEStop = false;
			accelPeriod = ACCEL_PERIOD(DEFAULT_ACCEL);
		}
	} else if (isStopping) {
		// Stopping is distinct from e-stopping.  In an e-stop recovery this will be called on the second poll
		if (speedActual == 0) {
			isStopping = false;
			accelPeriod = ACCEL_PERIOD(DEFAULT_ACCEL);
		}
	} else {
		// Check for reasons to stop the motor:

		if (speedActual < 0) {
			if (ADCGetBridgeCurrent() > currentLimitInward) {
				MotorEStop();
				currentTripsInward++;
			}
		} else if (ADCGetBridgeCurrent() > currentLimitOutward) {
			MotorEStop();
			currentTripsOutward++;
		}

		if (ADCGetBattVoltage() > MAX_BATT_VOLTAGE) {
			MotorEStop();
			battVoltageTrips++;
		}

		// Actuator endstop microswitches stop movement
		if ((speedActual < 0) && HardwareGetInwardEndstop()) {
			MotorStop();
			inwardEndstops++;
		}

		if ((speedActual > 0) && HardwareGetOutwardEndstop()) {
			MotorStop();
			outwardEndstops++;
		}

		if(heartbeatTimeout && speedActual && TimerCheckExpired(TIMER_HEARTBEAT)) {
			MotorEStop();
			heartbeatExpiries++;
		}
	}


	if (TimerCheckExpired(TIMER_MOTOR)) {
		TimerSetDurationMs(TIMER_MOTOR, accelPeriod);

		lastSpeedActual = speedActual;

		// speedActual must transition through zero to trigger direction change
		if (speedSetpoint > speedActual) {
			speedActual++;

			// Rapid direction change
			//if (speedActual == -MIN_SPEED) {
			//	speedActual = -1;
			//}
		} else if (speedSetpoint < speedActual) {
			speedActual--;

			// Rapid direction change
			//if (speedActual == MIN_SPEED) {
			//	speedActual = 1;
			//}
		}

		// For lock antiphase drive
		//PWMDuty = ((uint32_t)(speedActual + 101) * PWMPeriod) / 202;  // avoid 0 when speed = -100

		// For sign magnitude drive
		PWMDuty = ((uint32_t)(abs(speedActual)) * PWMPeriod) / 100;

		if ((speedActual == 0) && (speedSetpoint == 0) && (lastSpeedActual == 0)) {
			// Stopped

			NVIC_DisableIRQ(TIMER_32_1_IRQn);
			NVIC_ClearPendingIRQ(TIMER_32_1_IRQn);

			Chip_GPIO_SetPinState(LPC_GPIO, BRIDGE_1_PORT, BRIDGE_1_PIN, FALSE);
			Chip_GPIO_SetPinState(LPC_GPIO, BRIDGE_3_PORT, BRIDGE_3_PIN, FALSE);
			Chip_GPIO_SetPinState(LPC_GPIO, BRIDGE_2_PORT, BRIDGE_2_PIN, FALSE);
			Chip_GPIO_SetPinState(LPC_GPIO, BRIDGE_4_PORT, BRIDGE_4_PIN, FALSE);

		} else if (abs(speedActual) < MIN_SPEED) {
			// Min speed cutout since interrupt timing constrains min pulse width
			// Hold lower FETs on, upper off

			NVIC_DisableIRQ(TIMER_32_1_IRQn);
			NVIC_ClearPendingIRQ(TIMER_32_1_IRQn);

			Chip_GPIO_SetPinState(LPC_GPIO, BRIDGE_1_PORT, BRIDGE_1_PIN, FALSE);
			Chip_GPIO_SetPinState(LPC_GPIO, BRIDGE_3_PORT, BRIDGE_3_PIN, FALSE);
			Chip_GPIO_SetPinState(LPC_GPIO, BRIDGE_2_PORT, BRIDGE_2_PIN, TRUE);
			Chip_GPIO_SetPinState(LPC_GPIO, BRIDGE_4_PORT, BRIDGE_4_PIN, TRUE);

		} else if ((lastSpeedActual == MIN_SPEED) && (speedActual > 0)) {
			// Transition to forward
			// Ensure FETs are set up correctly, then enable interrupt to control PWM FETs

			NVIC_DisableIRQ(TIMER_32_1_IRQn);
			NVIC_ClearPendingIRQ(TIMER_32_1_IRQn);

			// Find pins to toggle up front so transition time is deterministic
			bridgeUpperPort = BRIDGE_1_PORT;
			bridgeUpperPin = BRIDGE_1_PIN;
			bridgeLowerPort = BRIDGE_2_PORT;
			bridgeLowerPin = BRIDGE_2_PIN;

			Chip_GPIO_SetPinState(LPC_GPIO, BRIDGE_1_PORT, BRIDGE_1_PIN, FALSE);
			Chip_GPIO_SetPinState(LPC_GPIO, BRIDGE_3_PORT, BRIDGE_3_PIN, FALSE);
			Chip_GPIO_SetPinState(LPC_GPIO, BRIDGE_2_PORT, BRIDGE_2_PIN, FALSE);
			Chip_GPIO_SetPinState(LPC_GPIO, BRIDGE_4_PORT, BRIDGE_4_PIN, TRUE);

			NVIC_EnableIRQ(TIMER_32_1_IRQn);

		} else if ((lastSpeedActual == -MIN_SPEED) && (speedActual < 0)) {
			// Transition to reverse
			// Ensure FETs are set up correctly, then enable interrupt to control PWM FETs

			NVIC_DisableIRQ(TIMER_32_1_IRQn);
			NVIC_ClearPendingIRQ(TIMER_32_1_IRQn);

			bridgeUpperPort = BRIDGE_3_PORT;
			bridgeUpperPin = BRIDGE_3_PIN;
			bridgeLowerPort = BRIDGE_4_PORT;
			bridgeLowerPin = BRIDGE_4_PIN;

			Chip_GPIO_SetPinState(LPC_GPIO, BRIDGE_1_PORT, BRIDGE_1_PIN, FALSE);
			Chip_GPIO_SetPinState(LPC_GPIO, BRIDGE_3_PORT, BRIDGE_3_PIN, FALSE);
			Chip_GPIO_SetPinState(LPC_GPIO, BRIDGE_4_PORT, BRIDGE_4_PIN, FALSE);
			Chip_GPIO_SetPinState(LPC_GPIO, BRIDGE_2_PORT, BRIDGE_2_PIN, TRUE);

			NVIC_EnableIRQ(TIMER_32_1_IRQn);

		}

	}

	// Flash LED at a rate proportional to speed
	if (TimerCheckExpired(TIMER_MOTOR_LED)) {
		TimerSetDurationMs(TIMER_MOTOR_LED, 10 * (MAX_SPEED - abs(speedSetpoint) + 1));
		if (speedSetpoint) {
			Chip_GPIO_SetPinToggle(LPC_GPIO, BLUE_LED_PORT, BLUE_LED_PIN);
		} else {
			Chip_GPIO_SetPinState(LPC_GPIO, BLUE_LED_PORT, BLUE_LED_PIN, FALSE);
		}
	}

}

void MotorSetSpeed(int8_t percent) {
	if (!isStopping && (percent >= -MAX_SPEED) && (percent <= MAX_SPEED)) {
		speedSetpoint = percent;
	}
}

int8_t MotorGetSetpoint(void) {
	return (speedSetpoint);
}

int8_t MotorGetSpeed(void) {
	return (speedActual);
}

void MotorSetAccel(uint8_t percent) {
	if (!isStopping && (percent >= 0) && (percent <= 100)) {
		accelPercent = percent;
		accelPeriod = ACCEL_PERIOD(percent);
	}
}

uint8_t MotorGetAccel(void) {
	return (accelPercent);
}

void MotorSetCurrentLimitInward(uint16_t limit) {
	if (limit <= MAX_CURRENT_LIMIT) {
		currentLimitInward = limit;
	}
}

void MotorSetCurrentLimitOutward(uint16_t limit) {
	if (limit <= MAX_CURRENT_LIMIT) {
		currentLimitOutward = limit;
	}
}

uint16_t MotorGetCurrentLimitInward(void) {
	return (currentLimitInward);
}

uint16_t MotorGetCurrentLimitOutward(void) {
	return (currentLimitOutward);
}

uint16_t MotorGetCurrentTripsInward(void) {
	return (currentTripsInward);
}

uint16_t MotorGetCurrentTripsOutward(void) {
	return (currentTripsOutward);
}

uint16_t MotorGetVoltageTrips(void) {
	return (battVoltageTrips);
}

uint16_t MotorGetInwardEndstops(void) {
	return (inwardEndstops);
}

uint16_t MotorGetOutwardEndstops(void) {
	return (outwardEndstops);
}

void MotorNotifyHeartbeat(void) {
	if (heartbeatTimeout) {
		TimerSetDurationMs(TIMER_HEARTBEAT, heartbeatTimeout * 1000);
	}
}

uint16_t MotorGetHeartbeatExpiries(void) {
	return (heartbeatExpiries);
}

void MotorSetHeartbeatTimeout(uint16_t seconds) {
	heartbeatTimeout = seconds;
}

uint16_t MotorGetHeartbeatTimeout(void) {
	return (heartbeatTimeout);
}

uint32_t MotorGetPWMPeriod(void) {
	return (PWMPeriod);
}

uint32_t MotorGetPWMDuty(void) {
	return (PWMDuty);
}

void MotorResetEStop(void) {
	resetEStop = true;
}

bool MotorGetEStopState(void) {
	return (isEStopping);
}
