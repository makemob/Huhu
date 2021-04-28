/*
 * hardware.c
 *
 *  Created on: 12 Jun 2016
 *      Author: Chris Mock
 *
 *	Configured for V1.0 Scarab PCB
 *	github.com/scarab
 *
 */

#include "chip.h"
#include "hardware.h"
#include "analogue.h"
#include "timer.h"

#define MICROSWITCH_DEBOUNCE_PERIOD (50)  // ms

typedef struct {
	bool lastState;
	bool debouncedState;
	TimerNum timer;
	uint8_t port;
	uint8_t pin;
	uint8_t debounceMs;
} Debouncer;

const uint8_t ScarabID[16] = "Scarab**********";

static uint16_t maxBattVoltage = 0;
static uint16_t minBattVoltage = 0;

static Debouncer inwardEndstopDebouncer;
static Debouncer outwardEndstopDebouncer;

#if (HW_TYPE == HW_TYPE_UKI)
#define ENCODER_DEBOUNCE_PERIOD (5)       // ms

#define POSITION_ENCODER_DIRECTION (1)   // 1 = counterclockwise extend, -1 = clockwise extend

#define POSITION_ENCODER_MUX_PERIOD (20)   // ms.  Long enough to let input cap charge, short enough to catch all transitions

#define DEFAULT_POSITION_ENCODER_TENTH_MM_PER_COUNT (16)  // Set to zero to disable encoder by default

static Debouncer positionEncoderADebouncer;
static Debouncer positionEncoderBDebouncer;
static bool lastPositionEncoderA = FALSE;
static bool positionEncoderMuxState;
#elif (HW_TYPE == HW_TYPE_HELIOS)
#define DEFAULT_POSITION_ENCODER_TENTH_MM_PER_COUNT (4)  // Set to zero to disable encoder by default

static int16_t positionADCOffset = 0;
#endif // HW_TYPE

static int16_t positionEncoderCounts = 0;
static uint16_t positionEncoderTenthMillimetresPerCount = DEFAULT_POSITION_ENCODER_TENTH_MM_PER_COUNT;  // Set to zero to disable encoder
static bool isPositionEncoderCalibrated = FALSE;
//static bool isPositionEncoderCalibrated = TRUE;   // DEBUG: Start in calibrated state
static bool forceCalibrateEncoder = FALSE;

static void startDebouncer(Debouncer* db, uint8_t port, uint8_t pin, TimerNum timer, uint8_t debouncePeriod) {
	db->port = port;
	db->pin = pin;
	db->timer = timer;
	db->debounceMs = debouncePeriod;
	db->lastState = Chip_GPIO_GetPinState(LPC_GPIO, db->port, db->pin);
	db->debouncedState = db->lastState;

	TimerSetDurationMs(db->timer, db->debounceMs);
}

static void checkDebouncer(Debouncer* db) {
	bool currentState = Chip_GPIO_GetPinState(LPC_GPIO, db->port, db->pin);

	if (currentState != db->lastState) {
		TimerSetDurationMs(db->timer, db->debounceMs);
	} else if (TimerCheckExpired(db->timer)) {
		db->debouncedState = currentState;
	}

	db->lastState = currentState;
}

#if (HW_TYPE == HW_TYPE_UKI)
// Process quadrature encoder state change
// Assumes detent stable on 00 or 11, A dominant
static void updateEncoderCounts(int16_t *counts, bool A, bool B, bool *lastA) {
	// Note this ignores invalid state transfers (00 -> 11 and 11 -> 00), will glitch

	// One pulse per detent
	if (*lastA != A) {
		*lastA = A;
		if (A) {
			*counts += B ? -POSITION_ENCODER_DIRECTION : POSITION_ENCODER_DIRECTION;
		} else {
			*counts += B ? POSITION_ENCODER_DIRECTION : -POSITION_ENCODER_DIRECTION;
		}
	}
/*
	// Two pulses per detent
	if (*lastA != A) {
		*lastA = A;
		if (!A) {
			*counts += B ? -1 : 1;
		}
	}
*/
}

// Use pullups/downs to multiplex position encoder with inward endstop switch
// If inverted, diodes cause endstop to be detected on C input (C is common for encoder if non-inverted)
static void setPositionEncoderMux(bool invert) {

	// Mode1 = Pulldown, Mode2 = Pullup.  Function differs by pin
	// In this order: Set relevant pin(s) to inputs, setup next output pin(s) low, enable weak pullup(s) on input(s)
	// Note output value must be set _after_ setting the direction, potentially glitching high in the process
	//  (can't avoid this, there's only one pin reg)
	if (!invert) {
		// Non-inverted, A & B weak pullup, C hard pulldown
		Chip_GPIO_SetPinDIRInput(LPC_GPIO, POSITION_ENCODER_A_PORT, POSITION_ENCODER_A_PIN);
		Chip_GPIO_SetPinDIRInput(LPC_GPIO, POSITION_ENCODER_B_PORT, POSITION_ENCODER_B_PIN);
		Chip_IOCON_PinMuxSet(LPC_IOCON, POSITION_ENCODER_C_IOCON, (IOCON_FUNC0 | IOCON_MODE_INACT | IOCON_DIGMODE_EN));
		Chip_GPIO_SetPinDIROutput(LPC_GPIO, POSITION_ENCODER_C_PORT, POSITION_ENCODER_C_PIN);
		Chip_GPIO_SetPinOutLow(LPC_GPIO, POSITION_ENCODER_C_PORT, POSITION_ENCODER_C_PIN);
		Chip_IOCON_PinMuxSet(LPC_IOCON, POSITION_ENCODER_A_IOCON, (IOCON_FUNC0 | IOCON_MODE_PULLUP | IOCON_DIGMODE_EN));
		Chip_IOCON_PinMuxSet(LPC_IOCON, POSITION_ENCODER_B_IOCON, (IOCON_FUNC1 | IOCON_MODE_PULLUP | IOCON_DIGMODE_EN));  // Note FUNC1 on this pin
	} else {
		// Inverted, A & B hard pulldown, C weak pullup
		Chip_GPIO_SetPinDIRInput(LPC_GPIO, POSITION_ENCODER_C_PORT, POSITION_ENCODER_C_PIN);
		Chip_IOCON_PinMuxSet(LPC_IOCON, POSITION_ENCODER_A_IOCON, (IOCON_FUNC0 | IOCON_MODE_INACT | IOCON_DIGMODE_EN));
		Chip_IOCON_PinMuxSet(LPC_IOCON, POSITION_ENCODER_B_IOCON, (IOCON_FUNC1 | IOCON_MODE_INACT | IOCON_DIGMODE_EN));   // Note FUNC1 on this pin
		Chip_GPIO_SetPinDIROutput(LPC_GPIO, POSITION_ENCODER_A_PORT, POSITION_ENCODER_A_PIN);
		Chip_GPIO_SetPinDIROutput(LPC_GPIO, POSITION_ENCODER_B_PORT, POSITION_ENCODER_B_PIN);
		Chip_GPIO_SetPinOutLow(LPC_GPIO, POSITION_ENCODER_A_PORT, POSITION_ENCODER_A_PIN);
		Chip_GPIO_SetPinOutLow(LPC_GPIO, POSITION_ENCODER_B_PORT, POSITION_ENCODER_B_PIN);
		Chip_IOCON_PinMuxSet(LPC_IOCON, POSITION_ENCODER_C_IOCON, (IOCON_FUNC0 | IOCON_MODE_PULLUP | IOCON_DIGMODE_EN));
	}
}
#endif // HW_TYPE_UKI


// Set up all the pins in one place so we can keep track of them
void HardwareInit(void) {

	uint16_t delay;

	// H-Bridge
	Chip_GPIO_SetPortDIR(LPC_GPIO, BRIDGE_1_PORT, (1 << BRIDGE_1_PIN), true);
	Chip_GPIO_SetPortDIR(LPC_GPIO, BRIDGE_2_PORT, (1 << BRIDGE_2_PIN), true);
	Chip_GPIO_SetPortDIR(LPC_GPIO, BRIDGE_3_PORT, (1 << BRIDGE_3_PIN), true);
	Chip_GPIO_SetPortDIR(LPC_GPIO, BRIDGE_4_PORT, (1 << BRIDGE_4_PIN), true);

	Chip_GPIO_SetPortDIR(LPC_GPIO, BLUE_LED_PORT, (1 << BLUE_LED_PIN), true);    // Blue LED / EXT_6
	Chip_GPIO_SetPortDIR(LPC_GPIO, GREEN_LED_PORT, (1 << GREEN_LED_PIN), true);  // Green LED / EXT_5

	Chip_GPIO_SetPortDIR(LPC_GPIO, RS485_TX_EN_PORT, (1 << RS485_TX_EN_PIN), true);  // 485 TX EN

	Chip_IOCON_PinMuxSet(LPC_IOCON, UART_RX_PIN, (IOCON_FUNC1 | IOCON_MODE_INACT | IOCON_MODE_PULLDOWN)); // UART RXD (without pulldown, MAX3485 can latch up)
	Chip_IOCON_PinMuxSet(LPC_IOCON, UART_TX_PIN, (IOCON_FUNC1 | IOCON_MODE_INACT)); // UART TXD

	// ADC (Caution: mode number differs pin to pin!)
	Chip_IOCON_PinMuxSet(LPC_IOCON, TEMPERATURE_ADC_IOCON, (IOCON_FUNC2 | IOCON_MODE_INACT | IOCON_ADMODE_EN));
	Chip_IOCON_PinMuxSet(LPC_IOCON, BATT_V_ADC_IOCON, (IOCON_FUNC2 | IOCON_MODE_INACT | IOCON_ADMODE_EN));
	Chip_IOCON_PinMuxSet(LPC_IOCON, CURR_SENSE_ADC_IOCON, (IOCON_FUNC1 | IOCON_MODE_INACT | IOCON_ADMODE_EN));
#ifdef ENABLE_EXT_1_ADC
	Chip_IOCON_PinMuxSet(LPC_IOCON, EXT_1_IOCON, (IOCON_FUNC2 | IOCON_MODE_INACT | IOCON_ADMODE_EN));
#endif
#ifdef ENABLE_EXT_2_ADC
	Chip_IOCON_PinMuxSet(LPC_IOCON, EXT_2_IOCON, (IOCON_FUNC2 | IOCON_MODE_INACT | IOCON_ADMODE_EN));
#endif

#if (HW_TYPE == HW_TYPE_UKI)
	// Set up position encoder multiplexing:
	setPositionEncoderMux(FALSE);  // Need to set mux state temporarily to ensure debouncers are correctly initialised
	startDebouncer(&positionEncoderADebouncer, POSITION_ENCODER_A_PORT, POSITION_ENCODER_A_PIN, TIMER_POSITION_ENCODER_A_DEBOUNCE, ENCODER_DEBOUNCE_PERIOD);
	startDebouncer(&positionEncoderBDebouncer, POSITION_ENCODER_B_PORT, POSITION_ENCODER_B_PIN, TIMER_POSITION_ENCODER_B_DEBOUNCE, ENCODER_DEBOUNCE_PERIOD);

	positionEncoderMuxState = TRUE;                 // Start with mux set to limit switches
	setPositionEncoderMux(positionEncoderMuxState);
	for (delay = 0; delay < 60000; delay++)         // Spin to allow pin voltage to stabilise
		;
	TimerSetDurationMs(TIMER_POSITION_ENCODER_MUX, POSITION_ENCODER_MUX_PERIOD);
#elif (HW_TYPE == HW_TYPE_HELIOS)
	Chip_GPIO_SetPinDIRInput(LPC_GPIO, INWARD_ENDSTOP_PORT, INWARD_ENDSTOP_PIN);
	Chip_GPIO_SetPinDIRInput(LPC_GPIO, OUTWARD_ENDSTOP_PORT, OUTWARD_ENDSTOP_PIN);

	Chip_IOCON_PinMuxSet(LPC_IOCON, INWARD_ENDSTOP_IOCON, (IOCON_FUNC0 | IOCON_MODE_INACT));  // disable pullup
	Chip_IOCON_PinMuxSet(LPC_IOCON, OUTWARD_ENDSTOP_IOCON, (IOCON_FUNC1 | IOCON_MODE_INACT | IOCON_DIGMODE_EN)); // disable pullup
#endif // HW_TYPE

	startDebouncer(&inwardEndstopDebouncer, INWARD_ENDSTOP_PORT, INWARD_ENDSTOP_PIN, TIMER_INWARD_ENDSTOP_DEBOUNCE, MICROSWITCH_DEBOUNCE_PERIOD);
	startDebouncer(&outwardEndstopDebouncer, OUTWARD_ENDSTOP_PORT, OUTWARD_ENDSTOP_PIN, TIMER_OUTWARD_ENDSTOP_DEBOUNCE, MICROSWITCH_DEBOUNCE_PERIOD);

	// Flash blue LED as powerup/reboot indicator
	Chip_GPIO_SetPinOutHigh(LPC_GPIO, BLUE_LED_PORT, BLUE_LED_PIN);
	for (delay = 0; delay < 10000; delay++)
		;
	Chip_GPIO_SetPinOutLow(LPC_GPIO, BLUE_LED_PORT, BLUE_LED_PIN);

}

void HardwarePoll(void) {
	// Check battery voltage
	uint16_t volts = ADCGetBattVoltage();

	if (volts > maxBattVoltage) {
		maxBattVoltage = volts;
	} else if (volts < minBattVoltage) {
		minBattVoltage = volts;  // This might need a holdoff, ends up being zero
	}

#if (HW_TYPE == HW_TYPE_UKI)
	// Check debounced inputs
	checkDebouncer(&outwardEndstopDebouncer);

	// Position encoder and inward endstop microswitch are multiplexed

	if (TimerCheckExpired(TIMER_POSITION_ENCODER_MUX)) {
		TimerSetDurationMs(TIMER_POSITION_ENCODER_MUX, POSITION_ENCODER_MUX_PERIOD);

		// Read input state at END of mux period to allow pullup voltage to stabilise across cap
		if (positionEncoderMuxState) {
			checkDebouncer(&inwardEndstopDebouncer);
		} else {
			checkDebouncer(&positionEncoderADebouncer);
			checkDebouncer(&positionEncoderBDebouncer);

			updateEncoderCounts(&positionEncoderCounts, positionEncoderADebouncer.debouncedState, positionEncoderBDebouncer.debouncedState, &lastPositionEncoderA);
		}

		if (positionEncoderTenthMillimetresPerCount > 0) {
			// Toggle pullup/downs to mux encoder & inward endstop switch
			positionEncoderMuxState = !positionEncoderMuxState;
		} else {
			// Set positionEncoderTenthMillimetresPerCount to zero to disable encoder (and multiplexing)
			positionEncoderMuxState = TRUE;
		}
		setPositionEncoderMux(positionEncoderMuxState);
	}

	// Reset position to zero when inward endstop switch detected, or when forced
	if ((inwardEndstopDebouncer.debouncedState ^ INWARD_ENDSTOP_SENSE) || forceCalibrateEncoder) {
		positionEncoderCounts = 0;
		isPositionEncoderCalibrated = TRUE;
		forceCalibrateEncoder = FALSE;
	}

#elif (HW_TYPE == HW_TYPE_HELIOS)
	// Check debounced inputs
	checkDebouncer(&inwardEndstopDebouncer);
	checkDebouncer(&outwardEndstopDebouncer);

	// Take analog position reading (ADC counts)
	uint16_t reading;
	if (POSITION_ADC == EXT_1_ADC) {
		reading = (int16_t)ADCGetExt1();
	} else if (POSITION_ADC == EXT_2_ADC) {
		reading = (int16_t)ADCGetExt2();
	}

	// Reset position to zero when inward endstop switch detected, or when forced
	if ((inwardEndstopDebouncer.debouncedState ^ INWARD_ENDSTOP_SENSE) || forceCalibrateEncoder) {
		positionADCOffset = reading;
		isPositionEncoderCalibrated = TRUE;
		forceCalibrateEncoder = FALSE;
	}

	positionEncoderCounts = reading - positionADCOffset;
#endif

}

uint16_t HardwareGetMaxBattVoltage(void) {
	return(maxBattVoltage);
}

uint16_t HardwareGetMinBattVoltage(void) {
	return(minBattVoltage);
}

bool HardwareGetInwardEndstop(void) {
	return(inwardEndstopDebouncer.debouncedState ^ INWARD_ENDSTOP_SENSE);
}

bool HardwareGetOutwardEndstop(void) {
	return(outwardEndstopDebouncer.debouncedState ^ OUTWARD_ENDSTOP_SENSE);
}

int16_t HardwareGetPositionEncoderCounts(void) {
	return(positionEncoderCounts);
}

// Get position encoder distance in tenths of a mm
int16_t HardwareGetPositionEncoderDistance(void) {
	if (!isPositionEncoderCalibrated || (positionEncoderTenthMillimetresPerCount == 0)) {
			return(POSITION_ENCODER_UNCALIBRATED);
	} else {
		return(positionEncoderCounts * positionEncoderTenthMillimetresPerCount);
	}
}

void HardwareZeroPositionEncoder(void) {
	positionEncoderCounts = 0;
}

uint16_t HardwareGetPositionEncoderScaling(void) {
	return(positionEncoderTenthMillimetresPerCount);
}

void HardwareSetPositionEncoderScaling(uint16_t tenthMillimetrePerCount) {
	// Set to zero to disable encoder
	positionEncoderTenthMillimetresPerCount = tenthMillimetrePerCount;
	isPositionEncoderCalibrated = FALSE;
}

bool HardwareIsPositionEncoderEnabled(void) {
	return (positionEncoderTenthMillimetresPerCount > 0);
}

void HardwareForceCalibrateEncoder(void) {
	// Force the encoder to calibrate to zero in the current position
	// Used to calibrate actuators that do not have endstop microswitches
	forceCalibrateEncoder = TRUE;
}
