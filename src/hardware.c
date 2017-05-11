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

#define DEBOUNCE_PERIOD (50)  // ms

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
	Chip_IOCON_PinMuxSet(LPC_IOCON, TEMPERATURE_ADC_PIN, (IOCON_FUNC2 | IOCON_MODE_INACT | IOCON_ADMODE_EN));
	Chip_IOCON_PinMuxSet(LPC_IOCON, BATT_V_ADC_PIN, (IOCON_FUNC2 | IOCON_MODE_INACT | IOCON_ADMODE_EN));
	Chip_IOCON_PinMuxSet(LPC_IOCON, CURR_SENSE_ADC_PIN, (IOCON_FUNC1 | IOCON_MODE_INACT | IOCON_ADMODE_EN));
	Chip_IOCON_PinMuxSet(LPC_IOCON, EXT_1_ADC_PIN, (IOCON_FUNC2 | IOCON_MODE_INACT | IOCON_ADMODE_EN));
	Chip_IOCON_PinMuxSet(LPC_IOCON, EXT_2_ADC_PIN, (IOCON_FUNC2 | IOCON_MODE_INACT | IOCON_ADMODE_EN));

	startDebouncer(&inwardEndstopDebouncer, INWARD_ENDSTOP_PORT, INWARD_ENDSTOP_PIN, TIMER_INWARD_ENDSTOP_DEBOUNCE, DEBOUNCE_PERIOD);
	startDebouncer(&outwardEndstopDebouncer, OUTWARD_ENDSTOP_PORT, OUTWARD_ENDSTOP_PIN, TIMER_OUTWARD_ENDSTOP_DEBOUNCE, DEBOUNCE_PERIOD);

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

	// Check debounced inputs
	checkDebouncer(&inwardEndstopDebouncer);
	checkDebouncer(&outwardEndstopDebouncer);
}

uint16_t HardwareGetMaxBattVoltage(void) {
	return(maxBattVoltage);
}

uint16_t HardwareGetMinBattVoltage(void) {
	return(minBattVoltage);
}

bool HardwareGetInwardEndstop(void) {
	// Invert, active low input
	return(!inwardEndstopDebouncer.debouncedState);
}

bool HardwareGetOutwardEndstop(void) {
	// Invert, active low input
	return(!outwardEndstopDebouncer.debouncedState);
}
