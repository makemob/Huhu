/*
===============================================================================
 Name        : huhu.c
 Author      : Chris Mock
 Version     :
 Copyright   : $(copyright)
 Description : main definition
===============================================================================
*/

#include "chip.h"
#include "hardware.h"
#include "timer.h"
#include "motor.h"
#include "comms.h"
#include "analogue.h"
#include "modbus.h"

#include <cr_section_macros.h>

#include <stdio.h>  // Note floating-point printf is enabled (sizeable)

#define WATCHDOG_SECONDS (2)  // approx

/* TODO:
 * Test heartbeat and adjust timeout
 * Enable crystal
 * Adjust PWM based on battery voltage
 * Cap max PWM to 2/3 (assuming 36v input)
 * Watchdog reset indicator
 * Nonfunctional modbus regs
 * Remove deprecated modbus regs / rework map
 *
 * Analog array name improvement sample/samples
 * Nonvolatile storage
 */

int main(void) {
	uint32_t lastTicker = TimerGetTicker();  // Used to gate watchdog

    // Read clock settings and update SystemCoreClock variable
    SystemCoreClockUpdate();

    // Configure watchdog
    Chip_WWDT_Init(LPC_WWDT);
    Chip_SYSCTL_PowerUp(SYSCTL_POWERDOWN_WDTOSC_PD);
    Chip_Clock_SetWDTOSC(WDTLFO_OSC_1_05, 20);
    Chip_Clock_SetWDTClockSource(SYSCTL_WDTCLKSRC_WDTOSC, 1);
    Chip_WWDT_SetTimeOut(LPC_WWDT, (Chip_Clock_GetWDTOSCRate() / 4) * WATCHDOG_SECONDS);
    Chip_WWDT_ClearStatusFlag(LPC_WWDT, WWDT_WDMOD_WDTOF);
    Chip_WWDT_SetOption(LPC_WWDT, WWDT_WDMOD_WDRESET);
    Chip_WWDT_Start(LPC_WWDT);
    Chip_WWDT_Feed(LPC_WWDT);

    TimerInit();
    HardwareInit();
    ADCInit();
    MotorInit();
    CommsInit(MODBUS_BAUD_RATE);
    ModbusInit(MODBUS_BAUD_RATE, MODBUS_SLAVE_ID, FALSE);

    TimerSetDurationMs(TIMER_TEST, 1000);

    // Wait for ADC sample buffer to fill
    while(!ADCIsReady())
    	;

    while(1) {

    	HardwarePoll();
    	MotorPoll();
    	CommsPoll();
    	ModbusPoll();

    	// Hit watchdog, gate on both interrupt and main loop time
    	if (lastTicker != TimerGetTicker()) {
    		Chip_WWDT_Feed(LPC_WWDT);
    		lastTicker = TimerGetTicker();
    	}

    	// wait for interrupt __WFI();

    }

    return(0);
}
