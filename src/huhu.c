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


/* TODO:
 * Enable crystal
 * Adjust PWM based on battery voltage
 * Cap max PWM to 2/3 (assuming 36v input)
 * Comms heartbeat
 * Watchdog
 * Report estop state
 *
 * Analog array name improvement sample/samples
 * Nonvolatile storage
 */

int main(void) {

    // Read clock settings and update SystemCoreClock variable
    SystemCoreClockUpdate();

    HardwareInit();
    TimerInit();
    ADCInit();
    MotorInit();
    CommsInit();
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

    	// If comms heartbeat timer expires, MotorEStop();

    	// hit watchdog

    	//__WFI();
    }

    return(0);
}
