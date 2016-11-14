/*
 * modbus.h
 *
 *  Created on: 25 Jun 2016
 *      Author: Chris Mock
 */

#ifndef MODBUS_H_
#define MODBUS_H_



typedef enum {
	MB_SCARAB_ID1 = 0,
	// ...
	MB_SCARAB_ID15 = 15,
	MB_BOARD_VERSION,
	MB_FW_VERSION_MAJOR,
	MB_FW_VERSION_MINOR,
	MB_MODBUS_ERROR_COUNT,
	MB_UPTIME_MSW,
	MB_UPTIME_LSW,

	MB_BRIDGE_CURRENT = 100,
	MB_BATT_VOLTAGE,
	MB_MAX_BATT_VOLTAGE,
	MB_MIN_BATT_VOLTAGE,
	MB_BOARD_TEMPERATURE,
	MB_EXT_1_ADC,
	MB_EXT_2_ADC,
	MB_EXT_1_DIG,
	MB_EXT_2_DIG,
	MB_EXT_3_DIG,
	MB_EXT_4_DIG,
	MB_EXT_5_DIG,
	MB_EXT_6_DIG,
	MB_BLUE_LED,
	MB_GREEN_LED,
	MB_INWARD_ENDSTOP_STATE,
	MB_OUTWARD_ENDSTOP_STATE,

	MB_MOTOR_SETPOINT = 200,
	MB_MOTOR_SPEED,
	MB_MOTOR_ACCEL,
	MB_CURRENT_LIMIT_INWARD,
	MB_CURRENT_LIMIT_OUTWARD,
	MB_CURRENT_TRIPS_INWARD,
	MB_CURRENT_TRIPS_OUTWARD,
	MB_VOLTAGE_TRIPS,
	MB_ESTOP,
	MB_RESET_ESTOP,    // Write 0x5050 to reset emergency stop
	MB_MOTOR_PWM_FREQ_MSW,
	MB_MOTOR_PWM_FREQ_LSW,
	MB_MOTOR_PWM_DUTY_MSW,
	MB_MOTOR_PWM_DUTY_LSW,
	MB_INWARD_ENDSTOP_COUNT,
	MB_OUTWARD_ENDSTOP_COUNT,
	MB_HEARTBEAT_EXPIRIES,

	// Position info etc. = 300


	MB_UNLOCK_CONFIG = 9000,    // Write 0xA0A0 to unlock regs, anything else to lock
	MB_MODBUS_ADDRESS,
	MB_OPERATING_MODE,   // eg. Limit switches, encoders
	MB_OPERATING_CONFIG, // specific config for the selected mode
	MB_DEFAULT_CURRENT_LIMIT_INWARD,
	MB_DEFAULT_CURRENT_LIMIT_OUTWARD,
	MB_MAX_CURRENT_LIMIT_INWARD,
	MB_MAX_CURRENT_LIMIT_OUTWARD,
	MB_HEARTBEAT_TIMEOUT,  // seconds until heartbeat timer trips

	NUM_MODBUS_REGS
} modbusRegMap_t;

#define MODBUS_BAUD_RATE (9600)

#define MODBUS_SLAVE_ID (11)

uint16_t ModbusPoll(void);

void ModbusInit(uint32_t baud, uint8_t _slaveID, uint8_t _lowLatency);

#endif /* MODBUS_H_ */
