/*
 * modbus.c
 *
 *  Created on: 22 Jun 2016
 *      Author: Chris Mock
 *
 *      Based on Simple Modbus Slave with small mods to use LPC comms:
 *       https://github.com/angeloc/simplemodbusng/tree/master/SimpleModbusSlave
 *
 */

#include "chip.h"
#include "hardware.h"
#include "modbus.h"
#include "comms.h"
#include "timer.h"
#include "analogue.h"
#include "motor.h"

#define BUFFER_SIZE 128

// frame[] is used to receive and transmit packages.
// The maximum serial ring buffer size is 128
static uint8_t frame[BUFFER_SIZE];
static uint8_t broadcastFlag;
static uint8_t slaveID;
static uint8_t function;
// static uint8_t TxEnablePin;
static uint16_t errorCount;
static uint16_t T1_5; // inter character time out
static uint16_t T3_5; // frame delay
static uint8_t lastByteCount;
//static uint16_t holdingRegs[NUM_MODBUS_REGS];

// function definitions
static void exceptionResponse(uint8_t exception);
static uint16_t calculateCRC(uint8_t bufferSize);
// static void sendPacket(uint8_t bufferSize);
static void receivePacket(uint8_t byteCount);

uint16_t ModbusPoll(void)
{
	uint8_t byteCount = CommsGetNumRxChars();

	if (byteCount > BUFFER_SIZE) {

		// Overflow, dump buffer
		CommsFlushRxBuffer();
		TimerForceExpiry(TIMER_MODBUS_INTERCHAR);
		lastByteCount = 0;
		errorCount++;

	} else {

		if (byteCount != lastByteCount) {
			// Incoming char(s) within interchar timeout
			// Only have a millisecond timer, so be liberal and round up
			TimerSetDurationMs(TIMER_MODBUS_INTERCHAR, (T1_5 + 500) / 1000);
			lastByteCount = byteCount;
		}

		if (TimerCheckExpired(TIMER_MODBUS_INTERCHAR)) {
			// Incoming frame
			byteCount = CommsReadChars(frame, BUFFER_SIZE);
			lastByteCount = 0;
			receivePacket(byteCount);
		}
	}

	return(errorCount);
}

static uint16_t fetchHoldingReg(uint16_t index) {
	uint16_t response = 0;

	if ((index >= MB_SCARAB_ID1) && (index <= MB_SCARAB_ID15)) {
		response = ScarabID[MB_SCARAB_ID1 + index];
	}

	switch (index) {
	// case MB_SCARAB_ID1:
	// ...
	// case MB_SCARAB_ID15:

	case MB_BOARD_VERSION:
		response = BOARD_VERSION;
		break;
	case MB_FW_VERSION_MAJOR:
		response = FW_VERSION_MAJOR;
		break;
	case MB_FW_VERSION_MINOR:
		response = FW_VERSION_MINOR;
		break;
	case MB_MODBUS_ERROR_COUNT:
		response = errorCount;
		break;
	case MB_UPTIME_MSW:
		response = (uint16_t)(TimerGetUptimeSeconds() >> 8);
		break;
	case MB_UPTIME_LSW:
		response = (uint16_t)(TimerGetUptimeSeconds() & 0xFFFF);
		break;

	// 100 block:
	case MB_BRIDGE_CURRENT:
		response = ADCGetBridgeCurrent();
		break;
	case MB_BATT_VOLTAGE:
		response = ADCGetBattVoltage();
		break;
	case MB_MAX_BATT_VOLTAGE:
		response = HardwareGetMaxBattVoltage();
		break;
	case MB_MIN_BATT_VOLTAGE:
		response = HardwareGetMinBattVoltage();
		break;
	case MB_BOARD_TEMPERATURE:
		response = ADCGetTemperature();
		break;
	case MB_EXT_1_ADC:
		response = ADCGetExt1();
		break;
	case MB_EXT_2_ADC:
		response = ADCGetExt2();
		break;
	case MB_EXT_1_DIG:
		response = Chip_GPIO_GetPinState(LPC_GPIO, EXT_1_PORT, EXT_1_PIN);
		break;
	case MB_EXT_2_DIG:
		response = Chip_GPIO_GetPinState(LPC_GPIO, EXT_2_PORT, EXT_2_PIN);
		break;
	case MB_EXT_3_DIG:
		response = Chip_GPIO_GetPinState(LPC_GPIO, EXT_3_PORT, EXT_3_PIN);
		break;
	case MB_EXT_4_DIG:
		response = Chip_GPIO_GetPinState(LPC_GPIO, EXT_4_PORT, EXT_4_PIN);
		break;
	case MB_EXT_5_DIG:
		response = Chip_GPIO_GetPinState(LPC_GPIO, EXT_5_PORT, EXT_5_PIN);
		break;
	case MB_EXT_6_DIG:
		response = Chip_GPIO_GetPinState(LPC_GPIO, EXT_6_PORT, EXT_6_PIN);
		break;
	case MB_BLUE_LED:
		response = Chip_GPIO_GetPinState(LPC_GPIO, BLUE_LED_PORT, BLUE_LED_PIN);
		break;
	case MB_GREEN_LED:
		response = Chip_GPIO_GetPinState(LPC_GPIO, GREEN_LED_PORT, GREEN_LED_PIN);
		break;
	// MB_INWARD_ENDSTOP_STATE_DEPRECATED
	// MB_OUTWARD_ENDSTOP_STATE_DEPRECATED
	case MB_POSITION_ENCODER_COUNTS:
		response = HardwareGetPositionEncoderCounts();
		break;

	// 200 block:
	case MB_MOTOR_SETPOINT:
		response = (uint16_t)MotorGetSetpoint();
		break;
	case MB_MOTOR_SPEED:
		response = (uint16_t)MotorGetSpeed();
		break;
	case MB_MOTOR_ACCEL:
		response = MotorGetAccel();
		break;
	case MB_CURRENT_LIMIT_INWARD:
		response = MotorGetCurrentLimitInward();
		break;
	case MB_CURRENT_LIMIT_OUTWARD:
		response = MotorGetCurrentLimitOutward();
		break;
	// MB_CURRENT_TRIPS_INWARD_DEPRECATED
	// MB_CURRENT_TRIPS_OUTWARD_DEPRECATED
	case MB_EXTENSION_LIMIT_INWARD:
		response = (uint16_t)MotorGetExtensionLimitInward();
		break;
	case MB_EXTENSION_LIMIT_OUTWARD:
		response = (uint16_t)MotorGetExtensionLimitOutward();
		break;
	case MB_POSITION_ENCODER_SCALING:
		response = HardwareGetPositionEncoderScaling();
		break;
	// MB_VOLTAGE_TRIPS_DEPRECATED
	// MB_INWARD_ENDSTOP_COUNT_DEPRECATED
	// MB_OUTWARD_ENDSTOP_COUNT_DEPRECATED
	// MB_HEARTBEAT_EXPIRIES_DEPRECATED
	//case MB_ESTOP:
		//
	//	break;
	//case MB_RESET_ESTOP:    // Write 0x5050 to reset emergency stop
		//
	//	break;
	case MB_MOTOR_PWM_FREQ_MSW:
		response = (uint16_t)(MotorGetPWMPeriod() / 0xFFFF);
		break;
	case MB_MOTOR_PWM_FREQ_LSW:
		response = (uint16_t)(MotorGetPWMPeriod() & 0xFFFF);
		break;
	case MB_MOTOR_PWM_DUTY_MSW:
		response = (uint16_t)(MotorGetPWMDuty() / 0xFFFF);
		break;
	case MB_MOTOR_PWM_DUTY_LSW:
		response = (uint16_t)(MotorGetPWMDuty() & 0xFFFF);
		break;

	case MB_GOTO_POSITION:
		response = MotorGetGotoPosition();
		break;
	case MB_GOTO_SPEED_SETPOINT:
		response = MotorGetGotoSpeedSetpoint();
		break;
	// MB_FORCE_CALIBRATE_ENCODER

	// 300 block:
	case MB_EXTENSION:  // 299
		response = (uint16_t)HardwareGetPositionEncoderDistance();
		break;
	case MB_ESTOP_STATE:
		response = MotorGetEStopState();
		break;
	case MB_CURRENT_TRIPS_INWARD:
		response = MotorGetCurrentTripsInward();
		break;
	case MB_CURRENT_TRIPS_OUTWARD:
		response = MotorGetCurrentTripsOutward();
		break;
	case MB_INWARD_ENDSTOP_STATE:
	case MB_INWARD_ENDSTOP_STATE_DEPRECATED:
		response = HardwareGetInwardEndstop();
		break;
	case MB_OUTWARD_ENDSTOP_STATE:
	case MB_OUTWARD_ENDSTOP_STATE_DEPRECATED:
		response = HardwareGetOutwardEndstop();
		break;
	case MB_INWARD_ENDSTOP_COUNT:
	case MB_INWARD_ENDSTOP_COUNT_DEPRECATED:
			response = MotorGetInwardEndstops();
			break;
	case MB_OUTWARD_ENDSTOP_COUNT:
	case MB_OUTWARD_ENDSTOP_COUNT_DEPRECATED:
		response = MotorGetOutwardEndstops();
		break;
	case MB_VOLTAGE_TRIPS:
		response = MotorGetVoltageTrips();
		break;
	case MB_HEARTBEAT_EXPIRIES:
	case MB_HEARTBEAT_EXPIRIES_DEPRECATED:
		response = MotorGetHeartbeatExpiries();
		break;
	case MB_EXTENSION_TRIPS_INWARD:
		response = MotorGetExtensionTripsInward();
		break;
	case MB_EXTENSION_TRIPS_OUTWARD:
		response = MotorGetExtensionTripsOutward();
		break;
	case MB_ENCODER_FAIL_TRIPS:
		response = MotorGetEncoderFailTrips();
		break;

	// 9000 block:
	case MB_UNLOCK_CONFIG:    // Write 0xA0A0 to unlock regs, anything else to lock
		break;
	case MB_MODBUS_ADDRESS:
		response = slaveID;
		break;
	case MB_OPERATING_MODE:   // eg. Limit switches, encoders
		break;
	case MB_OPERATING_CONFIG: // specific config for the selected mode
		break;
	case MB_DEFAULT_CURRENT_LIMIT_INWARD:
		break;
	case MB_DEFAULT_CURRENT_LIMIT_OUTWARD:
		break;
	case MB_MAX_CURRENT_LIMIT_INWARD:
		break;
	case MB_MAX_CURRENT_LIMIT_OUTWARD:
		break;
	case MB_HEARTBEAT_TIMEOUT:
		response = MotorGetHeartbeatTimeout();
		break;
	case MB_ENCODER_FAIL_TIMEOUT:
		response = MotorGetEncoderFailTimeout();
		break;

	default:
		break;
}


	return(response);
}

static void processHoldingRegChange(uint16_t index, uint16_t value) {

	switch (index) {
	//case MB_SCARAB_ID1:
	//	break;
	// ...
	//case MB_SCARAB_ID15:
	//	break;
	//case MB_BOARD_VERSION:
	//	break;
	//case MB_FW_VERSION_MAJOR:
	//	break;
	//case MB_FW_VERSION_MINOR:
	//	break;
	//case MB_MODBUS_ERROR_COUNT:
	//	break;
	//case MB_UPTIME_MSW:
	//	break;
	//case MB_UPTIME_LSW:
	//	break;

	//case MB_BRIDGE_CURRENT:
	//	break;
	//case MB_BATT_VOLTAGE:
	//	break;
	//case MB_MAX_BATT_VOLTAGE:
	//	break;
	//case MB_MIN_BATT_VOLTAGE:
	//	break;
	//case MB_BOARD_TEMPERATURE:
	//	break;
	//case MB_EXT_1_ADC:
	//	break;
	//case MB_EXT_2_ADC:
	//	break;
	//case MB_EXT_1_DIG:
	//	break;
	//case MB_EXT_2_DIG:
	//	break;
	//case MB_EXT_3_DIG:
	//	break;
	//case MB_EXT_4_DIG:
	//	break;
	//case MB_EXT_5_DIG:
	//	break;
	//case MB_EXT_6_DIG:
	//	break;
	case MB_BLUE_LED:
		if (value) {
			Chip_GPIO_SetPinOutHigh(LPC_GPIO, BLUE_LED_PORT, BLUE_LED_PIN);
		} else {
			Chip_GPIO_SetPinOutLow(LPC_GPIO, BLUE_LED_PORT, BLUE_LED_PIN);
		}
		break;
	case MB_GREEN_LED:
		if (value) {
			Chip_GPIO_SetPinOutHigh(LPC_GPIO, GREEN_LED_PORT, GREEN_LED_PIN);
		} else {
			Chip_GPIO_SetPinOutLow(LPC_GPIO, GREEN_LED_PORT, GREEN_LED_PIN);
		}
		break;
	//case MB_INWARD_ENDSTOP_STATE:
	//  break;
	//case MB_OUTWARD_ENDSTOP_STATE:
	//  break;

	case MB_MOTOR_SETPOINT:
		MotorSetSpeed((int8_t)value);
		break;
	//case MB_MOTOR_SPEED:
	//	break;
	case MB_MOTOR_ACCEL:
		MotorSetAccel((uint8_t)value);
		break;
	case MB_CURRENT_LIMIT_INWARD:
		MotorSetCurrentLimitInward(value);
		break;
	case MB_CURRENT_LIMIT_OUTWARD:
		MotorSetCurrentLimitOutward(value);
		break;
	case MB_EXTENSION_LIMIT_INWARD:
		MotorSetExtensionLimitInward(value);
		break;
	case MB_EXTENSION_LIMIT_OUTWARD:
		MotorSetExtensionLimitOutward(value);
		break;
	case MB_POSITION_ENCODER_SCALING:
		HardwareSetPositionEncoderScaling(value);
		break;
	//case MB_CURRENT_TRIPS_INWARD_DEPRECATED:
	//	break;
	//case MB_CURRENT_TRIPS_OUTWARD_DEPRECATED:
	//	break;
	//case MB_VOLTAGE_TRIPS_DEPRECATED:
	//	break;
	case MB_ESTOP:
		MotorEStop();
		break;
	case MB_RESET_ESTOP:    // Write 0x5050 to reset emergency stop
		if (value == 0x5050) {
			MotorResetEStop();
		}
		break;
	//case MB_MOTOR_PWM_FREQ_MSW:
	//	break;
	//case MB_MOTOR_PWM_FREQ_LSW:
	//	break;
	//case MB_MOTOR_PWM_DUTY_MSW:
	//	break;
	//case MB_MOTOR_PWM_DUTY_LSW:
	//	break;
	//case MB_INWARD_ENDSTOP_COUNT:
	//	break;
	//case MB_OUTWARD_ENDSTOP_COUNT:
	//	break;
	//case MB_HEARTBEAT_EXPIRIES:
	//  break;
	case MB_GOTO_POSITION:
		MotorSetGotoPosition(value);
		break;
	case MB_GOTO_SPEED_SETPOINT:
		MotorSetGotoSpeedSetpoint(value);
		break;
	case MB_FORCE_CALIBRATE_ENCODER:   // write 0xA0A0 to force encoder to calibrate to zero in current position
		if (value == 0xA0A0) {
			HardwareForceCalibrateEncoder();
		}
		break;

//	case MB_UNLOCK_CONFIG:    // Write 0xA0A0 to unlock regs, anything else to lock
//		break;
//	case MB_MODBUS_ADDRESS:
//		break;
//	case MB_OPERATING_MODE:   // eg. Limit switches, encoders
//		break;
//	case MB_OPERATING_CONFIG: // specific config for the selected mode
//		break;
//	case MB_DEFAULT_CURRENT_LIMIT_INWARD:
//		break;
//	case MB_DEFAULT_CURRENT_LIMIT_OUTWARD:
//		break;
//	case MB_MAX_CURRENT_LIMIT_INWARD:
//		break;
//	case MB_MAX_CURRENT_LIMIT_OUTWARD:
//		break;
	case MB_HEARTBEAT_TIMEOUT:
		MotorSetHeartbeatTimeout(value);
		break;
	case MB_ENCODER_FAIL_TIMEOUT:
		MotorSetEncoderFailTimeout(value);
		break;
	default:
		break;
	}

}

/*
	uint8_t buffer = 0;
	uint8_t overflow = 0;

	while (Serial.available())
	{
		// The maximum number of bytes is limited to the serial buffer size of 128 bytes
		// If more bytes is received than the BUFFER_SIZE the overflow flag will be set and the
		// serial buffer will be read until all the data is cleared from the receive buffer.
		if (overflow)
			Serial.read();
		else
		{
			if (buffer == BUFFER_SIZE)
				overflow = 1;
			frame[buffer] = Serial.read();
			buffer++;
		}
		delayMicroseconds(T1_5); // inter character time out
	}
*/



static void receivePacket(uint8_t byteCount) {

/*
	// If an overflow occurred increment the errorCount
	// variable and return to the main sketch without
	// responding to the request i.e. force a timeout
	if (overflow)
		return errorCount++;
*/

	// The minimum request packet is 8 bytes for function 3 & 16
	if (byteCount > 6)
	{
		uint8_t id = frame[0];

		broadcastFlag = 0;

		if (id == 0)
			broadcastFlag = 1;

		if (id == slaveID || broadcastFlag) // if the received ID matches the slaveID or broadcasting id (0), continue
		{
			uint16_t crc = ((frame[byteCount - 2] << 8) | frame[byteCount - 1]); // combine the crc Low & High bytes
			if (calculateCRC(byteCount - 2) == crc) // if the calculated crc matches the received crc continue
			{
				function = frame[1];
				uint16_t startingAddress = ((frame[2] << 8) | frame[3]); // combine the starting address bytes
				uint16_t no_of_registers = ((frame[4] << 8) | frame[5]); // combine the number of register bytes
				uint16_t maxData = startingAddress + no_of_registers;
				uint16_t index;
				uint8_t address;
				uint16_t crc16;

				// valid message to this address received, send heartbeat to motor drive
				MotorNotifyHeartbeat();

				// broadcasting is not supported for function 3
				if (!broadcastFlag && (function == 3))
				{
					if (startingAddress < NUM_MODBUS_REGS) // check exception 2 ILLEGAL DATA ADDRESS
					{
						if (maxData <= NUM_MODBUS_REGS) // check exception 3 ILLEGAL DATA VALUE
						{
							uint8_t noOfBytes = no_of_registers * 2;
							uint8_t responseFrameSize = 5 + noOfBytes; // ID, function, noOfBytes, (dataLo + dataHi) * number of registers, crcLo, crcHi
							frame[0] = slaveID;
							frame[1] = function;
							frame[2] = noOfBytes;
							address = 3; // PDU starts at the 4th byte
							uint16_t temp;

							for (index = startingAddress; index < maxData; index++)
							{
								//temp = holdingRegs[index];
								temp = fetchHoldingReg(index);
								frame[address] = temp >> 8; // split the register into 2 bytes
								address++;
								frame[address] = temp & 0xFF;
								address++;
							}

							crc16 = calculateCRC(responseFrameSize - 2);
							frame[responseFrameSize - 2] = crc16 >> 8; // split crc into 2 bytes
							frame[responseFrameSize - 1] = crc16 & 0xFF;
							CommsSendChars(frame, responseFrameSize);
						}
						else
							exceptionResponse(3); // exception 3 ILLEGAL DATA VALUE
					}
					else
						exceptionResponse(2); // exception 2 ILLEGAL DATA ADDRESS
				}
				else if (function == 6)
				{
					if (startingAddress < NUM_MODBUS_REGS) // check exception 2 ILLEGAL DATA ADDRESS
					{
						uint16_t startingAddress = ((frame[2] << 8) | frame[3]);
						uint16_t regStatus = ((frame[4] << 8) | frame[5]);
						uint8_t responseFrameSize = 8;

						//holdingRegs[startingAddress] = regStatus;
						processHoldingRegChange(startingAddress, regStatus);

						crc16 = calculateCRC(responseFrameSize - 2);
						frame[responseFrameSize - 2] = crc16 >> 8; // split crc into 2 bytes
						frame[responseFrameSize - 1] = crc16 & 0xFF;
						CommsSendChars(frame, responseFrameSize);
					}
					else
						exceptionResponse(2); // exception 2 ILLEGAL DATA ADDRESS
				}
				else if (function == 16)
				{
					// check if the received number of bytes matches the calculated bytes minus the request bytes
					// id + function + (2 * address bytes) + (2 * no of register bytes) + byte count + (2 * CRC bytes) = 9 bytes
					if (frame[6] == (byteCount - 9))
					{
						if (startingAddress < NUM_MODBUS_REGS) // check exception 2 ILLEGAL DATA ADDRESS
						{
							if (maxData <= NUM_MODBUS_REGS) // check exception 3 ILLEGAL DATA VALUE
							{
								address = 7; // start at the 8th byte in the frame

								for (index = startingAddress; index < maxData; index++)
								{
									// holdingRegs[index] = ((frame[address] << 8) | frame[address + 1]);
									processHoldingRegChange(index, ((frame[address] << 8) | frame[address + 1]));
									address += 2;
								}

								// only the first 6 bytes are used for CRC calculation
								crc16 = calculateCRC(6);
								frame[6] = crc16 >> 8; // split crc into 2 bytes
								frame[7] = crc16 & 0xFF;

								// a function 16 response is an echo of the first 6 bytes from the request + 2 crc bytes
								if (!broadcastFlag) // don't respond if it's a broadcast message
									CommsSendChars(frame, 8);
							}
							else
								exceptionResponse(3); // exception 3 ILLEGAL DATA VALUE
						}
						else
							exceptionResponse(2); // exception 2 ILLEGAL DATA ADDRESS
					}
					else
						errorCount++; // corrupted packet
				}
				else
					exceptionResponse(1); // exception 1 ILLEGAL FUNCTION
			}
			else // checksum failed
				errorCount++;
		} // incorrect id
	}
	else if (byteCount > 1 && byteCount < 8) // ignore byteCount == 1 packets, these can get triggered when switching to TX
		errorCount++; // corrupted packet

}

static void exceptionResponse(uint8_t exception)
{
	errorCount++; // each call to exceptionResponse() will increment the errorCount
	if (!broadcastFlag) // don't respond if its a broadcast message
	{
		frame[0] = slaveID;
		frame[1] = (function | 0x80); // set the MSB bit high, informs the master of an exception
		frame[2] = exception;
		uint16_t crc16 = calculateCRC(3); // ID, function + 0x80, exception code == 3 bytes
		frame[3] = crc16 >> 8;
		frame[4] = crc16 & 0xFF;
		CommsSendChars(frame, 5);  // exception response is always 5 bytes ID, function + 0x80, exception code, 2 bytes crc
	}
}

void ModbusInit(uint32_t baud, uint8_t _slaveID, uint8_t _lowLatency)
{
	slaveID = _slaveID;

	/* Dealt with in comms layer

	Serial.begin(baud);

	if (_TxEnablePin > 1)
	{ // pin 0 & pin 1 are reserved for RX/TX. To disable set txenpin < 2
		TxEnablePin = _TxEnablePin;
		pinMode(TxEnablePin, OUTPUT);
		digitalWrite(TxEnablePin, LOW);
	}

	*/

	// Modbus states that a baud rate higher than 19200 must use a fixed 750 us
	// for inter character time out and 1.75 ms for a frame delay.
	// For baud rates below 19200 the timeing is more critical and has to be calculated.
	// E.g. 9600 baud in a 10 bit packet is 960 characters per second
	// In milliseconds this will be 960characters per 1000ms. So for 1 character
	// 1000ms/960characters is 1.04167ms per character and finaly modbus states an
	// intercharacter must be 1.5T or 1.5 times longer than a normal character and thus
	// 1.5T = 1.04167ms * 1.5 = 1.5625ms. A frame delay is 3.5T.
	// Added sperimentally low latency delays. This makes the implementation
	// non-standard but practically it works with all major modbus master implementations.

	if (baud == 1000000 && _lowLatency)
	{
		T1_5 = 1;
		T3_5 = 10;
	}
	else if (baud >= 115200 && _lowLatency){
		T1_5 = 75;
		T3_5 = 175;
	}
	else if (baud > 19200)
	{
		T1_5 = 750;
		T3_5 = 1750;
	}
	else
	{
		T1_5 = 15000000/baud; // 1T * 1.5 = T1.5
		T3_5 = 35000000/baud; // 1T * 3.5 = T3.5
	}

	errorCount = 0; // initialize errorCount
	lastByteCount = 0;
	CommsSetTxEnableDelay(T3_5 / 1000);  // Note inaccurate frame delay to use millisecond timer (rounds down)
}

static uint16_t calculateCRC(uint8_t bufferSize)
{
	uint16_t temp, temp2, flag;
	temp = 0xFFFF;
	for (uint8_t i = 0; i < bufferSize; i++)
	{
		temp = temp ^ frame[i];
		for (uint8_t j = 1; j <= 8; j++)
		{
			flag = temp & 0x0001;
			temp >>= 1;
			if (flag)
				temp ^= 0xA001;
		}
	}
	// Reverse byte order.
	temp2 = temp >> 8;
	temp = (temp << 8) | temp2;
	temp &= 0xFFFF;
	return(temp); // the returned value is already swapped - crcLo byte is first & crcHi byte is last
}

/*
static void sendPacket(uint8_t bufferSize)
{


	if (TxEnablePin > 1)
		digitalWrite(TxEnablePin, HIGH);

	for (uint8_t i = 0; i < bufferSize; i++)
		Serial.write(frame[i]);

	Serial.flush();

	// allow a frame delay to indicate end of transmission
	delayMicroseconds(T3_5);

	if (TxEnablePin > 1)
		digitalWrite(TxEnablePin, LOW);
}
*/

