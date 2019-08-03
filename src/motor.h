/*
 * motor.h
 *
 *  Created on: 12 Jun 2016
 *      Author: Chris Mock
 */

#ifndef MOTOR_H_
#define MOTOR_H_

void MotorInit(void);

void MotorSetSpeed(int8_t percent);

void MotorEStop(void);

void MotorPoll(void);

int8_t MotorGetSetpoint(void);

int8_t MotorGetSpeed(void);

void MotorSetAccel(uint8_t percent);

uint8_t MotorGetAccel(void);

int16_t MotorGetGotoPosition(void);

void MotorSetGotoPosition(int16_t position);

void MotorSetGotoSpeedSetpoint(uint8_t speed);

uint8_t MotorGetGotoSpeedSetpoint(void);

void MotorSetCurrentLimitInward(uint16_t limit);

void MotorSetCurrentLimitOutward(uint16_t limit);

uint16_t MotorGetCurrentLimitInward(void);

uint16_t MotorGetCurrentLimitOutward(void);

void MotorSetExtensionLimitInward(int16_t limitTenthMillimetres);

void MotorSetExtensionLimitOutward(int16_t limitTenthMillimetres);

int16_t MotorGetExtensionLimitInward(void);

int16_t MotorGetExtensionLimitOutward(void);

uint16_t MotorGetCurrentTripsInward(void);

uint16_t MotorGetCurrentTripsOutward(void);

uint16_t MotorGetExtensionTripsInward(void);

uint16_t MotorGetExtensionTripsOutward(void);

uint16_t MotorGetVoltageTrips(void);

uint16_t MotorGetInwardEndstops(void);

uint16_t MotorGetOutwardEndstops(void);

uint16_t MotorGetEncoderFailTrips(void);

void MotorNotifyHeartbeat(void);

uint16_t MotorGetHeartbeatExpiries(void);

void MotorSetHeartbeatTimeout(uint16_t seconds);

uint16_t MotorGetHeartbeatTimeout(void);

void MotorSetEncoderFailTimeout(uint16_t milliseconds);

uint16_t MotorGetEncoderFailTimeout(void);

uint32_t MotorGetPWMPeriod(void);

uint32_t MotorGetPWMDuty(void);

void MotorResetEStop(void);

bool MotorGetEStopState(void);

#endif /* MOTOR_H_ */
