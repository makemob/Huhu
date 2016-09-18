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

void MotorSetCurrentLimitInward(uint16_t limit);

void MotorSetCurrentLimitOutward(uint16_t limit);

uint16_t MotorGetCurrentLimitInward(void);

uint16_t MotorGetCurrentLimitOutward(void);

uint16_t MotorGetCurrentTripsInward(void);

uint16_t MotorGetCurrentTripsOutward(void);

uint16_t MotorGetVoltageTrips(void);

uint16_t MotorGetInwardEndstops(void);

uint16_t MotorGetOutwardEndstops(void);

uint32_t MotorGetPWMPeriod(void);

uint32_t MotorGetPWMDuty(void);

void MotorResetEStop(void);

#endif /* MOTOR_H_ */
