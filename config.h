/*
 * TongSheng TSDZ2 motor controller firmware/
 *
 * Copyright (C) Casainho, 2018.
 *
 * Released under the GPL License, Version 3
 */

#ifndef _CONFIG_H_
#define _CONFIG_H_

// This file is the firmware configuration for the TSDZ2 motor controller,
// to run the 2 different available motors of 36V or 48V motor,
// and from 24V battery (7S) up to 52V battery pack (14S).


// *************************************************************************** //
// LCD

// you can tune LCD assist level
#define ASSIST_LEVEL_0      0.0
#define ASSIST_LEVEL_1      1.0
#define ASSIST_LEVEL_2      2.0
#define ASSIST_LEVEL_3      3.0
#define ASSIST_LEVEL_4      6.0
#define ASSIST_LEVEL_5      9.0

// *************************************************************************** //
// PAS configurations

// MAX cadence, where the output value of PAS processing will be the max value
#define PAS_MAX_CADENCE_RPM 120

// *************************************************************************** //
// BATTERY
// Choose your battery pack voltage

// the cells number can be a custom value from 7S up to 14S, choose the value of your battery pack
#define BATTERY_LI_ION_CELLS_NUMBER  13
//#define BATTERY_LI_ION_CELLS_NUMBER	10 // 10S = 36V battery pack
//#define BATTERY_LI_ION_CELLS_NUMBER	13 // 13S = 48V battery pack
//#define BATTERY_LI_ION_CELLS_NUMBER 14 // 14S = 52V battery pack

// This is the current that motor will draw from the battery
// Higher value will give higher torque and the limit of the controller is 16 amps
#define ADC_BATTERY_CURRENT_MAX 29 // 18 amps (0.625 amps each unit)

// Considering the follow voltage values for each li-ion battery cell
// State of charge 		| voltage
#define LI_ION_CELL_VOLTS_100   4.06
#define LI_ION_CELL_VOLTS_80    3.93
#define LI_ION_CELL_VOLTS_60    3.78
#define LI_ION_CELL_VOLTS_40    3.60
#define LI_ION_CELL_VOLTS_20    3.38
#define LI_ION_CELL_VOLTS_10    3.25
#define LI_ION_CELL_VOLTS_0     3.00

// *************************************************************************** //
// MOTOR CONTROLLER

// Choose PWM ramp up/down step (higher value will make the motor acceleration slower)
//
// For a 24V battery, 25 for ramp up seems ok. For an higher voltage battery, this values should be higher
#define PWM_DUTY_CYCLE_RAMP_UP_INVERSE_STEP 75
#define PWM_DUTY_CYCLE_RAMP_DOWN_INVERSE_STEP 25

// define whether the motor can start without pedal rotation.
// for safety, this should be turned off when not using e-brakes
#define MOTOR_ASSISTANCE_CAN_START_WITHOUT_PEDAL_ROTATION 1

// *************************************************************************** //
// MOTOR

// Choose some parameters for your motor (if you don't know, just keep the following original values because they should work ok)
//
// This value should be near 0.
// You can try to tune with the whell on the air, full throttle and look at batttery current: adjust for lower battery current
#define MOTOR_ROTOR_OFFSET_ANGLE 10

#define ADC_MOTOR_PHASE_CURRENT_MAX 48 // 30 amps (0.625 amps each unit)

// This value is ERPS speed after which a transition happens from sinewave no interpolation to have
// interpolation 60 degrees and must be found experimentally but a value of 25 may be good
#define MOTOR_ROTOR_ERPS_START_INTERPOLATION_60_DEGREES 10

#endif /* _CONFIG_H_ */
