/*
 * TongSheng TSDZ2 motor controller firmware/
 *
 * Copyright (C) Casainho, 2018.
 *
 * Released under the GPL License, Version 3
 */

#ifndef CONFIG_H_
#define CONFIG_H_

// This file is the firmware configuration for the TSDZ2 motor controller,
// to run the 2 different available motors of 36V or 48V motor,
// and from 24V battery (7S) up to 52V battery pack (14S).

// *************************************************************************** //
// THROTTLE
//

// next, choose one of the both (only apply to throttle and/or PAS)
//#define EBIKE_THROTTLE_TYPE_THROTTLE_PAS_PWM_DUTY_CYCLE // direct PWM duty_cycle control, important for developers
#define EBIKE_THROTTLE_TYPE_THROTTLE_PAS_CURRENT_SPEED // control using motor current/torque and/or wheel speed

// next, if enabled, output of torque sensor algorithm is the human power (torque * cadence) other way will be the same as the torque signal
// (only apply to torque sensor)
#define EBIKE_THROTTLE_TYPE_TORQUE_SENSOR_HUMAN_POWER

// PAS configurations
// MAX cadence, where the output value of PAS processiing will be the max value
#define PAS_MAX_CADENCE_RPM 	90
// *************************************************************************** //

// *************************************************************************** //
// LCD

// you can tune LCD assist level
#define ASSIST_LEVEL_0			0.00
#define ASSIST_LEVEL_1 			0.44
#define ASSIST_LEVEL_2 			0.66
#define ASSIST_LEVEL_3 			1.00
#define ASSIST_LEVEL_4 			1.5
#define ASSIST_LEVEL_5 			2.25
// *************************************************************************** //

// *************************************************************************** //
// BATTERY
// Choose your battery pack voltage

// the cells number can be a custom value from 7S up to 14S, choose the value of your battery pack
#define BATTERY_LI_ION_CELLS_NUMBER	 7 // 7S = 24V battery pack
//#define BATTERY_LI_ION_CELLS_NUMBER	10 // 10S = 36V battery pack
//#define BATTERY_LI_ION_CELLS_NUMBER	13 // 13S = 48V battery pack
//#define BATTERY_LI_ION_CELLS_NUMBER 14 // 14S = 52V battery pack

// This is the current that motor will draw from the battery
// Higher value will give higher torque and the limit of the controller is 16 amps
#define ADC_BATTERY_CURRENT_MAX		224 // (14 ADC bits step per 1 amp) so, 224 for 16 amps

// Considering the follow voltage values for each li-ion battery cell
// State of charge 		| voltage
#define LI_ION_CELL_VOLTS_100 	4.06 // this value is used to help finding the battery SOC
#define LI_ION_CELL_VOLTS_80 		3.93 // this value is used to help finding the battery SOC
#define LI_ION_CELL_VOLTS_60 		3.78 // this value is used to help finding the battery SOC
#define LI_ION_CELL_VOLTS_40 		3.60 // this value is used to help finding the battery SOC
#define LI_ION_CELL_VOLTS_20 		3.38 // this value is used to help finding the battery SOC
#define LI_ION_CELL_VOLTS_10 		3.25 // this value is used to help finding the battery SOC
#define LI_ION_CELL_VOLTS_0 		3.00 // this value is used to help finding the battery SOC and is the minimum value after which the firmware don't ask more current to battery, to run the motor
// *************************************************************************** //

// *************************************************************************** //
// MOTOR CONTROLLER

// Choose PWM ramp up/down step (higher value will make the motor acceleration slower)
//
// For a 24V battery, 25 for ramp up seems ok. For an higher voltage battery, this values should be higher
#define PWM_DUTY_CYCLE_RAMP_UP_INVERSE_STEP	25
#define PWM_DUTY_CYCLE_RAMP_DOWN_INVERSE_STEP	10
// *************************************************************************** //

// *************************************************************************** //
// MOTOR

// Choose some parameters for your motor (if you don't know, just keep the following original values because they should work ok)
//
// This value should be near 0.
// You can try to tune with the whell on the air, full throttle and look at batttery current: adjust for lower battery current
#define MOTOR_ROTOR_OFFSET_ANGLE 0

// This value is ERPS speed after which a transition happens from sinewave no interpolation to have
// interpolation 60 degrees and must be found experimentally but a value of 25 may be good
#define MOTOR_ROTOR_ERPS_START_INTERPOLATION_60_DEGREES 25
// *************************************************************************** //

#endif /* CONFIG_H_ */
