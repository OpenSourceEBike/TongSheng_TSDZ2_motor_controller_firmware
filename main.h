/*
 * TongSheng TSDZ2 motor controller firmware/
 *
 * Copyright (C) Casainho, 2018.
 *
 * Released under the GPL License, Version 3
 */

#ifndef _MAIN_H_
#define _MAIN_H_

//#include "config.h"

#define DEBUG_UART

// *************************************************************************** //
// Throotle and PAS

#define EBIKE_THROTTLE_TYPE_TORQUE_SENSOR_AND_THROTTLE		1
#define EBIKE_THROTTLE_TYPE_PAS_AND_THROTTLE              2
#define EBIKE_THROTTLE_TYPE_THROTTLE_ONLY                 3
// *************************************************************************** //

#define LCD_TYPE_TSDZ2      1
#define LCD_TYPE_KUNTENG    2

#define PWM_CYCLES_COUNTER_MAX 3125 // 5 erps minimum speed; 1/5 = 200ms; 200ms/64us = 3125

#define PWM_CYCLES_SECOND 15625L // 1 / 64us(PWM period)

#define PWM_DUTY_CYCLE_MAX 254
#define PWM_DUTY_CYCLE_MIN 20
#define MIDDLE_PWM_DUTY_CYCLE_MAX (PWM_DUTY_CYCLE_MAX/2)

#define MOTOR_ROTOR_ANGLE_90    (63  + MOTOR_ROTOR_OFFSET_ANGLE)
#define MOTOR_ROTOR_ANGLE_150   (106 + MOTOR_ROTOR_OFFSET_ANGLE)
#define MOTOR_ROTOR_ANGLE_210   (148 + MOTOR_ROTOR_OFFSET_ANGLE)
#define MOTOR_ROTOR_ANGLE_270   (191 + MOTOR_ROTOR_OFFSET_ANGLE)
#define MOTOR_ROTOR_ANGLE_330   (233 + MOTOR_ROTOR_OFFSET_ANGLE)
#define MOTOR_ROTOR_ANGLE_30    (20  + MOTOR_ROTOR_OFFSET_ANGLE)

// this value of 174 (244 degrees; 170 would be 239 degrees) was found experimentaly
// seems to be about 180 + 60 degrees; were I expected to be 180 degrees
#define MOTOR_ROTOR_ANGLE_FOC   (174 + MOTOR_ROTOR_OFFSET_ANGLE)

#define MOTOR_OVER_SPEED_ERPS 520 // motor max speed, protection max value | 30 points for the sinewave at max speed

#define WHEEL_SPEED_PI_CONTROLLER_KP_DIVIDEND	100
#define WHEEL_SPEED_PI_CONTROLLER_KP_DIVISOR	4
#define WHEEL_SPEED_PI_CONTROLLER_KI_DIVIDEND	40
#define WHEEL_SPEED_PI_CONTROLLER_KI_DIVISOR	6

// Possible values: 0, 1, 2, 3, 4, 5, 6
// 0 equal to no filtering and no delay, higher values will increase filtering but will also add bigger delay
#define THROTTLE_FILTER_COEFFICIENT     1
#define ADC_THROTTLE_THRESHOLD      4 // value in ADC 8 bits step

#define CRUISE_CONTROL_MIN      20

// Max voltage value for throttle, in ADC 8 bits step
// each ADC 8 bits step = (5V / 256) = 0.0195
#define ADC_THROTTLE_MAX_VALUE 200
#define ADC_TORQUE_SENSOR_MAX_VALUE 229

// *************************************************************************** //
// PAS
#define PAS_NUMBER_MAGNETS  24

#define PAS_THRESHOLD 1.7

// (1/(150RPM/60)) / (PAS_NUMBER_MAGNETS * 0.000064)
#define PAS_ABSOLUTE_MAX_CADENCE_PWM_CYCLE_TICKS  (6250 / PAS_NUMBER_MAGNETS) // max hard limit to 150RPM PAS cadence
#define PAS_ABSOLUTE_MIN_CADENCE_PWM_CYCLE_TICKS  (156250 / PAS_NUMBER_MAGNETS) // min hard limit to 6RPM PAS cadence

#define PAS_NUMBER_MAGNETS_X2 (PAS_NUMBER_MAGNETS * 2)
// *************************************************************************** //

// *************************************************************************** //
// Wheel speed sensor
#define WHEEL_SPEED_SENSOR_MAX_PWM_CYCLE_TICKS  135 // something like 200m/h with a 6'' wheel
#define WHEEL_SPEED_SENSOR_MIN_PWM_CYCLE_TICKS  32767 // could be a bigger number but will make slow detecting wheel stopped
// *************************************************************************** //

// *************************************************************************** //
// EEPROM memory variables default values
#define DEFAULT_VALUE_ASSIST_LEVEL              2
#define DEFAULT_VALUE_MOTOR_CHARACTARISTIC      202 // for Q85 motor (12.6 * 16)
#define DEFAULT_VALUE_WHEEL_SIZE                20 // 26''
#define DEFAULT_VALUE_MAX_SPEED                 25
#define DEFAULT_VALUE_POWER_ASSIST_CONTROL_MODE 1
#define DEFAULT_VALUE_CONTROLLER_MAX_CURRENT    10
// *************************************************************************** //

// *************************************************************************** //
// BATTERY

// ADC Battery voltage
// 0.0862 per ADC_8bits step: 17.9V --> ADC_8bits = 52; 40V --> ADC_8bits = 116; this signal atenuated by the opamp 358
#define ADC10BITS_BATTERY_VOLTAGE_PER_ADC_STEP_X512 44
#define ADC8BITS_BATTERY_VOLTAGE_PER_ADC_STEP 0.343

// ADC Battery current
// 1A per 5 steps of ADC_10bits
#define ADC_BATTERY_CURRENT_PER_ADC_STEP_X512 102

#define COMMUNICATIONS_BATTERY_VOLTAGE  (uint8_t) (((float) BATTERY_LI_ION_CELLS_NUMBER) * 3.45) // example: 7S battery, should be = 24
#define ADC_BATTERY_VOLTAGE_MAX   (uint8_t) ((float) (BATTERY_LI_ION_CELLS_NUMBER * LI_ION_CELL_VOLTS_MAX) / ADC8BITS_BATTERY_VOLTAGE_PER_ADC_STEP)
#define ADC_BATTERY_VOLTAGE_10    (uint8_t) ((float) (BATTERY_LI_ION_CELLS_NUMBER * LI_ION_CELL_VOLTS_10)  / ADC8BITS_BATTERY_VOLTAGE_PER_ADC_STEP)
#define ADC_BATTERY_VOLTAGE_MIN   (uint8_t) ((float) (BATTERY_LI_ION_CELLS_NUMBER * LI_ION_CELL_VOLTS_0) / ADC8BITS_BATTERY_VOLTAGE_PER_ADC_STEP)

#define BATTERY_PACK_VOLTS_100  ((float) LI_ION_CELL_VOLTS_100 * (BATTERY_LI_ION_CELLS_NUMBER << 8))
#define BATTERY_PACK_VOLTS_80 ((float) LI_ION_CELL_VOLTS_80  * (BATTERY_LI_ION_CELLS_NUMBER << 8))
#define BATTERY_PACK_VOLTS_60 ((float) LI_ION_CELL_VOLTS_60  * (BATTERY_LI_ION_CELLS_NUMBER << 8))
#define BATTERY_PACK_VOLTS_40 ((float) LI_ION_CELL_VOLTS_40  * (BATTERY_LI_ION_CELLS_NUMBER << 8))
#define BATTERY_PACK_VOLTS_20 ((float) LI_ION_CELL_VOLTS_20  * (BATTERY_LI_ION_CELLS_NUMBER << 8))
#define BATTERY_PACK_VOLTS_10 ((float) LI_ION_CELL_VOLTS_10  * (BATTERY_LI_ION_CELLS_NUMBER << 8))
#define BATTERY_PACK_VOLTS_0  ((float) LI_ION_CELL_VOLTS_0   * (BATTERY_LI_ION_CELLS_NUMBER << 8))

// Possible values: 0, 1, 2, 3, 4, 5, 6
// 0 equal to no filtering and no delay, higher values will increase filtering but will also add bigger delay
#define READ_BATTERY_CURRENT_FILTER_COEFFICIENT 2
#define READ_BATTERY_VOLTAGE_FILTER_COEFFICIENT 2
// *************************************************************************** //

#endif // _MAIN_H_
