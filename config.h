/*
 * config.h
 *
 *  Automatically created by Flexible OpenSource firmware - Configuration tool
 *  Author: stancecoke
 *  Author: casainho
 */

#ifndef _CONFIG_H_
#define _CONFIG_H_

//#define EBIKE_THROTTLE_TYPE EBIKE_THROTTLE_TYPE_THROTTLE_ONLY
#define EBIKE_THROTTLE_TYPE EBIKE_THROTTLE_TYPE_PAS_AND_THROTTLE
#define ASSIST_LEVEL_0      0.0
#define ASSIST_LEVEL_1      2.0
#define ASSIST_LEVEL_2      4.0
#define ASSIST_LEVEL_3      6.0
#define ASSIST_LEVEL_4      8.0
#define ASSIST_LEVEL_5      10.0
#define PAS_MAX_CADENCE_RPM 120
#define PWM_DUTY_CYCLE_RAMP_UP_INVERSE_STEP 75
#define PWM_DUTY_CYCLE_RAMP_DOWN_INVERSE_STEP 25
#define MOTOR_ROTOR_OFFSET_ANGLE 10
#define MOTOR_ROTOR_ERPS_START_INTERPOLATION_60_DEGREES 10
#define ADC_BATTERY_CURRENT_MAX 29 // 18 amps (0.625 amps each unit)
#define ADC_MOTOR_PHASE_CURRENT_MAX 64 // 40 amps
#define BATTERY_LI_ION_CELLS_NUMBER  13
#define LI_ION_CELL_VOLTS_100   4.06
#define LI_ION_CELL_VOLTS_80    3.93
#define LI_ION_CELL_VOLTS_60    3.78
#define LI_ION_CELL_VOLTS_40    3.60
#define LI_ION_CELL_VOLTS_20    3.38
#define LI_ION_CELL_VOLTS_10    3.25
#define LI_ION_CELL_VOLTS_0     3.00

#endif /* _CONFIG_H_ */
