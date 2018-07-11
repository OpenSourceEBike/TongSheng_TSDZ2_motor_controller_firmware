/*
 * TongSheng TSDZ2 motor controller firmware/
 *
 * Copyright (C) Casainho, 2018.
 *
 * Released under the GPL License, Version 3
 */

#ifndef _EBIKE_APP_H_
#define _EBIKE_APP_H_

#include <stdint.h>
#include "main.h"

#define EBIKE_APP_STATE_MOTOR_COAST   0
#define EBIKE_APP_STATE_MOTOR_STOP  1
#define EBIKE_APP_STATE_MOTOR_STARTUP   2
#define EBIKE_APP_STATE_MOTOR_COOL  3
#define EBIKE_APP_STATE_MOTOR_RUNNING   4

extern volatile uint8_t ui8_throttle_value;
extern volatile uint8_t ui8_adc_throttle_offset;
extern volatile uint8_t ui8_adc_torque_sensor_offset;
extern volatile uint8_t ui8_adc_battery_current_offset;
extern volatile uint8_t ui8_ebike_app_state;
extern volatile uint8_t ui8_adc_target_battery_current_max;

extern volatile uint16_t ui16_pas_pwm_cycles_ticks;
extern volatile uint8_t ui8_pas_direction;
extern uint8_t ui8_pas_cadence_rpm;

extern volatile uint16_t ui16_wheel_speed_sensor_pwm_cycles_ticks;
extern volatile uint8_t ui8_wheel_speed_sensor_is_disconnected;

void ebike_app_init (void);
void ebike_app_controller (void);
void read_pas_cadence (void);

#endif /* _EBIKE_APP_H_ */
