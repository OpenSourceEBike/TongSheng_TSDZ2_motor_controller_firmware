/*
 * TongSheng TSDZ2 motor controller firmware/
 *
 * Copyright (C) Casainho, 2018.
 *
 * Released under the GPL License, Version 3
 */

#include "ebike_app.h"

#include <stdint.h>
#include <stdio.h>
#include "stm8s.h"
#include "stm8s_gpio.h"
#include "main.h"
#include "interrupts.h"
#include "adc.h"
#include "utils.h"
#include "motor.h"
#include "pwm.h"
#include "uart.h"
#include "brake.h"
#include "eeprom.h"
#include "config.h"

volatile uint8_t ui8_throttle_value = 0;
volatile uint8_t ui8_throttle_value_filtered = 0;
volatile uint8_t ui8_adc_throttle_offset;
volatile uint8_t ui8_adc_torque_sensor_offset;
volatile uint8_t ui8_adc_battery_current_offset;
volatile uint8_t ui8_ebike_app_state = EBIKE_APP_STATE_MOTOR_STOP;
volatile uint8_t ui8_adc_target_battery_current_max;

// function prototypes
void throttle_value_remove_offset (uint8_t *ui8_p_throttle_value);
static void ebike_control_motor (void);
void ebike_app_battery_set_current_max (uint8_t ui8_value);

void throttle_value_remove_offset (uint8_t *ui8_p_throttle_value)
{
  uint8_t ui8_temp;

  // remove throttle signal offset and consider active after a safe threshold
  ui8_temp = ui8_adc_throttle_offset + ADC_THROTTLE_THRESHOLD;
  if (*ui8_p_throttle_value > ui8_temp)
  {
    *ui8_p_throttle_value -= ui8_temp;
  }
  else
  {
    *ui8_p_throttle_value = 0;
  }
}

void throttle_read (void)
{
  // read torque sensor signal
  ui8_throttle_value = UI8_ADC_THROTTLE;

  throttle_value_remove_offset (&ui8_throttle_value);

  // map throttle value from 0 up to 255
  ui8_throttle_value = (uint8_t) (map (
      ui8_throttle_value,
      (uint8_t) 0,
      (uint8_t) ADC_THROTTLE_MAX_VALUE,
      (uint8_t) 0,
      (uint8_t) 255));

  ui8_throttle_value_filtered = ui8_throttle_value;
}

void ebike_app_init (void)
{
  ebike_app_battery_set_current_max (ADC_BATTERY_CURRENT_MAX);
}

void ebike_app_controller (void)
{
  throttle_read ();

  ebike_control_motor ();
}

static void ebike_control_motor (void)
{
#if (EBIKE_THROTTLE_TYPE == EBIKE_THROTTLE_TYPE_THROTTLE_ONLY)
  uint8_t ui8_temp;
  float f_temp;

//  f_temp = (float) (((float) ui8_throttle_value_filtered) * f_get_assist_level ());
f_temp = (float) (((float) ui8_throttle_value_filtered) * 1.0);

  ui8_temp = (uint8_t) (map ((uint32_t) f_temp,
         (uint32_t) 0,
         (uint32_t) ADC_THROTTLE_MAX_VALUE,
         (uint32_t) 0,
         (uint32_t) 255));

  motor_set_pwm_duty_cycle_target (ui8_temp);
#endif
}

void ebike_app_battery_set_current_max (uint8_t ui8_value)
{
  ui8_adc_target_battery_current_max = ui8_adc_battery_current_offset + ui8_value;
}
