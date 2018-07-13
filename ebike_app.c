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

uint8_t ui8_adc_battery_max_current = ADC_BATTERY_CURRENT_MAX;
uint8_t ui8_target_battery_max_power_x10 = ADC_BATTERY_CURRENT_MAX;

volatile uint8_t ui8_throttle_value = 0;
volatile uint8_t ui8_throttle_value_filtered = 0;
volatile uint8_t ui8_adc_throttle_offset;
volatile uint8_t ui8_adc_torque_sensor_offset;
volatile uint8_t ui8_adc_battery_current_offset;
volatile uint8_t ui8_ebike_app_state = EBIKE_APP_STATE_MOTOR_STOP;
volatile uint8_t ui8_adc_target_battery_current_max;

volatile uint16_t ui16_pas_pwm_cycles_ticks = (uint16_t) PAS_ABSOLUTE_MIN_CADENCE_PWM_CYCLE_TICKS;
volatile uint8_t ui8_pas_direction = 0;
uint8_t ui8_pas_cadence_rpm = 0;

// wheel speed
volatile uint16_t ui16_wheel_speed_sensor_pwm_cycles_ticks = (uint16_t) WHEEL_SPEED_SENSOR_MAX_PWM_CYCLE_TICKS;
uint8_t ui8_wheel_speed_max = 0;
float f_wheel_speed_x10;
uint16_t ui16_wheel_speed_x10;

volatile struct_lcd_configuration_variables lcd_configuration_variables;

// UART
volatile uint8_t ui8_received_package_flag = 0;
volatile uint8_t ui8_rx_buffer[7];
volatile uint8_t ui8_rx_counter = 0;
volatile uint8_t ui8_tx_buffer[21];
volatile uint8_t ui8_tx_counter = 0;
volatile uint8_t ui8_i;
volatile uint8_t ui8_checksum;
volatile uint8_t ui8_checksum_1st_package;
volatile uint8_t ui8_checksum_2nd_package;
volatile uint8_t ui8_byte_received;
volatile uint8_t ui8_state_machine = 0;
volatile uint8_t ui8_uart_received_first_package = 0;

// function prototypes
void throttle_value_remove_offset (uint8_t *ui8_p_throttle_value);
void torque_sensor_value_remove_offset (uint8_t *ui8_p_torque_sensor_value);
static void ebike_control_motor (void);
void ebike_app_battery_set_current_max (uint8_t ui8_value);
void communications_controller (void);
void uart_send_package (void);
void calc_wheel_speed (void);

void read_pas_cadence (void)
{
  // cadence in RPM =  60 / (ui16_pas_timer2_ticks * PAS_NUMBER_MAGNETS * 0.000064)
  if (ui16_pas_pwm_cycles_ticks >= ((uint16_t) PAS_ABSOLUTE_MIN_CADENCE_PWM_CYCLE_TICKS)) { ui8_pas_cadence_rpm = 0; }
  else
  {
    ui8_pas_cadence_rpm = (uint8_t) (60 / (((float) ui16_pas_pwm_cycles_ticks) * ((float) PAS_NUMBER_MAGNETS) * 0.000064));

    if (ui8_pas_cadence_rpm > ((uint8_t) PAS_MAX_CADENCE_RPM))
    {
      ui8_pas_cadence_rpm = ((uint8_t) PAS_MAX_CADENCE_RPM);
    }
  }
}

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

void torque_sensor_value_remove_offset (uint8_t *ui8_p_torque_sensor_value)
{
  uint8_t ui8_temp;

  // remove throttle signal offset and consider active after a safe threshold
//  ui8_temp = ui8_adc_torque_sensor_offset + ADC_THROTTLE_THRESHOLD;
ui8_temp = 38 + ADC_THROTTLE_THRESHOLD;
  if (*ui8_p_torque_sensor_value > ui8_temp)
  {
    *ui8_p_torque_sensor_value -= ui8_temp;
  }
  else
  {
    *ui8_p_torque_sensor_value = 0;
  }
}

void throttle_read (void)
{
  // read torque sensor signal
  ui8_throttle_value = UI8_ADC_THROTTLE;
//ui8_throttle_value = UI8_ADC_TORQUE_SENSOR;

  throttle_value_remove_offset (&ui8_throttle_value);
//torque_sensor_value_remove_offset (&ui8_throttle_value);

  // map throttle value from 0 up to 255
  ui8_throttle_value = (uint8_t) (map (
      ui8_throttle_value,
      (uint8_t) 0,
      (uint8_t) ADC_THROTTLE_MAX_VALUE,
//      (uint8_t) ADC_TORQUE_SENSOR_MAX_VALUE,
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
  read_pas_cadence ();
  calc_wheel_speed ();
  ebike_control_motor ();
  communications_controller ();
}

void communications_controller (void)
{
#ifndef DEBUG_UART
  if (ui8_received_package_flag)
  {
    // validation of the package data
    // last byte is the checksum
    ui8_checksum = 0;
    for (ui8_i = 0; ui8_i <= 5; ui8_i++)
    {
      ui8_checksum += ui8_rx_buffer[ui8_i];
    }

    // see if checksum is ok...
    if (ui8_rx_buffer [6] == ui8_checksum)
    {
      // assist level
      if (ui8_rx_buffer [1] & (1 << 4)) { lcd_configuration_variables.ui8_assist_level = 0; }
      if (ui8_rx_buffer [1] & (1 << 7)) { lcd_configuration_variables.ui8_assist_level = 1; }
      if (ui8_rx_buffer [1] & (1 << 6)) { lcd_configuration_variables.ui8_assist_level = 2; }
      if (ui8_rx_buffer [1] & (1 << 1)) { lcd_configuration_variables.ui8_assist_level = 3; }
      if (ui8_rx_buffer [1] & (1 << 2)) { lcd_configuration_variables.ui8_assist_level = 4; }
      if (ui8_rx_buffer [1] & (1 << 3)) { lcd_configuration_variables.ui8_assist_level = 5; }
      // head light
      lcd_configuration_variables.ui8_head_light = (ui8_rx_buffer [1] & (1 << 0)) ? 1: 0;
      // walk assist
      lcd_configuration_variables.ui8_walk_assist = (ui8_rx_buffer [1] & (1 << 5)) ? 1: 0;
      // target battery max power
      lcd_configuration_variables.ui8_target_battery_max_power = ui8_rx_buffer [2];
      // wheel perimeter
      lcd_configuration_variables.ui16_wheel_perimeter = (((uint16_t) ui8_rx_buffer [4]) << 8) + ((uint16_t) ui8_rx_buffer [3]);
      // target battery max power
      lcd_configuration_variables.ui8_max_speed = ui8_rx_buffer [5];

      // signal that we processed the full package
      ui8_received_package_flag = 0;

      ui8_uart_received_first_package = 1;
    }

    // enable UART2 receive interrupt as we are now ready to receive a new package
    UART2->CR2 |= (1 << 5);
  }

  uart_send_package ();
#endif
}

void uart_send_package (void)
{
  //send the data to the LCD
  // start up byte
  ui8_tx_buffer[0] = 0x43;

  // wheel speed
  ui8_tx_buffer[6] = (uint8_t) (ui16_wheel_speed_x10 & 0xff);
  ui8_tx_buffer[7] = (uint8_t) (ui16_wheel_speed_x10 >> 8);

  // battery current
  ui8_tx_buffer[10] = (uint8_t) ((float) motor_get_adc_battery_current_filtered () * 0.875);

  // prepare checksum of the 2 packages
  ui8_checksum = 0;
  for (ui8_i = 0; ui8_i <= 7; ui8_i++)
  {
    ui8_checksum += ui8_tx_buffer[ui8_i];
  }
  ui8_tx_buffer[8] = ui8_checksum;

  ui8_checksum = 0;
  for (ui8_i = 9; ui8_i <= 19; ui8_i++)
  {
    ui8_checksum += ui8_tx_buffer[ui8_i];
  }
  ui8_tx_buffer[20] = ui8_checksum;

  // send the full package to UART
  for (ui8_i = 0; ui8_i <= 20; ui8_i++)
  {
    putchar (ui8_tx_buffer[ui8_i]);
  }
}

static void ebike_control_motor (void)
{
#if (EBIKE_THROTTLE_TYPE == EBIKE_THROTTLE_TYPE_THROTTLE_ONLY)
  uint8_t ui8_temp;
  float f_temp;

//  f_temp = (float) (((float) ui8_throttle_value_filtered) * f_get_assist_level ());
f_temp = (float) (((float) ui8_throttle_value_filtered) * 2.0);

  if (f_temp > 255)
    ui8_temp = 255;
  else
    ui8_temp = (uint8_t) f_temp;

  motor_set_pwm_duty_cycle_target (ui8_temp);

#elif (EBIKE_THROTTLE_TYPE == EBIKE_THROTTLE_TYPE_PAS_AND_THROTTLE)

  uint8_t ui8_temp;
  float f_temp;
  uint16_t ui16_battery_voltage_filtered;
  uint16_t ui16_battery_current;

  // calc battery voltage
  ui16_battery_voltage_filtered = (uint16_t) motor_get_adc_battery_voltage_filtered () * ADC10BITS_BATTERY_VOLTAGE_PER_ADC_STEP_X512;
  ui16_battery_voltage_filtered = ui16_battery_voltage_filtered >> 9;

  //calc I
  ui16_battery_current = 0;
  if (ui8_target_battery_max_power_x10 > 0)
  {
    ui16_battery_current = ((uint16_t) ui8_target_battery_max_power_x10) / ui16_battery_voltage_filtered;
  }


  ui8_throttle_value_filtered -= 10;
  f_temp = (float) (((float) ui8_throttle_value_filtered) * 3.0);

  ui8_temp = (uint8_t) (map ((uint32_t) f_temp,
         (uint32_t) 0,
         (uint32_t) 255,
         (uint32_t) 0,
         (uint32_t) 16));
//         (uint32_t) 5));

  //  ebike_app_battery_set_current_max ((uint8_t) ((float) ui16_battery_current * 1.8)); // each 1 unit = 0.625 amps
  ebike_app_battery_set_current_max (ui8_temp);

  if (ui8_throttle_value_filtered > 10)
    motor_set_pwm_duty_cycle_target (255);
  else
    motor_set_pwm_duty_cycle_target (0);

#endif
}

// each 1 unit = 0.625 amps
void ebike_app_battery_set_current_max (uint8_t ui8_value)
{
  ui8_adc_target_battery_current_max = ui8_adc_battery_current_offset + ui8_value;
}

// This is the interrupt that happens when UART2 receives data. We need it to be the fastest possible and so
// we do: receive every byte and assembly as a package, finally, signal that we have a package to process (on main slow loop)
// and disable the interrupt. The interrupt should be enable again on main loop, after the package being processed
void UART2_IRQHandler(void) __interrupt(UART2_IRQHANDLER)
{
  if(UART2_GetFlagStatus(UART2_FLAG_RXNE) == SET)
  {
    UART2->SR &= (uint8_t)~(UART2_FLAG_RXNE); // this may be redundant

    ui8_byte_received = UART2_ReceiveData8 ();

    switch (ui8_state_machine)
    {
      case 0:
      if (ui8_byte_received == 0x59) // see if we get start package byte
      {
        ui8_rx_buffer[ui8_rx_counter] = ui8_byte_received;
        ui8_rx_counter++;
        ui8_state_machine = 1;
      }
      else
      {
        ui8_rx_counter = 0;
        ui8_state_machine = 0;
      }
      break;

      case 1:
      ui8_rx_buffer[ui8_rx_counter] = ui8_byte_received;
      ui8_rx_counter++;

      // see if is the last byte of the package
      if (ui8_rx_counter > 7)
      {
        ui8_rx_counter = 0;
        ui8_state_machine = 0;
        ui8_received_package_flag = 1; // signal that we have a full package to be processed
        UART2->CR2 &= ~(1 << 5); // disable UART2 receive interrupt
      }
      break;

      default:
      break;
    }
  }
}

void calc_wheel_speed (void)
{
  // calc wheel speed in km/h
  if (ui16_wheel_speed_sensor_pwm_cycles_ticks < WHEEL_SPEED_SENSOR_MIN_PWM_CYCLE_TICKS)
  {
    f_wheel_speed_x10 = ((float) PWM_CYCLES_SECOND) / ((float) ui16_wheel_speed_sensor_pwm_cycles_ticks); // rps
    f_wheel_speed_x10 *= lcd_configuration_variables.ui16_wheel_perimeter; // millimeters per second
    f_wheel_speed_x10 *= 0.036; // ((3600 / (1000 * 1000)) * 10) kms per hour * 10
    ui16_wheel_speed_x10 = (uint16_t) f_wheel_speed_x10;
  }
  else
  {
    ui16_wheel_speed_x10 = 0;
  }
}
