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
volatile uint8_t ui8_torque_sensor_value = 0;
volatile uint8_t ui8_torque_sensor_value_filtered = 0;
volatile uint8_t ui8_adc_torque_sensor_offset;
volatile uint8_t ui8_adc_battery_current_offset;
volatile uint8_t ui8_ebike_app_state = EBIKE_APP_STATE_MOTOR_STOP;
volatile uint8_t ui8_adc_target_battery_max_current;
uint8_t ui8_adc_battery_current_max;

volatile uint16_t ui16_pas_pwm_cycles_ticks = (uint16_t) PAS_ABSOLUTE_MIN_CADENCE_PWM_CYCLE_TICKS;
volatile uint8_t ui8_pas_direction = 0;
uint8_t ui8_pas_cadence_rpm = 0;
uint8_t ui8_pedal_human_power = 0;

// wheel speed
volatile uint16_t ui16_wheel_speed_sensor_pwm_cycles_ticks = (uint16_t) WHEEL_SPEED_SENSOR_MAX_PWM_CYCLE_TICKS;
uint8_t ui8_wheel_speed_max = 0;
float f_wheel_speed_x10;
uint16_t ui16_wheel_speed_x10;

volatile struct_lcd_configuration_variables lcd_configuration_variables;

// UART
volatile uint8_t ui8_received_package_flag = 0;
volatile uint8_t ui8_rx_buffer[8];
volatile uint8_t ui8_rx_counter = 0;
volatile uint8_t ui8_tx_buffer[19];
volatile uint8_t ui8_tx_counter = 0;
volatile uint8_t ui8_i;
volatile uint8_t ui8_checksum;
volatile uint8_t ui8_checksum_1st_package;
volatile uint8_t ui8_checksum_2nd_package;
volatile uint8_t ui8_byte_received;
volatile uint8_t ui8_state_machine = 0;
volatile uint8_t ui8_uart_received_first_package = 0;

// function prototypes
static void ebike_control_motor (void);
void ebike_app_set_battery_max_current (uint8_t ui8_value);
void ebike_app_set_target_adc_battery_max_current (uint8_t ui8_value);
void communications_controller (void);
void uart_send_package (void);
void calc_wheel_speed (void);
float f_get_assist_level ();
void throttle_read (void);
void torque_sensor_read (void);

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

void torque_sensor_read (void)
{
  // map value from 0 up to 255
  ui8_torque_sensor_value = (uint8_t) (map (
      UI8_ADC_TORQUE_SENSOR,
      (uint8_t) ADC_TORQUE_SENSOR_MIN_VALUE,
      (uint8_t) ADC_TORQUE_SENSOR_MAX_VALUE,
      (uint8_t) 0,
      (uint8_t) 255));

  ui8_torque_sensor_value_filtered = ui8_torque_sensor_value;
}

void throttle_read (void)
{
  // map value from 0 up to 255
  ui8_throttle_value = (uint8_t) (map (
      UI8_ADC_THROTTLE,
      (uint8_t) ADC_THROTTLE_MIN_VALUE,
      (uint8_t) ADC_THROTTLE_MAX_VALUE,
      (uint8_t) 0,
      (uint8_t) 255));

  ui8_throttle_value_filtered = ui8_throttle_value;
}

void ebike_app_init (void)
{
  ebike_app_set_battery_max_current (ADC_BATTERY_CURRENT_MAX);
}

void ebike_app_controller (void)
{
  throttle_read ();
  torque_sensor_read ();
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
    for (ui8_i = 0; ui8_i <= 6; ui8_i++)
    {
      ui8_checksum += ui8_rx_buffer[ui8_i];
    }

    // see if checksum is ok...
    if (ui8_rx_buffer [7] == ui8_checksum)
    {
      // assist level
      lcd_configuration_variables.ui8_assist_level = ui8_rx_buffer [1] & 0x0f;
      // head light
      lcd_configuration_variables.ui8_head_light = (ui8_rx_buffer [1] & (1 << 4)) ? 1: 0;
      // walk assist
      lcd_configuration_variables.ui8_walk_assist = (ui8_rx_buffer [1] & (1 << 5)) ? 1: 0;
      // battery max current
      ebike_app_set_battery_max_current ( ui8_rx_buffer [2]);
      // target battery max power
      lcd_configuration_variables.ui8_target_battery_max_power_div10 = ui8_rx_buffer [3];
      // wheel perimeter
      lcd_configuration_variables.ui16_wheel_perimeter = (((uint16_t) ui8_rx_buffer [5]) << 8) + ((uint16_t) ui8_rx_buffer [4]);
      // wheel max speed
      lcd_configuration_variables.ui8_wheel_max_speed = ui8_rx_buffer [6];

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
  uint16_t ui16_temp;
  uint16_t ui16_battery_volts_x256;
  uint16_t ui8_temp;

  //send the data to the LCD
  // start up byte
  ui8_tx_buffer[0] = 0x43;

  ui16_temp = motor_get_adc_battery_voltage_filtered_10b ();
  // battery voltage
  ui8_tx_buffer[1] = (ui16_temp & 0xff);
  ui8_tx_buffer[2] = ((uint8_t) (ui16_temp >> 4)) & 0x30;

  // calc battery pack state of charge (SOC)
  ui16_battery_volts_x256 = ((uint16_t) ui16_temp) * ((uint16_t) ADC10BITS_BATTERY_VOLTAGE_PER_ADC_STEP_X256);
  if (ui16_battery_volts_x256 > ((uint16_t) BATTERY_PACK_VOLTS_80_X256)) { ui8_temp = 5; } // 4 bars | full
  else if (ui16_battery_volts_x256 > ((uint16_t) BATTERY_PACK_VOLTS_60_X256)) { ui8_temp = 4; } // 3 bars
  else if (ui16_battery_volts_x256 > ((uint16_t) BATTERY_PACK_VOLTS_40_X256)) { ui8_temp = 3; } // 2 bars
  else if (ui16_battery_volts_x256 > ((uint16_t) BATTERY_PACK_VOLTS_20_X256  )) { ui8_temp = 2; } // 1 bar
  else if (ui16_battery_volts_x256 > ((uint16_t) BATTERY_PACK_VOLTS_10_X256  )) { ui8_temp = 1; } // empty
  else { ui8_temp = 0; } // flashing

  // battery state of charge (SOC)
  ui8_tx_buffer[2] |= ui8_temp & 0x0f;

  // battery current x5
  ui8_tx_buffer[3] = (uint8_t) ((float) motor_get_adc_battery_current_filtered_10b () * 0.826);

  // wheel speed
  ui8_tx_buffer[4] = (uint8_t) (ui16_wheel_speed_x10 & 0xff);
  ui8_tx_buffer[5] = (uint8_t) (ui16_wheel_speed_x10 >> 8);

  // brake state
  if (motor_controller_state_is_set (MOTOR_CONTROLLER_STATE_BRAKE))
  {
    ui8_tx_buffer[6] |= 1;
  }
  else
  {
    ui8_tx_buffer[6] &= ~1;
  }

  // error states
  ui8_tx_buffer[7] = 0;

  // ADC throttle
  ui8_tx_buffer[8] = UI8_ADC_THROTTLE;
  // throttle value with offset removed and mapped to 255
  ui8_tx_buffer[9] = ui8_throttle_value_filtered;
  // ADC torque_sensor
  ui8_tx_buffer[10] = UI8_ADC_TORQUE_SENSOR;
  // throttle value with offset removed and mapped to 255
  ui8_tx_buffer[11] = ui8_torque_sensor_value_filtered;
  // PAS cadence
  ui8_tx_buffer[12] = ui8_pas_cadence_rpm;
  // pedal human power mapped to 255
  ui8_tx_buffer[13] = ui8_pedal_human_power;
  // PWM duty_cycle
  ui8_tx_buffer[14] = ui8_duty_cycle;
  // motor speed in ERPS
  ui16_temp = ui16_motor_get_motor_speed_erps(),
  ui8_tx_buffer[15] = (uint8_t) (ui16_temp & 0xff);
  ui8_tx_buffer[16] = (uint8_t) (ui16_temp >> 8) & 0xff;
  // FOC angle
  ui8_tx_buffer[17] = ui8_foc_angle;

  // prepare checksum of the package
  ui8_checksum = 0;
  for (ui8_i = 0; ui8_i <= 17; ui8_i++)
  {
    ui8_checksum += ui8_tx_buffer[ui8_i];
  }
  ui8_tx_buffer[18] = ui8_checksum;

  // send the full package to UART
  for (ui8_i = 0; ui8_i <= 18; ui8_i++)
  {
    putchar (ui8_tx_buffer[ui8_i]);
  }
}

static void ebike_control_motor (void)
{
#if (EBIKE_THROTTLE_TYPE == EBIKE_THROTTLE_TYPE_THROTTLE_ONLY)

  motor_set_pwm_duty_cycle_target (ui8_torque_sensor_throttle_value_filtered);

#elif (EBIKE_THROTTLE_TYPE == EBIKE_THROTTLE_TYPE_PAS_AND_THROTTLE)

  uint8_t ui16_temp;
  float f_torque_sensor;
  uint8_t ui8_torque_sensor;
  uint8_t ui8_throttle;
  uint16_t ui16_battery_voltage_filtered;
  uint16_t ui16_max_battery_current;
  uint8_t ui8_battery_target_current;

  // scale torque sensor signal using assist level
  f_torque_sensor = ((float) ui8_torque_sensor_value_filtered) * f_get_assist_level ();
  // limit to max
  if (f_torque_sensor > 255)
    ui8_torque_sensor = 255;
  else
    ui8_torque_sensor = (uint8_t) f_torque_sensor;

  // cadence percentage (in x256)
//  ui16_temp = (((uint16_t) ui8_pas_cadence_rpm) << 8) / ((uint16_t) PAS_MAX_CADENCE_RPM);

  // human power: pedal torque * pedal cadence
  // do not apply human power with lower cadence
//  if (ui8_pas_cadence_rpm > 25)
//  {
//    ui8_pedal_human_power = ((((uint16_t) ui8_torque_sensor) * ui16_temp) >> 8);
//  }
//  else
//  {
    ui8_pedal_human_power = ui8_torque_sensor;
//  }

  // use the value that is the max of both signals: throttle or torque sensor (human power)
  ui8_throttle = ui8_max (ui8_throttle_value_filtered, ui8_pedal_human_power);

  // map previous value to battery current
  ui8_battery_target_current = (uint8_t) (map ((uint32_t) ui8_throttle,
         (uint32_t) 0,
         (uint32_t) 255,
         (uint32_t) 0,
         (uint32_t) ADC_BATTERY_CURRENT_MAX));

  // now let's calc max battery current based on the target max power
  // calc battery voltage
  ui16_battery_voltage_filtered = (uint16_t) motor_get_adc_battery_voltage_filtered_10b () * ADC10BITS_BATTERY_VOLTAGE_PER_ADC_STEP_X512;
  ui16_battery_voltage_filtered = ui16_battery_voltage_filtered >> 9;

  // calc max battery current
  ui16_max_battery_current = 0;
  if (lcd_configuration_variables.ui8_target_battery_max_power_div10 > 0)
  {
    // 1.6 = 1 / 0.625(each adc step for current)
    // 1.6 * 10 = 16
    ui16_max_battery_current = ((uint16_t) lcd_configuration_variables.ui8_target_battery_max_power_div10 * 16) / ui16_battery_voltage_filtered;
  }

  // now let's limit the target battery current to battery max current (use min value of both)
  ui8_battery_target_current = ui8_min (ui8_battery_target_current, ui16_max_battery_current);
  // finally set the target battery current to the current controller
  ebike_app_set_target_adc_battery_max_current (ui8_battery_target_current);

  // set the target duty_cycle to max, as the battery current controller will manage it
  // if battery_target_current == 0, put duty_cycle at 0
  if (ui8_battery_target_current > 0)
    motor_set_pwm_duty_cycle_target (255);
  else
    motor_set_pwm_duty_cycle_target (0);
#endif
}

// each 1 unit = 0.625 amps
void ebike_app_set_target_adc_battery_max_current (uint8_t ui8_value)
{
  // limit max number of amps
  if (ui8_value > ui8_adc_battery_current_max)
    ui8_value = ui8_adc_battery_current_max;

  ui8_adc_target_battery_max_current = ui8_adc_battery_current_offset + ui8_value;
}

// in amps
void ebike_app_set_battery_max_current (uint8_t ui8_value)
{
  // each 1 unit = 0.625 amps (0.625 * 256 = 160)
  ui8_adc_battery_current_max = ((((uint16_t) ui8_value) << 8) / 160);

  if (ui8_adc_battery_current_max > ADC_BATTERY_CURRENT_MAX)
    ui8_adc_battery_current_max = ADC_BATTERY_CURRENT_MAX;
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

float f_get_assist_level ()
{
  float f_temp;

  switch (lcd_configuration_variables.ui8_assist_level)
  {
    case 0:
    f_temp = ASSIST_LEVEL_0;
    break;

    case 1:
    f_temp = ASSIST_LEVEL_1;
    break;

    case 2:
    f_temp = ASSIST_LEVEL_2;
    break;

    case 3:
    f_temp = ASSIST_LEVEL_3;
    break;

    case 4:
    f_temp = ASSIST_LEVEL_4;
    break;

    case 5:
    f_temp = ASSIST_LEVEL_5;
    break;

    default:
    f_temp = ASSIST_LEVEL_5;
    break;
  }

  return f_temp;
}
