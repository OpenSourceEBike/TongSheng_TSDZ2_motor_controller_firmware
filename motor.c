/*
 * TongSheng TSDZ2 motor controller firmware/
 *
 * Copyright (C) Casainho, 2018.
 *
 * Released under the GPL License, Version 3
 */

#include "motor.h"
#include <stdint.h>
#include <stdio.h>
#include "interrupts.h"
#include "stm8s_gpio.h"
#include "stm8s_tim1.h"
#include "motor.h"
#include "ebike_app.h"
#include "pins.h"
#include "pwm.h"
#include "config.h"
#include "adc.h"
#include "utils.h"
#include "uart.h"
#include "adc.h"
#include "watchdog.h"
#include "math.h"

#define SVM_TABLE_LEN 256
#define SIN_TABLE_LEN 60

uint8_t ui8_svm_table [SVM_TABLE_LEN] =
{
    239 ,
    241 ,
    242 ,
    243 ,
    245 ,
    246 ,
    247 ,
    248 ,
    249 ,
    250 ,
    251 ,
    251 ,
    252 ,
    253 ,
    253 ,
    254 ,
    254 ,
    254 ,
    255 ,
    255 ,
    255 ,
    255 ,
    255 ,
    255 ,
    254 ,
    254 ,
    254 ,
    253 ,
    253 ,
    252 ,
    251 ,
    250 ,
    250 ,
    249 ,
    248 ,
    247 ,
    245 ,
    244 ,
    243 ,
    242 ,
    240 ,
    239 ,
    236 ,
    231 ,
    227 ,
    222 ,
    217 ,
    212 ,
    207 ,
    202 ,
    197 ,
    191 ,
    186 ,
    181 ,
    176 ,
    170 ,
    165 ,
    160 ,
    154 ,
    149 ,
    144 ,
    138 ,
    133 ,
    127 ,
    122 ,
    116 ,
    111 ,
    106 ,
    100 ,
    95  ,
    89  ,
    84  ,
    79  ,
    74  ,
    68  ,
    63  ,
    58  ,
    53  ,
    48  ,
    43  ,
    38  ,
    33  ,
    28  ,
    23  ,
    18  ,
    16  ,
    14  ,
    13  ,
    12  ,
    10  ,
    9 ,
    8 ,
    7 ,
    6 ,
    5 ,
    4 ,
    3 ,
    3 ,
    2 ,
    1 ,
    1 ,
    1 ,
    0 ,
    0 ,
    0 ,
    0 ,
    0 ,
    0 ,
    0 ,
    0 ,
    0 ,
    1 ,
    1 ,
    2 ,
    2 ,
    3 ,
    4 ,
    5 ,
    6 ,
    6 ,
    8 ,
    9 ,
    10  ,
    11  ,
    12  ,
    14  ,
    15  ,
    17  ,
    15  ,
    14  ,
    12  ,
    11  ,
    10  ,
    9 ,
    8 ,
    6 ,
    6 ,
    5 ,
    4 ,
    3 ,
    2 ,
    2 ,
    1 ,
    1 ,
    0 ,
    0 ,
    0 ,
    0 ,
    0 ,
    0 ,
    0 ,
    0 ,
    0 ,
    1 ,
    1 ,
    1 ,
    2 ,
    3 ,
    3 ,
    4 ,
    5 ,
    6 ,
    7 ,
    8 ,
    9 ,
    10  ,
    12  ,
    13  ,
    14  ,
    16  ,
    18  ,
    23  ,
    28  ,
    33  ,
    38  ,
    43  ,
    48  ,
    53  ,
    58  ,
    63  ,
    68  ,
    74  ,
    79  ,
    84  ,
    89  ,
    95  ,
    100 ,
    106 ,
    111 ,
    116 ,
    122 ,
    127 ,
    133 ,
    138 ,
    144 ,
    149 ,
    154 ,
    160 ,
    165 ,
    170 ,
    176 ,
    181 ,
    186 ,
    191 ,
    197 ,
    202 ,
    207 ,
    212 ,
    217 ,
    222 ,
    227 ,
    231 ,
    236 ,
    239 ,
    240 ,
    242 ,
    243 ,
    244 ,
    245 ,
    247 ,
    248 ,
    249 ,
    250 ,
    250 ,
    251 ,
    252 ,
    253 ,
    253 ,
    254 ,
    254 ,
    254 ,
    255 ,
    255 ,
    255 ,
    255 ,
    255 ,
    255 ,
    254 ,
    254 ,
    254 ,
    253 ,
    253 ,
    252 ,
    251 ,
    251 ,
    250 ,
    249 ,
    248 ,
    247 ,
    246 ,
    245 ,
    243 ,
    242 ,
    241 ,
    239 ,
    238 ,
};

uint8_t ui8_sin_table [SVM_TABLE_LEN] =
{
    0 ,
    3 ,
    6 ,
    9 ,
    12  ,
    16  ,
    19  ,
    22  ,
    25  ,
    28  ,
    31  ,
    34  ,
    37  ,
    40  ,
    43  ,
    46  ,
    49  ,
    52  ,
    54  ,
    57  ,
    60  ,
    63  ,
    66  ,
    68  ,
    71  ,
    73  ,
    76  ,
    78  ,
    81  ,
    83  ,
    86  ,
    88  ,
    90  ,
    92  ,
    95  ,
    97  ,
    99  ,
    101 ,
    102 ,
    104 ,
    106 ,
    108 ,
    109 ,
    111 ,
    113 ,
    114 ,
    115 ,
    117 ,
    118 ,
    119 ,
    120 ,
    121 ,
    122 ,
    123 ,
    124 ,
    125 ,
    125 ,
    126 ,
    126 ,
    127
};

uint16_t ui16_PWM_cycles_counter = 1;
uint16_t ui16_PWM_cycles_counter_6 = 1;
uint16_t ui16_PWM_cycles_counter_total = 0xffff;

volatile uint16_t ui16_motor_speed_erps = 0;
uint8_t ui8_sinewave_table_index = 0;
uint8_t ui8_motor_rotor_absolute_angle;
uint8_t ui8_motor_rotor_angle;

volatile uint8_t ui8_foc_angle_correction = 0;
uint8_t ui8_interpolation_angle = 0;

uint8_t ui8_motor_commutation_type = BLOCK_COMMUTATION;
volatile uint8_t ui8_motor_controller_state = MOTOR_CONTROLLER_STATE_OK;

uint8_t ui8_hall_sensors_state = 0;
uint8_t ui8_hall_sensors_state_last = 0;

uint8_t ui8_half_erps_flag = 0;

volatile uint8_t ui8_duty_cycle = 0;
uint8_t ui8_duty_cycle_target;
uint16_t ui16_duty_cycle_ramp_up_inverse_step;
uint16_t ui16_duty_cycle_ramp_down_inverse_step;
uint16_t ui16_counter_duty_cycle_ramp_up = 0;
uint16_t ui16_counter_duty_cycle_ramp_down = 0;
uint8_t ui8_phase_a_voltage;
uint8_t ui8_phase_b_voltage;
uint8_t ui8_phase_c_voltage;
uint16_t ui16_value;

uint8_t ui8_first_time_run_flag = 1;

uint16_t ui16_adc_battery_voltage_accumulated = 0;
uint16_t ui16_adc_battery_voltage_filtered;

uint16_t ui16_adc_battery_current_accumulated = 0;
uint16_t ui16_adc_battery_current_filtered;

uint16_t ui16_foc_angle_accumulated = 0;
uint8_t ui8_foc_angle_filtered;

void read_battery_voltage (void);
void read_battery_current (void);
uint8_t asin_table (uint8_t ui8_inverted_angle_x128);

void motor_controller (void)
{
  uint8_t ui8_foc_angle;
  uint16_t ui16_temp;
  uint32_t ui32_temp;
  uint16_t ui16_e_phase_voltage;
  uint32_t ui32_i_phase_current_x2;
  uint32_t ui32_l_x1048576;
  uint32_t ui32_w_angular_velocity_x16;
  uint16_t ui16_iwl_128;

  // reads battery voltage and current
  read_battery_voltage ();
  read_battery_current ();

  //************************************************************************************
  // FOC implementation by calculating the angle between phase current and rotor magnetic flux (BEMF)
  // 1. phase voltage is calculate
  // 2. I*w*L is calculated, where I is the phase current. L was a measured value for 48V motor.
  // 3. inverse sin is calculated of (I*w*L) / phase voltage, were we obtain the angle
  // 4. previous calculated angle is applied to phase voltage vector angle and so the
  // angle between phase current and rotor magnetic flux (BEMF) is kept at 0 (max torque per amp)

  // calc E phase voltage
  ui16_temp = ui16_adc_battery_voltage_filtered * ADC_BATTERY_VOLTAGE_PER_ADC_STEP_X512;
  ui16_temp = (ui16_temp >> 8) * ui8_duty_cycle;
  ui16_e_phase_voltage = ui16_temp >> 9;

  // calc I phase current
  if (ui8_duty_cycle > 10)
  {
    ui16_temp = ui16_adc_battery_current_filtered * ADC_BATTERY_CURRENT_PER_ADC_STEP_X512;
    ui32_i_phase_current_x2 = ui16_temp / ui8_duty_cycle;
  }
  else
  {
    ui32_i_phase_current_x2 = 0;
  }

  // calc W angular velocity: erps * 6.3
  ui32_w_angular_velocity_x16 = ui16_motor_speed_erps * 101;

  // 36V motor: L = 76uH
  // 48V motor: L = 135uH
  ui32_l_x1048576 = 142; // 1048576 = 2^20

  // calc IwL
  ui32_temp = ui32_i_phase_current_x2 * ui32_l_x1048576;
  ui32_temp *= ui32_w_angular_velocity_x16;
  ui16_iwl_128 = ui32_temp >> 18;

  // calc FOC angle
  ui8_foc_angle = asin_table (ui16_iwl_128 / ui16_e_phase_voltage);

  // low pass filter FOC angle
  ui16_foc_angle_accumulated -= ui16_foc_angle_accumulated >> 4;
  ui16_foc_angle_accumulated += ui8_foc_angle;
  ui8_foc_angle_filtered = ui16_foc_angle_accumulated >> 4;

  // apply FOC angle
  ui8_foc_angle_correction = -ui8_foc_angle_filtered;
}


// Measures did with a 24V Q85 328 RPM motor, rotating motor backwards by hand:
// Hall sensor A positivie to negative transition | BEMF phase B at max value / top of sinewave
// Hall sensor B positivie to negative transition | BEMF phase A at max value / top of sinewave
// Hall sensor C positive to negative transition | BEMF phase C at max value / top of sinewave

// runs every 64us (PWM frequency)
void TIM1_CAP_COM_IRQHandler(void) __interrupt(TIM1_CAP_COM_IRQHANDLER)
{
  uint8_t ui8_temp;

  /****************************************************************************/
  // trigger ADC conversion of all channels (scan conversion, buffered)
  ADC1->CSR = 0x07; // clear EOC flag first (selected also channel 7)
  ADC1->CR1 |= ADC1_CR1_ADON; // start ADC1 conversion
  /****************************************************************************/

  /****************************************************************************/
  // read hall sensor signals and:
  // - find the motor rotor absolute angle
  // - read FOC Id current and calc FOC (adjust ui8_angle_correction)
  // - calc motor speed in erps (ui16_motor_speed_erps)

  // read hall sensors signal pins and mask other pins
  // hall sensors sequence with motor forward rotation: 4, 6, 2, 3, 1, 5
  ui8_hall_sensors_state = ((HALL_SENSOR_A__PORT->IDR & HALL_SENSOR_A__PIN) >> 5) |
  ((HALL_SENSOR_B__PORT->IDR & HALL_SENSOR_B__PIN) >> 1) |
  ((HALL_SENSOR_C__PORT->IDR & HALL_SENSOR_C__PIN) >> 3);
  // make sure we run next code only when there is a change on the hall sensors signal
  if (ui8_hall_sensors_state != ui8_hall_sensors_state_last)
  {
    ui8_hall_sensors_state_last = ui8_hall_sensors_state;

    switch (ui8_hall_sensors_state)
    {
      case 3:
      ui8_motor_rotor_absolute_angle = (uint8_t) MOTOR_ROTOR_ANGLE_150;
      break;

      case 1:
      if (ui8_half_erps_flag == 1)
      {
        ui8_half_erps_flag = 0;
        ui16_PWM_cycles_counter_total = ui16_PWM_cycles_counter;
        ui16_PWM_cycles_counter = 1;

        // this division takes 4.4us and without the cast (uint16_t) PWM_CYCLES_SECOND, would take 111us!! Verified on 2017.11.20
        // avoid division by 0
        if (ui16_PWM_cycles_counter_total > 0) { ui16_motor_speed_erps = ((uint16_t) PWM_CYCLES_SECOND) / ui16_PWM_cycles_counter_total; }
        else { ui16_motor_speed_erps = ((uint16_t) PWM_CYCLES_SECOND); }

        // update motor commutation state based on motor speed
        if (ui16_motor_speed_erps > MOTOR_ROTOR_ERPS_START_INTERPOLATION_60_DEGREES)
        {
          if (ui8_motor_commutation_type == BLOCK_COMMUTATION)
          {
            ui8_motor_commutation_type = SINEWAVE_INTERPOLATION_60_DEGREES;
            ui8_ebike_app_state = EBIKE_APP_STATE_MOTOR_RUNNING;
          }
        }
        else
        {
          if (ui8_motor_commutation_type == SINEWAVE_INTERPOLATION_60_DEGREES)
          {
            ui8_motor_commutation_type = BLOCK_COMMUTATION;
            ui8_foc_angle_correction = 0;
          }
        }
      }

      ui8_motor_rotor_absolute_angle = (uint8_t) MOTOR_ROTOR_ANGLE_210;
      break;

      case 5:
      ui8_motor_rotor_absolute_angle = (uint8_t) MOTOR_ROTOR_ANGLE_270;
      break;

      case 4:
      ui8_motor_rotor_absolute_angle = (uint8_t) MOTOR_ROTOR_ANGLE_330;
      break;

      case 6:
      ui8_half_erps_flag = 1;

      ui8_motor_rotor_absolute_angle = (uint8_t) MOTOR_ROTOR_ANGLE_30;
      break;

      // BEMF is always 90 degrees advanced over motor rotor position degree zero
      // and here (hall sensor C blue wire, signal transition from positive to negative),
      // phase B BEMF is at max value (measured on osciloscope by rotating the motor)
      case 2:
      ui8_motor_rotor_absolute_angle = (uint8_t) MOTOR_ROTOR_ANGLE_90;
      break;

      default:
      return;
      break;
    }

    ui16_PWM_cycles_counter_6 = 1;
  }
  /****************************************************************************/

  /****************************************************************************/
  // count number of fast loops / PWM cycles and reset some states when motor is near zero speed
  if (ui16_PWM_cycles_counter < ((uint16_t) PWM_CYCLES_COUNTER_MAX))
  {
    ui16_PWM_cycles_counter++;
    ui16_PWM_cycles_counter_6++;
  }
  else // happens when motor is stopped or near zero speed
  {
    ui16_PWM_cycles_counter = 1; // don't put to 0 to avoid 0 divisions
    ui16_PWM_cycles_counter_6 = 1;
    ui8_half_erps_flag = 0;
    ui16_motor_speed_erps = 0;
    ui16_PWM_cycles_counter_total = 0xffff;
    ui8_foc_angle_correction = 0;
    ui8_motor_commutation_type = BLOCK_COMMUTATION;
    ui8_hall_sensors_state_last = 0; // this way we force execution of hall sensors code next time
//    ebike_app_cruise_control_stop ();
//    if (ui8_ebike_app_state == EBIKE_APP_STATE_MOTOR_RUNNING) { ui8_ebike_app_state = EBIKE_APP_STATE_MOTOR_STOP; }
  }
  /****************************************************************************/

  /****************************************************************************/
  // - calc interpolation angle and sinewave table index
#define DO_INTERPOLATION 1 // may be usefull to disable interpolation when debugging
#if DO_INTERPOLATION == 1
  // calculate the interpolation angle (and it doesn't work when motor starts and at very low speeds)
  if (ui8_motor_commutation_type == SINEWAVE_INTERPOLATION_60_DEGREES)
  {
    // division by 0: ui16_PWM_cycles_counter_total should never be 0
    // TODO: verifiy if (ui16_PWM_cycles_counter_6 << 8) do not overflow
    ui8_interpolation_angle = (ui16_PWM_cycles_counter_6 << 8) / ui16_PWM_cycles_counter_total; // this operations take 4.4us
    ui8_motor_rotor_angle = ui8_motor_rotor_absolute_angle + ui8_interpolation_angle;
    ui8_sinewave_table_index = ui8_motor_rotor_angle + ui8_foc_angle_correction;
  }
  else
#endif
  {
    ui8_sinewave_table_index = ui8_motor_rotor_absolute_angle + ui8_foc_angle_correction;
  }

  // we need to put phase voltage 90 degrees ahead of rotor position, to get current 90 degrees ahead and have max torque per amp
  ui8_sinewave_table_index -= 63;
  /****************************************************************************/

  /****************************************************************************/
  // PWM duty_cycle controller:
  // - limit battery undervoltage
  // - limit battery max current
  // - limit motor max ERPS
  // - ramp up/down PWM duty_cycle value

//  if ((UI8_ADC_BATTERY_CURRENT > ui8_adc_target_battery_current_max) || // battery max current, reduce duty_cycle
//      (ui16_motor_speed_erps > MOTOR_OVER_SPEED_ERPS) || // motor speed over max ERPS, reduce duty_cycle
//      (UI8_ADC_BATTERY_VOLTAGE < ((uint8_t) ADC_BATTERY_VOLTAGE_MIN))) // battery voltage under min voltage, reduce duty_cycle
//  {
//    if (ui8_duty_cycle > 0)
//    {
//      ui8_duty_cycle--;
//    }
//  }
//  else // nothing to limit, so, adjust duty_cycle to duty_cycle_target, including ramping
//  {
    if (ui8_duty_cycle_target > ui8_duty_cycle)
    {
      if (ui16_counter_duty_cycle_ramp_up++ >= ui16_duty_cycle_ramp_up_inverse_step)
      {
        ui16_counter_duty_cycle_ramp_up = 0;
        ui8_duty_cycle++;
      }
    }
    else if (ui8_duty_cycle_target < ui8_duty_cycle)
    {
      if (ui16_counter_duty_cycle_ramp_down++ >= ui16_duty_cycle_ramp_down_inverse_step)
      {
        ui16_counter_duty_cycle_ramp_down = 0;
        ui8_duty_cycle--;
      }
    }
//  }
  /****************************************************************************/

  /****************************************************************************/
  // calc final PWM duty_cycle values to be applied to TIMER1

  // scale and apply PWM duty_cycle for the 3 phases
  // phase A is advanced 240 degrees over phase B
  ui8_temp = ui8_svm_table [(uint8_t) (ui8_sinewave_table_index + 171 /* 240º */)];
  if (ui8_temp > MIDDLE_PWM_DUTY_CYCLE_MAX)
  {
    ui16_value = ((uint16_t) (ui8_temp - MIDDLE_PWM_DUTY_CYCLE_MAX)) * ui8_duty_cycle;
    ui8_temp = (uint8_t) (ui16_value >> 8);
    ui8_phase_a_voltage = MIDDLE_PWM_DUTY_CYCLE_MAX + ui8_temp;
  }
  else
  {
    ui16_value = ((uint16_t) (MIDDLE_PWM_DUTY_CYCLE_MAX - ui8_temp)) * ui8_duty_cycle;
    ui8_temp = (uint8_t) (ui16_value >> 8);
    ui8_phase_a_voltage = MIDDLE_PWM_DUTY_CYCLE_MAX - ui8_temp;
  }

  // phase B as reference phase
  ui8_temp = ui8_svm_table [ui8_sinewave_table_index];
  if (ui8_temp > MIDDLE_PWM_DUTY_CYCLE_MAX)
  {
    ui16_value = ((uint16_t) (ui8_temp - MIDDLE_PWM_DUTY_CYCLE_MAX)) * ui8_duty_cycle;
    ui8_temp = (uint8_t) (ui16_value >> 8);
    ui8_phase_b_voltage = MIDDLE_PWM_DUTY_CYCLE_MAX + ui8_temp;
  }
  else
  {
    ui16_value = ((uint16_t) (MIDDLE_PWM_DUTY_CYCLE_MAX - ui8_temp)) * ui8_duty_cycle;
    ui8_temp = (uint8_t) (ui16_value >> 8);
    ui8_phase_b_voltage = MIDDLE_PWM_DUTY_CYCLE_MAX - ui8_temp;
  }

  // phase C is advanced 120 degrees over phase B
  ui8_temp = ui8_svm_table [(uint8_t) (ui8_sinewave_table_index + 85 /* 120º */)];
  if (ui8_temp > MIDDLE_PWM_DUTY_CYCLE_MAX)
  {
    ui16_value = ((uint16_t) (ui8_temp - MIDDLE_PWM_DUTY_CYCLE_MAX)) * ui8_duty_cycle;
    ui8_temp = (uint8_t) (ui16_value >> 8);
    ui8_phase_c_voltage = MIDDLE_PWM_DUTY_CYCLE_MAX + ui8_temp;
  }
  else
  {
    ui16_value = ((uint16_t) (MIDDLE_PWM_DUTY_CYCLE_MAX - ui8_temp)) * ui8_duty_cycle;
    ui8_temp = (uint8_t) (ui16_value >> 8);
    ui8_phase_c_voltage = MIDDLE_PWM_DUTY_CYCLE_MAX - ui8_temp;
  }

  // set final duty_cycle value
  // phase B
  TIM1->CCR3H = (uint8_t) (ui8_phase_b_voltage >> 7);
  TIM1->CCR3L = (uint8_t) (ui8_phase_b_voltage << 1);
  // phase C
  TIM1->CCR2H = (uint8_t) (ui8_phase_c_voltage >> 7);
  TIM1->CCR2L = (uint8_t) (ui8_phase_c_voltage << 1);
  // phase A
  TIM1->CCR1H = (uint8_t) (ui8_phase_a_voltage >> 7);
  TIM1->CCR1L = (uint8_t) (ui8_phase_a_voltage << 1);

  // enable PWM signals only when MOTOR_CONTROLLER_STATE_OK
  if (ui8_motor_controller_state == MOTOR_CONTROLLER_STATE_OK)
  {
    TIM1->BKR |= TIM1_BKR_MOE;
  }
  /****************************************************************************/

  /****************************************************************************/
  // reload watchdog timer, every PWM cycle to avoid automatic reset of the microcontroller
  if (ui8_first_time_run_flag)
  { // from the init of watchdog up to first reset on PWM cycle interrupt,
    // it can take up to 250ms and so we need to init here inside the PWM cycle
    ui8_first_time_run_flag = 0;
    watchdog_init ();
  }
  else
  {
    IWDG->KR = IWDG_KEY_REFRESH; // reload watch dog timer counter
  }
  /****************************************************************************/

  /****************************************************************************/
  // clears the TIM1 interrupt TIM1_IT_UPDATE pending bit
  TIM1->SR1 = (uint8_t)(~(uint8_t)TIM1_IT_CC4);
  /****************************************************************************/
}

void motor_disable_PWM (void)
{
  TIM1_CtrlPWMOutputs(DISABLE);
}

void motor_enable_PWM (void)
{
  TIM1_CtrlPWMOutputs(ENABLE);
}

void motor_controller_set_state (uint8_t ui8_state)
{
  ui8_motor_controller_state |= ui8_state;
}

void motor_controller_reset_state (uint8_t ui8_state)
{
  ui8_motor_controller_state &= ~ui8_state;
}

uint8_t motor_controller_state_is_set (uint8_t ui8_state)
{
  return ui8_motor_controller_state & ui8_state;
}

void hall_sensor_init (void)
{
  GPIO_Init (HALL_SENSOR_A__PORT, (GPIO_Pin_TypeDef) HALL_SENSOR_A__PIN, GPIO_MODE_IN_FL_NO_IT);
  GPIO_Init (HALL_SENSOR_B__PORT, (GPIO_Pin_TypeDef) HALL_SENSOR_B__PIN, GPIO_MODE_IN_FL_NO_IT);
  GPIO_Init (HALL_SENSOR_C__PORT, (GPIO_Pin_TypeDef) HALL_SENSOR_C__PIN, GPIO_MODE_IN_FL_NO_IT);
}

void motor_init (void)
{
  motor_set_pwm_duty_cycle_ramp_up_inverse_step (PWM_DUTY_CYCLE_RAMP_UP_INVERSE_STEP); // each step = 64us
  motor_set_pwm_duty_cycle_ramp_down_inverse_step (PWM_DUTY_CYCLE_RAMP_DOWN_INVERSE_STEP); // each step = 64us
}

void motor_set_pwm_duty_cycle_target (uint8_t ui8_value)
{
  if (ui8_value > PWM_DUTY_CYCLE_MAX) { ui8_value = PWM_DUTY_CYCLE_MAX; }

  ui8_duty_cycle_target = ui8_value;
}

void motor_set_pwm_duty_cycle_ramp_up_inverse_step (uint16_t ui16_value)
{
  ui16_duty_cycle_ramp_up_inverse_step = ui16_value;
}

void motor_set_pwm_duty_cycle_ramp_down_inverse_step (uint16_t ui16_value)
{
  ui16_duty_cycle_ramp_down_inverse_step = ui16_value;
}

uint16_t ui16_motor_get_motor_speed_erps (void)
{
  return ui16_motor_speed_erps;
}

void read_battery_voltage (void)
{
  // low pass filter the voltage readed value, to avoid possible fast spikes/noise
  ui16_adc_battery_voltage_accumulated -= ui16_adc_battery_voltage_accumulated >> READ_BATTERY_VOLTAGE_FILTER_COEFFICIENT;
  ui16_adc_battery_voltage_accumulated += ui16_adc_read_battery_voltage_10b ();
  ui16_adc_battery_voltage_filtered = ui16_adc_battery_voltage_accumulated >> READ_BATTERY_VOLTAGE_FILTER_COEFFICIENT;
}

void read_battery_current (void)
{
  // low pass filter the positive battery readed value (no regen current), to avoid possible fast spikes/noise
  ui16_adc_battery_current_accumulated -= ui16_adc_battery_current_accumulated >> READ_BATTERY_CURRENT_FILTER_COEFFICIENT;
  ui16_adc_battery_current_accumulated += ui16_adc_read_battery_current_10b ();
  ui16_adc_battery_current_filtered = ui16_adc_battery_current_accumulated >> READ_BATTERY_CURRENT_FILTER_COEFFICIENT;
}

uint8_t asin_table (uint8_t ui8_inverted_angle_x128)
{
  uint8_t ui8_index = 0;

  while (ui8_index < SIN_TABLE_LEN)
  {
    if (ui8_inverted_angle_x128 < ui8_sin_table [ui8_index])
    {
      break;
    }

    ui8_index++;
  }

  // first value of table is 0 so ui8_index will always increment to at least 1 and return 0
  return ui8_index--;
}
