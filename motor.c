/*
 * TongSheng TSDZ2 motor controller firmware/
 *
 * Copyright (C) Casainho, 2018.
 *
 * Released under the GPL License, Version 3
 */

#include <stdint.h>
#include <stdio.h>
#include "interrupts.h"
#include "motor.h"
#include "gpio.h"
#include "pwm.h"

uint8_t ui8_hall_sensors_state = 0;
uint8_t ui8_hall_sensors_last_state = 0;

// runs every 64us (PWM frequency)
void TIM1_CAP_COM_IRQHandler(void) __interrupt(TIM1_CAP_COM_IRQHANDLER)
{
  // read hall sensors signal
  ui8_hall_sensors_state = ((HALL_SENSOR_A__PORT->IDR & HALL_SENSOR_A__PIN) >> 5) |
      ((HALL_SENSOR_B__PORT->IDR & HALL_SENSOR_B__PIN) >> 4) |
      (HALL_SENSOR_C__PORT->IDR & HALL_SENSOR_C__PIN);

  // hall sensors sequence: 2, 6, 4, 5, 1, 3
  // make sure we run next code only when there is a change on the hall sensors signal
  if (ui8_hall_sensors_state != ui8_hall_sensors_last_state)
  {
    ui8_hall_sensors_last_state = ui8_hall_sensors_state;

    switch (ui8_hall_sensors_state)
    {
      case 2:
      break;

      case 6:
      break;

      case 4:
      break;

      case 5:
      break;

      case 1:
      break;

      case 3:
      break;

      default:
      return;
      break;
    }
  }

  // clears the TIM1 interrupt TIM1_IT_UPDATE pending bit
  TIM1->SR1 = (uint8_t)(~(uint8_t)TIM1_IT_CC4);
}

void hall_sensor_init (void)
{
  GPIO_Init(HALL_SENSOR_A__PORT, (GPIO_Pin_TypeDef)HALL_SENSOR_A__PIN, GPIO_MODE_IN_FL_NO_IT);
  GPIO_Init(HALL_SENSOR_B__PORT, (GPIO_Pin_TypeDef)HALL_SENSOR_B__PIN, GPIO_MODE_IN_FL_NO_IT);
  GPIO_Init(HALL_SENSOR_C__PORT, (GPIO_Pin_TypeDef)HALL_SENSOR_C__PIN, GPIO_MODE_IN_FL_NO_IT);
}

void motor_init (void)
{

}
