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
#include "stm8s_tim1.h"

#define TIM1_CHANNEL_PHASE_C TIM1_CHANNEL_1
#define TIM1_CHANNEL_PHASE_A TIM1_CHANNEL_2
#define TIM1_CHANNEL_PHASE_B TIM1_CHANNEL_3

#define PHASE_A_HIGH_ENABLE_LOW_DISABLE \
  TIM1_SelectOCxM(TIM1_CHANNEL_PHASE_A,(TIM1_OCMode_TypeDef)TIM1_FORCEDACTION_ACTIVE); \
  TIM1_CCxCmd(TIM1_CHANNEL_PHASE_A, ENABLE); \
  TIM1_CCxNCmd(TIM1_CHANNEL_PHASE_A, ENABLE);

#define PHASE_A_HIGH_DISABLE_LOW_ENABLE \
  TIM1_SelectOCxM(TIM1_CHANNEL_PHASE_A,(TIM1_OCMode_TypeDef)TIM1_FORCEDACTION_INACTIVE); \
  TIM1_CCxCmd(TIM1_CHANNEL_PHASE_A, ENABLE); \
  TIM1_CCxNCmd(TIM1_CHANNEL_PHASE_A, ENABLE);

#define PHASE_A_HIGH_DISABLE_LOW_DISABLE \
  TIM1_CCxCmd(TIM1_CHANNEL_PHASE_A, DISABLE); \
  TIM1_CCxNCmd(TIM1_CHANNEL_PHASE_A, DISABLE);

#define PHASE_A_HIGH_PWM_LOW_PWM \
  TIM1_SelectOCxM(TIM1_CHANNEL_PHASE_A,(TIM1_OCMode_TypeDef)TIM1_OCMODE_PWM1); \
  TIM1_CCxCmd(TIM1_CHANNEL_PHASE_A, ENABLE); \
  TIM1_CCxNCmd(TIM1_CHANNEL_PHASE_A, ENABLE);


#define PHASE_B_HIGH_ENABLE_LOW_DISABLE \
  TIM1_SelectOCxM(TIM1_CHANNEL_PHASE_B,(TIM1_OCMode_TypeDef)TIM1_FORCEDACTION_ACTIVE); \
  TIM1_CCxCmd(TIM1_CHANNEL_PHASE_B, ENABLE); \
  TIM1_CCxNCmd(TIM1_CHANNEL_PHASE_B, ENABLE);

#define PHASE_B_HIGH_DISABLE_LOW_ENABLE \
  TIM1_SelectOCxM(TIM1_CHANNEL_PHASE_B,(TIM1_OCMode_TypeDef)TIM1_FORCEDACTION_INACTIVE); \
  TIM1_CCxCmd(TIM1_CHANNEL_PHASE_B, ENABLE); \
  TIM1_CCxNCmd(TIM1_CHANNEL_PHASE_B, ENABLE);

#define PHASE_B_HIGH_DISABLE_LOW_DISABLE \
  TIM1_CCxCmd(TIM1_CHANNEL_PHASE_B, DISABLE); \
  TIM1_CCxNCmd(TIM1_CHANNEL_PHASE_B, DISABLE);

#define PHASE_B_HIGH_PWM_LOW_PWM \
  TIM1_SelectOCxM(TIM1_CHANNEL_PHASE_B,(TIM1_OCMode_TypeDef)TIM1_OCMODE_PWM1); \
  TIM1_CCxCmd(TIM1_CHANNEL_PHASE_B, ENABLE); \
  TIM1_CCxNCmd(TIM1_CHANNEL_PHASE_B, ENABLE);


#define PHASE_C_HIGH_ENABLE_LOW_DISABLE \
  TIM1_SelectOCxM(TIM1_CHANNEL_PHASE_C,(TIM1_OCMode_TypeDef)TIM1_FORCEDACTION_ACTIVE); \
  TIM1_CCxCmd(TIM1_CHANNEL_PHASE_C, ENABLE); \
  TIM1_CCxNCmd(TIM1_CHANNEL_PHASE_C, ENABLE);

#define PHASE_C_HIGH_DISABLE_LOW_ENABLE \
  TIM1_SelectOCxM(TIM1_CHANNEL_PHASE_C,(TIM1_OCMode_TypeDef)TIM1_FORCEDACTION_INACTIVE); \
  TIM1_CCxCmd(TIM1_CHANNEL_PHASE_C, ENABLE); \
  TIM1_CCxNCmd(TIM1_CHANNEL_PHASE_C, ENABLE);

#define PHASE_C_HIGH_DISABLE_LOW_DISABLE \
  TIM1_CCxCmd(TIM1_CHANNEL_PHASE_C, DISABLE); \
  TIM1_CCxNCmd(TIM1_CHANNEL_PHASE_C, DISABLE);

#define PHASE_C_HIGH_PWM_LOW_PWM \
  TIM1_SelectOCxM(TIM1_CHANNEL_PHASE_C,(TIM1_OCMode_TypeDef)TIM1_OCMODE_PWM1); \
  TIM1_CCxCmd(TIM1_CHANNEL_PHASE_C, ENABLE); \
  TIM1_CCxNCmd(TIM1_CHANNEL_PHASE_C, ENABLE);



uint8_t ui8_hall_sensors_state = 0;
uint8_t ui8_hall_sensors_last_state = 0;

// runs every 64us (PWM frequency)
void TIM1_CAP_COM_IRQHandler (void) __interrupt(TIM1_CAP_COM_IRQHANDLER)
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
      PHASE_A_HIGH_PWM_LOW_PWM
      PHASE_B_HIGH_DISABLE_LOW_DISABLE
      PHASE_C_HIGH_ENABLE_LOW_DISABLE
	    break;

	    case 6:
      PHASE_A_HIGH_DISABLE_LOW_DISABLE
      PHASE_B_HIGH_DISABLE_LOW_ENABLE
      PHASE_C_HIGH_PWM_LOW_PWM
	    break;

      // hall sensor A transition from high to low happens here
      // and considering BEMF phase A max value as the motor rotor 0 degrees position,
      // motor rotor should be at 0 degrees at this hall sensor signal transition
	    case 4:
      PHASE_A_HIGH_ENABLE_LOW_DISABLE
      PHASE_B_HIGH_PWM_LOW_PWM
      PHASE_C_HIGH_DISABLE_LOW_DISABLE
	    break;

	    case 5:
      PHASE_A_HIGH_PWM_LOW_PWM
      PHASE_B_HIGH_DISABLE_LOW_DISABLE
      PHASE_B_HIGH_DISABLE_LOW_ENABLE
	    break;

	    case 1:
      PHASE_A_HIGH_DISABLE_LOW_DISABLE
      PHASE_B_HIGH_ENABLE_LOW_DISABLE
      PHASE_C_HIGH_PWM_LOW_PWM
	    break;

	    case 3:
      PHASE_A_HIGH_DISABLE_LOW_ENABLE
      PHASE_B_HIGH_PWM_LOW_PWM
      PHASE_C_HIGH_DISABLE_LOW_DISABLE
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
  GPIO_Init (HALL_SENSOR_A__PORT, (GPIO_Pin_TypeDef) HALL_SENSOR_A__PIN, GPIO_MODE_IN_FL_NO_IT);
  GPIO_Init (HALL_SENSOR_B__PORT, (GPIO_Pin_TypeDef) HALL_SENSOR_B__PIN, GPIO_MODE_IN_FL_NO_IT);
  GPIO_Init (HALL_SENSOR_C__PORT, (GPIO_Pin_TypeDef) HALL_SENSOR_C__PIN, GPIO_MODE_IN_FL_NO_IT);
}

void motor_init (void)
{

}

