/*
 * TongSheng TSDZ2 motor controller firmware/
 *
 * Copyright (C) Casainho, 2018.
 *
 * Released under the GPL License, Version 3
 */

#include <stdint.h>
#include <stdio.h>
#include "stm8s_tim1.h"
#include "interrupts.h"
#include "pwm.h"
#include "gpio.h"

void pwm_init (void)
{
  TIM1_TimeBaseInit(0, // TIM1_Prescaler = 0
    TIM1_COUNTERMODE_UP,
    (1024 - 1), // clock = 16MHz; counter period = 1024; PWM freq = 16MHz / 1024 = 15.625kHz
    0); // TIM1_RepetitionCounter = 0

  TIM1_OC1Init(TIM1_OCMODE_PWM1,
    TIM1_OUTPUTSTATE_ENABLE,
    TIM1_OUTPUTSTATE_ENABLE,
    0, // initial duty_cycle value
    TIM1_OCPOLARITY_HIGH,
    TIM1_OCNPOLARITY_HIGH,
    TIM1_OCIDLESTATE_RESET,
    TIM1_OCNIDLESTATE_RESET);

  TIM1_OC2Init(TIM1_OCMODE_PWM1,
    TIM1_OUTPUTSTATE_ENABLE,
    TIM1_OUTPUTSTATE_ENABLE,
    0, // initial duty_cycle value
    TIM1_OCPOLARITY_HIGH,
    TIM1_OCNPOLARITY_HIGH,
    TIM1_OCIDLESTATE_RESET,
    TIM1_OCNIDLESTATE_RESET);

  TIM1_OC3Init(TIM1_OCMODE_PWM1,
   TIM1_OUTPUTSTATE_ENABLE,
   TIM1_OUTPUTSTATE_ENABLE,
    0, // initial duty_cycle value
    TIM1_OCPOLARITY_HIGH,
    TIM1_OCNPOLARITY_HIGH,
    TIM1_OCIDLESTATE_RESET,
    TIM1_OCNIDLESTATE_RESET);

  // break, dead time and lock configuration
  TIM1_BDTRConfig(TIM1_OSSISTATE_ENABLE,
    TIM1_LOCKLEVEL_OFF,
    // FAN7888 already takes care of dead time so we use 0 value
    0,
    TIM1_BREAK_DISABLE,
    TIM1_BREAKPOLARITY_LOW,
    TIM1_AUTOMATICOUTPUT_DISABLE);

  // OC4 is always syncronized with PWM
  TIM1_OC4Init(TIM1_OCMODE_TIMING,
	       TIM1_OUTPUTSTATE_DISABLE,
	       0, // timming for interrupt firing
	       TIM1_OCPOLARITY_HIGH,
	       TIM1_OCIDLESTATE_RESET);


  TIM1_ITConfig(TIM1_IT_CC4, ENABLE);
  TIM1_Cmd(ENABLE); // TIM1 counter enable
  TIM1_CtrlPWMOutputs(ENABLE);
}
