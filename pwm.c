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
  TIM1_CtrlPWMOutputs(DISABLE);

  TIM1_TimeBaseInit(0, // TIM1_Prescaler = 0
    TIM1_COUNTERMODE_UP,
    (1024 - 1), // clock = 16MHz; counter period = 1024; PWM freq = 16MHz / 1024 = 15.625kHz
    0); // TIM1_RepetitionCounter = 0


#define DISABLE_PWM_CHANNELS_1_3

  TIM1_OC1Init(TIM1_OCMODE_PWM1,
#ifdef DISABLE_PWM_CHANNELS
    TIM1_OUTPUTSTATE_DISABLE,
    TIM1_OUTPUTNSTATE_DISABLE,
#else
    TIM1_OUTPUTSTATE_ENABLE,
    TIM1_OUTPUTNSTATE_DISABLE,
#endif
    0, // initial duty_cycle value
    TIM1_OCPOLARITY_HIGH,
    TIM1_OCNPOLARITY_HIGH,
    TIM1_OCIDLESTATE_RESET,
    TIM1_OCNIDLESTATE_RESET);

  TIM1_OC2Init(TIM1_OCMODE_PWM1,
    TIM1_OUTPUTSTATE_ENABLE,
    TIM1_OUTPUTNSTATE_DISABLE,
    0, // initial duty_cycle value
    TIM1_OCPOLARITY_HIGH,
    TIM1_OCNPOLARITY_HIGH,
    TIM1_OCIDLESTATE_RESET,
    TIM1_OCNIDLESTATE_RESET);

  TIM1_OC3Init(TIM1_OCMODE_PWM1,
#ifdef DISABLE_PWM_CHANNELS
    TIM1_OUTPUTSTATE_DISABLE,
    TIM1_OUTPUTNSTATE_DISABLE,
#else
    TIM1_OUTPUTSTATE_ENABLE,
    TIM1_OUTPUTNSTATE_DISABLE,
#endif
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

  // OC4 is always ssyncronized with PWM
  TIM1_OC4Init(TIM1_OCMODE_PWM1,
	       TIM1_OUTPUTSTATE_DISABLE,
	       0, // timming for interrupt firing
	       TIM1_OCPOLARITY_HIGH,
	       TIM1_OCIDLESTATE_RESET);

//  // put the phases to a valid state
//  pwm_phase_a_disable ();
//  pwm_phase_b_disable ();
//  pwm_phase_c_disable ();

  TIM1_ITConfig(TIM1_IT_CC4, ENABLE);
  TIM1_Cmd(ENABLE); // TIM1 counter enable
  TIM1_CtrlPWMOutputs(ENABLE);
}

//void pwm_phase_a_disable (void)
//{
////  TIM1_OC3Init(TIM1_OCMODE_PWM1,
////	       TIM1_OUTPUTNSTATE_DISABLE,
////  	       TIM1_OUTPUTNSTATE_DISABLE,
////	       0,
////  	       TIM1_OCPOLARITY_HIGH,
////  	       TIM1_OCNPOLARITY_LOW,
////  	       TIM1_OCIDLESTATE_RESET,
////  	       TIM1_OCNIDLESTATE_SET);
////
////  // disable pin
////  GPIO_Init(PMW_PHASE_A_HIGH__PORT,
////	    PMW_PHASE_A_HIGH__PIN,
////	    GPIO_MODE_OUT_PP_LOW_FAST);
////
////  // disable pin
////  GPIO_Init(PMW_PHASE_A_LOW__PORT,
////	    PMW_PHASE_A_LOW__PIN,
////	    GPIO_MODE_OUT_PP_HIGH_FAST);
//
//  // PWM channel N as IO pin and output high (disable power mosfet)
//  PMW_PHASE_A_LOW__PORT->ODR |= (uint8_t)PMW_PHASE_A_LOW__PIN;
//  PMW_PHASE_A_LOW__PORT->DDR |= (uint8_t)PMW_PHASE_A_LOW__PIN;
//  // disable PWM channel pins
//  TIM1->CCER2 &= (uint8_t)(~( TIM1_CCER2_CC3E | TIM1_CCER2_CC3NE));
//}
//
//void pwm_phase_a_enable_pwm (void)
//{
////  TIM1_OC3Init(TIM1_OCMODE_PWM1,
////	       TIM1_OUTPUTSTATE_ENABLE,
////	       TIM1_OUTPUTNSTATE_DISABLE,
////	       ui8_duty_cycle,
////	       TIM1_OCPOLARITY_HIGH,
////	       TIM1_OCNPOLARITY_LOW,
////	       TIM1_OCIDLESTATE_RESET,
////	       TIM1_OCNIDLESTATE_SET);
////
////  // disable pin
////  GPIO_Init(PMW_PHASE_A_LOW__PORT,
////	    PMW_PHASE_A_LOW__PIN,
////	    GPIO_MODE_OUT_PP_HIGH_FAST);
//
//  // PWM channel N as IO pin and output high (disable power mosfet)
//  PMW_PHASE_A_LOW__PORT->ODR |= (uint8_t)PMW_PHASE_A_LOW__PIN;
//  PMW_PHASE_A_LOW__PORT->DDR |= (uint8_t)PMW_PHASE_A_LOW__PIN;
//  // disable PWM n channel
//  TIM1->CCER2 &= (uint8_t)(~(TIM1_CCER2_CC3NE));
//  // enable PWM p channel
//  TIM1->CCER2 |= (uint8_t)(TIM1_CCER2_CC3E);
//}
//
//void pwm_phase_a_enable_low (void)
//{
////  TIM1_OC3Init(TIM1_OCMODE_PWM1,
////	       TIM1_OUTPUTNSTATE_DISABLE,
////  	       TIM1_OUTPUTNSTATE_DISABLE,
////	       0,
////	       TIM1_OCPOLARITY_HIGH,
////	       TIM1_OCNPOLARITY_LOW,
////	       TIM1_OCIDLESTATE_RESET,
////	       TIM1_OCNIDLESTATE_SET);
////
////  // disable pin
////  GPIO_Init(PMW_PHASE_A_HIGH__PORT,
////	    PMW_PHASE_A_HIGH__PIN,
////	    GPIO_MODE_OUT_PP_LOW_FAST);
////
////  // enable pin
////  GPIO_Init(PMW_PHASE_A_LOW__PORT,
////	    PMW_PHASE_A_LOW__PIN,
////	    GPIO_MODE_OUT_PP_LOW_FAST);
//
//  // disable PWM channel pins
//  TIM1->CCER2 &= (uint8_t)(~( TIM1_CCER2_CC3E | TIM1_CCER2_CC3NE));
//  // PWM channel N as IO pin and output low (enable power mosfet)
//  PMW_PHASE_A_LOW__PORT->ODR &= (uint8_t)~PMW_PHASE_A_LOW__PIN;
//  PMW_PHASE_A_LOW__PORT->DDR |= (uint8_t)PMW_PHASE_A_LOW__PIN;
//}
//
//void pwm_phase_b_disable (void)
//{
////  TIM1_OC2Init(TIM1_OCMODE_PWM1,
////	       TIM1_OUTPUTNSTATE_DISABLE,
////  	       TIM1_OUTPUTNSTATE_DISABLE,
////	       0,
////  	       TIM1_OCPOLARITY_HIGH,
////  	       TIM1_OCNPOLARITY_LOW,
////  	       TIM1_OCIDLESTATE_RESET,
////  	       TIM1_OCNIDLESTATE_SET);
////
////  // disable pin
////  GPIO_Init(PMW_PHASE_B_HIGH__PORT,
////	    PMW_PHASE_B_HIGH__PIN,
////	    GPIO_MODE_OUT_PP_LOW_FAST);
////
////  // disable pin
////  GPIO_Init(PMW_PHASE_B_LOW__PORT,
////	    PMW_PHASE_B_LOW__PIN,
////	    GPIO_MODE_OUT_PP_HIGH_FAST);
//
//  // PWM channel N as IO pin and output high (disable power mosfet)
//  PMW_PHASE_B_LOW__PORT->ODR |= (uint8_t)PMW_PHASE_B_LOW__PIN;
//  PMW_PHASE_B_LOW__PORT->DDR |= (uint8_t)PMW_PHASE_B_LOW__PIN;
//  // disable PWM channel pins
//  TIM1->CCER1 &= (uint8_t)(~( TIM1_CCER1_CC2E | TIM1_CCER1_CC2NE));
//}
//
//void pwm_phase_b_enable_pwm (void)
//{
////  TIM1_OC2Init(TIM1_OCMODE_PWM1,
////	       TIM1_OUTPUTSTATE_ENABLE,
////	       TIM1_OUTPUTNSTATE_DISABLE,
////	       ui8_duty_cycle,
////	       TIM1_OCPOLARITY_HIGH,
////	       TIM1_OCNPOLARITY_LOW,
////	       TIM1_OCIDLESTATE_RESET,
////	       TIM1_OCNIDLESTATE_SET);
////
////  // disable pin
////  GPIO_Init(PMW_PHASE_B_LOW__PORT,
////	    PMW_PHASE_B_LOW__PIN,
////	    GPIO_MODE_OUT_PP_HIGH_FAST);
//
//  // PWM channel N as IO pin and output high (disable power mosfet)
//  PMW_PHASE_B_LOW__PORT->ODR |= (uint8_t)PMW_PHASE_B_LOW__PIN;
//  PMW_PHASE_B_LOW__PORT->DDR |= (uint8_t)PMW_PHASE_B_LOW__PIN;
//  // disable PWM n channel
//  TIM1->CCER1 &= (uint8_t)(~(TIM1_CCER1_CC2NE));
//  // enable PWM p channel
//  TIM1->CCER1 |= (uint8_t)(TIM1_CCER1_CC2E);
//}
//
//void pwm_phase_b_enable_low (void)
//{
////  TIM1_OC2Init(TIM1_OCMODE_PWM1,
////	       TIM1_OUTPUTNSTATE_DISABLE,
////  	       TIM1_OUTPUTNSTATE_DISABLE,
////	       0,
////	       TIM1_OCPOLARITY_HIGH,
////	       TIM1_OCNPOLARITY_LOW,
////	       TIM1_OCIDLESTATE_RESET,
////	       TIM1_OCNIDLESTATE_SET);
////
////  // disable pin
////  GPIO_Init(PMW_PHASE_B_HIGH__PORT,
////	    PMW_PHASE_B_HIGH__PIN,
////	    GPIO_MODE_OUT_PP_LOW_FAST);
////
////  // enable pin
////  GPIO_Init(PMW_PHASE_B_LOW__PORT,
////	    PMW_PHASE_B_LOW__PIN,
////	    GPIO_MODE_OUT_PP_LOW_FAST);
//
//  // disable PWM channel pins
//  TIM1->CCER1 &= (uint8_t)(~( TIM1_CCER1_CC2E | TIM1_CCER1_CC2NE));
//  // PWM channel N as IO pin and output low (enable power mosfet)
//  PMW_PHASE_B_LOW__PORT->ODR &= (uint8_t)~PMW_PHASE_B_LOW__PIN;
//  PMW_PHASE_B_LOW__PORT->DDR |= (uint8_t)PMW_PHASE_B_LOW__PIN;
//}
//
//void pwm_phase_c_disable (void)
//{
////  TIM1_OC1Init(TIM1_OCMODE_PWM1,
////	       TIM1_OUTPUTNSTATE_DISABLE,
////  	       TIM1_OUTPUTNSTATE_DISABLE,
////	       0,
////  	       TIM1_OCPOLARITY_HIGH,
////  	       TIM1_OCNPOLARITY_LOW,
////  	       TIM1_OCIDLESTATE_RESET,
////  	       TIM1_OCNIDLESTATE_SET);
////
////  // disable pin
////  GPIO_Init(PMW_PHASE_C_HIGH__PORT,
////	    PMW_PHASE_C_HIGH__PIN,
////	    GPIO_MODE_OUT_PP_LOW_FAST);
////
////  // disable pin
////  GPIO_Init(PMW_PHASE_C_LOW__PORT,
////	    PMW_PHASE_C_LOW__PIN,
////	    GPIO_MODE_OUT_PP_HIGH_FAST);
//
//  // PWM channel N as IO pin and output high (disable power mosfet)
//  PMW_PHASE_C_LOW__PORT->ODR |= (uint8_t)PMW_PHASE_C_LOW__PIN;
//  PMW_PHASE_C_LOW__PORT->DDR |= (uint8_t)PMW_PHASE_C_LOW__PIN;
//  // disable PWM channel pins
//  TIM1->CCER1 &= (uint8_t)(~( TIM1_CCER1_CC1E | TIM1_CCER1_CC1NE));
//}
//
//void pwm_phase_c_enable_pwm (void)
//{
////  TIM1_OC1Init(TIM1_OCMODE_PWM1,
////	       TIM1_OUTPUTSTATE_ENABLE,
////	       TIM1_OUTPUTNSTATE_DISABLE,
////	       ui8_duty_cycle,
////	       TIM1_OCPOLARITY_HIGH,
////	       TIM1_OCNPOLARITY_LOW,
////	       TIM1_OCIDLESTATE_RESET,
////	       TIM1_OCNIDLESTATE_SET);
////
////  // disable pin
////  GPIO_Init(PMW_PHASE_C_LOW__PORT,
////	    PMW_PHASE_C_LOW__PIN,
////	    GPIO_MODE_OUT_PP_HIGH_FAST);
//
//  // PWM channel N as IO pin and output high (disable power mosfet)
//  PMW_PHASE_C_LOW__PORT->ODR |= (uint8_t)PMW_PHASE_C_LOW__PIN;
//  PMW_PHASE_C_LOW__PORT->DDR |= (uint8_t)PMW_PHASE_C_LOW__PIN;
//  // disable PWM n channel
//  TIM1->CCER1 &= (uint8_t)(~(TIM1_CCER1_CC1NE));
//  // enable PWM p channel
//  TIM1->CCER1 |= (uint8_t)(TIM1_CCER1_CC1E);
//}
//
//void pwm_phase_c_enable_low (void)
//{
////  TIM1_OC1Init(TIM1_OCMODE_PWM1,
////	       TIM1_OUTPUTNSTATE_DISABLE,
////  	       TIM1_OUTPUTNSTATE_DISABLE,
////	       0,
////	       TIM1_OCPOLARITY_HIGH,
////	       TIM1_OCNPOLARITY_LOW,
////	       TIM1_OCIDLESTATE_RESET,
////	       TIM1_OCNIDLESTATE_SET);
////
////  // disable pin
////  GPIO_Init(PMW_PHASE_C_HIGH__PORT,
////	    PMW_PHASE_C_HIGH__PIN,
////	    GPIO_MODE_OUT_PP_LOW_FAST);
////
////  // enable pin
////  GPIO_Init(PMW_PHASE_C_LOW__PORT,
////	    PMW_PHASE_C_LOW__PIN,
////	    GPIO_MODE_OUT_PP_LOW_FAST);
//
//  // disable PWM channel pins
//  TIM1->CCER1 &= (uint8_t)(~( TIM1_CCER1_CC1E | TIM1_CCER1_CC1NE));
//  // PWM channel N as IO pin and output low (enable power mosfet)
//  PMW_PHASE_C_LOW__PORT->ODR &= (uint8_t)~PMW_PHASE_C_LOW__PIN;
//  PMW_PHASE_C_LOW__PORT->DDR |= (uint8_t)PMW_PHASE_C_LOW__PIN;
//}
