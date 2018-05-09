/*
 * TongSheng TSDZ2 motor controller firmware/
 *
 * Copyright (C) Casainho, 2018.
 *
 * Released under the GPL License, Version 3
 */

#include <stdint.h>
#include <stdio.h>
#include "stm8s.h"
#include "gpio.h"
#include "stm8s_adc1.h"
#include "adc.h"

void adc_init (void)
{
  uint16_t ui16_counter;

  //init GPIO for the used ADC pins
  GPIO_Init(GPIOB,
	    (GPIO_PIN_7 | GPIO_PIN_6 | GPIO_PIN_5 | GPIO_PIN_3),
	    GPIO_MODE_IN_FL_NO_IT);

  //init ADC1 peripheral
  ADC1_Init(ADC1_CONVERSIONMODE_SINGLE,
      ADC1_CHANNEL_7,
      ADC1_PRESSEL_FCPU_D2,
      ADC1_EXTTRIG_TIM,
      DISABLE,
      ADC1_ALIGN_LEFT,
      (ADC1_SCHMITTTRIG_CHANNEL3 | ADC1_SCHMITTTRIG_CHANNEL5 | ADC1_SCHMITTTRIG_CHANNEL6 | ADC1_SCHMITTTRIG_CHANNEL7),
            DISABLE);

  ADC1_ScanModeCmd (ENABLE);
  ADC1_Cmd (ENABLE);

  //********************************************************************************
  // next code is for "calibrating" the offset value of some ADC channels
  // read and discard few samples of ADC, to make sure the next samples are ok

  // read and average a few values of ADC battery current
  ui16_adc_battery_current_offset_10b = 0;
  for (ui8_i = 0; ui8_i < 16; ui8_i++)
  {
    ui16_counter = TIM3_GetCounter () + 10; // delay ~10ms
    while (TIM3_GetCounter () < ui16_counter) ; // delay ~10ms
    adc_trigger ();
    while (!ADC1_GetFlagStatus (ADC1_FLAG_EOC)) ; // wait for end of conversion
    ui16_adc_battery_current_offset_10b += ui16_adc_read_battery_current_10b ();
  }
  ui16_adc_battery_current_offset_10b >>= 4;
  ui8_adc_battery_current_offset = ui16_adc_battery_current_offset_10b >> 2;

  // read and average a few values of ADC throttle
  ui16_adc_throttle_offset = 0;
  for (ui8_i = 0; ui8_i < 16; ui8_i++)
  {
    ui16_counter = TIM3_GetCounter () + 10; // delay ~10ms
    while (TIM3_GetCounter () < ui16_counter) ; // delay ~10ms
    adc_trigger ();
    while (!ADC1_GetFlagStatus (ADC1_FLAG_EOC)) ; // wait for end of conversion
    ui16_adc_throttle_offset += UI8_ADC_THROTTLE;
  }
  ui16_adc_throttle_offset >>= 4;
  ui8_adc_throttle_offset = ui16_adc_throttle_offset;
}

void adc_trigger (void)
{
  // trigger ADC conversion of all channels (scan conversion, buffered)
  ADC1->CSR &= 0x09; // clear EOC flag first (selected also channel 9)
  ADC1->CR1 |= ADC1_CR1_ADON; // Start ADC1 conversion
}
