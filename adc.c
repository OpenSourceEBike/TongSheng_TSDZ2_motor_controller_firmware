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

void adc_trigger (void);

void adc_init (void)
{
  //init GPIO for the used ADC pins
  GPIO_Init(GPIOB,
	    (GPIO_PIN_7 | GPIO_PIN_6 | GPIO_PIN_5),
	    GPIO_MODE_IN_FL_NO_IT);

  //init ADC1 peripheral
  ADC1_Init(ADC1_CONVERSIONMODE_CONTINUOUS,
	    ADC1_CHANNEL_9,
	    ADC1_PRESSEL_FCPU_D2,
      ADC1_EXTTRIG_TIM,
	    DISABLE,
	    ADC1_ALIGN_LEFT,
	    0,
	    DISABLE);

  ADC1_DataBufferCmd (ENABLE);
  ADC1_ScanModeCmd (ENABLE);

  ADC1_Cmd (ENABLE);
  ADC1_StartConversion();
}

void adc_trigger (void)
{
  // trigger ADC conversion of all channels (scan conversion, buffered)
  ADC1->CSR &= 0x09; // clear EOC flag first (selected also channel 9)
  ADC1->CR1 |= ADC1_CR1_ADON; // Start ADC1 conversion
}
