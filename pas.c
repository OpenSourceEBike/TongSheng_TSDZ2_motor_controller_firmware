/*
 * TongSheng TSDZ2 motor controller firmware/
 *
 * Copyright (C) Casainho, 2018.
 *
 * Released under the GPL License, Version 3
 */

#include <stdint.h>
#include "stm8s.h"
#include "gpio.h"

void pas1_init (void)
{
  //PAS1 pin as external input pin
  GPIO_Init(PAS1__PORT,
	    PAS1__PIN,
	    GPIO_MODE_IN_PU_NO_IT); // input pull-up, no external interrupt
}

void pas2_init (void)
{
  //PAS2 pin as external input pin interrupt
  GPIO_Init(PAS2__PORT,
	    PAS2__PIN,
	    GPIO_MODE_IN_PU_NO_IT); // input pull-up, no external interrupt
}

