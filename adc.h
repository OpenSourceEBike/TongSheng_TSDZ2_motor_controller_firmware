/*
 * TongSheng TSDZ2 motor controller firmware/
 *
 * Copyright (C) Casainho, 2018.
 *
 * Released under the GPL License, Version 3
 */

#ifndef _ADC_H
#define _ADC_H

#include "main.h"

#define UI8_ADC_BATTERY_VOLTAGE 			(*(uint8_t*)(0x53EC))
#define UI8_ADC_BATTERY_CURRENT				(*(uint8_t*)(0x53EA))
#define UI8_ADC_THROTTLE 				      (*(uint8_t*)(0x53EE))

void adc_init (void);

#endif /* _ADC_H */
