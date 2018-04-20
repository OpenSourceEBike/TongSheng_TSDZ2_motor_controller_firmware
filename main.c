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
#include "stm8s.h"
#include "gpio.h"
#include "uart.h"
#include "pwm.h"
#include "motor.h"
#include "wheel_speed_sensor.h"
#include "brake.h"
#include "pas.h"
#include "adc.h"
#include "timers.h"

/////////////////////////////////////////////////////////////////////////////////////////////
//// Functions prototypes

// main -- start of firmware and main loop
int main (void);

// With SDCC, interrupt service routine function prototypes must be placed in the file that contains main ()
// in order for an vector for the interrupt to be placed in the the interrupt vector space.  It's acceptable
// to place the function prototype in a header file as long as the header file is included in the file that
// contains main ().  SDCC will not generate any warnings or errors if this is not done, but the vector will
// not be in place so the ISR will not be executed when the interrupt occurs.

// Calling a function from interrupt not always works, SDCC manual says to avoid it. Maybe the best is to put
// all the code inside the interrupt

// Local VS global variables
// Sometimes I got the following error when compiling the firmware: motor.asm:750: Error: <r> relocation error
// when I have this code inside a function: "static uint8_t ui8_cruise_counter = 0;"
// and the solution was define the variable as global instead
// Another error example:
// *** buffer overflow detected ***: sdcc terminated
// Caught signal 6: SIGABRT

// UART2 Receive interrupt
void UART2_IRQHandler(void) __interrupt(UART2_IRQHANDLER);
// PWM cycle interrupt
void TIM1_CAP_COM_IRQHandler(void) __interrupt(TIM1_CAP_COM_IRQHANDLER);
void EXTI_PORTA_IRQHandler(void) __interrupt(EXTI_PORTA_IRQHANDLER);

/////////////////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////////////////

// This is the interrupt that happesn when UART2 receives data.
void UART2_IRQHandler(void) __interrupt(UART2_IRQHANDLER)
{
  if(UART2_GetFlagStatus(UART2_FLAG_RXNE) == SET)
  {
//    ui8_byte_received = UART2_ReceiveData8 ();
  }
}

int main (void)
{
  static uint32_t ui32_temp;

  gpio_init ();
  brake_init ();
  while (brake_is_set()) ; // hold here while brake is pressed -- this is a protection for development
  pas1_init ();
  pas2_init ();
  wheel_speed_sensor_init ();
  uart_init ();
  pwm_init ();
  hall_sensor_init ();
  motor_init ();
  timer2_init ();
//  enableInterrupts ();
  TIM1_SetCompare1 (511);
  TIM1_SetCompare2 (511);
  TIM1_SetCompare3 (511);

  adc_init ();

  while (1)
  {
//    printf ("%d - %d - %d - %d - %d\n", GPIOA->IDR, GPIOB->IDR, GPIOC->IDR, GPIOD->IDR, GPIOE->IDR);
//    printf ("%d - %d\n", GPIO_ReadInputPin(PAS1__PORT, PAS1__PIN), GPIO_ReadInputPin(PAS2__PORT, PAS2__PIN));

    printf ("%d,%d,%d,%d,%d,%d\n",
	    (uint16_t) ADC1_GetBufferValue(3) >> 2,
	    (uint16_t) ADC1_GetBufferValue(4) >> 2,
	    (uint16_t) ADC1_GetBufferValue(5) >> 2,
	    (uint16_t) ADC1_GetBufferValue(6) >> 2,
	    (uint16_t) ADC1_GetBufferValue(7) >> 2,
	    (uint16_t) ADC1_GetBufferValue(9) >> 2);

//      printf ("%d\n",
//  	    (uint16_t) ADC1_GetBufferValue(4));
  }

  return 0;
}

