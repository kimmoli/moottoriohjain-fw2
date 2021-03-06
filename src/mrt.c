/**************************************************************************/
/*!
    @file     mrt.c
    @author   K. Townsend
    @brief    Multi-rate timer (mrt) helper functions

    @section LICENSE

    Software License Agreement (BSD License)

    Copyright (c) 2013, K. Townsend (microBuilder.eu)
    All rights reserved.

    Redistribution and use in source and binary forms, with or without
    modification, are permitted provided that the following conditions are met:
    1. Redistributions of source code must retain the above copyright
    notice, this list of conditions and the following disclaimer.
    2. Redistributions in binary form must reproduce the above copyright
    notice, this list of conditions and the following disclaimer in the
    documentation and/or other materials provided with the distribution.
    3. Neither the name of the copyright holders nor the
    names of its contributors may be used to endorse or promote products
    derived from this software without specific prior written permission.

    THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS ''AS IS'' AND ANY
    EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
    WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
    DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER BE LIABLE FOR ANY
    DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
    (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
    LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
    ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
    (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
    SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*/
/**************************************************************************/
#include <stdio.h>
#include "LPC8xx.h"
#include "mrt.h"

volatile uint32_t mrt_counter = 0;
volatile uint32_t mrtDelay_counter = 0;
int signalok = 0;

void MRT_IRQHandler(void)
{
  if ( LPC_MRT->Channel[0].STAT & MRT_STAT_IRQ_FLAG )
  {
    LPC_MRT->Channel[0].STAT = MRT_STAT_IRQ_FLAG;      /* clear interrupt flag */
    mrt_counter++;
    mrtDelay_counter++;
  }

  if ( LPC_MRT->Channel[1].STAT & MRT_STAT_IRQ_FLAG ) // M1
  {
    LPC_MRT->Channel[1].STAT = MRT_STAT_IRQ_FLAG;      /* clear interrupt flag */

	if (st1 == CW)
	{
		LPC_GPIO_PORT->CLR0 = 0x2 << 6;
		LPC_GPIO_PORT->SET0 = 0x1 << 6;
	}
	else if (st1 == CCW)
	{
		LPC_GPIO_PORT->CLR0 = 0x1 << 6;
		LPC_GPIO_PORT->SET0 = 0x2 << 6;
	}
	if (st2 == CW)
	{
		LPC_GPIO_PORT->CLR0 = 0x2 << 8;
		LPC_GPIO_PORT->SET0 = 0x1 << 8;
	}
	else if (st2 == CCW)
	{
		LPC_GPIO_PORT->CLR0 = 0x1 << 8;
		LPC_GPIO_PORT->SET0 = 0x2 << 8;
	}
  }

  if ( LPC_MRT->Channel[2].STAT & MRT_STAT_IRQ_FLAG )
  {
    LPC_MRT->Channel[2].STAT = MRT_STAT_IRQ_FLAG;      /* clear interrupt flag */

    LPC_GPIO_PORT->CLR0 = 0x3C0; // clear all outputs

    LPC_MRT->Channel[1].INTVAL = rate | 0x1UL<<31;  // reload the other timer to generate PWM
  }

  if ( LPC_MRT->Channel[3].STAT & MRT_STAT_IRQ_FLAG )
  {
    LPC_MRT->Channel[3].STAT = MRT_STAT_IRQ_FLAG;      /* clear interrupt flag */

	signalok = 1;

    // Replicate GPIO 11 state to GPIO 10
    if (LPC_GPIO_PORT->PIN0 & (1 << 11))
    {
		if ((LPC_GPIO_PORT->PIN0 & (1 << 10)) == 0)
		{
			printf("Relay on\r\n");
		}
		LPC_GPIO_PORT->SET0 = (1 << 10); // Relay on
	}
    else
    {
		if ((LPC_GPIO_PORT->PIN0 & (1 << 10)) != 0)
		{
			printf("Relay off\r\n");
		}
		LPC_GPIO_PORT->CLR0 = (1 << 10); // Relay off
    }
  }

  return;
}

void mrtInit(uint32_t delay)
{
  /* Enable clock to MRT and reset the MRT peripheral */
  LPC_SYSCON->SYSAHBCLKCTRL |= (0x1<<10);
  LPC_SYSCON->PRESETCTRL &= ~(0x1<<7);
  LPC_SYSCON->PRESETCTRL |= (0x1<<7);

  mrt_counter = 0;
  LPC_MRT->Channel[0].INTVAL = delay;
  LPC_MRT->Channel[0].INTVAL |= 0x1UL<<31;

  LPC_MRT->Channel[0].CTRL = MRT_REPEATED_MODE|MRT_INT_ENA;
  LPC_MRT->Channel[1].CTRL = MRT_ONE_SHOT_INT|MRT_INT_ENA;
  LPC_MRT->Channel[2].CTRL = MRT_REPEATED_MODE|MRT_INT_ENA;
  LPC_MRT->Channel[3].CTRL = MRT_ONE_SHOT_INT|MRT_INT_ENA;

  /* Enable the MRT Interrupt */
#if NMI_ENABLED
  NVIC_DisableIRQ( MRT_IRQn );
  NMI_Init( MRT_IRQn );
#else
  NVIC_EnableIRQ(MRT_IRQn);
#endif
  return;
}

void mrtDelay(uint32_t ticks)
{
	mrtDelay_counter = 0;
	while(mrtDelay_counter < ticks);
}
