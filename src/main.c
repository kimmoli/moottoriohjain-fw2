/**************************************************************************/
/*!
    @file     main.c

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
#include <string.h>
#include "LPC8xx.h"
#include "gpio.h"
#include "mrt.h"
#include "uart.h"
//#include "crc.h"
//#include "sct.h"
//#include "spi.h"

#if defined(__CODE_RED)
  #include <cr_section_macros.h>
  #include <NXP/crp.h>
  __CRP const unsigned int CRP_WORD = CRP_NO_CRP ;
#endif

#include "sct_fsm.h"

#define MIN 1500

/* This define should be enabled if you want to      */
/* maintain an SWD/debug connection to the LPC810,   */
/* but it will prevent you from having access to the */
/* LED on the LPC810 Mini Board, which is on the     */
/* SWDIO pin (PIO0_2).                               */
// #define USE_SWD


  /*
	PIO0_6	M1CW	Output
	PIO0_7	M1CCW	Output
	PIO0_8	M2CW	Output
	PIO0_9	M2CCW	Output

	PIO0_1	ST Led	I/O (open drain)

	PIO0_13	LED	Output
	PIO0_4	TXD
	PIO0_0	RXD

	PIO0_3	S2	Input
	PIO0_2	S1	Input

	PIO0_10 S3    Input
	PIO0_11 RELAY Output
   * */

int st1 = NOSIG;
int st2 = NOSIG;

int turn = STOP;
int drive = STOP;

int rate = 0;
int pp = 0;

void configurePins()
{
    /* Enable SWM clock */
    LPC_SYSCON->SYSAHBCLKCTRL |= (1<<7);


    /* Pin Assign 8 bit Configuration */
    /* U0_TXD */
    /* U0_RXD */
    LPC_SWM->PINASSIGN0 = 0xffff0004UL;

    /* Pin Assign 1 bit Configuration */
    /* RESET */
    LPC_SWM->PINENABLE0 = 0xffffffbfUL;

    /* CTIN_0 */
    LPC_SWM->PINASSIGN5 = 0x02ffffffUL;	// CTIN_0 -> PIO0_2
    /* CTIN_1 */
    LPC_SWM->PINASSIGN6 = 0xffffff03UL;	// CTIN_1 -> PIO0_3
    /* CTOUT_0 */
    // LPC_SWM->PINASSIGN6 = 0x06ffffffUL; // CTOUT_0 -> PIO0_6
    /* CTOUT_1 */
    // LPC_SWM->PINASSIGN7 = 0xffffff07UL;	// CTOUT_1 -> PIO0_7

	LPC_GPIO_PORT->CLR0 = 0x3C0; // clear all outputs
    /* Outputs */
    LPC_GPIO_PORT->DIR0 |= (1 << 6);
    LPC_GPIO_PORT->DIR0 |= (1 << 7);
    LPC_GPIO_PORT->DIR0 |= (1 << 8);
    LPC_GPIO_PORT->DIR0 |= (1 << 9);
    LPC_GPIO_PORT->DIR0 |= (1 << 13); // PIO0_13 = LED~ out

	/* Pin I/O Configuration */
	/* LPC_IOCON->PIO0_0 = 0x90; */
	LPC_IOCON->PIO0_1 = 0x490; // Opendrain
	LPC_IOCON->PIO0_2 = 0x88;
	LPC_IOCON->PIO0_3 = 0x88;
	/* LPC_IOCON->PIO0_4 = 0x90; */
	/* LPC_IOCON->PIO0_5 = 0x90; */
	LPC_IOCON->PIO0_6 = 0x88;
	LPC_IOCON->PIO0_7 = 0x88;
	LPC_IOCON->PIO0_8 = 0x88;
	LPC_IOCON->PIO0_9 = 0x88;
	LPC_IOCON->PIO0_10 = 0x180;
	LPC_IOCON->PIO0_11 = 0x180;
	/* LPC_IOCON->PIO0_12 = 0x90; */
	/* LPC_IOCON->PIO0_13 = 0x90; */

	// ST led
	LPC_GPIO_PORT->SET0 = (1 << 1);
	LPC_GPIO_PORT->DIR0 |= (1 << 1);

	// Relay control output
	LPC_GPIO_PORT->CLR0 = (1 << 10);
	LPC_GPIO_PORT->DIR0 |= (1 << 10);
}

int scale(int x, int a, int b, int min, int max)
{
	int val = ( (b - a) * (x - min) ) / (max - min) + a;

	if (val < a)
	    val = a;
	else if (val > b)
	    val = b;

	return val;
}

void PININT0_IRQHandler(void)
{
	// rising edge on GPIO 11. 
	// Start timer to check is the input still high after 1500µs
	// 24 MHz 

	LPC_MRT->Channel[3].INTVAL = 36000 | 0x1UL << 31;
	LPC_PIN_INT->RISE = 0x1;
}

void SCT_IRQHandler (void)
{
	uint32_t status = LPC_SCT->EVFLAG;
	int p = 0;

	if ((status & (1u << SCT_IRQ_EVENT_signal1_no_signal)) || (status & (1u << SCT_IRQ_EVENT_signal2_no_signal)))
	{
		/* Time-out (no signal) */

	    LPC_GPIO_PORT->CLR0 = 0x3C0; // clear all outputs

	    st1 = NOSIG;
		st2 = NOSIG;
		turn = STOP;
		drive = STOP;
		rate = 0;

		LPC_SCT->EVFLAG = status;
		status = 0;
	}

	if (drive == STOP && (status & (1u << SCT_IRQ_EVENT_signal1_width))) // vasen-oikea
	{
		/* New measurement result */

		if (SCT_CAPTURE_cap_signal1_width >= CW_MIN)
		{
			rate = scale(SCT_CAPTURE_cap_signal1_width, MIN_RATE, MAX_RATE, CW_MAX, CW_MIN);
			st1 = CW;
			st2 = CW;
			if (turn != CW) printf("TURN CW ");
			turn = CW;
		}
		else if (SCT_CAPTURE_cap_signal1_width <= CCW_MIN)
		{
			rate = scale(SCT_CAPTURE_cap_signal1_width, MIN_RATE, MAX_RATE, CCW_MAX, CCW_MIN);
			st1 = CCW;
			st2 = CCW;
			if (turn != CCW) printf("TURN CCW ");
			turn = CCW;
		}
		else
		{
			rate = 0;
			st1 = STOP;
			st2 = STOP;
			if (turn != STOP) printf("TURN STOP\r\n");
			turn = STOP;
		}
	}

	if (turn == STOP && (status & (1u << SCT_IRQ_EVENT_signal2_width))) // eteen-taakse
	{
		/* New measurement result */

		if (SCT_CAPTURE_cap_signal2_width >= CW_MIN)
		{
			rate = scale(SCT_CAPTURE_cap_signal2_width, MIN_RATE, MAX_RATE, CW_MAX, CW_MIN);
			st1 = CW;
			st2 = CCW;
			if (drive != CW) printf("DRIVE FWD ");
			drive = CW;
		}
		else if (SCT_CAPTURE_cap_signal2_width <= CCW_MIN)
		{
			rate = scale(SCT_CAPTURE_cap_signal2_width, MIN_RATE, MAX_RATE, CCW_MAX, CCW_MIN);
			st1 = CCW;
			st2 = CW;
			if (drive != CCW) printf("DRIVE REV ");
			drive = CCW;
		}
		else
		{
			rate = 0;
			st1 = STOP;
			st2 = STOP;
			if (drive != STOP) printf("DRIVE STOP\r\n");
			drive = STOP;
		}
	}

	p = 100 - ((rate * 100) / PERIOD_RELOAD);

	if ((turn != STOP || drive != STOP) && pp != p)
	{
		printf("%d%% ", p);
		pp = p;
	}

	/* Acknowledge interrupts */
	LPC_SCT->EVFLAG = status;
}


int main(void)
{
  /* Configure the core clock/PLL via CMSIS */
  SystemCoreClockUpdate();

  /* Initialize the GPIO block */
  LPC_GPIO_PORT->CLR0 = 0xf << 6; // clear all outputs
  gpioInit();

  /* Initialize the UART0 block for printf output */
  uart0Init(115200);

  /* Configure the multi-rate timer for 1ms ticks */
  mrtInit(SystemCoreClock/1000);

  /* Configure the switch matrix (setup pins for UART0 and GPIO) */
  configurePins();

  LPC_GPIO_PORT->CLR0 = 1 << 13; // lit led

  printf("** MOOTTORIOHJAIN **\r\n");
  printf("SystemCoreClock %d\r\n", SystemCoreClock );
  printf("MainClock %d\r\n", MainClock );
  printf("LPC_SYSCON->UARTFRGMULT %d\r\n", LPC_SYSCON->UARTFRGMULT);
  printf("LPC_SYSCON->UARTFRGDIV %d\r\n", LPC_SYSCON->UARTFRGDIV);
  printf("LPC_USART0->BRG %d\r\n", LPC_USART0->BRG);
  printf("LPC_SYSCON->SYSAHBCLKDIV %d\r\n", LPC_SYSCON->SYSAHBCLKDIV);
  printf("LPC_SYSCON->SYSPLLCTRL %x\r\n", LPC_SYSCON->SYSPLLCTRL);

  LPC_GPIO_PORT->CLR0 = (1 << 1);
  mrtDelay(2000);

  // enable the SCT clock
  LPC_SYSCON->SYSAHBCLKCTRL |= (1 << 8);

  // clear peripheral reset the SCT:
  LPC_SYSCON->PRESETCTRL |= ( 1<< 8);


  // Initialize it:
  sct_fsm_init();							/* Init code from RedState tool */
  LPC_SCT->CTRL_U = (LPC_SCT->CTRL_U & ~SCT_CTRL_U_PRE_L_Msk) | ((SCT_PRESCALER << SCT_CTRL_U_PRE_L_Pos) & SCT_CTRL_U_PRE_L_Msk);
  LPC_SCT->CTRL_U = (LPC_SCT->CTRL_U & ~SCT_CTRL_U_PRE_H_Msk) | ((SCT_PRESCALER << SCT_CTRL_U_PRE_H_Pos) & SCT_CTRL_U_PRE_H_Msk);

  // unhalt it: - clearing bit 2 of the CTRL register
  LPC_SCT->CTRL_U &= ~SCT_CTRL_U_HALT_L_Msk;
  LPC_SCT->CTRL_U &= ~SCT_CTRL_U_HALT_H_Msk;

  // Enable SCT interrupt
  NVIC_EnableIRQ(SCT_IRQn);

  volatile uint32_t mrt_last = 0;

  LPC_MRT->Channel[2].INTVAL = PERIOD_RELOAD | 0x1UL<<31;

  LPC_SYSCON->SYSAHBCLKCTRL |= (1 << 6); // enable GPIO Interrupt controller clock
  LPC_SYSCON->PINTSEL[0] = 11; // pin 11 interrupt 0
  LPC_PIN_INT->ISEL = 0x0; // edge sensitive
  LPC_PIN_INT->IENR = 0x1; // rising
  LPC_PIN_INT->IST = 0xFF;

  NVIC_EnableIRQ(PININT0_IRQn); // Enable GPIO interrupt

  while(1)
  {
	  __WFI();
	  if (mrt_counter > (mrt_last + 200))
	  {
		  mrt_last = mrt_counter;
		  LPC_GPIO_PORT->NOT0 = 1 << 13; // toggle led

		  if (!signalok)
		  {
			  printf("No signal!\r\n");
			  LPC_GPIO_PORT->CLR0 = (1 << 10); // Relay off
			  LPC_GPIO_PORT->CLR0 = (1 << 1); // ST Led on
		  }
		  else
		  {
			  LPC_GPIO_PORT->SET0 = (1 << 1); // ST Led off
		  }

		  signalok = 0;
	  }
  }
}



