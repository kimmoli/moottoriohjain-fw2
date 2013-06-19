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
#include "LPC8xx.h"
#include "gpio.h"
#include "mrt.h"
#include "uart.h"

#if defined(__CODE_RED)
  #include <cr_section_macros.h>
  #include <NXP/crp.h>
  __CRP const unsigned int CRP_WORD = CRP_NO_CRP ;
#endif

/* This define should be enabled if you want to      */
/* maintain an SWD/debug connection to the LPC810,   */
/* but it will prevent you from having access to the */
/* LED on the LPC810 Mini Board, which is on the     */
/* SWDIO pin (PIO0_2).                               */
// #define USE_SWD

void configurePins()
{
  /* Enable SWM clock */
  LPC_SYSCON->SYSAHBCLKCTRL |= (1 << 7);

  /* Pin Assign 8 bit Configuration */
  /* U0_TXD */
  /* U0_RXD */
  LPC_SWM->PINASSIGN0 = 0xffff0004UL;
  /* U1_TXD */
  /* U1_RXD */
  LPC_SWM->PINASSIGN1 = 0xff0103ffUL;

  /* Pin Assign 1 bit Configuration */
  /* RESET */
  LPC_SWM->PINENABLE0 = 0xffffffbfUL;
}


int main(void)
{

  char iobusbuf[64] = "\0kimmo was here!" ;

  /* Configure the core clock/PLL via CMSIS */
  SystemCoreClockUpdate();

  /* Initialise the GPIO block */
  gpioInit();

  /* Initialise the UART0 block for printf output */
  uart0Init(115200);

  /* Initialise UART1 for IOBUS */
  uart1Init(200000);

  /* Configure the multi-rate timer for 1ms ticks */
  mrtInit(SystemCoreClock/1000);

  /* Configure the switch matrix (setup pins for UART0 and GPIO) */
  configurePins();

  /* Set the TXEN pin to output (1 = output, 0 = input) */
  LPC_GPIO_PORT->DIR0 |= (1 << TXEN_IO);

  printf("SystemCoreClock %d\r\n", SystemCoreClock );
  printf("MainClock %d\r\n", MainClock );
  printf("LPC_SYSCON->UARTFRGMULT %d\r\n", LPC_SYSCON->UARTFRGMULT);
  printf("LPC_SYSCON->UARTFRGDIV %d\r\n", LPC_SYSCON->UARTFRGDIV);
  printf("LPC_USART0->BRG %d\r\n", LPC_USART0->BRG);
  printf("LPC_SYSCON->SYSAHBCLKDIV %d\r\n", LPC_SYSCON->SYSAHBCLKDIV);
  printf("LPC_SYSCON->SYSPLLCTRL %x\r\n", LPC_SYSCON->SYSPLLCTRL);
  while(1)
  {

    /* Just insert a 1 second delay */
    mrtDelay(1000);

    /* Send some text (printf is redirected to UART0) */
    printf(".");

    /* Send something over RS-485 */
    uart1Send( iobusbuf, 16);
    uart0Send( iobusbuf, 16);

  }
}
