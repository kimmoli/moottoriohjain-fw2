/**************************************************************************/
/*!
    @file     uart.c
    @author   K. Townsend

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
#include <string.h>
#include <stdio.h>

#include "uart.h"
#include "mrt.h"

volatile uint16_t rxBuf[128];
volatile uint32_t rxCount;
volatile uint32_t rxRead;


void UART1_IRQHandler(void)
{
	int i;

	NVIC_DisableIRQ(UART1_IRQn);
    /* Clear UART1 RXRDYEN interrupt */
	LPC_USART1->INTENCLR |= UART_INT_RXRDYEN;

	rxBuf[rxCount] = (LPC_USART1->RXDATA_STAT & 0x1FF);

 	if ((LPC_USART1->STAT & 0x0002))
 		rxBuf[rxCount] |=  0x0200;

	rxCount++;
	rxCount &= 0x7F;

//	printf("%d %d\r\n", rxCount, rxRead);

	NVIC_EnableIRQ(UART1_IRQn);
	/* Enable UART1 RXRDYEN interrupt */
	LPC_USART1->INTENSET |= UART_INT_RXRDYEN;


}

void uart0Init(uint32_t baudRate)
{
  uint32_t clk;

  /* Setup the clock and reset UART0 */
  LPC_SYSCON->UARTCLKDIV = 1;
  NVIC_DisableIRQ(UART0_IRQn);
  LPC_SYSCON->SYSAHBCLKCTRL |=  (1 << 14);
  LPC_SYSCON->PRESETCTRL    &= ~(1 << 3);
  LPC_SYSCON->PRESETCTRL    |=  (1 << 3);

  /* Configure UART0 */
  clk = MainClock/LPC_SYSCON->UARTCLKDIV;
  LPC_USART0->CFG = UART_DATA_LENGTH_8 | UART_PARITY_NONE | UART_STOP_BIT_1;
  LPC_USART0->BRG = clk / 16 / baudRate - 1;
  LPC_SYSCON->UARTFRGDIV = 0xFF;
  LPC_SYSCON->UARTFRGMULT = (((clk / 16) * (LPC_SYSCON->UARTFRGDIV + 1)) /
    (baudRate * (LPC_USART0->BRG + 1))) - (LPC_SYSCON->UARTFRGDIV + 1);

  /* Clear the status bits */
  LPC_USART0->STAT = UART_STATUS_CTSDEL | UART_STATUS_RXBRKDEL;

  /* Enable UART0 interrupt */
  NVIC_EnableIRQ(UART0_IRQn);

  /* Enable UART0 */
  LPC_USART0->CFG |= UART_ENABLE;
}

void uart1Init(uint32_t baudRate)
{
  uint32_t clk;

  NVIC_DisableIRQ(UART1_IRQn);

  LPC_SYSCON->SYSAHBCLKCTRL |=  (1 << 15);
  LPC_SYSCON->PRESETCTRL    &= ~(1 << 4);
  LPC_SYSCON->PRESETCTRL    |=  (1 << 4);

  /* Configure UART1 */
  clk = MainClock/LPC_SYSCON->UARTCLKDIV;
  LPC_USART1->CFG = UART_DATA_LENGTH_9 | UART_PARITY_NONE | UART_STOP_BIT_1;
  //LPC_USART1->CTRL |= UART_ADDRDET;
  LPC_USART1->BRG = clk / 16 / baudRate - 1;

  /* Clear the status bits */
  LPC_USART1->STAT = UART_STATUS_CTSDEL | UART_STATUS_RXBRKDEL;

  /* Enable UART0 interrupt */
  NVIC_EnableIRQ(UART1_IRQn);

  /* Enable UART0 */
  LPC_USART1->CFG |= UART_ENABLE;

  /* Enable UART1 RXRDYEN interrupt */
  LPC_USART1->INTENSET |= UART_INT_RXRDYEN;

  rxCount = 0;
  thisIsForMe = 0;
}


void uart0SendChar(char buffer)
{
  /* Wait until we're ready to send */
  while (!(LPC_USART0->STAT & UART_STATUS_TXRDY));
  LPC_USART0->TXDATA = buffer;
}

void uart1SendChar(char buffer, int addrbit)
{
  /* Wait until we're ready to send */
  while (!(LPC_USART1->STAT & UART_STATUS_TXRDY));
  LPC_USART1->TXDATA = buffer | (addrbit & 1) << 8;
}


void uart0Send(char *buffer, uint32_t length)
{
  while (length != 0)
  {
    uart0SendChar(*buffer);
    buffer++;
    length--;
  }
}

void uart1Send(char *buffer, uint32_t length)
{
	int first = 1;

	/* Enable transmitter */
	LPC_GPIO_PORT->SET0 = 1 << TXEN_IO;

	/* Send buffer */

	while (length != 0)
	{
		uart1SendChar(*buffer, first);
		first = 0;
		buffer++;
		length--;
	}

	/* Wait until transmitter becomes idle */
	while (!(LPC_USART1->STAT & UART_STATUS_TXIDLE));

	/* Disable transmitter */
	LPC_GPIO_PORT->CLR0 = 1 << TXEN_IO;

}

