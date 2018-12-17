/*
 * echo.c
 *
 * Created: 13-12-2018 19:38:46
 * Author : ERTS 1
 */ 

#define F_CPU 14745600
#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/delay.h>

#define RX  (1<<4)
#define TX  (1<<3)
#define TE  (1<<5)
#define RE  (1<<7)

volatile unsigned char data;

void uart0_init()
{
	UCSR0B = 0x00;							//disable while setting baud rate
	UCSR0A = 0x00;
	UCSR0C = 0x06;
	UBRR0L = 0x5F; 							//9600BPS at 14745600Hz
	UBRR0H = 0x00;
	UCSR0B = 0x98;
	//UCSR0C = 3<<1;							//setting 8-bit character and 1 stop bit
	//UCSR0B = RX | TX;
}


void uart_tx(char data)
{
	while(!(UCSR0A & TE));						//waiting to transmit
	UDR0 = data;
}

ISR(USART0_RX_vect)
{
	data = UDR0;
}

char uart_rx()
{
	while(!(UCSR0A & RE));						//waiting to receive
	return UDR0;
}


int main(void)
{
    /* Replace with your application code */
    while (1) 
    {
    }
}

