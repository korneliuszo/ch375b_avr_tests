/*
 * ch375bbridge.cpp
 *
 *  Created on: May 31, 2023
 *      Author: Korneliusz Osmenda
 */

#include <avr/io.h>
#include <util/delay.h>

static inline unsigned char rx(void)
{
	while ( !(UCSR0A & (1<<RXC0)) );
	return UDR0;
}

static inline void tx(unsigned char byte)
{
	while ( !( UCSR0A & (1<<UDRE0)) );
	UDR0 = byte;
}


//internal use macro
#define CONCAT(a, b) a ## b
#define CONCAT3(a, b, c) a ## b ## c
#define CALL(macro, ...)  macro(__VA_ARGS__)


#define PORTREG(No,pin) CONCAT(PORT, No)
#define PINREG(No,pin) CONCAT(PIN, No)
#define DDRREG(No,pin) CONCAT(DDR, No)
#define TMPREG(No,pin) CONCAT(TMP, No)
#define PINNO(No,pin) pin

#define COUT(No,pin) DDRREG(No,pin) |= (1<<PINNO(No,pin))
#define CIN(No,pin) DDRREG(No,pin) &= ~(1<<PINNO(No,pin))
#define V1(No,pin) PORTREG(No,pin) |= (1<<PINNO(No,pin))
#define V0(No,pin) PORTREG(No,pin) &= ~(1<<PINNO(No,pin))
#define T1(No,pin) TMPREG(No,pin) |= (1<<PINNO(No,pin))


#define ARD_A0 C, 0
#define ARD_A1 C, 1
#define ARD_A2 C, 2
#define ARD_A3 C, 3
#define ARD_A4 C, 4
#define ARD_A5 C, 5
#define ARD_D0 D, 0 //USED
#define ARD_D1 D, 1 //USED
#define ARD_D2 D, 2 //USED
#define ARD_D3 D, 3
#define ARD_D4 D, 4
#define ARD_D5 D, 5
#define ARD_D6 D, 6
#define ARD_D7 D, 7
#define ARD_D8 B, 0 // USED after switch
#define ARD_D9 B, 1
#define ARD_D10 B, 2
#define ARD_D11 B, 3
#define ARD_D12 B, 4 //USED
#define ARD_D13 B, 5 //USED

#define B_WR ARD_D11
#define B_RD ARD_D10
#define B_CS ARD_D8
#define B_A0 ARD_D9
#define B_D0 ARD_D7
#define B_D1 ARD_D6
#define B_D2 ARD_D5
#define B_D3 ARD_D4
#define B_D4 ARD_D3
#define B_D5 ARD_A5
#define B_D6 ARD_A4
#define B_D7 ARD_A3


template <char regnum>
consteval uint8_t set_masks()
{
	uint8_t TMPB=0;
	uint8_t TMPC=0;
	uint8_t TMPD=0;
	CALL(T1,B_D0);
	CALL(T1,B_D1);
	CALL(T1,B_D2);
	CALL(T1,B_D3);
	CALL(T1,B_D4);
	CALL(T1,B_D5);
	CALL(T1,B_D6);
	CALL(T1,B_D7);
	if(regnum == 'B')
		return TMPB;
	else if (regnum == 'C')
		return TMPC;
	else if (regnum == 'D')
		return TMPD;
}

constexpr uint8_t mb = set_masks<'B'>();
constexpr uint8_t mc = set_masks<'C'>();
constexpr uint8_t md = set_masks<'D'>();

template<uint8_t dstbit, uint8_t srcbit>
uint8_t inline set_bit(uint8_t dst,uint8_t src)
{
	asm(
	  "bst %[src],%[srcbit]\n\t"
	  "bld %[dst],%[dstbit]\n\t"
	: [dst] "=r" (dst)
	: "0" (dst),
	  [src] "r" (src),
	  [srcbit] "I" (srcbit),
	  [dstbit] "I" (dstbit)
	: "cc"
	);
	return dst;
}

void set_byte(uint8_t in)
{
	uint8_t TMPB=0;
	uint8_t TMPC=0;
	uint8_t TMPD=0;

	CALL(TMPREG,B_D0) = set_bit<CALL(PINNO,B_D0),0>(CALL(TMPREG,B_D0),in);
	CALL(TMPREG,B_D1) = set_bit<CALL(PINNO,B_D1),1>(CALL(TMPREG,B_D1),in);
	CALL(TMPREG,B_D2) = set_bit<CALL(PINNO,B_D2),2>(CALL(TMPREG,B_D2),in);
	CALL(TMPREG,B_D3) = set_bit<CALL(PINNO,B_D3),3>(CALL(TMPREG,B_D3),in);
	CALL(TMPREG,B_D4) = set_bit<CALL(PINNO,B_D4),4>(CALL(TMPREG,B_D4),in);
	CALL(TMPREG,B_D5) = set_bit<CALL(PINNO,B_D5),5>(CALL(TMPREG,B_D5),in);
	CALL(TMPREG,B_D6) = set_bit<CALL(PINNO,B_D6),6>(CALL(TMPREG,B_D6),in);
	CALL(TMPREG,B_D7) = set_bit<CALL(PINNO,B_D7),7>(CALL(TMPREG,B_D7),in);

	PORTB = (PORTB & ~mb) | TMPB;
	PORTC = (PORTC & ~mc) | TMPC;
	PORTD = (PORTD & ~md) | TMPD;
}

uint8_t get_byte()
{
	uint8_t TMPB=PINB;
	uint8_t TMPC=PINC;
	uint8_t TMPD=PIND;

	uint8_t byte = 0;
	byte = set_bit<0,CALL(PINNO,B_D0)>(byte,CALL(TMPREG,B_D0));
	byte = set_bit<1,CALL(PINNO,B_D1)>(byte,CALL(TMPREG,B_D1));
	byte = set_bit<2,CALL(PINNO,B_D2)>(byte,CALL(TMPREG,B_D2));
	byte = set_bit<3,CALL(PINNO,B_D3)>(byte,CALL(TMPREG,B_D3));
	byte = set_bit<4,CALL(PINNO,B_D4)>(byte,CALL(TMPREG,B_D4));
	byte = set_bit<5,CALL(PINNO,B_D5)>(byte,CALL(TMPREG,B_D5));
	byte = set_bit<6,CALL(PINNO,B_D6)>(byte,CALL(TMPREG,B_D6));
	byte = set_bit<7,CALL(PINNO,B_D7)>(byte,CALL(TMPREG,B_D7));

	return byte;
}

void switch_to_rx()
{
	PORTB |= mb;
	PORTC |= mc;
	PORTD |= md;

	// all bus input
	DDRB = DDRB & ~mb;
	DDRC = DDRC & ~mc;
	DDRD = DDRD & ~md;
}

int main()
{

#define BAUD 76800
#include <util/setbaud.h>

	UCSR0A = 0;

	UBRR0H = UBRRH_VALUE;
	UBRR0L = UBRRL_VALUE;
	#if USE_2X
	UCSR0A |= (1 << U2X0);
	#else
	UCSR0A &= ~(1 << U2X0);
	#endif

	UCSR0B = (1<<RXEN0) | (1<<TXEN0);
	UCSR0C = (1<<UCSZ01) | (1<<UCSZ00);


	CALL(V1,B_WR);
	CALL(V1,B_RD);
	CALL(V1,B_CS);
	CALL(V1,B_A0);
	CALL(COUT,B_WR);
	CALL(COUT,B_RD);
	CALL(COUT,B_CS);
	CALL(COUT,B_A0);

	switch_to_rx();

	while(1)
	{
		switch(rx())
		{
			default:
			case 0x00: //RESET_STATE
				break;
			case 'r':
			{
				CALL(V0,B_A0);
				asm volatile("nop");
				CALL(V0,B_CS);
				CALL(V0,B_RD);
				asm volatile("nop");
				uint8_t byte = get_byte();
				CALL(V1,B_RD);
				CALL(V1,B_CS);
				asm volatile("nop");
				tx(byte);
				break;
			}
			case 'R':
			{
				CALL(V1,B_A0);
				asm volatile("nop");
				CALL(V0,B_CS);
				CALL(V0,B_RD);
				asm volatile("nop");
				uint8_t byte = get_byte();
				CALL(V1,B_RD);
				CALL(V1,B_CS);
				asm volatile("nop");
				tx(byte);
				break;
			}
			case 'w':
			{
				uint8_t byte = rx();
				set_byte(byte);
				DDRB = DDRB | mb;
				DDRC = DDRC | mc;
				DDRD = DDRD | md;
				CALL(V0,B_A0);
				asm volatile("nop");
				CALL(V0,B_CS);
				CALL(V0,B_WR);
				asm volatile("nop");
				CALL(V1,B_WR);
				CALL(V1,B_CS);
				asm volatile("nop");
				switch_to_rx();
				break;
			}
			case 'W':
			{
				uint8_t byte = rx();
				set_byte(byte);
				DDRB = DDRB | mb;
				DDRC = DDRC | mc;
				DDRD = DDRD | md;
				CALL(V1,B_A0);
				asm volatile("nop");
				CALL(V0,B_CS);
				CALL(V0,B_WR);
				asm volatile("nop");
				CALL(V1,B_WR);
				CALL(V1,B_CS);
				asm volatile("nop");
				switch_to_rx();
				break;
			}
		}
	}
}
