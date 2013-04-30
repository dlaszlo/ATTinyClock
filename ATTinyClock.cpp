#include <avr/io.h>
#include <avr/interrupt.h>
#include <avr/wdt.h>
#include <util/delay.h>
#include "OneWire.h"
#include "USI_TWI_Master.h"

#define PORT_SHIFT_OUT PORTB
#define DS_PIN PINB0
#define LATCH_PIN PINB1
#define CLOCK_PIN PINB2

#define cbi(sfr, bit) (_SFR_BYTE(sfr) &= ~_BV(bit))
#define sbi(sfr, bit) (_SFR_BYTE(sfr) |= _BV(bit))

#define PCF8583_WRITE_ADDRESS ( 0xA0 & ~(0x01) )
#define PCF8583_READ_ADDRESS  ( PCF8583_WRITE_ADDRESS | 0x01 )

#define CHAR_C 10
#define CHAR_EMPTY 11
#define CHAR_MINUS 12
#define BLINK_DELAY 8

#define DOT_MASK 0b00100000

uint8_t charset[] =
{ 0b00101000, 0b11101110, 0b00110010, 0b10100010, 0b11100100, 0b10100001,
		0b00100001, 0b11101010, 0b00100000, 0b10100000, 0b00111001, 0b11111111,
		0b11110111 };

uint8_t pos[] =
{ 0b00001110, 0b00001101, 0b00001011, 0b00000111 };

uint8_t tempfract[] =
{ 0, 0, 1, 2, 3, 3, 4, 4, 5, 6, 6, 7, 8, 8, 9, 9 };

uint8_t buffer_i2c[9] =
{ 0 };

volatile int8_t hour;
volatile int8_t minute;
volatile int8_t second;

volatile uint8_t tick = 0;
volatile uint8_t display[4] =
{ 0 };
volatile uint8_t dot = 0;
volatile uint8_t current_char = 0;
volatile uint8_t blink = 0;

void shift_out(uint8_t d)
{
	uint8_t i;
	for (i = 0; i < 8; i++)
	{
		if ((d << i) & _BV(7))
		{
			PORT_SHIFT_OUT |= _BV(DS_PIN);
		}
		else
		{
			PORT_SHIFT_OUT &= ~_BV(DS_PIN);
		}
		PORT_SHIFT_OUT |= _BV(CLOCK_PIN);
		PORT_SHIFT_OUT &= ~_BV(CLOCK_PIN);
	}
}

void display_char(uint8_t p, uint8_t n)
{
	PORTB &= ~_BV(LATCH_PIN);
	shift_out(pos[p]);
	uint8_t c = charset[n];
	if (((p < 2 && (blink & 1))
			|| (p >= 2 && (blink & 2)))
			&& (tick & 1) && tick < BLINK_DELAY)
	{
		c = charset[CHAR_EMPTY];
	}
	if (tick >= BLINK_DELAY)
	{
		blink = 0;
	}
	if (dot && p == 1)
	{
		if (blink || !(dot & 2) || (tick & 1))
		{
			c ^= DOT_MASK;
		}
	}
	shift_out(c);
	PORTB |= _BV(LATCH_PIN);
}

void set_display(uint8_t h, uint8_t l, uint8_t d)
{
	cli();
	display[0] = (h / 10) % 10;
	display[1] = h % 10;
	display[2] = (l / 10) % 10;
	display[3] = l % 10;
	dot = d;
	sei();
}

void get_clock(void)
{
	buffer_i2c[0] = PCF8583_WRITE_ADDRESS;
	buffer_i2c[1] = 0x01; // start of time data
	USI_TWI_Start_Transceiver_With_Data(buffer_i2c, 2);
	buffer_i2c[0] = PCF8583_READ_ADDRESS;
	USI_TWI_Start_Transceiver_With_Data(buffer_i2c, 7);
	second = (buffer_i2c[2] & 0x0F) + (buffer_i2c[2] >> 4) * 10;
	minute = ((buffer_i2c[3] & 0x0F) + (buffer_i2c[3] >> 4) * 10);
	hour = ((buffer_i2c[4] & 0x0F) + (buffer_i2c[4] >> 4) * 10);
}

void set_clock(void)
{
	buffer_i2c[0] = PCF8583_WRITE_ADDRESS;
	buffer_i2c[1] = 0x01; // start of time data
	buffer_i2c[2] = 0; /* set 1/100 seconds to 0 */
	buffer_i2c[3] = 0;
	buffer_i2c[4] = (((minute / 10) << 4) | (minute % 10));
	buffer_i2c[5] = (((hour / 10) << 4) | (hour % 10));
	buffer_i2c[6] = 0;
	buffer_i2c[7] = 0;
	USI_TWI_Start_Transceiver_With_Data(buffer_i2c, 8);
}

void setup()
{
	DDRB = 0b00000111;
	DDRD = 0b00001100;
	PORTD = 0b00001100;

	// Timer
	TIMSK |= (1 << TOIE0);
	TCCR1B |= (1 << WGM12);
	TIMSK |= (1 << OCIE1A);
	OCR1A = 15625;

	TCCR0B |= (1 << CS01);
	TCCR1B |= (1 << CS11) + (1 << CS10);

	sei();

	USI_TWI_Master_Initialise();

	if (!ow_reset())
	{
		// Skip ROM command
		ow_write(0xCC);
		// Start Conversion command
		ow_write(0x44);
	}

}

void settings(void)
{
	if (!(PIND & (1 << PD2)))
	{
		_delay_ms(10);
		cli();
		if (!(PIND & (1 << PD2)))
		{
			hour++;
			if (hour == 24)
			{
				hour = 0;
			}
			set_clock();
			tick = 0;
			blink = 1;
		}
		sei();
	}

	if (!(PIND & (1 << PD3)))
	{
		_delay_ms(10);
		cli();
		if (!(PIND & (1 << PD3)))
		{
			minute++;
			if (minute == 60)
			{
				minute = 0;
			}
			set_clock();
			tick = 0;
			blink = 2;
		}
		sei();
	}
}

void thermometer(void)
{
	if (!ow_reset())
	{
		// Skip ROM command
		ow_write(0xCC);
		// Start Conversion command
		ow_write(0x44);
		_delay_ms(100);
		if (!ow_reset())
		{
			// Skip ROM command
			ow_write(0xCC);
			// Read Scratchpad command
			ow_write(0xBE);
			uint8_t low = ow_read();
			uint8_t high = ow_read();
			for (int i = 0; i < 7; i++)
			{
				ow_read();
			}
			uint16_t temp = (high << 8) | low;
			uint8_t f = temp & 15;
			temp >>= 4;

			cli();
			dot = 1;
			if (temp & 0b100000000000)
			{
				display[0] = CHAR_MINUS;
				temp = 0b111111111111 ^ temp;
				dot = 0;
			}
			display[1] = (temp / 10) % 10;
			display[2] = temp % 10;
			if (temp > 99)
			{
				display[0] = (temp / 100) % 10;
				dot = 0;
			}
			else if (dot)
			{
				display[0] = (temp / 10) % 10;
				display[1] = temp % 10;
				display[2] = tempfract[f];
				dot = 1;
			}
			display[3] = CHAR_C;
			sei();
		}
	}
}

int main(void)
{
	setup();
	for (;;)
	{
		get_clock();
		settings();
		int current_display = (tick >> 4) & 3;
		if (current_display < 2)
		{
			set_display(hour, minute, 2);
		}
		else if (current_display == 2)
		{
			set_display(minute, second, 1);
		}
		else
		{
			thermometer();
		}
		_delay_ms(100);
	}
}

ISR(TIMER0_OVF_vect)
{	// multiplex code
	display_char(current_char, display[current_char]);
	current_char++;
	current_char &= 3;
}

ISR(TIMER1_COMPA_vect)
{	// 1 Hz counter
	tick++;
	tick &= 255;
}

