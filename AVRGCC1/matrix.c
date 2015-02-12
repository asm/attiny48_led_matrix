#define F_CPU 8000000UL
#include <inttypes.h>
#include <stdlib.h>
#include <string.h>
#include <avr/io.h>
#include <avr/interrupt.h>
#include <avr/eeprom.h>
#include <util/delay.h>
#include <avr/pgmspace.h>

static volatile uint16_t counter = 0;           // used for delay function
static uint8_t screen_mem[8];			        // screen memory
static uint8_t buffer[60];                      // stores the active message or sprite
static uint8_t active_row;			            // active row

const prog_uint8_t PROGMEM heart4[8] =
{
  0b00000000,
  0b01100110,
  0b10111101,
  0b10000001,
  0b10000001,
  0b01000010,
  0b00100100,
  0b00011000 
};

const prog_uint8_t PROGMEM heart3[8] =
{
  0x00,    // ________ 
  0x66,    // _XX__XX_
  0xFF,    // XXXXXXXX
  0xFF,    // XXXXXXXX
  0xFF,    // XXXXXXXX
  0x7E,    // _XXXXXX_
  0x3C,    // __XXXX__
  0x18     // ___XX___
};

const prog_uint8_t PROGMEM heart2[8] =
{
  0b00000000,
  0b00000000,
  0b00100100,
  0b01111110,
  0b00111100,
  0b00011000,
  0b00000000,
  0b00000000 
};

const prog_uint8_t PROGMEM heart1[8] =
{
  0b00000000,
  0b00000000,
  0b00000000,
  0b00011000,
  0b00011000,
  0b00000000,
  0b00000000,
  0b00000000 
};

const prog_uint8_t PROGMEM test[8] =
{
  0xFF,  
  0xFF,  
  0xFF,  
  0xFF,  
  0x0F,  
  0xFF,  
  0xFF,  
  0xFF,   
};

/*
 * delay_ms
 * Uses the counter that is incremented by the ISR.
 * Max delay is 32767ms.
 */
void delay_ms(uint16_t delay) {
	uint16_t t = delay * 2;
	counter = 0;
	while (counter < t) {}
}

void display_active_row(void) {
	uint8_t col;

	active_row = (active_row + 1) % 8;
	col = screen_mem[active_row];

    /*
	PORTC = col;
	PORTB = col & 0x40;
	PORTD = ~(1 << active_row);
	*/
	// Start with a clean slate
	PORTB = 1 << PB5 | 1 << PB6;
	PORTC = 1 << PC2 | 1 << PC4 | 1 << PC5 | 1 << PC7;
	PORTD = 1 << PD0 | 1 << PD3;
	
	// Row
	if ((col & 0x80) == 0x80) {
		PORTC &= ~(1 << PC2);
	}
	if ((col & 0x40) == 0x40) {
		PORTC &= ~(1 << PC7);    
	}
	if ((col & 0x20) == 0x20) {
		PORTC &= ~(1 << PC4);    
	}
	if ((col & 0x10) == 0x10) {
		PORTB &= ~(1 << PB5);    
	}
	if ((col & 0x08) == 0x08) {
		PORTD &= ~(1 << PD3);    
	}
	if ((col & 0x04) == 0x04) {
		PORTC &= ~(1 << PC5);    
	}
	if ((col & 0x02) == 0x02) {
		PORTB &= ~(1 << PB6);    
	}
	if ((col & 0x01) == 0x01) {
		PORTD &= ~(1 << PD0);    
	}
	
	// Column
	switch (active_row) {
		case 0:
			PORTD |= 1 << PD4;
			break;
		case 1:
			PORTC |= 1 << PC0;
			break;
		case 2:
			PORTC |= 1 << PC1;
			break;
		case 3:
			PORTD |= 1 << PD1;
			break;
		case 4:
			PORTC |= 1 << PC3;
			break;
		case 5:
			PORTD |= 1 << PD2;
			break;
		case 6:
			PORTB |= 1 << PB7;
			break;
		case 7:
			PORTD |= 1 << PD5;
			break;
	}
}

//void interrupt [TIMER0_OVF0_vect] ISR_TOV0 (void)
ISR(TIMER0_OVF_vect) {	
	display_active_row();
	counter++;
}

/*
 * copy_to_display
 * Copies sprite data to the screen memory at the given position. 
 */
void copy_to_display(int8_t x, int8_t y, uint8_t sprite[8]) {
	int8_t i, t;
	uint8_t row;

	for (i = 0; i < 8; i++) {
		t = i-y;
		row = ((t >= 0) && (t < 8)) ? sprite[t] : 0x00;
		row = (x >= 0) ? (row >> x) : (row << -x);
		screen_mem[i] = row;
	}
}

/*
 * copy_to_buffer
 * Copies the given sprite from PROGMEM to RAM.
 */
void copy_to_buffer(const prog_uint8_t sprite[8]) {
  memcpy_P(buffer, sprite, 8);
}

int main (void) {
	int delay = 35;
	
	// timer 0 setup, prescaler 8
	TCCR0A =  1 << CS00 | 1 << CS01;
 
	// clear pending interrupts
	TIFR0 = 1<<TOV0;

	// enable timer 0 interrupt
	TIMSK0 = (1 << TOIE0);

	DDRC = 0xFF;
	PORTC = 0x00;

	DDRD = 0xFF;
	PORTD = 0x00;

	DDRB = 0xFF;
	PORTB = 0x00;

	

	sei();

	while (1) {
		copy_to_buffer(heart1);
		copy_to_display(0, 0, buffer);
		
		delay_ms(delay);
		
		copy_to_buffer(heart2);
		copy_to_display(0, 0, buffer);
		
		delay_ms(delay);
		
		copy_to_buffer(heart3);
		copy_to_display(0, 0, buffer);
		
		delay_ms(delay);
		
		copy_to_buffer(heart4);
		copy_to_display(0, 0, buffer);
		
		delay_ms(delay);
		
		copy_to_buffer(heart3);
		copy_to_display(0, 0, buffer);
		
		delay_ms(delay);
		
		copy_to_buffer(heart2);
		copy_to_display(0, 0, buffer);
		
		delay_ms(delay);
	}
	return 0;
}