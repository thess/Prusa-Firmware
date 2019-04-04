//timer02.c
// use atmega timer2 as main system timer instead of timer0
// timer0 is used for fast pwm (OC0B output)
// original OVF handler is disabled

#include <avr/io.h>
#include <avr/interrupt.h>
#include "system_timer.h"

#ifdef SYSTEM_TIMER_2

uint8_t timer02_pwm0 = 0;

void timer02_set_pwm0(uint8_t pwm0)
{
	if (timer02_pwm0 == pwm0) return;
	if (pwm0)
	{
		TCCR0A |= (2 << COM0B0);
		OCR0B = pwm0 - 1;
	}
	else
	{
		TCCR0A &= ~(2 << COM0B0);
		OCR0B = 0;
	}
	timer02_pwm0 = pwm0;
}

extern void soft_pwm_isr();
extern volatile bool in_temp_isr, temp_isr_enable;

void timer02_init(void)
{
	//save sreg
	uint8_t _sreg = SREG;
	//disable interrupts for sure
	cli();
	//mask timer0 interrupts - disable all
	TIMSK0 &= ~(1<<TOIE0);
	TIMSK0 &= ~(1<<OCIE0A);
	TIMSK0 &= ~(1<<OCIE0B);
	//setup timer0
	TCCR0A = 0x00; //COM_A-B=00, WGM_0-1=00
	TCCR0B = (1 << CS00); //WGM_2=0, CS_0-2=011
	//switch timer0 to fast pwm mode
	TCCR0A |= (3 << WGM00); //WGM_0-1=11
	//set OCR0B register to zero
	OCR0B = 0;
	//disable OCR0B output (will be enabled in timer02_set_pwm0)
	TCCR0A &= ~(2 << COM0B0);
	//setup timer3 CTC
	TCCR3A = (1 << WGM30); //COM_A-C=00, WGM30=01
	TCCR3B = (1 << WGM32) | (3 << CS30); //WGM32=01, CS_0-2=011
	//set timer3 OCRA registers
	OCR3A = 0;
	TCNT3 = 0;
	//mask timer3 interrupts - enable TOV, disable others
	TIMSK3 = (1 << TOIE3);
	// Enable temperature pwm int
	in_temp_isr = false;
	temp_isr_enable = true;
	//restore sreg (enable interrupts)
	SREG = _sreg;
}


//following code is COMPA handler for timer 3
//it is copy-paste from wiring.c and modified for timer3
//variables timer0_overflow_count and timer0_millis are declared in wiring.c



// the prescaler is set so that timer3 ticks every 64 clock cycles, and the
// the compare handler is called every 256 ticks.
#define MICROSECONDS_PER_TIMER0_OVERFLOW (clockCyclesToMicroseconds(64 * 256))

// the whole number of milliseconds per timer0 overflow
#define MILLIS_INC (MICROSECONDS_PER_TIMER0_OVERFLOW / 1000)

// the fractional number of milliseconds per timer0 overflow. we shift right
// by three to fit these numbers into a byte. (for the clock speeds we care
// about - 8 and 16 MHz - this doesn't lose precision.)
#define FRACT_INC ((MICROSECONDS_PER_TIMER0_OVERFLOW % 1000) >> 3)
#define FRACT_MAX (1000 >> 3)

//extern volatile unsigned long timer0_overflow_count;
//extern volatile unsigned long timer0_millis;
//unsigned char timer0_fract = 0;
volatile unsigned long timer2_overflow_count;
volatile unsigned long timer2_millis;
unsigned char timer2_fract = 0;

ISR(TIMER3_OVF_vect)
{
	// copy these to local variables so they can be stored in registers
	// (volatile variables must be read from memory on every access)
	unsigned long m = timer2_millis;
	unsigned char f = timer2_fract;
	m += MILLIS_INC;
	f += FRACT_INC;
	if (f >= FRACT_MAX)
	{
		f -= FRACT_MAX;
		m += 1;
	}
	timer2_fract = f;
	timer2_millis = m;
	timer2_overflow_count++;

	// Temp & fans soft pwm driven by system clock (~1ms)
	// NOTE: interrupts are (re-)enabled in this routine
	if (temp_isr_enable)
		soft_pwm_isr();
}

unsigned long millis2(void)
{
	unsigned long m;
	uint8_t oldSREG = SREG;

	// disable interrupts while we read timer0_millis or we might get an
	// inconsistent value (e.g. in the middle of a write to timer0_millis)
	cli();
	m = timer2_millis;
	SREG = oldSREG;

	return m;
}

unsigned long micros2(void)
{
	unsigned long m;
	uint8_t oldSREG = SREG, t;
	cli();
	m = timer2_overflow_count;
	t = TCNT3L;
	if ((TIFR3 & _BV(TOV3)) && (t < 255))
		m++;
	SREG = oldSREG;	
	return ((m << 8) + t) * (64 / clockCyclesPerMicrosecond());
}

void delay2(unsigned long ms)
{
	uint32_t start = micros2();
	while (ms > 0)
	{
		yield();
		while ( ms > 0 && (micros2() - start) >= 1000)
		{
			ms--;
			start += 1000;
		}
	}
}

#endif //SYSTEM_TIMER_2
