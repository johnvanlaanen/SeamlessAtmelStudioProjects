/*
 * PopTest1.c
 *
 * Created: 9/25/2013 6:29:32 PM
 *  Author: John
 */ 


#include <avr/io.h>
#include <avr/interrupt.h>

#include "moduleIO.h"
#include "PopTest1.h"

#ifdef USE_TIMER1_ISR
/************************************
The timer 1 interrupt service routine
Simply Toggle the output port
************************************/
static volatile uint16_t icnt=0;

ISR(TIM1_COMPA_vect)
{
	static uint8_t control_state=0;
	
	icnt += 1;	// will wrap
	
	// Startup delay - lets voltages settle
	if(control_state == 0) {
		control_state = 1;
		digitalWrite(kPIN_BOOST_CTRL3, HIGH);
	} else {
		control_state = 0;
		digitalWrite(kPIN_BOOST_CTRL3, LOW);
	}
}

	void PulseActuate(void) {
		digitalWrite(kPIN_ACTUATE, HIGH);
		icnt = 0;
		while(icnt < kActuatePulseIntCount) {}
		digitalWrite(kPIN_ACTUATE, LOW);
	}

#else
	void PulseActuate(void) {
		digitalWrite(kPIN_ACTUATE, HIGH);
		for( uint16_t icnt=0; icnt<20; icnt++ ) {
			for(uint16_t icnt2=0; icnt2<2013; icnt2++) {}	// about 1mS
		}
		digitalWrite(kPIN_ACTUATE, LOW);
	}

#endif


void FireSolenoid(void)
{
	uint8_t fcnt;
	for(fcnt=0; fcnt<kNumTriggerFires; fcnt++) {
		while( analogRead(KPin_BoostedVSense) < kFullChargeADCValue) {}
		PulseActuate();
	}
}

int main(void)
{
	// set up the i/o ports: set the output values
	digitalWrite(kPIN_ACTUATE,			LOW);
	digitalWrite(kPIN_BOOST_CTRL3,		LOW);
	digitalWrite(kPIN_LED_ON_L,			HIGH);
	
	// set up the i/o ports: set the port directions
	pinMode(kPIN_ACTUATE,		OUTPUT);
	pinMode(kPIN_IODATA,		INPUT_PULLUP);
	pinMode(kPIN_LED_ON_L,		OUTPUT);
	
	pinMode(kPIN_BOOST_CTRL1,	INPUT);
	pinMode(kPIN_BOOST_CTRL2,	INPUT);
	pinMode(kPIN_BOOST_CTRL3,	OUTPUT);

#ifdef USE_TIMER1_ISR
	// set up Timer 1 to generate a periodic interrupt for toggling the voltage booster switch
	OCR1AH = 0;		// have to load both high and low halves, with the high first
	OCR1AL = kBOOST_TIMER_COUNT_VALUE;								// sets the A compare value for clock generation
	TIMSK1 = (1 << OCIE1A);											// enable interrupt on compare A match

	TCCR1A = 0;
	TCCR1B = (1 << WGM12) | (kBOOST_TIMER_PRESCALE_SELECT << CS10);	// set timer CTC mode and prescaler divider ratio (turns on timer)
	sei();	// enable interrupts
#else
	// set up 8-bit timer 0 to generate a clock output on kPIN_TRIPLER_SWITCH3
	TCCR0A = (1<<COM0A0) | (1<<WGM01);				// Toggle OC0A on match, and CTC mode
	OCR0A = kBOOST_TIMER_COUNT_VALUE;				// sets the A compare value for clock generation
	TCNT0 = 0;										// make sure the count starts at 0 so no short first cycle
	GTCCR = (1<<PSR10);								// reset the prescaler
	TCCR0B = (kBOOST_TIMER_PRESCALE_SELECT<<CS00);	// set timer prescaler divider (turns on timer
#endif

	// set up the ADC
	ADCSRA = (1<<ADEN) | (7<<ADPS0); // Set the ADC clock prescaler to divide by 128, and enable the ADC
	
    while(1)
    {
		if( analogRead(KPin_BoostedVSense) > kFullChargeADCValue) {
			digitalWrite(kPIN_LED_ON_L, LOW);
		} else {
			digitalWrite(kPIN_LED_ON_L, HIGH);
		}
		
		if(digitalRead(kPIN_IODATA) == LOW) {
			digitalWrite(kPIN_LED_ON_L, HIGH);
			FireSolenoid();
			while(digitalRead(kPIN_IODATA) == LOW) {}	// wait until button released
		}
    }
}