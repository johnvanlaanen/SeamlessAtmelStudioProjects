/*
 * PwrMgr_Test2.c
 *
 * Cycles the LEDs once a second through the sequence Off-Red-Amber-Green
 * Gets a 50Hz interrupt running, and it does something every 50 times through...
 *
 * Created: 9/1/2013 9:29:28 PM
 *  Author: John VanLaanen
 */ 


#include <avr/io.h>
#include <avr/interrupt.h>
#include <avr/sleep.h>
#include <stdint.h>

#include "PwrMgr_Test2.h"
#include "moduleIO.h"

int main(void)
{

	// set up the output pins and turn the Group2 power on
	pinMode(kPIN_BATTERY_V_SENSE,			INPUT);
	pinMode(kPIN_USB_DMINUS_SENSE,			INPUT);
	pinMode(kPIN_USB_DPLUS_SENSE,			INPUT);
	pinMode(kPIN_ENABLE_BUS_POWER,			OUTPUT);
	pinMode(kPIN_REQ_SLEEP_MODE,			INPUT);
	pinMode(kPIN_USB_POWER_SENSE_L,			INPUT);
	pinMode(kPIN_TURN_POWER_OFF,			OUTPUT);
	pinMode(kPIN_POWER_BUTTON_PRESSED,		INPUT);

	pinMode(kPIN_CHARGER_SEL_HIGH_CURRENT,	INPUT);
	pinMode(kPIN_CHARGER_ENABLE,			OUTPUT);
	pinMode(kPIN_CHARGER_STAT2,				INPUT_PULLUP);
	pinMode(kPIN_CHARGER_STAT1,				INPUT_PULLUP);
	pinMode(kPIN_ENABLE_POWER_GRP2,			OUTPUT);
	pinMode(kPIN_GREEN_POWER_LED_ON_L,		OUTPUT);
	pinMode(kPIN_RED_POWER_LED_ON_L,		OUTPUT);

	digitalWrite(kPIN_TURN_POWER_OFF,		LOW);
	digitalWrite(kPIN_ENABLE_BUS_POWER,		HIGH);
	digitalWrite(kPIN_CHARGER_ENABLE,		LOW);
	digitalWrite(kPIN_ENABLE_POWER_GRP2,	HIGH);	
	digitalWrite(kPIN_GREEN_POWER_LED_ON_L, HIGH);
	digitalWrite(kPIN_RED_POWER_LED_ON_L,   HIGH);
	
	
	// set up 8-bit timer 0 to generate an interrupt at a 50Hz rate
	TCCR0A = (1<<WGM01);					// CTC mode - clear on compare
	ASSR = 0;								// normal synchronous mode
	OCR0A = TCCR0_COUNT_VALUE;		// sets the compare value for interrupt generation
	TCNT0 = 0;								// make sure the count starts at 0
	GTCCR = (1<<PSR0);						// reset the prescaler
	TCCR0B = TCCR0_PRESCALE_SELECT<<CS00;	// set timer prescaler divide by 1024. Clock = 8MHz/1024=7812.5Hz
	TIMSK0 = (1<<OCIE0A);					// enable interrupt on compare

	set_sleep_mode(SLEEP_MODE_IDLE);

	sei();									// enable interrupts
	
    while(1)
    {
		sleep_mode();		//just keep going to sleep
    }
}

// The Timer0 vector
ISR(TIMER0_COMPA_vect)
{
	static uint8_t icnt=0;
	static uint8_t state=0;
	
	icnt += 1;
	if(icnt >= 50) {
		icnt = 0;
		if(state == 0) {
			state = 1;
			digitalWrite(kPIN_RED_POWER_LED_ON_L, LOW);
			digitalWrite(kPIN_GREEN_POWER_LED_ON_L, LOW);
		} else if(state == 1) {
			state = 2;
			digitalWrite(kPIN_RED_POWER_LED_ON_L, HIGH);
			digitalWrite(kPIN_GREEN_POWER_LED_ON_L, LOW);
		} else if(state == 2) {
			state = 3;
			digitalWrite(kPIN_RED_POWER_LED_ON_L, HIGH);
			digitalWrite(kPIN_GREEN_POWER_LED_ON_L, HIGH);
		} else if(state == 3) {
			state = 0;
			digitalWrite(kPIN_RED_POWER_LED_ON_L, LOW);
			digitalWrite(kPIN_GREEN_POWER_LED_ON_L, HIGH);
		}
	}
}
