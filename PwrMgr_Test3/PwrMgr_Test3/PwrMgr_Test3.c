/*
 * PwrMgr_Test2.c
 *
 * Monitors the voltage on the USB D+ and D- lines, and turns on the red and green LEDS
 *	when they are in the desired range
 *
 * Created: 9/1/2013 9:29:28 PM
 *  Author: John VanLaanen
 */ 


#include <avr/io.h>
#include <avr/interrupt.h>
#include <avr/sleep.h>
#include <stdint.h>

#include "PwrMgr_Test3.h"
#include "moduleIO.h"

int main(void)
{

	// set up the output pins and turn the Group2 power on
	//pinMode(kPIN_BATTERY_V_SENSE,			INPUT);			// ADC0
	//pinMode(kPIN_USB_DMINUS_SENSE,			INPUT);			// ADC1
	//pinMode(kPIN_USB_DPLUS_SENSE,			INPUT);			// ADC2
	pinMode(kPIN_ENABLE_BUS_POWER,			OUTPUT);
	pinMode(kPIN_REQ_SLEEP_MODE,			INPUT);
	pinMode(kPIN_USB_POWER_SENSE_L,			INPUT);
	pinMode(kPIN_TURN_POWER_OFF_L,			OUTPUT);
	pinMode(kPIN_POWER_BUTTON_PRESSED,		INPUT);

	pinMode(kPIN_CHARGER_SEL_HIGH_CURRENT,	OUTPUT);
	pinMode(kPIN_CHARGER_ENABLE,			OUTPUT);
	pinMode(kPIN_CHARGER_STAT2,				INPUT_PULLUP);
	pinMode(kPIN_CHARGER_STAT1,				INPUT_PULLUP);
	pinMode(kPIN_ENABLE_POWER_GRP2,			OUTPUT);
	pinMode(kPIN_GREEN_POWER_LED_ON_L,		OUTPUT);
	pinMode(kPIN_RED_POWER_LED_ON_L,		OUTPUT);

	digitalWrite(kPIN_TURN_POWER_OFF_L,		HIGH);
	digitalWrite(kPIN_ENABLE_BUS_POWER,		LOW);
	digitalWrite(kPIN_CHARGER_ENABLE,		LOW);
	digitalWrite(kPIN_ENABLE_POWER_GRP2,	HIGH);	
	digitalWrite(kPIN_GREEN_POWER_LED_ON_L, HIGH);
	digitalWrite(kPIN_RED_POWER_LED_ON_L,   HIGH);
	
	// set up the ADC
	AMISCR = 0;
	ADCSRA = (1<<ADEN) | (7<<ADPS0); // Set the ADC clock prescaler to divide by 128, and enable the ADC
	
	// Turn off the digital input buffers on the analog input pins (saves some power)
	DIDR0 = ((kPIN_BATTERY_V_SENSE&0xFF) | (kPIN_USB_DMINUS_SENSE&0xFF) | (kPIN_USB_DPLUS_SENSE&0xFF));
	
	
	// set up 8-bit timer 0 to generate an interrupt at a 50Hz rate
	TCCR0A = (1<<WGM01);					// CTC mode - clear on compare
	ASSR = 0;								// normal synchronous mode
	OCR0A = TCCR0_COUNT_VALUE;				// sets the compare value for interrupt generation
	TCNT0 = 0;								// make sure the count starts at 0
	GTCCR = (1<<PSR0);						// reset the prescaler
	TCCR0B = TCCR0_PRESCALE_SELECT<<CS00;	// set timer prescaler divide by 1024. Clock = 8MHz/1024=7812.5Hz
	TIMSK0 = (1<<OCIE0A);					// enable interrupt on compare

	set_sleep_mode(SLEEP_MODE_IDLE);

	analogReadStart(kPIN_USB_DPLUS_SENSE);	// prime the pump for the ISR

	sei();									// enable interrupts
	
    while(1)
    {
		sleep_mode();		//just keep going to sleep
    }
}

// The Timer0 vector
ISR(TIMER0_COMPA_vect)
{
	static uint8_t linesel=0;
	static uint8_t bit_state=0;
	

	// toggle the CHARER_SEL_HIGH_CURRENT every interrupt to indicate proper operation.
	if(bit_state==0) {
		bit_state = 1;
		digitalWrite(kPIN_CHARGER_SEL_HIGH_CURRENT, HIGH);
	} else {
		bit_state = 0;
		digitalWrite(kPIN_CHARGER_SEL_HIGH_CURRENT, LOW);
	}

	int16_t adcval = analogReadFinish();
	
	if(linesel==0){
		// D- line measurement
		linesel = 1;
		analogReadStart(kPIN_USB_DPLUS_SENSE);		// start the next conversion
		
		// turn on the green LED if the voltage was within range
		if( (adcval >= kADCVAL_RANGE_MIN) && (adcval <= kADCVAL_RANGE_MAX))
			digitalWrite(kPIN_GREEN_POWER_LED_ON_L, LOW);
		else
			digitalWrite(kPIN_GREEN_POWER_LED_ON_L, HIGH);
	} else {
		// D+ line measurement
		linesel = 0;
		analogReadStart(kPIN_USB_DMINUS_SENSE);		
		
		// turn on the red LED if the voltage was within range
		if( (adcval >= kADCVAL_RANGE_MIN) && (adcval <= kADCVAL_RANGE_MAX))
			digitalWrite(kPIN_RED_POWER_LED_ON_L, LOW);
		else
			digitalWrite(kPIN_RED_POWER_LED_ON_L, HIGH);
	}
	
}
