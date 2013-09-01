/*
 * PwrMgr_Test1.c
 *
 * Created: 8/30/2013 9:29:28 PM
 *  Author: John
 */ 


#include <avr/io.h>
#include <stdint.h>

#include "PwrMgr_Test1.h"
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
	pinMode(kPIN_TURN_POWER_OFF_L,			OUTPUT);
	pinMode(kPIN_POWER_BUTTON_PRESSED,		INPUT);

	pinMode(kPIN_CHARGER_SEL_HIGH_CURRENT,	INPUT);
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
	digitalWrite(kPIN_GREEN_POWER_LED_ON_L, LOW);
	digitalWrite(kPIN_RED_POWER_LED_ON_L,   LOW);
	

    while(1)
    {
		if(digitalRead(kPIN_POWER_BUTTON_PRESSED))
			digitalWrite(kPIN_GREEN_POWER_LED_ON_L, HIGH);
		else
			digitalWrite(kPIN_GREEN_POWER_LED_ON_L, LOW);
    }
}


