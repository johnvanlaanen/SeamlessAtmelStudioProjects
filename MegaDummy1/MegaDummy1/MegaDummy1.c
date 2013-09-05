/*
 * MegaDummy1.c
 *
 * Created: 9/5/2013 8:23:05 AM
 *  Author: John
 */ 


#include <avr/io.h>

#include "AtomDefs.h"
#include "moduleIO.h"
#include "MegaDummy1.h"

int main(void)
{
	
	// set the state of outputs before enabling them
	digitalWrite(kPIN_IODATA1,			LOW);
	digitalWrite(kPIN_IODATA2,			LOW);
	digitalWrite(kPIN_IODATA3,			LOW);
	digitalWrite(kPIN_IODATA4,			LOW);
	digitalWrite(kPIN_IODATA5,			LOW);
	digitalWrite(kPIN_IODATA6,			LOW);
	digitalWrite(kPIN_IODATA7,			LOW);
	digitalWrite(kPIN_IODATA8,			LOW);
	
	digitalWrite(kPIN_LEDSPI_SS_L,		HIGH);
	digitalWrite(kPIN_LEDSPI_MOSI,		LOW);
	digitalWrite(kPIN_LEDSPI_SCK,		LOW);
	digitalWrite(kPIN_REQ_SLEEP_MODE,	LOW);
	digitalWrite(kPIN_BT_TX,			LOW);
	digitalWrite(kPIN_BT_RTS_L,			LOW);



	// setup the ports	
	pinMode( kPIN_IODATA2,					OUTPUT);
	pinMode( kPIN_IODATA1,					OUTPUT);
	pinMode( kPIN_LEDSPI_SS_L,				OUTPUT);
	pinMode( kPIN_LEDSPI_MOSI,				OUTPUT);
	pinMode( kPIN_ENABLE_BUS_POWER,			INPUT);
	pinMode( kPIN_LEDSPI_SCK,				OUTPUT);

	pinMode( kPIN_BATTERY_V_SENSE,			INPUT);
	pinMode( kPIN_REQ_SLEEP_MODE,			OUTPUT);
	pinMode( kPIN_CHARGER_ENABLE,			INPUT);
	pinMode( kPIN_CHARGER_SEL_HIGH_CURRENT,	INPUT);
	pinMode( kPIN_IODATA3,					OUTPUT);
	pinMode( kPIN_IODATA4,					OUTPUT);

	pinMode( kPIN_BT_RX,					INPUT);
	pinMode( kPIN_BT_TX,					OUTPUT);
	pinMode( kPIN_BT_CTS_L,					INPUT);
	pinMode( kPIN_BT_RTS_L,					OUTPUT);
	pinMode( kPIN_IODATA8,					OUTPUT);
	pinMode( kPIN_IODATA7,					OUTPUT);
	pinMode( kPIN_IODATA6,					OUTPUT);
	pinMode( kPIN_IODATA5,					OUTPUT);	
	
    while(1)
    {
        //TODO:: Please write your application code 
    }
}