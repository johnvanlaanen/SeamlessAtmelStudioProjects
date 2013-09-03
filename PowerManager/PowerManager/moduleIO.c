//
//  moduleIO.c
//  Atom
//
//  Created by Jay Hamlin on 6/20/13.
//
//
#include <stdint.h>
#include <avr/io.h>
#include <avr/interrupt.h>

#include "AtomDefs.h"
#include "moduleIO.h"


void pinMode(uint16_t ioPin, uint8_t mode)
{
	uint8_t  bit;

	bit = (ioPin & 0x00ff);

	if (mode == INPUT)
    {
		uint8_t oldSREG = SREG;
		cli();

        do
        {

#ifdef DDRA
            if(ioPin & kPORTA)
            {
                DDRA &= ~bit;       // 0 bit is input
                PORTA &= ~bit;      // A 0 bit on an input disables the pullup resistor
                break;
            }
#endif

#ifdef DDRB
            if(ioPin & kPORTB)
            {
                DDRB &= ~bit;       // 0 bit is input
                PORTB &= ~bit;      // A 0 bit on an input disables the pullup resistor
                break;
            }
#endif

#ifdef DDRC
            if(ioPin & kPORTC)
            {
                DDRC &= ~bit;       // 0 bit is input
                PORTC &= ~bit;      // A 0 bit on an input disables the pullup resistor
                break;
            }
#endif

#ifdef DDRD
            if(ioPin & kPORTD)
            {
                DDRD &= ~bit;       // 0 bit is input
                PORTD &= ~bit;      // A 0 bit on an input disables the pullup resistor
                break;
            }
#endif
        } while(0);

        SREG = oldSREG;
    }

    else if (mode == INPUT_PULLUP)
    {
        uint8_t oldSREG = SREG;
        cli();

        do
        {

#ifdef DDRA
            if(ioPin & kPORTA)
            {
                DDRA &= ~bit;       // 0 bit is input
                PORTA |= bit;       // A 1 bit on an input enables the pullup resistor
                break;
            }
#endif

#ifdef DDRB
            if(ioPin & kPORTB)
            {
                DDRB &= ~bit;       // 0 bit is input
                PORTB |= bit;       // A 1 bit on an input enables the pullup resistor
                break;
            }
#endif

#ifdef DDRC
            if(ioPin & kPORTC)
            {
                DDRC &= ~bit;       // 0 bit is input
                PORTC |= bit;       // A 1 bit on an input enables the pullup resistor
                break;
            }
#endif

#ifdef DDRD
            if(ioPin & kPORTD)
            {
                DDRD &= ~bit;       // 0 bit is input
                PORTD |= bit;       // A 1 bit on an input enables the pullup resistor
                break;
            }
#endif
        } while(0);

        SREG = oldSREG;
    }

    else if(mode == OUTPUT)
    {
        uint8_t oldSREG = SREG;
        cli();

        do
        {
#ifdef DDRA
            if(ioPin & kPORTA)
            {
                DDRA |= bit;
                break;
            }
#endif

#ifdef DDRB
            if(ioPin & kPORTB)
            {
                DDRB |= bit;
                break;
            }
#endif

#ifdef DDRC
            if(ioPin & kPORTC)
            {
                DDRC |= bit;
                break;
            }
#endif

#ifdef DDRD
            if(ioPin & kPORTD)
            {
                DDRD |= bit;
                break;
            }
#endif

        } while(0);

            DDRB |= bit;
        SREG = oldSREG;
    }
}

void digitalWrite(uint16_t ioPin, uint8_t val)
{
    uint8_t bit;

    bit = ioPin & 0x00ff;

    uint8_t oldSREG = SREG;
    cli();

    if (val == LOW)
    {
        do
        {

#ifdef PORTA
            if(ioPin & kPORTA)
            {
                PORTA &= ~bit;
                break;
            }
#endif

#ifdef PORTB
            if(ioPin & kPORTB)
            {
                PORTB &= ~bit;
                break;
            }
#endif

#ifdef PORTC
            if(ioPin & kPORTC)
            {
                PORTC &= ~bit;
                break;
            }
#endif

#ifdef PORTD
            if(ioPin & kPORTD)
            {
                PORTD &= ~bit;
                break;
            }
#endif
        } while(0);
    }

    else
    {
        do
        {

#ifdef PORTA
            if(ioPin & kPORTA)
            {
                PORTA |= bit;
                break;
            }
#endif

#ifdef PORTB
            if(ioPin & kPORTB)
            {
                PORTB |= bit;
                break;
            }
#endif

#ifdef PORTC
            if(ioPin & kPORTC)
            {
                PORTC |= bit;
                break;
            }
#endif

#ifdef PORTD
            if(ioPin & kPORTD)
            {
                PORTD |= bit;
                break;
            }
#endif
        } while(0);
    }

    SREG = oldSREG;
}

int digitalRead(uint16_t ioPin)
{
    uint8_t bit;
    uint8_t val = 0;

    bit = ioPin & 0x00ff;

    do
    {

#ifdef PINA
        if(ioPin & kPORTA)
        {
            val = (PINA & bit);
            break;
        }
#endif

#ifdef PINB
        if(ioPin & kPORTB)
        {
            val = (PINB & bit);
            break;
        }
#endif

#ifdef PINC
        if(ioPin & kPORTC)
        {
            val = (PINC & bit);
            break;
        }
#endif

#ifdef PIND
        if(ioPin & kPORTD)
        {
            val = (PIND & bit);
            break;
        }
#endif

    } while(0);

    if(val)
        return HIGH;
    else
        return LOW;
}


void analogWrite(uint16_t ioPin, int val)
{
}

int16_t analogRead(uint16_t ioPin)
{
    int16_t		value;
    uint8_t		mux,bit;
    uint16_t		lowb,highb;

    if(ioPin & kPORTA) {
        // setup the MUX
        bit = (ioPin & 0xff);
        mux=255;
        while(bit){
            mux++;
            bit>>=1;
        }
        // AREF on PA0, select channel
        ADMUX = ((1<<REFS0)| mux);	// single ended - unity gain.

        // start the conversion
        setbit(ADCSRA, ADSC);

        // ADSC is cleared when the conversion finishes
        while (bit_is_set(ADCSRA, ADSC));

        // we have to read ADCL first; doing so locks both ADCL
        // and ADCH until ADCH is read.  reading ADCL second would
        // cause the results of each conversion to be discarded,
        // as ADCL and ADCH would be locked when it completed.
        lowb  = ADCL;
        highb = ADCH;
        value= highb;
        value<<=8;
        value |=lowb;
    } else {
        value = -1;
    }

    return(value);
}

void analogReadStart(uint16_t ioPin)
{
	uint8_t        mux,bit;

	if(ioPin & kPORTA) {
		// setup the MUX
		bit = (ioPin & 0xff);
		mux=255;
		while(bit){
			mux++;
			bit>>=1;
		}

		// select channel, using AVcc as reference
		ADMUX = mux;

		// start the conversion
		setbit(ADCSRA, ADSC);
	}
}

int16_t analogReadFinish(void)
{	
	int16_t     value;
	uint16_t    lowb,highb;

	// ADSC is cleared when the conversion finishes
	while (bit_is_set(ADCSRA, ADSC)) {}

	// we have to read ADCL first; doing so locks both ADCL
	// and ADCH until ADCH is read.  reading ADCL second would
	// cause the results of each conversion to be discarded,
	// as ADCL and ADCH would be locked when it completed.
	lowb  = ADCL;
	highb = ADCH;
	value= highb;
	value<<=8;
	value |=lowb;

	return(value);
}


#if 0	// the new code doesn't use pin interrupts

static	uint8_t gportApins=0;
static	uint8_t gportAstate=0;
static	uint8_t gportBpins=0;
static	uint8_t gportBstate=0;

extern char	pulseDetectedPinIndex;

void	ReadPinIntEnables(uint8_t *portApins,uint8_t *portBpins)
{
    if(portApins) *portApins=gportApins;
    if(portBpins) *portBpins=gportBpins;
}

#if defined(__AVR_ATmega328P__)
 // The ATMega328P is used on our console project doesn't have PINA, IOData is on PINB0 .
 // Interrupt pin numbers are mapped differently than on the ATTinys.

void	EnablePinInterrupt(uint8_t portIndex)
{
	uint8_t		ppin;
	uint16_t	ioPin = module.port[portIndex].iopin;

	ppin = (ioPin & 0x00ff);

	pulseDetectedPinIndex=0;

	if(ioPin & kPORTB)  {
		gportBstate = PINB;		// read the state now so we can detect the change later
		gportBpins |= ppin;
		PCICR |= 0x02;  // bit 1 for port B pins
		PCMSK1 |= ppin;
		PCIFR |= 0x02;  // bit 1 for PCIF1 - port B pins
	}

}

void	DisablePinInterrupt(uint8_t portIndex)
{
	uint8_t		ppin;
	uint16_t	ioPin = module.port[portIndex].iopin;

	ppin = (uint8_t)(ioPin & 0x00ff);

	pulseDetectedPinIndex=0;

	if(ioPin & kPORTB)  {
		gportBpins &= ~ ppin;
		PCICR &= ~0x02;  // bit 1 for port B pins
		PCMSK1 &= ~ppin;
		PCIFR &= ~0x02;  // bit 1 for PCIF1 - port B pins
	}
}


ISR(PCINT0_vect)	// This is INT is for PA[0:7}
{					// We only handle 2 kinds of pin change interrupts
	// A positive going change to indicate the start of a INPUT port pulse
	// A negative going change for a SW uart start bit.
	uint8_t	portIndex,pin,xpin;

	pin = (PINB);
	// which enabled pin changed?  easy...
	xpin = (gportAstate^pin)&gportApins;
	// I am going to assume ther is only 1 pin change at a time.
	// may not be 100% true but it is a good assumption.
	if(xpin) {  // can this even be zero?

		portIndex=kNumberofINPUTPorts; // interrupts only come from OUT pins so we can start here
		while(((module.port[portIndex].iopin&0xff) != (xpin)) && (portIndex < kNumberofPorts)) {
			portIndex++;
		}

		pulseDetectedPinIndex = (portIndex | 0x40);

	}
}


ISR(PCINT1_vect)
{

}

ISR(PCINT2_vect)
{
}
			// ATMega328P code above
#else
			// ATtiny44/87 code below

void	EnablePinInterrupt(uint8_t portIndex)
{
	uint8_t		ppin;
	uint16_t	ioPin = module.port[portIndex].iopin;

	ppin = (ioPin & 0x00ff);

	pulseDetectedPinIndex=0;

#ifdef GIMSK			// global int msk register for the ATTiny44/84
	if(ioPin & kPORTA) {
		gportAstate = PINA;		// read the state now so we can detect the change later
		gportApins |= ppin;
		GIFR |= 0x10;  // bit 4 for PCIF0 - port A pins
		PCMSK0 |= ppin;
		GIMSK |=  0x10;  // bit 4 for PCIF0 - port A pins
	} else if(ioPin & kPORTB)  {
		gportBstate = PINB;		// read the state now so we can detect the change later
		gportBpins |= ppin;
		GIFR |= 0x20;  // bit 5 for PCIF1 - port B pins
		PCMSK1 |= ppin;
		GIMSK |= 0x20;  // bit 5 for PCIF1 - port B pins
	}
#endif

#ifdef	PCICR			// Pin change interrupt control register for the ATTiny87/167
	if(ioPin & kPORTA) {
		gportAstate = PINA;		// read the state now so we can detect the change later
		gportApins |= ppin;
		PCICR |= 0x01;  // bit 0 port A pins
		PCMSK0 |= ppin;
		PCIFR |=  0x01;  // bit 0 for PCIF0 - port A pins
	} else if(ioPin & kPORTB)  {
		gportBstate = PINB;		// read the state now so we can detect the change later
		gportBpins |= ppin;
		PCICR |= 0x02;  // bit 1 for port B pins
		PCMSK1 |= ppin;
		PCIFR |= 0x02;  // bit 1 for PCIF1 - port B pins
	}
#endif

}

void	DisablePinInterrupt(uint8_t portIndex)
{
	uint8_t		ppin;
	uint16_t	ioPin = module.port[portIndex].iopin;

	ppin = (uint8_t)(ioPin & 0x00ff);

#ifdef GIMSK			// global int msk register for the ATTiny44/84
	if(ioPin & kPORTA) {
		gportApins &= ~ ppin;
		GIFR &= ~ 0x10;
		PCMSK0 &= ~ ppin;
		GIMSK &= ~ 0x10;
	} else if(ioPin & kPORTB)  {
		gportBpins &= ~ ppin;
		GIFR &= ~ 0x20;
		PCMSK1 &= ~ ppin;
		GIMSK &= ~ 0x20;
	}
#endif
#ifdef	PCICR			// Pin change interrupt control register for the ATTiny87/167
	if(ioPin & kPORTA) {
		gportApins &= ~ ppin;
		PCICR &= ~0x01;  // bit 0 port A pins
		PCMSK0 &= ~ppin;
		PCIFR &= ~0x01;  // bit 0 for PCIF0 - port A pins
	} else if(ioPin & kPORTB)  {
		gportBpins &= ~ ppin;
		PCICR &= ~0x02;  // bit 1 for port B pins
		PCMSK1 &= ~ppin;
		PCIFR &= ~0x02;  // bit 1 for PCIF1 - port B pins
	}
#endif
}


ISR(PCINT0_vect)	// This is INT is for PA[0:7}
{					// We only handle 2 kinds of pin change interrupts
	// A positive going change to indicate the start of a INPUT port pulse
	// A negative going change for a SW uart start bit.
	uint8_t	portIndex,pin;
	uint16_t	xpin;

	pin = (PINA);
	// which enabled pin changed?  easy...
	xpin = (gportAstate^pin)&gportApins;
	// I am going to assume there is only 1 pin change at a time.
	// may not be 100% true but it is a good assumption.
	if(xpin) {  // can this even be zero?
		xpin |= (kPORTA|kOUTPUTPORT);		// we know this was port A

		portIndex=kNumberofINPUTPorts; // interrupts only come from OUT pins so we can start here
		while((module.port[portIndex].iopin != xpin) && (portIndex < kNumberofPorts)) {
			portIndex++;
		}

		pulseDetectedPinIndex = (portIndex | 0x40);
		
	}
}


ISR(PCINT1_vect)	// This is INT is for PB[0:7}
{					// We only handle 2 kinds of pin change interrupts
	// A positive going change to indicate the start of a INPUT port pulse
	// A negative going change for a SW uart start bit.
	uint8_t	portIndex,pin;
	uint16_t	xpin;

	pin = (PINB);
	// which enabled pin changed?  easy...
	xpin = (gportBstate^pin)&gportBpins;
	// I am going to assume ther is only 1 pin change at a time.
	// may not be 100% true but it is a good assumption.
	if(xpin) {  // can this even be zero?
		xpin |= (kPORTB|kOUTPUTPORT);		// we know this was port B

		portIndex=kNumberofINPUTPorts; // interrupts only come from OUT pins so we can start here
		while((module.port[portIndex].iopin != xpin) && (portIndex < kNumberofPorts)) {
			portIndex++;
		}
		
		pulseDetectedPinIndex = (portIndex | 0x40);

	}
}

#endif
#endif
