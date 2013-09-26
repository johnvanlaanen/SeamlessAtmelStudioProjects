//
//  moduleIO.h
//  Atom
//
//  Created by Jay Hamlin on 6/20/13.
//
//

#ifndef Atom_moduleIO_h
#define Atom_moduleIO_h

#define		kPORTA		0x0800
#define		kPORTB		0x1000

#define		INPUT 0x0
#define		OUTPUT 0x1
#define		INPUT_PULLUP 0x2

#define HIGH 0x1
#define LOW  0x0

#define true 0x1
#define false 0x0

#define		kBIT0		0x01
#define		kBIT1		0x02
#define		kBIT2		0x04
#define		kBIT3		0x08
#define		kBIT4		0x10
#define		kBIT5		0x20
#define		kBIT6		0x40
#define		kBIT7		0x80

void pinMode(uint16_t pin, uint8_t mode);
void digitalWrite(uint16_t pin, uint8_t val);
int digitalRead(uint16_t ioPin);
int16_t analogRead(uint16_t ioPin);
void analogWrite(uint16_t ioPin, int val);

// pin interrupts
//void	ReadPinIntEnables(uint8_t *portApins,uint8_t *portBpins);
//void	EnablePinInterrupt(uint8_t portIndex);
//void	DisablePinInterrupt(uint8_t portIndex);

#define clearbit(sfr, bit)	(_SFR_BYTE(sfr) &= ~_BV(bit))
#define setbit(sfr, bit)	(_SFR_BYTE(sfr) |= _BV(bit))


#endif
