//
//  moduleIO.h
//  Atom
//
//  Created by Jay Hamlin on 6/20/13.
//
//

#ifndef Atom_moduleIO_h
#define Atom_moduleIO_h

void pinMode(uint16_t pin, uint8_t mode);
void digitalWrite(uint16_t pin, uint8_t val);
int digitalRead(uint16_t ioPin);
int16_t analogRead(uint16_t ioPin);
void analogWrite(uint16_t ioPin, int val);
void analogReadStart(uint16_t ioPin);
int16_t analogReadFinish(void);

// pin interrupts
//void	ReadPinIntEnables(uint8_t *portApins,uint8_t *portBpins);
//void	EnablePinInterrupt(uint8_t portIndex);
//void	DisablePinInterrupt(uint8_t portIndex);

#define clearbit(sfr, bit)	(_SFR_BYTE(sfr) &= ~_BV(bit))
#define setbit(sfr, bit)	(_SFR_BYTE(sfr) |= _BV(bit))


#endif
