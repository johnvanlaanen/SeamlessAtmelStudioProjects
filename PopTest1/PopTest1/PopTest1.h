/*
 * PopTest1.h
 *
 * Created: 8/30/2013 9:34:05 PM
 *  Author: John
 */ 


#ifndef POPTEST1_H_
#define POPTEST1_H_


#define  kPIN_CAP_V_SENSE			   (kPORTA | kBIT0 )
#define  kPIN_ACTUATE				   (kPORTA | kBIT2 )
#define  kPIN_IODATA				   (kPORTA | kBIT3 )
#define  kPIN_LED_ON_L				   (kPORTA | kBIT7 )

#define  kPIN_TRIPLER_SWITCH1  (kPORTB | kBIT0 )
#define  kPIN_TRIPLER_SWITCH2  (kPORTB | kBIT1 )
#define  kPIN_TRIPLER_SWITCH3  (kPORTB | kBIT2 )


#define kCPU_CLOCK_FREQ_KHz 8000				// System clock frequency, in kHz
#define kTRIPLER_FREQ_HZ 2000					// desired clock rate, in Hz
const uint8_t  kTCCR0_PRESCALE_SELECT = 3;		// timer0 prescaler divider: prescaler out is i/o clock divided by 64

// have to calculate the following value outside of the compiler to get good accuracy (16-bit math limitations)
// count = (kCPU_CLOCK_FREQ_KHz * 1000 /64)  / (2*kTRIPLER_FREQ_HZ) )-1)
//const uint8_t  kTCCR0_COUNT_VALUE = 61;		// yields a clock freq of 2016Hz, or an output frequency of 1008Hz
//const uint8_t  kTCCR0_COUNT_VALUE = 30;		// yields a clock freq of 4032Hz, or an output frequency of 2016Hz
const uint8_t  kTCCR0_COUNT_VALUE = 20 ;		// yields a clock freq of 5952Hz, or an output frequency of 2976Hz
//const uint8_t  kTCCR0_COUNT_VALUE = 14 ;		// yields a clock freq of 8333Hz, or an output frequency of 4167Hz
//const uint8_t  kTCCR0_COUNT_VALUE = 11 ;		// yields a clock freq of 1-417Hz, or an output frequency of 5208Hz

const int16_t kFullChargeADCValue = 938;	// corresponds to 11V/4 with a 3V supply

#endif /* PWRMGR_TEST1_H_ */