/*
 * PopTest1.h
 *
 * Created: 8/30/2013 9:34:05 PM
 *  Author: John
 */ 


#ifndef POPTEST1_H_
#define POPTEST1_H_

#define USE_TIMER1_ISR


#define  kPIN_CAP_V_SENSE		(kPORTA | kBIT0 )
#define  kPIN_ACTUATE			(kPORTA | kBIT2 )
#define  kPIN_IODATA			(kPORTA | kBIT3 )
#define  kPIN_LED_ON_L			(kPORTA | kBIT7 )

#define  kPIN_TRIPLER_SWITCH1	(kPORTB | kBIT0 )
#define  kPIN_TRIPLER_SWITCH2	(kPORTB | kBIT1 )
#define  kPIN_TRIPLER_SWITCH3	(kPORTB | kBIT2 )

#define kCPU_CLOCK_FREQ_KHz 8000				// System clock frequency, in kHz
#define kTRIPLER_FREQ_HZ 3000					// desired clock rate, in Hz

#ifdef USE_TIMER1_ISR
	#define kBOOST_TIMER_PRESCALE_RATIO 64				// the timer clock prescale divider ratio
	#define kBOOST_TIMER_PRESCALE_SELECT  3				// timer prescal divider selection to corresponding to the above ratio
	// caution: calculation structured carefully to not overflow the 16-bit math of the compiler
	// note that the timer runs at twice the desired frequency since the output toggles with every full count
	#define kPreCountDenom  (kTRIPLER_FREQ_HZ/5)
	#define  kBOOST_TIMER_COUNT_VALUE  ( ( ( (100 * (kCPU_CLOCK_FREQ_KHz / 64)) + (kPreCountDenom/2) ) / kPreCountDenom ) - 1)
#else
	#define kkTCCR0_PRESCALE_RATIO 64				// the timer clock prescale divider ratio
	const uint8_t  kTCCR0_PRESCALE_SELECT = 3;		// timer0 prescal divider selection to correspond to the above ratio
	// caution: calculations structured carefully to not overflow 16-bit math
	// note that the timer runs at twice the desired frequency since the output toggles with every full count
	#define kPreCountDenom  (kTRIPLER_FREQ_HZ/5)
	const uint8_t  kTCCR0_COUNT_VALUE = ( ( (100 * (kCPU_CLOCK_FREQ_KHz / 64)) + (kPreCountDenom/2) ) / kPreCountDenom ) - 1;
#endif

#define kSupplyVoltage_x100		270				// 2.7V supply, used as ADC reference, times 100
#define kADCBits				10				// the number of bits in the ADC result
#define kFullChargeVolts_x100	900				// full charge boosted voltage, times 100
// caution: calculations structured carefully to not overflow 16-bit math
//const int16_t kFullChargeADCValue = ( (kFullChargeVolts_x100 / 4) * (1<<(kADCBits-3)) / (kSupplyVoltage_x100 / (1<<3)) );
#define kFullChargeADCValue ( (kFullChargeVolts_x100 / 4) * (1<<(kADCBits-kADC_Calc_shift_bits)) / (kSupplyVoltage_x100 / (1<<kADC_Calc_shift_bits)) )


#endif /* PWRMGR_TEST1_H_ */