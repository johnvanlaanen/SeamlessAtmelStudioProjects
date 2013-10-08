/*
 * PopTest1.h
 *
 * Created: 8/30/2013 9:34:05 PM
 *  Author: John VanLaanen for Seamless Toy Company
 */ 


#ifndef POPTEST1_H_
#define POPTEST1_H_

#define USE_TIMER1_ISR

#define kPIN_IODATA			(kPORTA | kBIT3 )
#define kPIN_LED_ON_L		(kPORTA | kBIT7 )

#define	KPin_BoostedVSense	(kPORTA | kBIT0)	// ADC input: boosted voltage sense
#define	kPIN_BUS_V_SENSE	(kPORTA | kBIT1)	// ADC input: bus voltage sense
#define	kPIN_ACTUATE		(kPORTA | kBIT2)	// digital output: solenoid current enable
#define	kPIN_BOOST_CTRL1	(kPORTB | kBIT0)	// not used, wired in parallel with portb pin 2
#define	kPIN_BOOST_CTRL2	(kPORTB | kBIT1)	// not used, wired in parallel with portb pin 2
#define	kPIN_BOOST_CTRL3	(kPORTB | kBIT2)	// Timer PWM output: voltage booster clock

#define kCPU_CLOCK_FREQ_KHz 8000				// System clock frequency, in kHz
#define kBOOSTER_FREQ_HZ 3000					// desired clock rate, in Hz
#define kBOOST_TIMER_PRESCALE_RATIO 64			// the timer clock prescale divider ratio
#define kBOOST_TIMER_PRESCALE_SELECT 3			// timer prescale divider selection to corresponding to the above ratio
// caution: calculation structured carefully to not overflow the 16-bit math of the compiler
// note that the timer runs at twice the desired frequency since the output toggles with every full count
#define kPreCountDenom  (kBOOSTER_FREQ_HZ/5)
#define  kBOOST_TIMER_COUNT_VALUE  ( ( (100 * (kCPU_CLOCK_FREQ_KHz / 64)) + (kPreCountDenom/2) ) / kPreCountDenom ) - 1

#define kActuatePulseWidth_mS 20
#define kActuatePulseIntCount ( (kActuatePulseWidth_mS/2) * kBOOSTER_FREQ_HZ / 250)
//#define kActuatePulseIntCount  240 // debug

// ADC conversion threshold
// Calculated offline in order to get the precision needed.
// The boosted voltage is sensed through a 1:5.7 voltage divider
// ADC value = 1024 * (BoostedV/5.7) / 3.0V

//#define kFullChargeADCValue 299		// threshold for 5.0V boosted charge threshold
//#define kFullChargeADCValue 359		// threshold for 6.0V boosted charge threshold
//#define kFullChargeADCValue 419		// threshold for 7.0V boosted charge threshold
//#define kFullChargeADCValue 449		// threshold for 7.5V boosted charge threshold
#define kFullChargeADCValue 464		// threshold for 7.75V boosted charge threshold
//#define kFullChargeADCValue 479		// threshold for 8.0V boosted charge threshold
//#define kFullChargeADCValue 509		// threshold for 8.5V boosted charge threshold

#define kNumTriggerFires 1

#endif /* PWRMGR_TEST1_H_ */