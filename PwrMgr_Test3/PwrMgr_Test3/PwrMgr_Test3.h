/*
 * PwrMgr_Test1.h
 *
 * Created: 8/30/2013 9:34:05 PM
 *  Author: John
 */ 


#ifndef PWRMGR_TEST1_H_
#define PWRMGR_TEST1_H_

#define  kPIN_BATTERY_V_SENSE          (kPORTA | kBIT0 )		// ADC0
#define  kPIN_USB_DMINUS_SENSE         (kPORTA | kBIT1 )		// ADC1
#define  kPIN_USB_DPLUS_SENSE          (kPORTA | kBIT2 )		// ADC2
#define  kPIN_ENABLE_BUS_POWER         (kPORTA | kBIT3 )
#define  kPIN_REQ_SLEEP_MODE           (kPORTA | kBIT4 )
#define  kPIN_USB_POWER_SENSE_L        (kPORTA | kBIT5 )
#define  kPIN_TURN_POWER_OFF_L         (kPORTA | kBIT6 )
#define  kPIN_POWER_BUTTON_PRESSED     (kPORTA | kBIT7 )

#define  kPIN_CHARGER_SEL_HIGH_CURRENT  (kPORTB | kBIT0 )
#define  kPIN_CHARGER_ENABLE            (kPORTB | kBIT1 )
#define  kPIN_CHARGER_STAT2             (kPORTB | kBIT2 )
#define  kPIN_CHARGER_STAT1             (kPORTB | kBIT3 )
#define  kPIN_ENABLE_POWER_GRP2         (kPORTB | kBIT4 )
#define  kPIN_GREEN_POWER_LED_ON_L      (kPORTB | kBIT5)
#define  kPIN_RED_POWER_LED_ON_L        (kPORTB | kBIT6 )


#define CPU_CLOCK_FREQ 8000000					// 8MHz clock
#define TCCR0_PRESCALE_SELECT 7					// divide by 1024
#define TCCR0_COUNT_VALUE ((8000000 / 1024 / 50)-1)	// produces a 50.08Hz counter period. Not quite sure where the factor of 2 comes from yet...

#define kSUPPLY_VOLTAGE_x10 28		// the supply voltage, times 10
#define kUSB_DATA_V_MIN_x10 26		// The min USB data line voltage indicating a high current device, times 10
#define kUSB_DATA_V_MAX_x10 30		// The max USB data line voltage indicating a high current device, times 10

#define kADCVAL_RANGE_MIN (kUSB_DATA_V_MIN_x10 * 1024 / (2*kSUPPLY_VOLTAGE_x10))	// corresponds to 2.6V on the D+/- lines

#define kADCVAL_RANGE_MAX (kUSB_DATA_V_MAX_x10 * 1024 / (2*kSUPPLY_VOLTAGE_x10))	// corresponds to 3.0V on the D+/- lines

#endif /* PWRMGR_TEST1_H_ */