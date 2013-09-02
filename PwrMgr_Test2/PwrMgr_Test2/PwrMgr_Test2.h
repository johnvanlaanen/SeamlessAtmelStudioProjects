/*
 * PwrMgr_Test1.h
 *
 * Created: 8/30/2013 9:34:05 PM
 *  Author: John
 */ 


#ifndef PWRMGR_TEST1_H_
#define PWRMGR_TEST1_H_

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

#define  kPIN_BATTERY_V_SENSE          (kPORTA | kBIT0 )
#define  kPIN_USB_DMINUS_SENSE         (kPORTA | kBIT1 )
#define  kPIN_USB_DPLUS_SENSE          (kPORTA | kBIT2 )
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



#endif /* PWRMGR_TEST1_H_ */