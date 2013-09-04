/*
 * PowerManager.h
 *
 * Created: 9/2/2013 12:29:48 PM
 *  Author: John VanLaanen for Seamless Toy Company
 */ 

#ifndef POWERMANAGER_H_
#define POWERMANAGER_H_


// configuration switches
#define TURN_POWER_OFF_ASSERT_LOW		// comment out if assertion level is high


// I/O Port assignemts
#define  kPIN_BATTERY_V_SENSE          (kPORTA | kBIT0 )		// ADC0
#define  kPIN_USB_DMINUS_SENSE         (kPORTA | kBIT1 )		// ADC1
#define  kPIN_USB_DPLUS_SENSE          (kPORTA | kBIT2 )		// ADC2
#define  kPIN_ENABLE_BUS_POWER         (kPORTA | kBIT3 )
#define  kPIN_REQ_SLEEP_MODE           (kPORTA | kBIT4 )
#define  kPIN_USB_POWER_SENSE_L        (kPORTA | kBIT5 )

#ifdef TURN_POWER_OFF_ASSERT_LOW
#define  kPIN_TURN_POWER_OFF_L         (kPORTA | kBIT6 )
#else
#define  kPIN_TURN_POWER_OFF           (kPORTA | kBIT6 )
#endif

#define  kPIN_POWER_BUTTON_PRESSED     (kPORTA | kBIT7 )

#define  kPIN_CHARGER_SEL_HIGH_CURRENT  (kPORTB | kBIT0 )
#define  kPIN_CHARGER_ENABLE            (kPORTB | kBIT1 )
#define  kPIN_CHARGER_STAT2             (kPORTB | kBIT2 )
#define  kPIN_CHARGER_STAT1             (kPORTB | kBIT3 )
#define  kPIN_ENABLE_POWER_GRP2         (kPORTB | kBIT4 )
#define  kPIN_GREEN_POWER_LED_ON_L      (kPORTB | kBIT5)
#define  kPIN_RED_POWER_LED_ON_L        (kPORTB | kBIT6 )

// 
#define CPU_CLOCK_FREQ 8000000					// 8MHz clock
#define kINTERRUPTS_PER_SEC 50					// 50Hz interrupt rate
const uint8_t  TCCR0_PRESCALE_SELECT =7;			// timer0 prescaler clock divide by 1024
const uint8_t  TCCR0_COUNT_VALUE = ((8000000 / 1024 / kINTERRUPTS_PER_SEC)-1);	// produces a 50.08Hz counter period.

#define kSUPPLY_VOLTAGE_x10 28		// the supply voltage, times 10
#define kUSB_DATA_V_MIN_x10 26		// The min USB data line voltage indicating a high current device, times 10
#define kUSB_DATA_V_MAX_x10 30		// The max USB data line voltage indicating a high current device, times 10
#define kBATTERY_FULL_VOLTS_x10 36	// The battery voltage above which to indicate a full charge, times 10
const int16_t kADCVAL_USB_MIN = (kUSB_DATA_V_MIN_x10 * 1024 / (2*kSUPPLY_VOLTAGE_x10));	// Min ADC value for high current sense
const int16_t kADCVAL_USB_MAX = (kUSB_DATA_V_MAX_x10 * 1024 / (2*kSUPPLY_VOLTAGE_x10));	// Max ADC value for high current sense
const int16_t kADCVAL_BATTERY_FULL = ( (kBATTERY_FULL_VOLTS_x10/2) * 1024) / (kSUPPLY_VOLTAGE_x10); // ADC value for full charge

const uint8_t kButtonDebounceCount = (10);
const uint8_t kButtonLongPressCount = (kINTERRUPTS_PER_SEC * 5);     // 5 seconds at 50 per second = 250
const uint8_t kUSBDebounceCount = (10);

const uint8_t kBatteryMeasureInterval = (kINTERRUPTS_PER_SEC * 1);     //measure the battery voltage once per second
const uint8_t kLEDBlinkInterval = ( kINTERRUPTS_PER_SEC / 2);     // LED blink rate when charging



#endif /* POWERMANAGER_H_ */