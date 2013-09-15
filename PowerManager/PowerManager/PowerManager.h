/*
 * PowerManager.h
 *
 * Created: 9/2/2013 12:29:48 PM
 *  Author: John VanLaanen for Seamless Toy Company
 */ 

#ifndef POWERMANAGER_H_
#define POWERMANAGER_H_

// configuration switches
//#define TURN_POWER_OFF_ASSERT_LOW		// comment out if assertion level is high
#define BUTTON_PRESS_ASSERT_LOW			// comment out if the button press signal assertion is high
#define BLUETOOTH_MODULE				// comment out for rechargeable battery module code

// I/O Port assignments
#define  kPIN_BATTERY_V_SENSE          (kPORTA | kBIT0 )		// ADC0
#define  kPIN_USB_DMINUS_SENSE         (kPORTA | kBIT1 )		// ADC1
#define  kPIN_USB_DPLUS_SENSE          (kPORTA | kBIT2 )		// ADC2
#define  kPIN_ENABLE_BUS_POWER         (kPORTA | kBIT3 )

#ifdef BLUETOOTH_MODULE
#define  kPIN_REQ_SLEEP_MODE           (kPORTA | kBIT4 )
#else
#define  kPIN_CHARGER_PG				(kPORTA | kBIT4 )
#endif

#define  kPIN_USB_POWER_SENSE_L        (kPORTA | kBIT5 )
#define  kPIN_TURN_POWER_OFF           (kPORTA | kBIT6 )

#define  kPIN_POWER_BUTTON_PRESSED     (kPORTA | kBIT7 )

#define  kPIN_CHARGER_SEL_HIGH_CURRENT  (kPORTB | kBIT0 )
#define  kPIN_CHARGER_ENABLE            (kPORTB | kBIT1 )
#define  kPIN_CHARGER_STAT2             (kPORTB | kBIT2 )
#define  kPIN_CHARGER_STAT1             (kPORTB | kBIT3 )
#define  kPIN_ENABLE_POWER_GRP2         (kPORTB | kBIT4 )
#define  kPIN_GREEN_POWER_LED_ON_L      (kPORTB | kBIT5)
#define  kPIN_RED_POWER_LED_ON_L        (kPORTB | kBIT6 )

// 
#define kCPU_CLOCK_FREQ 8000000					// 8MHz clock
#define kINTERRUPTS_PER_SEC 50					// 50Hz interrupt rate
const uint8_t  kTCCR0_PRESCALE_SELECT =7;		// timer0 prescaler clock divide by 1024
const uint8_t  kTCCR0_COUNT_VALUE = ((kCPU_CLOCK_FREQ / 1024 / kINTERRUPTS_PER_SEC)-1);	// produces a 50.08Hz counter period.

#define kSUPPLY_VOLTAGE_x10 28		// the supply voltage, times 10

#define kUSB_DATA_V_MIN_x10 25		// The min USB data line voltage indicating a high current device, times 10
#define kUSB_DATA_V_MAX_x10 31		// The max USB data line voltage indicating a high current device, times 10
const int16_t kADCVAL_USB_DATA_MIN = (kUSB_DATA_V_MIN_x10 * 1024 / (2*kSUPPLY_VOLTAGE_x10));	// Min ADC value for high current sense
const int16_t kADCVAL_USB_DATA_MAX = (kUSB_DATA_V_MAX_x10 * 1024 / (2*kSUPPLY_VOLTAGE_x10));	// Max ADC value for high current sense

#define kBATTERY_FULL_VOLTS_x10 36	// The battery voltage above which to indicate a full charge, times 10
const int16_t kADCVAL_BATTERY_FULL = ( (kBATTERY_FULL_VOLTS_x10/2) * 1024) / (kSUPPLY_VOLTAGE_x10); // ADC value for full charge

#define kPowerUpDelay_mS 100		// the amount of time to wait on powerup for things to settle
const uint8_t kPowerUpDelayCount = (kPowerUpDelay_mS * kINTERRUPTS_PER_SEC / 1000);

#define kPowerUpStateTimeout_Sec 2	// the max amount of time to wait in the powerup state to figure why power was turned on
const uint8_t kPowerUpStateTimeoutCount = (kPowerUpStateTimeout_Sec * kINTERRUPTS_PER_SEC);

#define kButtonDebounceTime_mS 20
const uint8_t kButtonDebounceCount = (kButtonDebounceTime_mS * kINTERRUPTS_PER_SEC / 1000);

#define kButtonLongPressTime_Sec 5
const uint8_t kButtonLongPressCount = (kButtonLongPressTime_Sec * kINTERRUPTS_PER_SEC);     // 5 seconds at 50 per second = 250

#define kUSBDebounceTime_mS 200
const uint8_t kUSBDebounceCount = (kUSBDebounceTime_mS * kINTERRUPTS_PER_SEC / 1000);

#define kUSBPowerupSettleTime_mS 200
const uint8_t kUSBPowerupSettleCount = (kUSBPowerupSettleTime_mS * kINTERRUPTS_PER_SEC / 1000);

#define kUSB_MeasureDelay_Sec 1
const uint8_t kUSBMeasureDelayCount = (kUSB_MeasureDelay_Sec * kINTERRUPTS_PER_SEC);


#define kBatteryMeasureInterval_Sec 1      //measure the battery voltage once per second
const uint8_t kBatteryMeasureCount = (kBatteryMeasureInterval_Sec * kINTERRUPTS_PER_SEC );

#define kLEDBlinkInterval_mS 500	// toggle the LED twice per second when blinking
const uint8_t kLEDBlinkCount = ( kLEDBlinkInterval_mS * kINTERRUPTS_PER_SEC / 1000);

#endif /* POWERMANAGER_H_ */