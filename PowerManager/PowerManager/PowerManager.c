/******************************************************************************
 * PowerManager.c - Power Manager microcontroller firmware for the Bluetooth Module (U10)
 *
 * Target device: ATTiny87
 *
 * Created: 9/2/2013 12:13:14 PM
 *  Author: John VanLaanen for Seamless Toy Company
 *****************************************************************************/ 


#include <avr/io.h>
#include <avr/interrupt.h>
#include <avr/sleep.h>
#include <stdint.h>

#include "AtomDefs.h"
#include "PowerManager.h"
#include "moduleIO.h"


typedef enum {
	kButtonWaitNotPressed=0,
	kButtonNotPressed,
	kButtonPressed,
	kButtonPressed_Long,
	kButtonJustReleased,
} tPowerButtonStates;

typedef enum {
	kSleepReq_NoRequest=0,
	kSleepReq_NewRequest,
	kSleepReq_Handshake,
} tSleepRequestStates;


typedef enum {
	kJustPoweredUp=0,
	kPowerCharging,
	kPowerSleep,
	kPowerActive,
	kPowerSleepCharging,
	kPowerActiveCharging,
} tModulePowerStates;

typedef enum {
	kBattery_NoBattery=0,
	kBattery_LowCharge,
	kBattery_FullCharge,
} tBatteryChargeStates;

typedef enum {
	kBatteryMeasureWaiting=0,
	kBatteryMeasureInProcess,
} tBatteryMeasureStates;


typedef enum {
	kUSB_PowerupSettle=0,
	kUSB_Unplugged,
	kUSB_JustUnplugged,
	kUSB_Debounce,
	kUSB_PluggedMeasureDPlus,
	kUSB_PluggedMeasureDMinus,
	kUSB_PluggedInLowCurrent,
	kUSB_PluggedInHighCurrent,
} tUSBStates;


volatile struct {
	uint8_t               button_pressed_count;
	uint8_t               led_blink_delay;
	uint8_t				  led_blink_on;
	uint8_t               usb_debounce_count;
	uint8_t               battery_measure_delay;
	uint8_t               adc_in_use;
	uint8_t				  new_power_state;
	tModulePowerStates    power_state;
	tPowerButtonStates    button_state;
	tSleepRequestStates	  sleep_req_state;
	tUSBStates            usb_state;
	tBatteryChargeStates  battery_charge_state;
	tBatteryMeasureStates battery_measure_state;
} SystemState= {.button_pressed_count	= 0,
				.led_blink_delay		= 0,
				.led_blink_on			= 0,
				.usb_debounce_count		= 0,
				.battery_measure_delay	= 0,
				.adc_in_use				= 0,
				.new_power_state		= 1,
				.power_state			= kJustPoweredUp,
				.button_state			= kButtonWaitNotPressed,
				.sleep_req_state		= kSleepReq_NoRequest,
				.usb_state				= kUSB_PowerupSettle,
				.battery_charge_state	= kBattery_NoBattery,
				.battery_measure_state	= kBatteryMeasureWaiting };

// Routine to turn off power. It's a dead-end call that will never return
void TurnPowerOff(void) {
	digitalWrite(kPIN_ENABLE_BUS_POWER, LOW);
	digitalWrite(kPIN_CHARGER_ENABLE, LOW);
	digitalWrite(kPIN_ENABLE_POWER_GRP2, LOW);
	digitalWrite(kPIN_RED_POWER_LED_ON_L, HIGH);
	digitalWrite(kPIN_GREEN_POWER_LED_ON_L, HIGH);
	
	while(1) {
#ifdef TURN_POWER_OFF_ASSERT_LOW
		digitalWrite(kPIN_TURN_POWER_OFF, LOW);
#else
		digitalWrite(kPIN_TURN_POWER_OFF, HIGH);
#endif
	}
}


// Update the button state based on it's current state and whether the button is currently pressed or not
void UpdateButtonState(void)
{
	int button_pressed = digitalRead(kPIN_POWER_BUTTON_PRESSED);

	if( button_pressed ) {
		if(SystemState.button_pressed_count < 255)
			SystemState.button_pressed_count+= 1;
	} else {
		SystemState.button_pressed_count = 0;
	}
	
	switch(SystemState.button_state) {
		case kButtonWaitNotPressed:
			// This state is only used only on powerup, and prevents a spurious ButtonJustReleased
			// state if the button was pressed when powered up.
			if(button_pressed==0)
				SystemState.button_state = kButtonNotPressed;
			break;
			
		case kButtonNotPressed:
			if(SystemState.button_pressed_count >= kButtonDebounceCount)
				SystemState.button_state = kButtonPressed;					
			break;
				
		case kButtonPressed:
			if(button_pressed == 0)
				SystemState.button_state = kButtonJustReleased;
			else if(SystemState.button_pressed_count >= kButtonLongPressCount)
				SystemState.button_state = kButtonPressed_Long;
		break;

		case kButtonJustReleased:
			SystemState.button_state = kButtonNotPressed;
			break;
								
		case kButtonPressed_Long:
			if(button_pressed == 0)
				SystemState.button_state = kButtonNotPressed;
			break;
			
		default:
			// should never get here
			SystemState.button_state = kButtonNotPressed;
			SystemState.button_pressed_count = 0;
	}
}

// update the sleep request signal state, detecting new assertions
void UpdateSleepReqState(void)
{
	int req_state = digitalRead(kPIN_REQ_SLEEP_MODE);
	
	switch(SystemState.sleep_req_state) {

		case kSleepReq_NoRequest:
			if(req_state == 1)
				SystemState.sleep_req_state = kSleepReq_NewRequest;
			break;

		case kSleepReq_NewRequest:
			SystemState.sleep_req_state = kSleepReq_Handshake;
			break;

		case kSleepReq_Handshake:
			if(req_state == 0)
				SystemState.sleep_req_state = kSleepReq_NoRequest;
			break;
	
	}
}


// Update the USB plugged-in state based on the power and data line voltage inputs
void UpdateUSBState(void)
{
	int16_t adcval;
		
	int usb_power_sense_l = digitalRead(kPIN_USB_POWER_SENSE_L);


	switch(SystemState.usb_state) {
		case kUSB_PowerupSettle:
			if( SystemState.usb_debounce_count < 255 )
				SystemState.usb_debounce_count += 1;
			if(SystemState.usb_debounce_count >=kUSBPowerupSettleCount)
				SystemState.usb_state = kUSB_Debounce;
			break;
			
		case kUSB_Unplugged:
			if(usb_power_sense_l == 0) {
				SystemState.usb_state = kUSB_Debounce;
				SystemState.usb_debounce_count = 0;
			}
			break;
			
		case kUSB_Debounce:
			if(usb_power_sense_l != 0) {
				SystemState.usb_state = kUSB_Unplugged;
			} else {
				if( SystemState.usb_debounce_count < 255 )
					SystemState.usb_debounce_count += 1;
				if( (SystemState.usb_debounce_count > kUSBDebounceCount) && (SystemState.adc_in_use==0) ) {
					SystemState.usb_state = kUSB_PluggedMeasureDPlus;
					// start an ADC measurement of the D+ voltage
					analogReadStart(kPIN_USB_DPLUS_SENSE);
					SystemState.adc_in_use = 1;
				}
			}
			break;
				
		case kUSB_PluggedMeasureDPlus:
			if(usb_power_sense_l != 0) {
				SystemState.usb_state = kUSB_Unplugged;
			} else {
				adcval = analogReadFinish();    // get the D+ voltage measurement
				if( (adcval >= kADCVAL_USB_MIN) && (adcval <= kADCVAL_USB_MAX) ) {
					// the D+ voltage looks good, so check the D- voltage
					analogReadStart(kPIN_USB_DMINUS_SENSE);
					SystemState.usb_state = kUSB_PluggedMeasureDMinus;
				}
				else {
					// the d+ voltage didn't look good for a high-current charger, so set the USB state
					// for low current charging
					SystemState.usb_state = kUSB_PluggedInLowCurrent;
					SystemState.adc_in_use = 0;
				}
			}
			break;
				
		case kUSB_PluggedMeasureDMinus:
			if(usb_power_sense_l != 0) {
				SystemState.usb_state = kUSB_Unplugged;
			} else {
				adcval = analogReadFinish();    // get the D- voltage measurement
				SystemState.adc_in_use = 0;
				if( (adcval >= kADCVAL_USB_MIN) && (adcval <= kADCVAL_USB_MAX) )
					SystemState.usb_state = kUSB_PluggedInHighCurrent;
				else
					SystemState.usb_state = kUSB_PluggedInLowCurrent;
			}
			break;
				
		case kUSB_PluggedInLowCurrent:
		case kUSB_PluggedInHighCurrent:
			if(usb_power_sense_l != 0)
				SystemState.usb_state = kUSB_JustUnplugged;
			break;
		
		case kUSB_JustUnplugged:
			SystemState.usb_state = kUSB_Unplugged;
			
		default:
			// should never get here
			SystemState.usb_state = kUSB_Unplugged;
	}
}

// Update the battery charging and charge level states
void UpdateBatteryState(void)
{
	int16_t adcval;
		
	// battery measurements are only done about once per second
	if(SystemState.battery_measure_delay > 0)
		SystemState.battery_measure_delay -= 1;
		
	switch(SystemState.battery_measure_state) {
		case kBatteryMeasureWaiting:
			if( (SystemState.battery_measure_delay == 0) && (SystemState.adc_in_use==0) ) {
				analogReadStart(kPIN_BATTERY_V_SENSE);     // start a new battery voltage measurement
				SystemState.battery_measure_state = kBatteryMeasureInProcess;
				SystemState.adc_in_use = 1;         // let everyone else know the ADC is being used
			}
			break;
			
		case  kBatteryMeasureInProcess:
			adcval = analogReadFinish();    // get the battery voltage measurement
			SystemState.adc_in_use = 0;             // done using the ADC
			SystemState.battery_measure_delay = kBatteryMeasureCount;
			SystemState.battery_measure_state = kBatteryMeasureWaiting;
			if(adcval >= kADCVAL_BATTERY_FULL)
				SystemState.battery_charge_state = kBattery_FullCharge;
			else
				SystemState.battery_charge_state = kBattery_LowCharge;
			break;
			
		default:
			// should never get here
			SystemState.battery_measure_delay = 0;
			SystemState.battery_measure_state = kBatteryMeasureWaiting;
			SystemState.adc_in_use = 0; 
	}
}


// update the power and battery charging modes
void UpdatePowerMode(void)
{
	uint8_t grp2_on, bus_on, charger_on;

	switch(SystemState.power_state) {
		
		case kJustPoweredUp:
			// This should be the first state visited after being powered on.
			// Once exited, this state is never returned to.
			// Power on can be caused by either USB being plugged in or, if USB not plugged in, the button being pressed
			// If the USB was plugged in and stays plugged in, this state will persist until the charge level is decided.
			if(SystemState.usb_state == kUSB_Unplugged) {
				// USB isn't plugged in, so must have been a button push
				SystemState.power_state = kPowerActive;
				SystemState.new_power_state = 1;
			} else if(SystemState.usb_state == kUSB_JustUnplugged) {
				// probably a USB connector bounce. Turn off
				TurnPowerOff();
			} else if( (SystemState.usb_state == kUSB_PluggedInLowCurrent) || (SystemState.usb_state == kUSB_PluggedInHighCurrent) ) {
				SystemState.power_state = kPowerCharging;
				SystemState.new_power_state = 1;
			}
			break;
		
		case kPowerCharging:
			// plugged into USB and charging the battery
			if( SystemState.button_state == kButtonJustReleased) {
				SystemState.power_state = kPowerActiveCharging;
				SystemState.new_power_state = 1;
			} else if( (SystemState.usb_state == kUSB_JustUnplugged) || (SystemState.usb_state == kUSB_Unplugged) ) {
				TurnPowerOff();
			}
			break;

		case kPowerSleep:
			// running on battery with the bus powered down but the rest of the system active.
			if(SystemState.button_state == kButtonPressed_Long) {
				TurnPowerOff();
			} else if( (SystemState.button_state == kButtonJustReleased) || (SystemState.sleep_req_state == kSleepReq_NewRequest) ) {
				SystemState.power_state = kPowerActive;
				SystemState.new_power_state = 1;
			} else if( (SystemState.usb_state == kUSB_PluggedInHighCurrent) || (SystemState.usb_state == kUSB_PluggedInLowCurrent) ) {
				SystemState.power_state = kPowerSleepCharging;
				SystemState.new_power_state = 1;
			}
			break;

		case kPowerActive:
			// Running on battery with the bus powered and the whole system active
			if(SystemState.button_state == kButtonPressed_Long) {
				TurnPowerOff();
			} else if( (SystemState.button_state == kButtonJustReleased) || (SystemState.sleep_req_state == kSleepReq_NewRequest) ) {
				SystemState.power_state = kPowerSleep;
				SystemState.new_power_state = 1;
			} else if( (SystemState.usb_state == kUSB_PluggedInHighCurrent) || (SystemState.usb_state == kUSB_PluggedInLowCurrent) ) {
				SystemState.power_state = kPowerActiveCharging;
				SystemState.new_power_state = 1;
			}
			break;

		case kPowerSleepCharging:
			// Running on USB, charging the battery, and with the bus not powered and the rest of the system all active
			if(SystemState.button_state == kButtonPressed_Long) {
				SystemState.power_state = kPowerCharging;
				SystemState.new_power_state = 1;
			} else if( (SystemState.button_state == kButtonJustReleased) || (SystemState.sleep_req_state == kSleepReq_NewRequest) ) {
				// transition to active mode
				SystemState.power_state = kPowerActiveCharging;
				SystemState.new_power_state = 1;
			} else if( (SystemState.usb_state != kUSB_PluggedInHighCurrent) && (SystemState.usb_state != kUSB_PluggedInLowCurrent) ) {
				// USB power removed - stay in sleep mode but stop charging
				SystemState.power_state = kPowerSleep;
				SystemState.new_power_state = 1;
			}
			break;

		case kPowerActiveCharging:
			// Running on USB, charging the battery, and with the bus powered and the whole system active
			if(SystemState.button_state == kButtonPressed_Long) {
				SystemState.power_state = kPowerCharging;
				SystemState.new_power_state = 1;
			} else if( (SystemState.button_state == kButtonJustReleased) || (SystemState.sleep_req_state == kSleepReq_NewRequest) ) {
				SystemState.power_state = kPowerSleepCharging;
				SystemState.new_power_state = 1;
			} else if( (SystemState.usb_state != kUSB_PluggedInHighCurrent) && (SystemState.usb_state != kUSB_PluggedInLowCurrent) ) {
				// USB power removed - stay in active mode but stop charging
				SystemState.power_state = kPowerActive;
				SystemState.new_power_state = 1;
			}
			break;
		
		default:
			// should never get here
			TurnPowerOff();
	}

	if(SystemState.new_power_state) {
		SystemState.new_power_state = 0;

		switch( SystemState.power_state) {
			case kPowerActive:
				grp2_on = HIGH;
				bus_on = HIGH;
				charger_on = LOW;
				break;

			case kPowerActiveCharging:
				grp2_on = HIGH;
				bus_on = HIGH;
				charger_on = HIGH;
				break;
			
			case kPowerSleep:
				grp2_on = HIGH;
				bus_on = LOW;
				charger_on = LOW;
				break;

			case kPowerSleepCharging:
				grp2_on = HIGH;
				bus_on = LOW;
				charger_on = HIGH;
				break;

			case kPowerCharging:
				grp2_on = LOW;
				bus_on = LOW;
				charger_on = HIGH;
				break;
				
			case kJustPoweredUp:
				grp2_on = LOW;
				bus_on = LOW;
				charger_on = LOW;

			default:
				// should never get here
				grp2_on = LOW;
				bus_on = LOW;
				charger_on = LOW;
		}

			digitalWrite(kPIN_ENABLE_POWER_GRP2, grp2_on);
			digitalWrite(kPIN_ENABLE_BUS_POWER, bus_on);

			if( (charger_on == HIGH) && (SystemState.usb_state == kUSB_PluggedInHighCurrent) )
				digitalWrite(kPIN_CHARGER_SEL_HIGH_CURRENT, HIGH);
			else
				digitalWrite(kPIN_CHARGER_SEL_HIGH_CURRENT, LOW);
			digitalWrite(kPIN_CHARGER_ENABLE, charger_on);
	}
}


// update the LEDs
void UpdateLEDs(void)
{
	uint8_t green_off, red_off, blink_change, blinking;

	// The blinking state free runs regardless of whether the LEDs are actually blinking or not
	// Whether they actually get turned off due to blinking is determined further down
	if( SystemState.led_blink_delay == 0) {
		SystemState.led_blink_delay = kLEDBlinkCount;
		blink_change = 1;
		if(SystemState.led_blink_on)
			SystemState.led_blink_on = 0;
		else
			SystemState.led_blink_on = 1;
	} else {
		SystemState.led_blink_delay -= 1;
		blink_change = 0;
	}
	
	// Only update the i/o ports when something changes
	if( (blink_change) || (SystemState.new_power_state)) {

		// set the default colors and no blinking. May get overridden further down
		green_off = 0;
		if(SystemState.battery_charge_state == kBattery_FullCharge)
			red_off = 1;	// green
		else
			red_off = 0;	// Amber
		blinking = 0;
		
		// determine that the LEDs are doing based on the current power state
		switch(SystemState.power_state) {
			case kJustPoweredUp:
				// keep the LEDs off until the power stabalizes
				green_off = 1;
				red_off = 1;
				break;
				
			case kPowerCharging:
				// set the color to red
				green_off = 1;
				red_off = 0;
				if(SystemState.battery_charge_state == kBattery_LowCharge)
					blinking = 1;
				break;

			case kPowerSleepCharging:
			case kPowerSleep:
				blinking = 1;
				break;

			default:
				break;
		}

		if( blinking && (SystemState.led_blink_on==0) ) {
			green_off = 1;
			red_off = 1;
		}

		digitalWrite(kPIN_GREEN_POWER_LED_ON_L, green_off);
		digitalWrite(kPIN_RED_POWER_LED_ON_L, red_off);
	}

}



/****************************
The timer interrupt service routine
Updates the state
****************************/
ISR(TIMER0_COMPA_vect)
{
	static uint8_t startupCount=0;
	
	// Startup delay - for debug and maybe longer...
	if(startupCount < 25) {
		startupCount += 1;
	} else {
	
		UpdateButtonState();
		UpdateSleepReqState();
		UpdateUSBState();
		UpdateBatteryState();
	
		UpdatePowerMode();
		UpdateLEDs();
	}
}

int main(void)
{
	// set the output pin states before enabling any of them
#ifdef TURN_POWER_OFF_ASSERT_LOW
	digitalWrite(kPIN_TURN_POWER_OFF,		HIGH);
#else
	digitalWrite(kPIN_TURN_POWER_OFF,		LOW);
#endif

	digitalWrite(kPIN_ENABLE_BUS_POWER,		LOW);
	digitalWrite(kPIN_CHARGER_ENABLE,		LOW);
	digitalWrite(kPIN_ENABLE_POWER_GRP2,	HIGH);
	digitalWrite(kPIN_GREEN_POWER_LED_ON_L, HIGH);
	digitalWrite(kPIN_RED_POWER_LED_ON_L,   HIGH);

	//pinMode(kPIN_BATTERY_V_SENSE,			INPUT);			// ADC0
	//pinMode(kPIN_USB_DMINUS_SENSE,		INPUT);			// ADC1
	//pinMode(kPIN_USB_DPLUS_SENSE,			INPUT);			// ADC2
	pinMode(kPIN_ENABLE_BUS_POWER,			OUTPUT);
	pinMode(kPIN_REQ_SLEEP_MODE,			INPUT);
	pinMode(kPIN_USB_POWER_SENSE_L,			INPUT);
	pinMode(kPIN_TURN_POWER_OFF,			OUTPUT);

	pinMode(kPIN_POWER_BUTTON_PRESSED,		INPUT);

	pinMode(kPIN_CHARGER_SEL_HIGH_CURRENT,	OUTPUT);
	pinMode(kPIN_CHARGER_ENABLE,			OUTPUT);
	pinMode(kPIN_CHARGER_STAT2,				INPUT_PULLUP);
	pinMode(kPIN_CHARGER_STAT1,				INPUT_PULLUP);
	pinMode(kPIN_ENABLE_POWER_GRP2,			OUTPUT);
	pinMode(kPIN_GREEN_POWER_LED_ON_L,		OUTPUT);
	pinMode(kPIN_RED_POWER_LED_ON_L,		OUTPUT);

	
	// set up the ADC
	AMISCR = 0;
	ADCSRA = (1<<ADEN) | (7<<ADPS0); // Set the ADC clock prescaler to divide by 128, and enable the ADC
	
	// Turn off the digital input buffers on the analog input pins (saves some power)
	DIDR0 = ((kPIN_BATTERY_V_SENSE&0xFF) | (kPIN_USB_DMINUS_SENSE&0xFF) | (kPIN_USB_DPLUS_SENSE&0xFF));
	
	// set up 8-bit timer 0 to generate an interrupt at a 50Hz rate
	TCCR0A = (1<<WGM01);					// CTC mode - clear on compare
	ASSR = 0;								// normal synchronous mode
	OCR0A = kTCCR0_COUNT_VALUE;				// sets the compare value for interrupt generation
	TCNT0 = 0;								// make sure the count starts at 0
	GTCCR = (1<<PSR0);						// reset the prescaler
	TCCR0B = kTCCR0_PRESCALE_SELECT<<CS00;	// set timer prescaler divide by 1024. Clock = 8MHz/1024=7812.5Hz
	TIMSK0 = (1<<OCIE0A);					// enable interrupt on compare

	set_sleep_mode(SLEEP_MODE_IDLE);

	sei();									// enable interrupts
	
	while(1)
	{
		sleep_mode();		//just keep going to sleep
	}
}



