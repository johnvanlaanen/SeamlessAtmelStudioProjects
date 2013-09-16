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
	kButtonState_NotPressed=0,
	kButtonState_Pressed,
	kButtonState_PressedLong,
	kButtonState_JustReleased,
} tPowerButtonStates;

typedef enum {
	kSleepReqState_NoRequest=0,
	kSleepReqState_NewRequest,
	kSleepReqState_Handshake,
} tSleepRequestStates;

typedef enum {
	kPowerState_JustPoweredUp=0,
	kPowerState_Charging,
	kPowerState_Sleep,
	kPowerState_Active,
	kPowerState_SleepCharging,
	kPowerState_ActiveCharging,
} tModulePowerStates;

typedef enum {
	kBatteryState_FullCharge=0,
	kBatteryState_LowCharge,
} tBatteryChargeStates;

typedef enum {
	kBatteryMeasureState_Waiting=0,
	kBatteryMeasureState_InProcess,
} tBatteryMeasureStates;


typedef enum {
	kUSBState_PowerupSettle=0,
	kUSBState_Unplugged,
	kUSBState_Debounce,
	kUSBState_PluggedMeasureDelay,
	kUSBState_PluggedMeasureDPlus,
	kUSBState_PluggedMeasureDMinus,
	kUSBState_PluggedIn,
} tUSBStates;


volatile struct {
	uint8_t				  power_state_timeout_count;
	uint8_t               button_pressed_count;
	uint8_t               led_blink_delay;
	uint8_t				  led_blink_on;
	uint8_t               usb_debounce_count;
	uint8_t               battery_measure_delay;
	uint8_t               adc_in_use;
	uint8_t				  new_power_state;
	uint8_t				  usb_power_available;
	uint8_t				  usb_high_power_available;
	tModulePowerStates    power_state;
	tPowerButtonStates    button_state;
	tSleepRequestStates	  sleep_req_state;
	tUSBStates            usb_state;
	tBatteryChargeStates  battery_charge_state;
	tBatteryMeasureStates battery_measure_state;
} SystemState= {.power_state_timeout_count = 0,
				.button_pressed_count	= 0,
				.led_blink_delay		= 0,
				.led_blink_on			= 0,
				.usb_debounce_count		= 0,
				.battery_measure_delay	= 0,
				.adc_in_use				= 0,
				.new_power_state		= 1,
				.usb_power_available	= 0,
				.usb_high_power_available = 0,
				.power_state			= kPowerState_JustPoweredUp,
				.button_state			= kButtonState_NotPressed,
				.sleep_req_state		= kSleepReqState_NoRequest,
				.usb_state				= kUSBState_PowerupSettle,
				.battery_charge_state	= kBatteryState_FullCharge,
				.battery_measure_state	= kBatteryMeasureState_Waiting };

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
#ifdef BUTTON_PRESS_ASSERT_LOW
	int button_pressed = !digitalRead(kPIN_POWER_BUTTON_PRESSED);
#else
	int button_pressed = digitalRead(kPIN_POWER_BUTTON_PRESSED);
#endif

	if( button_pressed ) {
		if(SystemState.button_pressed_count < 255)
			SystemState.button_pressed_count+= 1;
	} else {
		SystemState.button_pressed_count = 0;
	}
	
	switch(SystemState.button_state) {
		case kButtonState_NotPressed:
			if(SystemState.button_pressed_count >= kButtonDebounceCount)
				SystemState.button_state = kButtonState_Pressed;					
			break;
				
		case kButtonState_Pressed:
			if(button_pressed == 0)
				SystemState.button_state = kButtonState_JustReleased;
			else if(SystemState.button_pressed_count >= kButtonLongPressCount)
				SystemState.button_state = kButtonState_PressedLong;
		break;

		case kButtonState_JustReleased:
			SystemState.button_state = kButtonState_NotPressed;
			break;
								
		case kButtonState_PressedLong:
			if(button_pressed == 0)
				SystemState.button_state = kButtonState_NotPressed;
			break;
			
		default:
			// should never get here
			SystemState.button_state = kButtonState_NotPressed;
			SystemState.button_pressed_count = 0;
	}
}


// update the BT sleep request signal state, detecting new assertions
// function is a no-op for the rechargeable battery module
void UpdateSleepReqState(void)
{
#ifdef BLUETOOTH_MODULE
	int req_state = digitalRead(kPIN_REQ_SLEEP_MODE);
	
	switch(SystemState.sleep_req_state) {

		case kSleepReqState_NoRequest:
			if(req_state == 1)
				SystemState.sleep_req_state = kSleepReqState_NewRequest;
			break;

		case kSleepReqState_NewRequest:
			SystemState.sleep_req_state = kSleepReqState_Handshake;
			break;

		case kSleepReqState_Handshake:
			if(req_state == 0)
				SystemState.sleep_req_state = kSleepReqState_NoRequest;
			break;
	
	}
#endif
}


// Update the USB plugged-in state based on the power and data line voltage inputs
void UpdateUSBState(void)
{
	int16_t adcval;
		
	int usb_power_sense_l = digitalRead(kPIN_USB_POWER_SENSE_L);

	switch(SystemState.usb_state) {
		case kUSBState_PowerupSettle:
			// Provides some additional settling time on powerup, before moving to the debounce state
			// Should never return to this state once it's left
			SystemState.usb_power_available = 0;
			SystemState.usb_high_power_available = 0;
			if( SystemState.usb_debounce_count < 255 )
				SystemState.usb_debounce_count += 1;
			if(SystemState.usb_debounce_count >=kUSBPowerupSettleCount)
				SystemState.usb_state = kUSBState_Debounce;
				SystemState.new_power_state = 1;
			break;
			
		case kUSBState_Unplugged:
			// USB isn't plugged in. Leave to the debounce state if power detected.
			SystemState.usb_power_available = 0;
			SystemState.usb_high_power_available = 0;
			if(usb_power_sense_l == 0) {
				SystemState.usb_state = kUSBState_Debounce;
				SystemState.usb_debounce_count = 0;
			}
			break;
			
		case kUSBState_Debounce:
			// power was detected. Make sure it persists before saying it's on
			if(usb_power_sense_l != 0) {
				SystemState.usb_state = kUSBState_Unplugged;
			} else {
				if( SystemState.usb_debounce_count < 255 )
					SystemState.usb_debounce_count += 1;
				if(SystemState.usb_debounce_count > kUSBDebounceCount) {
					SystemState.usb_debounce_count = 0;	// reset count for the next state
					SystemState.usb_state = kUSBState_PluggedMeasureDelay;
					SystemState.usb_power_available = 1;
					SystemState.new_power_state = 1;
				}
			}
			break;
		
		case kUSBState_PluggedMeasureDelay:
			// USB power is on. Wait a bit to measure the D+/- voltages to determine charging level
			if(usb_power_sense_l != 0) {
				SystemState.usb_state = kUSBState_Unplugged;
				SystemState.usb_power_available = 0;
				SystemState.usb_high_power_available = 0;
				SystemState.new_power_state = 1;
			} else {		
				if( SystemState.usb_debounce_count < 255 )
					SystemState.usb_debounce_count += 1;
				
				if( (SystemState.usb_debounce_count > kUSBMeasureDelayCount)  && (SystemState.adc_in_use==0) ) {
					// start an ADC measurement of the D+ voltage once the delay expires
					analogReadStart(kPIN_USB_DPLUS_SENSE);
					SystemState.adc_in_use = 1;
					SystemState.usb_state = kUSBState_PluggedMeasureDPlus;
				}
			}	
			break;
		
		case kUSBState_PluggedMeasureDPlus:	
			// gets the ADC conversion result for the D+ line and decides what to do next.
			if(usb_power_sense_l != 0) {
				SystemState.usb_state = kUSBState_Unplugged;
				SystemState.usb_power_available = 0;
				SystemState.usb_high_power_available = 0;
				SystemState.new_power_state = 1;
				SystemState.adc_in_use = 0;
			} else {
				adcval = analogReadFinish();    // get the D+ voltage measurement
				
				if( (adcval >= kADCVAL_USB_DATA_MIN) && (adcval <= kADCVAL_USB_DATA_MAX) ) {
					// the D+ voltage looks good, so check the D- voltage, keeping ownership of the ADC
					analogReadStart(kPIN_USB_DMINUS_SENSE);
					SystemState.usb_state = kUSBState_PluggedMeasureDMinus;
				} else {
					// the d+ voltage didn't look good for a high-current charger, so leave the charging
					// current low and release the ADC
					SystemState.usb_state = kUSBState_PluggedIn;
					SystemState.adc_in_use = 0;
				}
			}
			break;
				
		case kUSBState_PluggedMeasureDMinus:
			// gets the ADC conversion result for the D- line and decides what to do next.
			SystemState.adc_in_use = 0;		// done using the ADC no matter what
			if(usb_power_sense_l != 0) {
				SystemState.usb_state = kUSBState_Unplugged;
				SystemState.usb_power_available = 0;
				SystemState.usb_high_power_available = 0;
				SystemState.new_power_state = 1;
			} else {
				adcval = analogReadFinish();    // get the D- voltage measurement
				SystemState.usb_state = kUSBState_PluggedIn;	// we go here no matter what at this point
				if( (adcval >= kADCVAL_USB_DATA_MIN) && (adcval <= kADCVAL_USB_DATA_MAX) ) {
					// the adc voltage looks good. Set the high charge current flag
					SystemState.usb_high_power_available = 1;
					SystemState.new_power_state = 1;
				}
			}
			break;
				
		case kUSBState_PluggedIn:
			// stay in this state until USB unplugged
			if(usb_power_sense_l != 0) {
				SystemState.usb_state = kUSBState_Unplugged;
				SystemState.usb_power_available = 0;
				SystemState.usb_high_power_available = 0;
				SystemState.new_power_state = 1;
			}
			break;

		default:
			// should never get here
			SystemState.usb_state = kUSBState_Unplugged;
			SystemState.usb_power_available = 0;
			SystemState.usb_high_power_available = 0;
			SystemState.new_power_state = 1;
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
		case kBatteryMeasureState_Waiting:
			if( (SystemState.battery_measure_delay == 0) && (SystemState.adc_in_use==0) ) {
				analogReadStart(kPIN_BATTERY_V_SENSE);     // start a new battery voltage measurement
				SystemState.battery_measure_state = kBatteryMeasureState_InProcess;
				SystemState.adc_in_use = 1;         // let everyone else know the ADC is being used
			}
			break;
			
		case  kBatteryMeasureState_InProcess:
			adcval = analogReadFinish();    // get the battery voltage measurement
			SystemState.adc_in_use = 0;             // done using the ADC
			SystemState.battery_measure_delay = kBatteryMeasureCount;
			SystemState.battery_measure_state = kBatteryMeasureState_Waiting;
			if(adcval >= kADCVAL_BATTERY_FULL)
				SystemState.battery_charge_state = kBatteryState_FullCharge;
			else
				SystemState.battery_charge_state = kBatteryState_LowCharge;
			break;
			
		default:
			// should never get here
			SystemState.battery_measure_delay = 0;
			SystemState.battery_measure_state = kBatteryMeasureState_Waiting;
			SystemState.adc_in_use = 0; 
	}
}


// update the power and battery charging modes
void UpdatePowerMode(void)
{
	uint8_t grp2_on, bus_on, charger_on;

	switch(SystemState.power_state) {
		
		case kPowerState_JustPoweredUp:
			// This is the first state visited after being powered on.
			// Once exited, this state is never returned to.
			// Power on can be caused by either USB being plugged in or the button being pressed
			// This state persists until it's determined which happened or until a timeout is reached
			if(SystemState.button_state == kButtonState_PressedLong) {
				// the button was pressed and held for >5 seconds - just turn off
				TurnPowerOff();
			} else if(SystemState.button_state == kButtonState_JustReleased) {
				// the button was pressed and released - go active
				SystemState.power_state = kPowerState_Active;
				SystemState.new_power_state = 1;
			} else if( SystemState.usb_power_available ) {
				// USB power available - go to charging state
				SystemState.power_state = kPowerState_Charging;
				SystemState.new_power_state = 1;
			} else if(SystemState.power_state_timeout_count >= kPowerUpStateTimeoutCount) {
				// nothing happened within timeout - turn off
				TurnPowerOff();
			}
			if(SystemState.power_state_timeout_count < 255)
				SystemState.power_state_timeout_count += 1;
			break;
		
		case kPowerState_Charging:
			// plugged into USB and charging the battery. Remainder of the system powered off.
			if( SystemState.button_state == kButtonState_JustReleased) {
				SystemState.power_state = kPowerState_ActiveCharging;
				SystemState.new_power_state = 1;
			} else if( SystemState.usb_power_available==0 ) {
				TurnPowerOff();
			}
			break;

		case kPowerState_Sleep:
			// running on battery with the bus powered down but the rest of the system active.
			if(SystemState.button_state == kButtonState_PressedLong) {
				TurnPowerOff();
			} else if( (SystemState.button_state == kButtonState_JustReleased) || (SystemState.sleep_req_state == kSleepReqState_NewRequest) ) {
				// the button was pressed or BT requested going active - go to active state
				SystemState.power_state = kPowerState_Active;
				SystemState.new_power_state = 1;
			} else if( SystemState.usb_power_available ) {
				// USB was plugged in - keep the bus asleep but start charging the battery
				SystemState.power_state = kPowerState_SleepCharging;
				SystemState.new_power_state = 1;
			}
			break;

		case kPowerState_Active:
			// Running on battery with the bus powered and the whole system active
			if(SystemState.button_state == kButtonState_PressedLong) {
				// long button press turns the system off
				TurnPowerOff();
			} else if( (SystemState.button_state == kButtonState_JustReleased) || (SystemState.sleep_req_state == kSleepReqState_NewRequest) ) {
				// the button was pressed or BT requested a mode change - go to sleep state
				SystemState.power_state = kPowerState_Sleep;
				SystemState.new_power_state = 1;
			} else if( SystemState.usb_power_available == 1) {
				// USB was plugged in - stay active but start charging the battery
				SystemState.power_state = kPowerState_ActiveCharging;
				SystemState.new_power_state = 1;
			}
			break;

		case kPowerState_SleepCharging:
			// Running on USB, charging the battery, with the bus powered but the rest of the system all active
			if(SystemState.button_state == kButtonState_PressedLong) {
				// long button press turns the system off but keeps charging the battery
				SystemState.power_state = kPowerState_Charging;
				SystemState.new_power_state = 1;
			} else if( (SystemState.button_state == kButtonState_JustReleased) || (SystemState.sleep_req_state == kSleepReqState_NewRequest) ) {
				// the button was pressed or BT requested a transition - switch to active and keep charging the battery
				SystemState.power_state = kPowerState_ActiveCharging;
				SystemState.new_power_state = 1;
			} else if( SystemState.usb_power_available == 0 ) {
				// USB power removed - stay in sleep mode but stop charging
				SystemState.power_state = kPowerState_Sleep;
				SystemState.new_power_state = 1;
			}
			break;


		case kPowerState_ActiveCharging:
			// Running on USB, charging the battery, and with the bus powered and the whole system active
			if(SystemState.button_state == kButtonState_PressedLong) {
				// long button press turns the system off but keeps charging the battery
				SystemState.power_state = kPowerState_Charging;
				SystemState.new_power_state = 1;
			} else if( (SystemState.button_state == kButtonState_JustReleased) || (SystemState.sleep_req_state == kSleepReqState_NewRequest) ) {
				// the button was pressed or BT requested a transition - switch to sleep and keep charging the battery
				SystemState.power_state = kPowerState_SleepCharging;
				SystemState.new_power_state = 1;
			} else if( SystemState.usb_power_available == 0 ) {
				// USB power removed - stay in active mode but stop charging
				SystemState.power_state = kPowerState_Active;
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
			case kPowerState_Active:
				grp2_on = HIGH;
				bus_on = HIGH;
				charger_on = LOW;
				break;

			case kPowerState_ActiveCharging:
				grp2_on = HIGH;
				bus_on = HIGH;
				charger_on = HIGH;
				break;
			
			case kPowerState_Sleep:
				grp2_on = HIGH;
				bus_on = LOW;
				charger_on = LOW;
				break;

			case kPowerState_SleepCharging:
				grp2_on = HIGH;
				bus_on = LOW;
				charger_on = HIGH;
				break;

			case kPowerState_Charging:
				grp2_on = LOW;
				bus_on = LOW;
				charger_on = HIGH;
				break;
				
			case kPowerState_JustPoweredUp:
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

		if( (charger_on == HIGH) && (SystemState.usb_high_power_available == 1) )
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
		if(SystemState.battery_charge_state == kBatteryState_LowCharge)
			red_off = 0;	// Amber
		else
			red_off = 1;	// Green
		blinking = 0;
		
		// determine that the LEDs are doing based on the current power state
		switch(SystemState.power_state) {
			case kPowerState_JustPoweredUp:
				// keep the LEDs off until the power stabilizes
				green_off = 1;
				red_off = 1;
				break;
				
			case kPowerState_Charging:
				// set the color to red
				green_off = 1;
				red_off = 0;
				if(SystemState.battery_charge_state == kBatteryState_LowCharge)
					blinking = 1;
				break;

			case kPowerState_SleepCharging:
			case kPowerState_Sleep:
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
	
	// Startup delay - lets voltages settle
	if(startupCount < kPowerUpDelayCount) {
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
	digitalWrite(kPIN_ENABLE_POWER_GRP2,	LOW);
	digitalWrite(kPIN_GREEN_POWER_LED_ON_L, HIGH);
	digitalWrite(kPIN_RED_POWER_LED_ON_L,   HIGH);

	//pinMode(kPIN_BATTERY_V_SENSE,			INPUT);			// ADC0
	//pinMode(kPIN_USB_DMINUS_SENSE,		INPUT);			// ADC1
	//pinMode(kPIN_USB_DPLUS_SENSE,			INPUT);			// ADC2
	pinMode(kPIN_ENABLE_BUS_POWER,			OUTPUT);
#ifdef BLUETOOTH_MODULE
	pinMode(kPIN_REQ_SLEEP_MODE,			INPUT);
#else
	pinMode(kPIN_CHARGER_PG,				INPUT);
#endif
	
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



