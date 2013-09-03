/*
 * PowerManager.c - Power Manager microcontroller firmware for the Bluetooth Module (U10)
 *
 * Target device: ATTiny87
 *
 * Created: 9/2/2013 12:13:14 PM
 *  Author: John VanLaanen for Seamless Toy Company
 */ 


#include <avr/io.h>
#include <avr/interrupt.h>
#include <avr/sleep.h>
#include <stdint.h>

#include "AtomDefs.h"
#include "PowerManager.h"
#include "moduleIO.h"


typedef enum {
	kButtonNotPressed=0,
	kButtonJustPressed,
	kButtonPressed,
	kButtonPressed_Long,
	kButtonPressed_Ignore,
} tPowerButtonStates;


typedef enum {
	kPowerOff=0,
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
	kLED_Off=0,
	kLED_Green,
	kLED_Red,
	kLED_Amber,
} tLEDColors;

typedef enum {
	kOff=0,
	kOn,
	kBlinking_On,
	kBlinking_Off,
} tLEDModes;

typedef enum {
	kUSB_Unplugged=0,
	kUSB_PluggedMeasureDPlus,
	kUSB_PluggedMeasureDMinus,
	kUSB_PluggedInLowCurrent,
	kUSB_PluggedInHighCurrent,
} tUSBStates;


static struct {
	uint8_t               button_pressed_count;
	uint8_t               led_blink_delay;
	uint8_t               usb_debounce_count;
	uint8_t               battery_measure_delay;
	uint8_t               adc_in_use;
	tModulePowerStates    power_state;
	tPowerButtonStates    button_state;
	tUSBStates            usb_state;
	tBatteryChargeStates  battery_charge_state;
	tBatteryMeasureStates battery_measure_state;
	tLEDColors            led_color;
	tLEDModes             led_mode;
	} SystemState={0,0,0,0,0,0,0,0,0,0,0};

// Update the button state based on it's current state and whether the button is currently pressed or not
void UpdateButtonState(void)
{
	int button_pressed = digitalRead(kPIN_POWER_BUTTON_PRESSED);
	if(button_pressed == 0) {
		// If the button signal ever indicates it's not pressed, set the button
		//  state unconditionally to not pressed.
		SystemState.button_state = kButtonNotPressed;
		SystemState.button_pressed_count = 0;
	} else {
		// the button is pressed
		if(SystemState.button_pressed_count < 255)
		SystemState.button_pressed_count+= 1;
			
		if(SystemState.button_state == kButtonNotPressed) {
			if(SystemState.button_pressed_count >= kButtonDebounceCount){
				SystemState.button_state = kButtonJustPressed;
			} else if(SystemState.button_state == kButtonJustPressed)  {
				SystemState.button_state = kButtonPressed;
			} else if(SystemState.button_state == kButtonPressed) {
				if(SystemState.button_pressed_count >= kButtonLongPressCount) {
					SystemState.button_state = kButtonPressed_Long;
				}
			} else if(SystemState.button_state == kButtonPressed_Long)  {
				SystemState.button_state = kButtonPressed_Ignore;
			} else {
				// should never get here
				SystemState.button_state = kButtonNotPressed;
				SystemState.button_pressed_count = 0;
			}
		}
	}
}

	// Update the USB plugged-in state based on the power and data line voltage inputa
	void UpdateUSBState(void)
	{
		int16_t adcval;
		
		int usb_power_sense_l = digitalRead(kPIN_USB_POWER_SENSE_L);
		if(usb_power_sense_l != 0) {
			// If the USB power sense ever indicates it's unplugged, unconditionally set
			//  the USB state to unplugged
			SystemState.usb_state = kUSB_Unplugged;
			SystemState.usb_debounce_count = 0;
		}
		else {
			// USB is plugged in
			if( SystemState.usb_debounce_count < 255 )
			SystemState.usb_debounce_count += 1;

			if( SystemState.usb_state == kUSB_Unplugged) {
				if( (SystemState.usb_debounce_count > kUSBDebounceCount) && (SystemState.adc_in_use==0) ) {
					SystemState.usb_state = kUSB_PluggedMeasureDPlus;
					
					// start an ADC measurement of the D+ voltage
					analogReadStart(kPIN_USB_DPLUS_SENSE);
					SystemState.adc_in_use = 1;
				}
				} else if( SystemState.usb_state == kUSB_PluggedMeasureDPlus) {
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
				} else if( SystemState.usb_state == kUSB_PluggedMeasureDMinus) {
				adcval = analogReadFinish();    // get the D- voltage measurement
				SystemState.adc_in_use = 0;
				if( (adcval >= kADCVAL_USB_MIN) && (adcval <= kADCVAL_USB_MAX) )
				SystemState.usb_state = kUSB_PluggedInHighCurrent;
				else
				SystemState.usb_state = kUSB_PluggedInLowCurrent;
				} else if( (SystemState.usb_state != kUSB_PluggedInLowCurrent) && (SystemState.usb_state != kUSB_PluggedInHighCurrent) ) {
				// should never get here
				SystemState.usb_state = kUSB_Unplugged;
				SystemState.usb_debounce_count = 0;
			}
		}
	}

// Update the battery charging and charge level states
void UpdateBatteryState(void)
{
	int16_t adcval;
		
	// battery measurements are only done about once per second
	if(SystemState.battery_measure_delay > 0)
	SystemState.battery_measure_delay -= 1;
		
	if(SystemState.battery_measure_state == kBatteryMeasureWaiting) {
		if( (SystemState.battery_measure_delay == 0) && (SystemState.adc_in_use==0) )
		analogReadStart(kPIN_BATTERY_V_SENSE);     // start a new battery voltage measurement
		SystemState.battery_measure_state = kBatteryMeasureInProcess;
		SystemState.adc_in_use = 1;         // let everyone else know the ADC is being used
	} else if(SystemState.battery_measure_state == kBatteryMeasureInProcess) {
		adcval = analogReadFinish();    // get the battery voltage measurement
		SystemState.adc_in_use = 0;             // done using the ADC
		SystemState.battery_measure_delay = kBatteryMeasureInterval;
		SystemState.battery_measure_state = kBatteryMeasureWaiting;
		if(adcval >= kADCVAL_BATTERY_FULL)
			SystemState.battery_charge_state = kBattery_FullCharge;
		else
			SystemState.battery_charge_state = kBattery_LowCharge;
	}
}


// update the power control mode
void UpdatePowerMode(void)
{
	if(SystemState.power_state == kPowerOff) {
		// power is off
	}

}


// update the LEDs
void UpdateLEDs(void)
{
	// determine the color based on the charge level and power mode

}

// Update the battery charging control
void UpdateCharger(void)
{


}


/****************************
The timer interrupt service routine
Updates the state
****************************/
ISR(TIMER0_COMPA_vect)
{
	UpdateButtonState();
	UpdateUSBState();
	UpdateBatteryState();
	
	UpdatePowerMode();
	UpdateLEDs();
	UpdateCharger();
}

int main(void)
{
	// set up the output pins and turn the Group2 power on
	//pinMode(kPIN_BATTERY_V_SENSE,			INPUT);			// ADC0
	//pinMode(kPIN_USB_DMINUS_SENSE,		INPUT);			// ADC1
	//pinMode(kPIN_USB_DPLUS_SENSE,			INPUT);			// ADC2
	pinMode(kPIN_ENABLE_BUS_POWER,			OUTPUT);
	pinMode(kPIN_REQ_SLEEP_MODE,			INPUT);
	pinMode(kPIN_USB_POWER_SENSE_L,			INPUT);
	pinMode(kPIN_TURN_POWER_OFF_L,			OUTPUT);
	pinMode(kPIN_POWER_BUTTON_PRESSED,		INPUT);

	pinMode(kPIN_CHARGER_SEL_HIGH_CURRENT,	OUTPUT);
	pinMode(kPIN_CHARGER_ENABLE,			OUTPUT);
	pinMode(kPIN_CHARGER_STAT2,				INPUT_PULLUP);
	pinMode(kPIN_CHARGER_STAT1,				INPUT_PULLUP);
	pinMode(kPIN_ENABLE_POWER_GRP2,			OUTPUT);
	pinMode(kPIN_GREEN_POWER_LED_ON_L,		OUTPUT);
	pinMode(kPIN_RED_POWER_LED_ON_L,		OUTPUT);

	digitalWrite(kPIN_TURN_POWER_OFF_L,		HIGH);
	digitalWrite(kPIN_ENABLE_BUS_POWER,		LOW);
	digitalWrite(kPIN_CHARGER_ENABLE,		LOW);
	digitalWrite(kPIN_ENABLE_POWER_GRP2,	HIGH);
	digitalWrite(kPIN_GREEN_POWER_LED_ON_L, HIGH);
	digitalWrite(kPIN_RED_POWER_LED_ON_L,   HIGH);
	
	// set up the ADC
	AMISCR = 0;
	ADCSRA = (1<<ADEN) | (7<<ADPS0); // Set the ADC clock prescaler to divide by 128, and enable the ADC
	
	// Turn off the digital input buffers on the analog input pins (saves some power)
	DIDR0 = ((kPIN_BATTERY_V_SENSE&0xFF) | (kPIN_USB_DMINUS_SENSE&0xFF) | (kPIN_USB_DPLUS_SENSE&0xFF));
	
	// set up 8-bit timer 0 to generate an interrupt at a 50Hz rate
	TCCR0A = (1<<WGM01);					// CTC mode - clear on compare
	ASSR = 0;								// normal synchronous mode
	OCR0A = TCCR0_COUNT_VALUE;				// sets the compare value for interrupt generation
	TCNT0 = 0;								// make sure the count starts at 0
	GTCCR = (1<<PSR0);						// reset the prescaler
	TCCR0B = TCCR0_PRESCALE_SELECT<<CS00;	// set timer prescaler divide by 1024. Clock = 8MHz/1024=7812.5Hz
	TIMSK0 = (1<<OCIE0A);					// enable interrupt on compare

	set_sleep_mode(SLEEP_MODE_IDLE);

	sei();									// enable interrupts
	
	while(1)
	{
		sleep_mode();		//just keep going to sleep
	}
}


