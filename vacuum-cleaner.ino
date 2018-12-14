//
// Brushless Vaccum Cleaner Controller
//
// Brushless motor upgrade for the "Balck&Decker DustBuster Flexi"
// vacuum cleaner.
// This sketch controls the Brushless ESC by simulating the behavior 
// of a RC receiver. 
// The throttle is set to minimum for a preset amount of time to 
// allow the ESC to arm. The power is then slowly ramped-up until 
// reaching the target operating power setting.
// This sketch also controls a MOSFET module that keeps the ESC armed
// for a preset amount of time (5 min) after the power switch has been
// toggled to off.
//
// For reference:
// RPM and current measurement of original brushed motor
// with impeller attached and vacuum chamber removed:
// 28990 RPM / 5 Amperes
// Loading the motor would result in increased current draw.
//
// Hobby Wing ESC is configured to run at:
// 30000 RPM / 6 Amperes
// No current increase when loading the motor, the ESC keeps
// the current flow constant at all loads.
// 
// Author:  Karim Hraibi
// Version: 1.0.0
// Date:    19.11.2017
//
#define VERSION_MAJOR 1  // major version
#define VERSION_MINOR 0  // minor version
#define VERSION_MAINT 0  // maintenance version


#include <Servo.h>
#include <avr/wdt.h>

// Settings:
#define PPM_LOW  32                // PPM value before ramp-up (0..180) (minimum power)
#define PPM_HIGH 64                // PPM value after ramp-up (0..180) (operating power)
// #define PPM_HIGH 30
#define INITIAL_DELAY 4500         // delay period before starting with power ramp-up
#define RAMPUP_TIME  2048          // power ramp-up duration in ms
#define POWEROFF_DELAY 300000      // delay ESC power off in ms (=5min)
//#define POWEROFF_DELAY 10*1000


#define STEP_DURATION  (RAMPUP_TIME / (PPM_HIGH - PPM_LOW))  // duration between two power increments
#define LED_PIN  13         // output LED pin number (digital pin)
#define PPM_PIN  9          // output pin number for the PPM output (digital pin with PWM support)
#define SWITCH_PIN 10       // input pin number for the motor stop switch signal (high = stop)
#define MOSFET_PIN 11       // output pin number connected to the MOSFET module

// available states for the main state machine
typedef enum {
  STATE_INIT = 0,
  STATE_RAMPUP = 1,
  STATE_ON = 2,
  STATE_STANDBY_PREPARE = 3,
  STATE_STANDBY = 4
} State_t;

Servo servo;                 // create servo object to genarate the PPM signal required by the ESC
uint16_t ppmVal = PPM_LOW;   // value of ppm output signal in degrees (0..180)
State_t state = STATE_INIT;  // initial state of the main state machine
uint32_t timestamp;          // timestamp value in ms used for general time interval measurment
uint32_t timestampPowerOff;  // timestamp for counting the time to power off


/*
 * Arduino initialization routine
 */
void setup () {
  uint8_t sr = MCUSR;  
  // disable the watchdog  
  cli ();
  MCUSR = 0;
  WDTCSR |= _BV(WDCE) | _BV(WDE);
  WDTCSR = 0;  
  sei ();

  // Initialize HW
  pinMode (LED_PIN, OUTPUT);             
  pinMode (PPM_PIN, OUTPUT);         
  pinMode (SWITCH_PIN, INPUT);
  pinMode (MOSFET_PIN, OUTPUT);
  digitalWrite (MOSFET_PIN, HIGH);       // activate the MOSFET module to keep the ESC and Arduino powered on
  digitalWrite (LED_PIN, HIGH);          // sets the LED on
  servo.attach (PPM_PIN, 1000, 2000);    // attaches the servo on pin 9 to the servo object and sets range to 1000..2000 us
  servo.write (ppmVal);                  // sets the servo position (0..180)
  timestamp = millis ();                 // get the current time
 
  // check if reboot due to watchdog 
  if (((sr >> WDRF) & 1) == 1) {
    uint8_t i = 0;
    while (i < 10) {
      i = i + ledBlink (LED_PIN, 250, 250);
    }
  }  
  
  // enable the watchdog timer
  // Arduino Pro Mini bootloader bug: upon watchdog expiery, gets stuck in the reboot loop until power cycle.
  // This is not an issue for the current application as watchdog reset will cause the MOSFET module to power-off
  // the Arduino thus introduce a power cycle.
  cli ();
  wdt_enable (WDTO_1S);  // WDTO_1S = 1s
  sei ();
}


/*
 * Arduino infinite loop
 */
void loop () {
  
  //delay (5);     // wait some time (in ms) as not to poll too fast
 
  // main state machine
  switch (state) {
    
    case STATE_INIT:
      ledBlink (LED_PIN, 900, 100); // blink LED
      if (millis () - timestamp >= INITIAL_DELAY) { 
        timestamp = millis ();
        state = STATE_RAMPUP;
      }
      wdt_reset ();
      break;
    
    case STATE_RAMPUP:
      // check the elapsed time
      if (millis () - timestamp >= STEP_DURATION) { 
        timestamp = millis ();                // capture current time
        ppmVal++;                             // increment PPM value
        servo.write(ppmVal);                  // sets the servo position (0..180)
        ledToggle (LED_PIN);                  // toggle the LED
      }
      if (digitalRead (SWITCH_PIN) == HIGH) state = STATE_STANDBY_PREPARE; // check if motor kill switch is on
      if (ppmVal >= PPM_HIGH) state = STATE_ON; // check if target power setting is reached
      wdt_reset ();                            // reset the watchdog timer
      break;

    case STATE_ON:
      ledBlink (LED_PIN, 500, 500); // blink LED
      if (digitalRead (SWITCH_PIN) == HIGH) state = STATE_STANDBY_PREPARE; // check if motor kill switch is on
      wdt_reset ();
      break;

    case STATE_STANDBY_PREPARE:
      ppmVal = PPM_LOW;
      servo.write(ppmVal);              // turn off the motor
      timestampPowerOff = millis ();    // power-off countdown begins
      state = STATE_STANDBY;
      break;
      
    case STATE_STANDBY:
      ledBlink (LED_PIN, 250, 1750); // blink LED
      // check if power off time was reached
      if (millis () - timestampPowerOff >= POWEROFF_DELAY) {
        digitalWrite (MOSFET_PIN, LOW);    // power-off the system
      }
      if (digitalRead (SWITCH_PIN) == LOW) state = STATE_RAMPUP; // check if motor kill switch is off -> start the motor
      wdt_reset ();  
      break;

    default: // invalid state
      ppmVal = PPM_LOW;
      servo.write (ppmVal);              // turn off the motor
      digitalWrite (MOSFET_PIN, LOW);    // power-off the system
      break;
     
  }

}



/*
 * toggle LED on->off->on->...
 */
inline void ledToggle (uint8_t ledPin) {
  if (digitalRead (ledPin) == HIGH) {
    digitalWrite (ledPin, LOW);
  }
  else {
    digitalWrite (ledPin, HIGH);
  }
}

/* 
 * Blink the LED with a specific on and off duration in milliseconds.
 * Returns 1 for every on->off transition, otheriwse 0.
 */
inline uint8_t ledBlink (uint8_t ledPin, uint32_t onDuration, uint32_t offDuration) {
  static uint32_t ts = millis ();
  if (digitalRead (ledPin) == HIGH) { 
    if (millis () - ts >= onDuration) {
      ts = millis ();
      digitalWrite (LED_PIN, LOW);
      return 1;
    }
  }
  else {
    if (millis () - ts >= offDuration) {
      ts = millis ();
      digitalWrite (LED_PIN, HIGH);
    }
  }
  return 0;
}
