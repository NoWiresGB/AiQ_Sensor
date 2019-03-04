#include <avr/sleep.h>
#include <avr/wdt.h>
#include <avr/power.h>; 

#define EN3 4 // ATTiny pin 3 (PB4)
#define EN5 3 // ATTiny pin 2 (PB3)
#define SLEEPSIG 0  // ATTiny Pin 5 (PB0) 

// sleep bit patterns:
#define SLEEP1 0b000110 //  1 second:
#define SLEEP2 0b000111 //  2 seconds:
#define SLEEP4 0b100000 //  4 seconds: 
#define SLEEP8 0b100001 //  8 seconds: 

int run_state = 0;


ISR(WDT_vect) 
  {
  wdt_disable();  
  }

void myWatchdogEnable(const byte interval) 
  {
  wdt_reset();
    
  MCUSR = 0;                          // reset various flags
  WDTCR |= 0b00011000;               // see docs, set WDCE, WDE
  WDTCR =  0b01000000 | interval;    // set WDIE, and appropriate delay

  byte old_ADCSRA = ADCSRA;
  ADCSRA = 0;
  power_all_disable(); // shut down ADC, Timer 0 and 1, serial etc
  
  // ADCSRA &= ~_BV(ADEN);  // Disable ADC

  set_sleep_mode (SLEEP_MODE_PWR_DOWN); 
  sleep_bod_disable();
  sei();
  sleep_mode();     

  power_all_enable();
  ADCSRA = old_ADCSRA;
  // ADCSRA |= _BV(ADEN);  // Enable ADC
  
  } 

void setup()
{
pinMode (EN3, OUTPUT);
pinMode (EN5, OUTPUT);
pinMode (SLEEPSIG, INPUT_PULLUP);  
digitalWrite(EN3, LOW);
digitalWrite(EN5, LOW);
}  

/*
 * State machine
 * 0 = Initial power on, power up the DC boosts and wait
 * 1 = Wait forever for device to wake up
 * 2 = Device has woken up, so wait for device to ask for power off sleep then go back to state 0
 * 
 * 
 * 
 */

void loop(){

  switch(run_state) {
    //TODO: 1/ Voltage monitoring, 2/ Low volt battery protection, 3/ High battery protect - short out solar
    case 0:
      digitalWrite (EN3,HIGH); // switch on 3.3v rail
      digitalWrite (EN5, HIGH); // switch on 5v rail
      run_state = 1; // initial power on complete wait for device to sleep
      myWatchdogEnable (SLEEP1); // 1 second short sleep to save power whilst waiting
      break;
    case 1:
      if ( digitalRead (SLEEPSIG) == HIGH) {
        // the companion device hasn't woke up yet;
        myWatchdogEnable (SLEEP1);  // device not yet fully awake go back to sleep to save power
      } else {
        run_state = 2;  // the device has pulled the SLEEPSIG pin low so is now awake
      }
      break;
    case 2:
      if (digitalRead (SLEEPSIG) == HIGH) {
        // SLEEPSIG has gone HIGH again - device has signalled to sleep
        digitalWrite (EN5, LOW); // switch off 5v rail
        digitalWrite (EN3, LOW) ; // switch off 3v3 rail
        for (int i=0; i < 75; i++) { // power down sleep for 5 minutes ( 75 x 4 seconds)
          myWatchdogEnable (SLEEP4); 
        }
        run_state = 0;  // time to wake up again
      } else {
        // device still awake - sleep then check again in a second
        myWatchdogEnable (SLEEP1);
      }
      break;
  }
}  


