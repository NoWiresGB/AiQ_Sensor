#include <avr/sleep.h>
#include <avr/wdt.h>
#include <avr/power.h>;
#include <SoftwareSerial.h>  // useful for debug, and if we end up doing signalling
#define rxPin 5    // We use a non-existant pin as we are not interested in receiving data
#define txPin 1
SoftwareSerial serial(rxPin, txPin);


#define EN3 4 // ATTiny pin 3 (PB4)
#define EN5 3 // ATTiny pin 2 (PB3)
#define SLEEPSIG 0  // ATTiny Pin 5 (PB0) 

// 0 1 0 Internal 1.1V Voltage Reference. 
// #define INTERNAL (2) 
#define INTERNAL1V1 INTERNAL  // for unambiguity

bool debug = true;  // runstate serial output

// Analog sensing pin
int VBatPin = A1;    // Reads in the analogue number of voltage
unsigned long VBat = 0; // This will hold the batery pack voltage 2000->3000mv
long Vanalog = 0; // Raw ADC readings of battery voltage
//unsigned long Vref = 1060; // calibrated internal reference - specific to mcu or set at 1100.
bool overcharge = false;  // flag to capture overcharge battery state

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
  // Set up IO pins
  pinMode(rxPin, INPUT);
  pinMode(txPin, OUTPUT);
  pinMode (EN3, OUTPUT);
  pinMode (EN5, OUTPUT);
  pinMode (SLEEPSIG, INPUT_PULLUP);  
  digitalWrite(EN3, LOW);
  digitalWrite(EN5, LOW);

  analogReference(INTERNAL1V1); 
  // Start the serial output for ATTiny85
  serial.begin(4800);
  delay(100);
  serial.println("reset");
}  

/*
 * State machine
 * 0 = Initial power on, check battery voltage, if all OK power up the DC boosts and wait
 * 1 = Wait for 10 seconds for the device to wake up else go into power down forever (charging state)
 * 2 = Device has woken up, so wait for device to ask for power off sleep then go back to state 0
 * 3 = No devices were detected, power down and go into a solar charging state until reset.
 * 
 * 
 */

void loop(){

  switch(run_state) {
    //TODO: All done
    case 0:
      if (debug) { serial.println(run_state);}
      
        //Let's check the battery off load before we go any further
        Vanalog = analogRead(VBatPin);
        serial.print("ADCraw: ");
        serial.println(Vanalog);
        // Calculate voltage: Internal Ref 1060mV..   VBAT---560k--^---220k---GND
        // Adjusted for actual reading but need more accurate resistors really! - 5% LOL.
        VBat = (Vanalog * 3695) / 1000;
        serial.print("VBat: ");
        serial.print(VBat);
        serial.println("mV");

        if (VBat < 2000) { // Batteries critically low @ 1v per cell.  Abort & sleep to allow solar charging
          // disable boosts and go back to sleep
          digitalWrite (EN5, LOW);
          digitalWrite (EN3, LOW);
          run_state = 2;
        }

        if (overcharge && (VBat < 2950) ) { // clear overcharge state, with some hysteresis protection
          overcharge = false;
          serial.println("Overcharge cleared");
        }

        if (VBat > 3050) { // Batteries are at risk of overcharging.  STAY AWAKE
          overcharge = true;
          serial.println("Overcharge");
        }

      // Now lets switch on the loads
      digitalWrite (EN3,HIGH); // switch on 3.3v rail
      digitalWrite (EN5, HIGH); // switch on 5v rail
      run_state = 1; // initial power on complete wait for device to sleep
      myWatchdogEnable (SLEEP1); // 1 second short sleep to save power whilst waiting
      if (debug) { serial.println(run_state);}
      break;
      
    case 1:
      {
        if (debug) { serial.println(run_state);}
        bool awake = false;
        for (int i=0; i < 10; i++) { // wait for 10 seconds before shutting down and going into charge only mode
          if (debug) { 
            serial.print("waiting ");
            serial.println(i);
            }
          if ( digitalRead (SLEEPSIG)   == HIGH) {
            // the companion device hasn't woke up yet;
            myWatchdogEnable (SLEEP1);  // device not yet fully awake go back to sleep to save power
          } else {
            awake = true;
            i = 60; // break out of for loop - case break; didn't seem to work ??
            if (debug) {serial.println("leaving"); }
          }  
        }
        
        // Sample battery voltage under load
        Vanalog = analogRead(VBatPin);
        serial.print("ADCraw: ");
        serial.println(Vanalog);
        // Calculate voltage: Internal Ref 1060mV..   VBAT---560k--^---220k---GND
        // Adjusted for actual reading but need more accurate resistors really! - 5% LOL.
        VBat = (Vanalog * 3695) / 1000;
        serial.print("VBat: ");
        serial.print(VBat);
        serial.println("mV");
        
        if (awake) {  
          run_state = 2;  // the device has pulled the SLEEPSIG pin low so is now awake
        } else {
          // device didn't wake up so we shut down the boost rails and move to sleep forever
          digitalWrite (EN5, LOW);
          digitalWrite (EN3, LOW);
          run_state = 3;
        }    
        break;
      }
      
    case 2:
      if (debug) { serial.println(run_state);}
      
      if (digitalRead (SLEEPSIG) == HIGH || run_state == 4) {
        // SLEEPSIG has gone HIGH again - device has signalled to sleep, or battery critically low
        // but if we're in a state of overcharge we're going to stay awake for the sleep period
        if (!overcharge) {
          digitalWrite (EN5, LOW); // switch off 5v rail
          digitalWrite (EN3, LOW) ; // switch off 3v3 rail
        }
        
        for (int i=0; i < 75; i++) { // power down sleep for 5 minutes ( 75 x 4 seconds)
          myWatchdogEnable (SLEEP4); 
        }
        run_state = 0;  // time to wake up again
      } else {
        // device still awake - sleep then check again in a second
        myWatchdogEnable (SLEEP1);
      }
      break;
      
      case 3:
      if (debug) { serial.println(run_state);}
      serial.println("forever");
      while (1) { // forever
        myWatchdogEnable (SLEEP8);
      } 
      // no change of runstate we keep doing this forever until reset
      break;
  }
}  


