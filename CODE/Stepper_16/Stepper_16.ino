// copyright 2012 Mike Stebbins
// Released under the MIT License - Please reuse, change and share
// Changed transitionsTotal to an unsigned long from unsigned int, think that line 214 was the issue
//
//-----------------------------------------------------------------------------------------------------------------
// USER INPUTS
//-----------------------------------------------------------------------------------------------------------------
//
const boolean startAtHome = false;        // true = begin by going to Home first, false = start from current position
const int dirFromHome = 1;                // (1 or 0): 1 = CCW, 0 = CW, direction to move to go to Home
const unsigned long stepsFromHome = 0;    // (steps, integer): number of steps to move from Home to Start
const int stepDir = 0;                    // (1 or 0): 0 = CCW, 1 = CW, direction of movement from Start position
const int stepDeg = 90;                   // (degrees, integer > 0): degrees for the camera to move through from Start
const int interval = 2;                   // (seconds, integer >= 0): interval of time between pictures being taken
const float moveTime = 10.0;              // (minutes, decimal > 0): time to move the camera through stepDeg 
//                                           (1 rev/15 seconds is fastest possible movement with jerky beginning)
//
//-----------------------------------------------------------------------------------------------------------------
// DEFINITIONS
//-----------------------------------------------------------------------------------------------------------------
//
// Includes
#include <avr/interrupt.h>
// Constants
#define TIMECTL_MAXTICKS 4294967295L    // IsTime function definition
#define TIMECTL_INIT 0                  // IsTime function definition
const int homeDir = 0;                  // (1 or 0): 1 = CCW, 0 = CW
const int homeDelay = 2000;             // (microseconds, integer > 0): delay between steps when traveling to Home
const int stepsPerRev = 200;            // (integer > 0): steps per 360 degrees revolution of stepper
const int microSteps = 16;              // (integer > 0): microsteps of stepper driver
const float ratio = 7.835;              // (decimal > 0): calibrated ratio of wheel to stepper pulley diameters (8.0,7.88, 7.869,7.85
const int transitionsPerStep = 2;       // function of stepper driver board
const int timeRes = 1024;               // pre-scaler used in interrupts
const int arduinoClock = 8;             // (8 or 16, MHz) Arduino: 3.3V = 8MHz, 5.0V = 16MHz
const unsigned long focusTime = 600;    // ( milliseconds, integer >= 0 ): time to focus camera
const unsigned long shutterTime = 200;  // ( milliseconds, integer > 0 ): time to take picture
unsigned long startTime = 0;            // check time when a switch was toggled in intervalometer
unsigned long TimeMark = 0;             // a millisecond time stamp used by the intervalometer
unsigned long int TimeInterval= interval * 1000; 
//                                      // Convert intervalometer interval (seconds) to milliseconds
unsigned long time;                     // used in Serial output for debugging of intervalometer
unsigned long TimeMark2 = 0;            // millisecond time stamp used by the intervalometer, initialize
unsigned long int TimeInterval2 = 2000;// period to write degrees completed and pc                                
//                                      // (milliseconds, integer >= 0), 0 = never write it out.

//
// Pins
const int dirPin = 6;                   // pin for setting stepper driver direction
const int stepPin = 8;                  // pin for setting stepper driver steps
const int cameraFocus = 2;              // pin for focusing camera in intervalometer
const int cameraShutter = 4;            // pin for taking picture in intervalometer
const int homeSwitch = A0;              // pin for Home location switch
const int goHoldSwitch = 11;            // pin for Go/Hold switch
const int goHoldSwitchLED = 10;         // pin for Go/Hold switch indicator LED
//
// Variables
float transitionsPer1Deg;               // numer of 1/2 steps (transitions) per 1 deg wheel movement
float degreesPerMinute;                 // degrees of wheel movement per 1 minute time
float secPerTransition;                 // seconds elapsed per 1/2 step (transition)
float timerRes;                         // timer resolution of the interrupt clock and chosen pre-scaler
int focusBool = 0;                      // boolean switch for state of focus timer
int shutterBool = 0;                    // boolean switch for state of shutter timer
long pictureCount = 0;                  // number of pictures taken since starting
long compareMatchRegister;              // number of timer oscillations to count before activating ISR
volatile boolean transition = false;    // (toggle in the ISR, used to determine if time to transition step pin
volatile unsigned long counter = 0;     // counter for number of steps executed
unsigned long transitionsTotal = 0;     // total number of transition (2 * total microsteps) to be executed
float clockRes;                         // (secs) clock period of Arduino
int goHoldBool = 0;                     // boolean variable for Go/Hold switch state
int writeSwitch;                        // boolean to track whether Hold switch has just been thrown
//
//-----------------------------------------------------------------------------------------------------------------
// SETUP
//-----------------------------------------------------------------------------------------------------------------
//
void setup()  {
  //
  // Assign pinmodes
  pinMode(homeSwitch, INPUT);
  pinMode(goHoldSwitch, INPUT);
  pinMode(goHoldSwitchLED, OUTPUT);
  pinMode(dirPin, OUTPUT);
  pinMode(stepPin, OUTPUT);
  pinMode(cameraFocus, OUTPUT);
  pinMode(cameraShutter, OUTPUT);
  //
  // Enable input switches' pull-up resistors
  digitalWrite(goHoldSwitch,HIGH);
  digitalWrite(homeSwitch, HIGH);
  //
  // Calculate the resolution of the clock for this Arduino
  clockRes = 1 / ((float)arduinoClock * pow(10,6) ); // (secs) clock period of Arduino
  // Calculate the number of transitions (1/2 a microstep) per 1 deg of bike wheel rotation
  transitionsPer1Deg = ratio * stepsPerRev * microSteps / 360 * transitionsPerStep;
  // Calculate the number of degrees moved by the bike wheel in 1 minute
  degreesPerMinute = (float)stepDeg / (float)moveTime;
  // Calculate the resolution of the timer using the defined pre-scaler
  timerRes = (float)timeRes * (float)clockRes;
  // Calculate the number of seconds needed per 1 transition
  secPerTransition = ( 1 / ((float)transitionsPer1Deg * (float)degreesPerMinute) ) * 60;
  // Calculate the Compare Match Register value to correlate with the desired movement
  compareMatchRegister = ( secPerTransition / timerRes ) - 1; 
  // Calculate the number of transitions (microsteps*2 for high/low logic step) for desired move
  transitionsTotal = degreesPerMinute * transitionsPer1Deg * moveTime;
  //
  // Activate serial communication line and print parameters
  Serial.begin(57600);
  Serial.println("");
  Serial.println("------------------------------------------------------------------------------------");
  Serial.println("Parameters:");
  Serial.print("Degrees to move = ");       Serial.println(stepDeg);
  Serial.print("Direction to move = ");     Serial.println(stepDir);
  Serial.print("Move time = ");             Serial.println(moveTime);
  Serial.print("Picture interval = ");      Serial.println(interval);
  Serial.print("Diameter ratio = ");        Serial.println(ratio);
  Serial.print("transitionsPer1Deg = ");    Serial.println(transitionsPer1Deg,10);
  Serial.print("degreesPerMinute = ");      Serial.println(degreesPerMinute,10);
  Serial.print("clockRes = ");              Serial.println(clockRes,10);
  Serial.print("secPerTransition = ");      Serial.println(secPerTransition,10);
  Serial.print("transitionsTotal = ");      Serial.println(transitionsTotal,10);
  Serial.print("compareMatchRegister = ");  Serial.println(compareMatchRegister,10);
  Serial.println("------------------------------------------------------------------------------------");
  Serial.println();
  //
  // Check Go/Hold switch, light LED, and stall in while loop if Hold position active
  writeSwitch = 1;
  while (digitalRead(goHoldSwitch) == LOW)  {
    digitalWrite(goHoldSwitchLED,HIGH);
    if (writeSwitch == 1)  {
      Serial.print("Waiting for Hold switch to be flipped....");
      writeSwitch = 0;
    }
  }  
  digitalWrite(goHoldSwitchLED,LOW);  // switch off LED if in Go position
  if (writeSwitch == 0)  {
    Serial.println("Go!");  Serial.println();
  }
  //
  // Move to Home location and then to the specified start position, if user-specified
  if (startAtHome == true)  {
    goHome();
  } 
  //
  // Check Go/Hold switch, light LED, and stall in while loop if Hold position active
  writeSwitch = 1;
  while (digitalRead(goHoldSwitch) == LOW)  {
    digitalWrite(goHoldSwitchLED,HIGH);
    if (writeSwitch == 1)  {
      Serial.print("Waiting for Hold switch to be flipped."); Serial.println();
      writeSwitch = 0;
    }
  }  
  digitalWrite(goHoldSwitchLED,LOW);  // switch off LED if in Go position
  if (writeSwitch == 0)  {
    Serial.println("Go!");  Serial.println();
  }
  //
  // Set the direction of the stepper motor for the main code movement
  int value = (stepDir == 1)? LOW:HIGH;
  digitalWrite(dirPin,value);
  //
  // Initialize Timer1 ISR
  cli();          // disable global interrupts
  TCCR1A = 0;     // set entire TCCR1A register to 0
  TCCR1B = 0;     // same for TCCR1B
  // set compare match register to desired timer count:
  OCR1A = compareMatchRegister;
  // turn on CTC mode:
  TCCR1B |= (1 << WGM12);
  // Set CS10 and CS12 bits for 1024 prescaler:
  TCCR1B |= (1 << CS10);
  TCCR1B |= (1 << CS12);
  // enable timer compare interrupt:
  TIMSK1 |= (1 << OCIE1A);
  // enable global interrupts:
  sei();
  // Send out start text to serial terminal
  Serial.println("Commence movement and picture taking:");
}
//
// ISR to control stepper steps
ISR(TIMER1_COMPA_vect)  {
  transition = true;
}
//
//-----------------------------------------------------------------------------------------------------------------
// LOOP
//-----------------------------------------------------------------------------------------------------------------
void loop()  { 
  //
  // Check Go/Hold switch, light LED, write status, and stall in while loop if Hold position active
  writeSwitch = 1;                                  // turn write parameters boolean on
  while (digitalRead(goHoldSwitch) == LOW)  {       // while switch is in Hold position...
    digitalWrite(goHoldSwitchLED,HIGH);             // light the LED
    if (writeSwitch == 1)  {                        // first time through while loop, write out parameters
      shutDownPrint();
      Serial.print("Waiting for Hold switch to be flipped....");
      writeSwitch = 2;                              // advance write boolean
    }
  } 
  //
  if (writeSwitch == 2)  {                          // if == 2, then this is the first loop since flipping switch
    digitalWrite(goHoldSwitchLED,LOW);              // switch off LED if in Go position
    Serial.println("Go!");  Serial.println();
    writeSwitch = 0;                                // reset the write boolean
  }
  // 
  if (transition == true)  {                        // if the ISR changed the transition boolean state
    digitalWrite(stepPin, !digitalRead(stepPin));   // change state of stepPin
    counter++;                                      // increment steps counter
    transition = false;                             // reset transition boolean for ISR
  }
  //
  if (counter >= transitionsTotal)  {               // check if the total motor steps have already been completed
    Serial.println();
    Serial.println("Finished!");
    shutDownPrint();                                // output current position in CW/CCW steps for future uses
    float val = counter/transitionsPer1Deg;
    Serial.print("Degrees: ");  Serial.print(val);  Serial.print("  /  "); // degrees wheel has travelled
    Serial.print("Pictures: "); Serial.println(pictureCount);              // number of pictures taken
    Serial.println("------------------------------------------------------------------------------------");
    Serial.println();
    while(true);                                    // put uC into an endless loop
  }
  else  {                                           // if not...
    intervalometer();                               // run the intervalometer function
    terminalUpdate();                               // update the serial terminal with completed stats if need be 
  } 
}
//
//-----------------------------------------------------------------------------------------------------------------
// FUNCTIONS
//-----------------------------------------------------------------------------------------------------------------
//
// goHome function, called from Setup if goHome = True
void goHome()  {
  Serial.print("Travelling to Home location....");
  int switchState = 0;                        // initialize the variable for reading the reed switch status
  int value = (homeDir == 1)? HIGH:LOW;       // set the direction of the stepper motor
  digitalWrite(dirPin,value);
  switchState = digitalRead(homeSwitch);      // read the reed switch value

  if (switchState == LOW) {                   // the switch is already closed, assume it is at home
  Serial.println("Found it."); Serial.println();
}

  else  {                                     // if not, then go find Home
    while (switchState == HIGH) {             // as long as the Home pin is HIGH, you aren't home
      for (int j = 0; j < 2; j++) {           // take one full step
        digitalWrite(stepPin, !digitalRead(stepPin));
        delayMicroseconds(homeDelay);
      }
      switchState = digitalRead(homeSwitch);  // read the Home switch again
    }
    Serial.println("Found it."); Serial.println();
  }

  delay(2000);                                // pause after reaching Home and before continuing to Start

  if (stepsFromHome != 0) {                            // now move from Home to start position
    Serial.print("Travelling to Start location....");
    int val = (dirFromHome == 1)? HIGH:LOW;            // set the direction of the stepper motor
    digitalWrite(dirPin, val);
    for(unsigned long i=0; i < stepsFromHome; i++)  {  // for all the steps to take from Home to Start
      for (int j = 0; j < 2; j++) {                    // take one full step
        digitalWrite(stepPin, !digitalRead(stepPin));
        delayMicroseconds(homeDelay);
      }
    } 
    Serial.println("Found it."); Serial.println();
  }
}
//-----------------------------------------------------------------------------------------------------------------
// terminalUpdate function, used to print progress status to serial terminal at user-specified time intervals
void terminalUpdate ()  {
  if (TimeInterval2 != 0)  {
    if (IsTime(&TimeMark2,TimeInterval2))  {
      float val = counter/transitionsPer1Deg;
      
      Serial.print("Degrees: ");  Serial.print(val);  Serial.print("  /  "); // degrees wheel has travelled
      Serial.print("Pictures: "); Serial.println(pictureCount);              // number of pictures taken
    }
  }
}
//-----------------------------------------------------------------------------------------------------------------
// Focus camera and take picture function, called from Loop
void intervalometer ()  { 
  time = millis();  

  // check that enough time has passed to raise the logic level of the focus pin
  if( IsTime(&TimeMark,TimeInterval ))  {    
    digitalWrite( cameraFocus, HIGH ); 
    focusBool = 1;                            // toggle the focusBool and record the start time
    startTime = millis();
  }

  // if focusing has already occurred and the focus time has elapsed...  
  if (( focusBool == 1 ) && (( time - startTime) >= focusTime ))  {
    digitalWrite( cameraShutter, HIGH );    // write the shutter pin high
    focusBool = 0;                          // reset the focusBool to zero
    shutterBool = 1;                        // toggle the shutterBool and record the start time
    startTime = millis();
  }

  // if shuttering (word?) has already occurred and the shutter time has elapsed...
  if (( shutterBool == 1 ) && (( time - startTime) >= shutterTime ))  {  
    digitalWrite( cameraShutter, LOW );        // write both shutter and focus pins to low
    digitalWrite( cameraFocus, LOW );
    shutterBool = 0;                           // reset the shutterBool
    pictureCount++;                            // increment the number of pictures taken
  }
}
//-----------------------------------------------------------------------------------------------------------------
//IsTime() function -from David Fowler, AKA uCHobby, http://www.uchobby.com 01/21/2012
int IsTime(unsigned long *timeMark, unsigned long timeInterval) {
  unsigned long timeCurrent;
  unsigned long timeElapsed;
  int result=false;

  timeCurrent=millis();
  if(timeCurrent<*timeMark) {                      // rollover detected
    // elapsed=all the ticks to overflow + all the ticks since overflow
    timeElapsed=(TIMECTL_MAXTICKS-*timeMark)+timeCurrent;  
  }
  else {
    timeElapsed=timeCurrent-*timeMark;  
  }

  if(timeElapsed>=timeInterval) {
    *timeMark=timeCurrent;
    result=true;
  }
  return(result);  
}
//-----------------------------------------------------------------------------------------------------------------
// shutDownPrint function - print direction and steps taken thus far, to reposition camera to current position
void shutDownPrint ()  {
  Serial.println("");
  if (stepDir == 1)  {
    Serial.print("CCW (1) steps = ");
  }
  else  {
    Serial.print("CW (0) steps = ");
  }
  Serial.println(counter/2);
  delay(200);                              // delay to ensure serial communication buffer is empty
}
//-----------------------------------------------------------------------------------------------------------------
