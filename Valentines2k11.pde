#include <EEPROM.h>
#include <Wire.h>
#include <avr/sleep.h>
#include <avr/power.h>
#include <avr/wdt.h>
#include "LEDController.h"

#ifndef cbi
#define cbi(sfr, bit) (_SFR_BYTE(sfr) &= ~_BV(bit))
#endif
#ifndef sbi
#define sbi(sfr, bit) (_SFR_BYTE(sfr) |= _BV(bit))
#endif

//IO pin constants
#define PIN_LIGHT          2    // analog pin for reading the photo diode sensor
#define PIN_DIST           3    // analog pin for reading the IR sensor
#define PIN_DIST_PWR       11   // digital pin for providing power to the IR sensor
#define PIN_TILT           2    // digital pin used for tilt switch (wakeup and sleep)
#define PIN_INDICATOR      10   // digital pin used for IR sensor control
#define PIN_LIGHT_PWR      12   // digital pin used for photo diode control

//Config constants
#define CONFIG_LIGHTMODE   0
#define CONFIG_LIGHTMAX    1
#define CONFIG_LIGHTMIN    2

//Operating constants
#define WDT_IDLE          0
#define WDT_OPERATING     1

#define LED_ON_FULL       127
#define LED_OFF_FULL      255
#define LED_OFF_MIN_PCT   10
#define LED_LOW_VAL       LED_ON_FULL
#define LED_HIGH_VAL      LED_OFF_FULL

#define LED_MID_VAL       (LED_HIGH_VAL-LED_LOW_VAL)/2
#define LED_QHIGH_VAL     LED_MID_VAL+(LED_MID_VAL/2)
#define LED_QLOW_VAL      LED_MID_VAL-(LED_MID_VAL/2)

#define DIST_MODE_CHG     10
#define DIST_WAKE_WDT     20
#define DIST_N_SAMPLE     5

#define T_AWAKE_MAX       500000L   //Total time to stay awake, milliseconds
#define T_DEBOUNCE        200L       // the debounce time; increase if the output flickers
#define T_LIGHT_SAMPLE    10L

//Watchdog timers
// 0=16ms, 1=32ms,2=64ms,3=128ms,4=250ms,5=500ms
// 6=1 sec,7=2 sec, 8=4 sec, 9=8sec
#define WDT_Q_SEC 4
#define WDT_H_SEC 5
#define WDT_1_SEC 6
#define WDT_2_SEC 7
#define WDT_4_SEC 8
#define WDT_8_SEC 9

//LED control variables
#define R 3
#define G 6
#define B 5
LEDController lR(R);
LEDController lG(G);
LEDController lB(B); 
LEDController* leds[] = {&lR,&lG,&lB};

//Lighting variables
int lightMode = 0;                         //Current light show mode
int lightMax = 700;                        //Light sensor reading when fully bright
int lightMin = 100;                        //Light sensor reading when fully dark
int lightCur = (lightMax - lightMin) / 2;  //Latest light sensor reading

//Timing variables
unsigned long tLastModeChange = 0;
unsigned long tLastWake = 0;       //Time last mode change was initiated (used to determine how long till sleep)
unsigned long tAwake = 0;          //Time unit has been awake
unsigned long tOperating = 0;      //Time unit has been performing the current lightShow

//Tilt switch variables
unsigned long tTiltDebounce = 0;     // The last time the output pin was toggled
int tiltState = HIGH;                // The current reading from the tilt switch
int lastTiltState  = LOW;            // The previous reading from the tilt switch pin

//Timer variables
int wdtType = WDT_OPERATING;
int fWatchDog = 0;

//Distance calculation variables
float distanceSampleSum = 0.0;
int distanceSampleCount = 0;
unsigned long tDistanceModeBegin = -1;

void setup()
{
  pinMode(PIN_TILT, INPUT);
  //Turn on internal pullup resistor for tilt switch
  digitalWrite(PIN_TILT, HIGH);

  //Setup the pin modes we need
  pinMode(R, OUTPUT);
  pinMode(G, OUTPUT);
  pinMode(B, OUTPUT);
  
  pinMode(PIN_INDICATOR, OUTPUT);
  digitalWrite(PIN_INDICATOR, LOW);

  pinMode(PIN_LIGHT_PWR, OUTPUT);
  digitalWrite(PIN_LIGHT_PWR, LOW);
  
  Serial.begin(9600);

  readConfig();

  LEDController *c;
  for ( int i=0; i<3; i++ ) {
    c = leds[i];
    c->usingPWM = true;
    c->analogHighValue = LED_HIGH_VAL; 
    c->analogMidValue = LED_MID_VAL;
    c->analogLowValue = LED_LOW_VAL;
    c->rampUpValue = 1;
    c->rampDownValue = 1;
    c->rampUpDelay = 10;
    c->rampDownDelay = 10;
  }

  setupLightShow();
  
  postSleep();  
}

void loop() {
  if ( fWatchDog == 1 ) {
    fWatchDog = 0;
    if ( wdtType == WDT_OPERATING )
      procWDTOperating();
    else
      procWDTIdle();
  }
  else {
    procLoop();
  }
}

void readConfig() {
  int v;
  
  v = EEPROM.read(CONFIG_LIGHTMODE);
  if ( v != 255 )
    lightMode = v;
    
//  v = EEPROM.read(CONFIG_LIGHTMAX);
//  if ( v != 255 )
//    lightMax = v*4;    
//    
//  v = EEPROM.read(CONFIG_LIGHTMIN);
//  if ( v != 255 )
//    lightMin = v*4;
    
//  Serial.println("readConfig");
//  Serial.print("lightMode: ");
//  Serial.println(lightMode);
//  Serial.print("lightMax: ");
//  Serial.println(lightMax);
//  Serial.print("lightMin: ");
//  Serial.println(lightMin);
}

void saveConfig() {
  EEPROM.write(CONFIG_LIGHTMODE, lightMode);
  EEPROM.write(CONFIG_LIGHTMAX, lightMax/4);
  EEPROM.write(CONFIG_LIGHTMIN, lightMin/4);
  
//  Serial.println("saveConfig");
//  Serial.print("lightMode: ");
//  Serial.println(lightMode);
//  Serial.print("lightMax: ");
//  Serial.println(lightMax);
//  Serial.print("lightMin: ");
//  Serial.println(lightMin);  
}

#define L_MAX 8
void setupLightShow() {
  LEDController *c;
  for ( int i=0; i<3; i++ ) {
    c = leds[i];
    c->action = TURN_HIGH;  //Turn off
    c->takeAction();
  }

  lightMode %= L_MAX;
  Serial.print("lightMode is: ");
  Serial.println(lightMode);  
  
  switch (lightMode) {
    case 7:
      lB.setLEDBrightness(LED_HIGH_VAL);
      lG.setLEDBrightness(LED_HIGH_VAL);
      lR.setLEDBrightness(LED_HIGH_VAL);
      lB.action = RAMP_CONT_HIGH_LOW;
      lG.action = RAMP_CONT_HIGH_LOW;
      lR.action = RAMP_CONT_HIGH_LOW;
      break;
    case 6:
      lB.setLEDBrightness(LED_HIGH_VAL);
      lG.setLEDBrightness(LED_MID_VAL);
      lR.setLEDBrightness(LED_LOW_VAL);
      lB.action = RAMP_CONT_HIGH_LOW;
      lG.action = RAMP_CONT_HIGH_LOW;
      lR.action = RAMP_CONT_HIGH_LOW;
      break;
    case 5:
      lB.setLEDBrightness(LED_LOW_VAL);
      lR.setLEDBrightness(LED_HIGH_VAL);
      lB.action = RAMP_CONT_HIGH_LOW;
      lR.action = RAMP_CONT_HIGH_LOW;
      break;
    case 4:
      lG.setLEDBrightness(LED_LOW_VAL);
      lB.setLEDBrightness(LED_HIGH_VAL);
      lG.action = RAMP_CONT_HIGH_LOW;
      lB.action = RAMP_CONT_HIGH_LOW;
      break;
    case 3:
      lR.setLEDBrightness(LED_LOW_VAL);
      lG.setLEDBrightness(LED_HIGH_VAL);
      lR.action = RAMP_CONT_HIGH_LOW;
      lG.action = RAMP_CONT_HIGH_LOW;
      break;
    case 2:
      lB.action = RAMP_CONT_HIGH_LOW;
      break;
    case 1:
      lG.action = RAMP_CONT_HIGH_LOW;
      break;
    case 0:
      lR.action = RAMP_CONT_HIGH_LOW;
      break;
  }
  
  for ( int i=0; i<3; i++ ) {
    c = leds[i];
    c->takeAction();
  }  
  
  saveConfig();
}

void disableLightShow() {
  LEDController *c;
  for ( int i=0; i<3; i++ ) {
    c = leds[i];
    c->action = TURN_HIGH; 
    c->takeAction();
  }  
}

void procLightShow()  {       
  LEDController *c;
  for ( int i=0; i<3; i++ ) {
    c = leds[i];
    c->takeAction();
  }  
}

void enableIRSensor() {
  pinMode(PIN_INDICATOR, OUTPUT);
  digitalWrite(PIN_INDICATOR, HIGH);
  
  sbi(ADCSRA,ADEN);                    // switch Analog to Digitalconverter ON

  pinMode(PIN_DIST_PWR, OUTPUT);
  digitalWrite(PIN_DIST_PWR, HIGH);  
  delay(60);  //Give the IR sensor time to come up to speed
}

void enableLightSensor() {
  digitalWrite(PIN_LIGHT_PWR, HIGH);
  delay(10);  //Give the IR sensor time to settle
}  

void disableLightSensor() {
  digitalWrite(PIN_LIGHT_PWR, LOW);
}  

void disableIRSensor() {
  cbi(ADCSRA,ADEN);                    // switch Analog to Digitalconverter OFF

  pinMode(PIN_DIST_PWR, OUTPUT);
  digitalWrite(PIN_DIST_PWR, LOW);
  
  pinMode(PIN_INDICATOR, OUTPUT);
  digitalWrite(PIN_INDICATOR, LOW);
}

void procLoop() {
  unsigned long n = millis();
  tAwake = n - tLastWake;
  tOperating = n - tLastModeChange;
  
  // display information about the counter
//  if ( tAwake % 1000 == 0 ) {
//    Serial.print("Awake for ");
//    Serial.println(tAwake);
//  }
//    Serial.print("tLastWake is ");
//    Serial.println(tLastWake);
//    Serial.print("tLastModeChange is ");
//    Serial.println(tLastModeChange);
//    Serial.print("Operating for ");
//    Serial.println(tOperating);

  procLightShow();

  checkSleep();
}

unsigned long tLastLight = 0;
void procWDTOperating() {
//  Serial.println("procWDTOperating");

    //Not using the light sensor yet.
//  unsigned long n = millis();
//  if ( n - tLastLight > T_LIGHT_SAMPLE ) {
//    enableLightSensor();
//    lightCur = analogRead(PIN_LIGHT);
//    disableLightSensor();
//    Serial.print("lightCur is: ");
//    Serial.println(lightCur);
//    
//    if ( lightCur < lightMin ) {
//      lightMin = lightCur;
//      saveConfig();
//    }
//    else if ( lightCur > lightMax ) {
//      lightMax = lightCur;
//      saveConfig();
//    }
//
//    mapLEDVals();
//    tLastLight = n;
//  }

  enableIRSensor();
  checkMode();
  disableIRSensor();
}

//int invertLightVal(int v) {
//  return (int)((1/(float)v)*100000);
//}
//
//void mapLEDVals() {
//  Serial.print("lightMax: ");
//  Serial.println(lightMax);
//  Serial.print("lightMin: ");
//  Serial.println(lightMin);
//  
//  if ( lightMax > lightMin ) {
//    //Invert the values for the mapping fn.
//    lightMax = invertLightVal(lightMax);
//    lightMin = invertLightVal(lightMin);
//    
//    Serial.println("INVERT LIGHT");
//    Serial.print("lightMax: ");
//    Serial.println(lightMax);
//    Serial.print("lightMin: ");
//    Serial.println(lightMin);      
//  }
//  
//  int v = invertLightVal(lightCur);
//  int vdash = map(v, lightMax, lightMin, LED_LOW_VAL, LED_HIGH_VAL);
//  int m = (LED_HIGH_VAL - v) / 2;
//  
//  LEDController *c;
//  for ( int i=0; i<3; i++ ) {
//    c = leds[i];
//    c->analogHighValue = LED_HIGH_VAL; 
//    c->analogMidValue = m;
//    c->analogLowValue = vdash;
//  }
//  
//  Serial.print("h,m,v: ");
//  Serial.print(LED_HIGH_VAL);
//  Serial.print(", ");
//  Serial.print(m);
//  Serial.print(", ");
//  Serial.println(v);
//}

void procWDTIdle() {
//  Serial.println("procWDTIdle");

  enableIRSensor();
  int nextState = checkWDTWakeup();
  disableIRSensor();
  
//    Serial.print("nextState is ");
//    Serial.println(nextState);
    
  switch ( nextState ) {
    case -1:
      wdt_disable();  // disable watchdog then go to idle sleep mode
      sleepNowTilt();
      break;
    case 0:  //Remain in the same state
      sleepNowWDT();
      break;
    case 1:  //Wake up
      postSleep();
      break;
  }
}

void wakeUpTilt() {        
  // here the interrupt is handled after wakeup
  // execute code here after wake-up before returning to the loop() function
  // timers and code using timers (serial.print and more...) will not work here.
  // we don't really need to execute any special functions here, since we
  // just want the thing to wake up
//  Serial.println("wakeUpNow");  
}

//Check IR sensor reading, if something is within range. wake up. (return 1)
//Check tilt switch, if the tiltswitch is deactivated. go to idle sleep (return -2)
//Remain in the same state, return 0.
int checkWDTWakeup() {    
  int distance = calcAveragedIRDistance();
//  int distance = calcRawIRDistance();
  
//  Serial.print("WDT distance is: ");
//  Serial.println(distance);
  
  if ( distance > 0 && distance < DIST_WAKE_WDT ) {
    Serial.println("WDT wake up!");
    return 1;
  }
  
  if ( isTiltActive() )
    return -1;
  
  return 0;
}

//Housekeeping before sleeping
void preSleep() {
  Serial.println("Pre sleep");
  delay(100);

  disableIRSensor();  

  disableLightShow();
}

void postSleep() {
  Serial.println("Post sleep");
  
  unsigned long n = millis();
  tLastWake = n;
  tLastModeChange = n;

  setupLightShow();
  setupWatchdog(WDT_H_SEC, WDT_OPERATING);
}

void sleepNowTimer()         // here we put the arduino to watchdog timer sleep mode because the run time passed a certain time.
{
  preSleep();

  // CPU Sleep Modes 
  // SM2 SM1 SM0 Sleep Mode
  // 0   0   0   Idle
  // 0   0   1   ADC Noise Reduction
  // 0   1   0   Power-down
  // 0   1   1   Power-save
  // 1   0   0   Reserved
  // 1   0   1   Reserved
  // 1   1   0   Standby(1)

  cbi( SMCR,SE );      // sleep enable, power down mode
  cbi( SMCR,SM0 );     // power down mode
  sbi( SMCR,SM1 );     // power down mode
  cbi( SMCR,SM2 );     // power down mode

  setupWatchdog(WDT_4_SEC, WDT_IDLE);  
  
  sleepNowWDT();
}

void setupWatchdog(int ii, int t) {
  byte bb;
  int ww;
  if (ii > 9 ) ii=9;
  bb=ii & 7;
  if (ii > 7) bb|= (1<<5);
  bb|= (1<<WDCE);
  ww=bb;
//  Serial.println(ww);


  MCUSR &= ~(1<<WDRF);
  // start timed sequence
  WDTCSR |= (1<<WDCE) | (1<<WDE);
  // set new watchdog timeout value
  WDTCSR = bb;
  WDTCSR |= _BV(WDIE);
  
  wdtType = t;
}

void sleepNowWDT() {
  Serial.println("TIMER: Entering WDT sleep mode");
  delay(100);     // this delay is needed, the sleep function may provoke a serial error otherwise!!
  
  cbi(ADCSRA,ADEN);                    // switch Analog to Digitalconverter OFF

  set_sleep_mode(SLEEP_MODE_PWR_DOWN); // sleep mode is set here
  sleep_enable();

  sleep_mode();                        // System sleeps here

  sleep_disable();                     // System continues execution here when watchdog timed out 
  sbi(ADCSRA,ADEN);                    // switch Analog to Digitalconverter ON
}

//****************************************************************  
// Watchdog Interrupt Service / is executed when  watchdog timed out
ISR(WDT_vect) {
//  Serial.print("fWatchDog = 1\n");
  fWatchDog = 1;  // set WDT flag
}

void sleepNowTilt()         // here we put the arduino to power down sleep mode because the tilt switch was de-activated
{
  Serial.println("TILT: Entering idle sleep mode");
  delay(100);     // this delay is needed, the sleep function may provoke a serial error otherwise!!
  
  wdt_disable();
  
  preSleep();

  /* Now is the time to set the sleep mode. In the Atmega8 datasheet
   * http://www.atmel.com/dyn/resources/prod_documents/doc2486.pdf on page 35
   * there is a list of sleep modes which explains which clocks and 
   * wake up sources are available in which sleep mode.
   *
   * In the avr/sleep.h file, the call names of these sleep modes are to be found:
   *
   * The 5 different modes are:
   *     SLEEP_MODE_IDLE         -the least power savings 
   *     SLEEP_MODE_ADC
   *     SLEEP_MODE_PWR_SAVE
   *     SLEEP_MODE_STANDBY
   *     SLEEP_MODE_PWR_DOWN     -the most power savings
   *
   * For now, we want as much power savings as possible, so we 
   * choose the according 
   * sleep mode: SLEEP_MODE_PWR_DOWN
   * 
   */  
  set_sleep_mode(SLEEP_MODE_PWR_DOWN);   // sleep mode is set here

  sleep_enable();          // enables the sleep bit in the mcucr register
                           // so sleep is possible. just a safety pin 

  /* Now it is time to enable an interrupt. We do it here so an 
   * accidentally pushed interrupt button doesn't interrupt 
   * our running program. if you want to be able to run 
   * interrupt code besides the sleep function, place it in 
   * setup() for example.
   * 
   * In the function call attachInterrupt(A, B, C)
   * A   can be either 0 or 1 for interrupts on pin 2 or 3.   
   * 
   * B   Name of a function you want to execute at interrupt for A.
   *
   * C   Trigger mode of the interrupt pin. can be:
   *             LOW        a low level triggers
   *             CHANGE     a change in level triggers
   *             RISING     a rising edge of a level triggers
   *             FALLING    a falling edge of a level triggers
   *
   * In all but the IDLE sleep modes only LOW can be used.
   */

  attachInterrupt(0,wakeUpTilt, LOW); // use interrupt 0 (pin 2) and run function
                                     // wakeUpNow when pin 2 gets LOW 
  
  sleep_mode();            // here the device is actually put to sleep!!
                           // THE PROGRAM CONTINUES FROM HERE AFTER WAKING UP
  sleep_disable();         // first thing after waking from sleep:
                           // disable sleep...
  detachInterrupt(0);      // disables interrupt 0 on pin 2 so the 
                           // wakeUpNow code will not be executed 
                           // during normal running time.  
                           
  postSleep();  
}

void checkMode() {
  int distance = calcAveragedIRDistance();
  
  if ( distance > 0 ) {
//    Serial.print("Distance is: ");
//    Serial.println(distance);
        
    if ( distance < DIST_MODE_CHG ) {
        //Switch modes
        lightMode++;
        tLastModeChange = millis();
        
        setupLightShow();
    }
//    else {
//      Serial.print("lightMode stays at ");
//      Serial.println(lightMode);  
//    }
  }
}

boolean flashState(unsigned long tDelta, int period) {
  int state = tDelta / period;
  return state % 2;
}

int calcRawIRDistance() {
  int d = (int)calcIRDistance(false, 1);
  return d > 0 ? d : 32768;
}

int calcAveragedIRDistance() {
  distanceSampleCount = 0;
  int i = 0, d = -1;
  
  while ( d < 0 && i++ <= DIST_N_SAMPLE ) {
    d = (int)calcIRDistance(true, DIST_N_SAMPLE);    
  } 
  
  return d > 0 ? d : 32767;
}

float calcIRDistance(boolean average, int nSamples) {
  float volts = analogRead(PIN_DIST)*0.0048828125;   // value from sensor * (5/1024) - if running 3.3.volts then change 5 to 3.3
  float rawDistance = 27*pow(volts, -1.075);

//  Serial.print("rawDistance is: ");
//  Serial.println(rawDistance);

  if ( !average ) {
      return rawDistance;
  }
    
  distanceSampleSum += rawDistance;
  distanceSampleCount++;
  
//  Serial.print("distanceSampleSum: ");
//  Serial.println(distanceSampleSum);
  if (distanceSampleCount >= nSamples) {    
    float result = distanceSampleSum / (float)distanceSampleCount;
//    Serial.print("distanceSampleSum / distanceSampleCount: ");
//    Serial.print(distanceSampleSum);
//    Serial.print(" / ");
//    Serial.print(distanceSampleCount);
//    Serial.print(" = ");
//    Serial.println(result);
    
//    Serial.print("calcIRDistance is: ");
//    Serial.println(result);

    distanceSampleSum = 0.0;
    distanceSampleCount = 0;    
    return result;
  }
//  else {
//    Serial.print("distance count: ");
//    Serial.println(distanceSampleCount);
//  }
  
  return -1;
}

void checkSleepSerial() {
  // Check for serial input for sleep the serial input
//  if (Serial.available()) {
//    int val = Serial.read();
//    if (val == 'S') {
//      Serial.println("Serial: Entering WDT sleep mode");
//      sleepNowTimer();     //Go into watchdog sleep mode
//    }
//    if (val == 'A') {
//      Serial.println("Hola Caracola"); // classic dummy message
//    }
//  }
}

void checkSleepTimer() {
   // check if it should go to sleep because of time
//  Serial.print("Operating for ");
//  Serial.println(tOperating);
  if (tOperating >= T_AWAKE_MAX) {
      sleepNowTimer();     //Go into watchdog sleep mode
  }
}

boolean isTiltActive() {
  return false;
  
  unsigned long n = millis();
  int reading = digitalRead(PIN_TILT);
//  Serial.print("Reading is: ");
//  Serial.println(reading);
  
  // If the switch changed, due to noise or pressing:
  if (reading != lastTiltState ) {
    // reset the debouncing timer
//    Serial.println("reset the debouncing timer");
    tTiltDebounce = n;
  } 
  
  if ((n - tTiltDebounce) > T_DEBOUNCE) {
    // whatever the reading is at, it's been there for longer
    // than the debounce delay, so take it as the actual current state:
    tiltState  = reading;
//    Serial.print("tiltState  changed to: ");
//    Serial.println(tiltState );
  }
  
//  Serial.print("tiltState  == HIGH? ");
//  Serial.println(tiltState  == HIGH);

  lastTiltState  = reading;
  
  //HIGH == tilt switch indicates we should sleep (return true)
  return tiltState  == HIGH;
}

void checkSleepTilt() {
  if ( isTiltActive() ) {
      sleepNowTilt();     // sleep function called here
  }
}

void checkSleep() {
  //checkSleepSerial();
  checkSleepTilt();
  checkSleepTimer(); 
}

