#include <avr/sleep.h>
#include <avr/power.h>
#include <avr/wdt.h>

#ifndef cbi
#define cbi(sfr, bit) (_SFR_BYTE(sfr) &= ~_BV(bit))
#endif
#ifndef sbi
#define sbi(sfr, bit) (_SFR_BYTE(sfr) |= _BV(bit))
#endif

int WDT_IDLE = 0;
int WDT_OPERATING = 1;
int wdtType = WDT_OPERATING;
int fWatchDog = 0;

//Variables for running distance calculation
//TODO: Convert to use integer math
float distanceSampleSum = 0.0;
int distanceSampleCount = 0;
unsigned long tDistanceModeBegin = -1;

int DIST_SENS_MIN = 19;
int DIST_MODE_CHG = 20;
int DIST_WAKE_WDT = 70;
int DIST_N_SAMPLE = 5;

int T_AWAKE_MAX = 5000;   //Total time to stay awake, milliseconds

int CA_LED_PWM_ON = 254;
int CA_LED_PWM_OFF = 255;

int MODE_A = 0;
int MODE_B = 1;
int MODE_C = 2;
int MODE_MAX = 2;
int MODES[] = { MODE_A, MODE_B, MODE_C };

int PIN_DIST           = 3;    // analog pin for reading the IR sensor
int PIN_DIST_PWR       = 11;   // digital pin for provising power to the IR sensor
int PIN_TILT           = 2;    // digital pin used for tilt switch (wakeup and sleep)
int PIN_INDICATOR      = 10;   // digital pin used for LED indicator

//****************************************************************
// 0=16ms, 1=32ms,2=64ms,3=128ms,4=250ms,5=500ms
// 6=1 sec,7=2 sec, 8=4 sec, 9=8sec
int WDT_1_SEC = 6;
int WDT_2_SEC = 7;
int WDT_4_SEC = 8;
int WDT_8_SEC = 9;

int R = 3;                      // RED LED connected to digital pin 9 for PWM
int G = 5;                      // GRN LED connected to digital pin 10 for PWM
int B = 6;                      // BLU LED connected to digital pin 11 for PWM
int LED_PINS[] = {R,G,B};       //PIN # for each color
int LED_VALS[] = {0,0,0};       //PWM values for each LED

int lightMode = MODES[0];          //Current light show mode
unsigned long tLastWake = 0;       //Time last mode change was initiated (used to determine how long till sleep)
unsigned long tAwake = 0;          //Time unit has been awake
unsigned long tOperating = 0;      //Time unit has been performing the current lightShow
unsigned long tLastModeChange = 0;

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
  
  Serial.begin(9600);

  postSleep();  
}

void enableIRSensor() {
  pinMode(PIN_INDICATOR, OUTPUT);
  digitalWrite(PIN_INDICATOR, HIGH);
  
  sbi(ADCSRA,ADEN);                    // switch Analog to Digitalconverter ON

  pinMode(PIN_DIST_PWR, OUTPUT);
  digitalWrite(PIN_DIST_PWR, HIGH);  
  delay(60);  //Give the IR sensor time to come up to speed
}

void disableIRSensor() {
  cbi(ADCSRA,ADEN);                    // switch Analog to Digitalconverter OFF

  pinMode(PIN_DIST_PWR, OUTPUT);
  digitalWrite(PIN_DIST_PWR, LOW);
  
  pinMode(PIN_INDICATOR, OUTPUT);
  digitalWrite(PIN_INDICATOR, LOW);
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

void procLoop() {
  unsigned long n = millis();
  tAwake = n - tLastWake;
  tOperating = n - tLastModeChange;
  
  // display information about the counter
  if ( tAwake % 1000 == 0 ) {
    Serial.print("Awake for ");
    Serial.println(tAwake);
  }
//    Serial.print("tLastWake is ");
//    Serial.println(tLastWake);
//    Serial.print("tLastModeChange is ");
//    Serial.println(tLastModeChange);
//    Serial.print("Operating for ");
//    Serial.println(tOperating);

  procLightShow();

  checkSleep();
}

void procWDTOperating() {
  Serial.println("procWDTOperating");

  enableIRSensor();
  checkMode();
  disableIRSensor();
}

void procWDTIdle() {
  Serial.println("procWDTIdle");

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
  
  Serial.print("WDT distance is: ");
  Serial.println(distance);
  
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

  analogWrite(R, CA_LED_PWM_OFF);
  analogWrite(G, CA_LED_PWM_OFF);
  analogWrite(B, CA_LED_PWM_OFF);
}

void postSleep() {
  Serial.println("Post sleep");
  
  unsigned long n = millis();
  tLastWake = n;
  tLastModeChange = n;

  setupWatchdog(WDT_2_SEC, WDT_OPERATING);
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
    Serial.print("Distance is: ");
    Serial.println(distance);
        
    if ( distance < DIST_MODE_CHG ) {
        //Switch modes
        if ( lightMode == MODE_MAX )
          lightMode = 0;
        else
          lightMode++;
                
        tLastModeChange = millis();
        
        Serial.print("lightMode changed to ");
        Serial.println(lightMode);  
    }
    else {
      Serial.print("lightMode stays at ");
      Serial.println(lightMode);  
    }
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


void procLightShow()  {       
  switch (lightMode) {
    case 0:
      analogWrite(R, CA_LED_PWM_ON);
      analogWrite(G, CA_LED_PWM_OFF);
      analogWrite(B, CA_LED_PWM_OFF);
      break;
    case 1:
      analogWrite(R, CA_LED_PWM_OFF);
      analogWrite(G, CA_LED_PWM_ON);
      analogWrite(B, CA_LED_PWM_OFF);
      break;
    case 2:
      analogWrite(R, CA_LED_PWM_OFF);
      analogWrite(G, CA_LED_PWM_OFF);
      analogWrite(B, CA_LED_PWM_ON);
      break;
  }
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
//  return false;
  
  /***************Production mode****************/
  //HIGH == tilt switch indicates we should sleep
  return digitalRead(PIN_TILT) == HIGH;


  /***************Test mode****************/
  //LOW == pushbutton switch indicates we should go to sleep
//  return digitalRead(PIN_TILT) == LOW;
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

