#include <avr/sleep.h>
#include <avr/wdt.h>

#ifndef cbi
#define cbi(sfr, bit) (_SFR_BYTE(sfr) &= ~_BV(bit))
#endif
#ifndef sbi
#define sbi(sfr, bit) (_SFR_BYTE(sfr) |= _BV(bit))
#endif

int fWatchDog = 0;

//Variables for running distance calculation
//TODO: Convert to use integer math
double distanceSampleSum = 0.0;
int distanceSampleCount = 0;
unsigned long tDistanceModeBegin = -1;

int DIST_SENS_MIN = 19;
int DIST_MODE_CHG = DIST_SENS_MIN * 2;
int DIST_WAKE_WDT = DIST_SENS_MIN * 4;
int DIST_N_SAMPLE = 250;

int T_AWAKE_MAX = 5000;   //Total time to stay awake, milliseconds
int T_MODE_HOLD = 2000;   //milliseconds

int MODE_A = 0;
int MODE_B = 1;
int MODE_C = 2;
int MODE_MAX = 2;
int MODES[] = { MODE_A, MODE_B, MODE_C };

int PIN_DIST           = 1;    // analog pin for reading the IR sensor
int PIN_TILT           = 2;    // digital pin used for tilt switch (wakeup and sleep)
int PIN_INDICATOR_LED  = 12;   // digital pin used for LED indicator

int R = 9;                      // RED LED connected to digital pin 9 for PWM
int G = 10;                     // GRN LED connected to digital pin 10 for PWM
int B = 11;                     // BLU LED connected to digital pin 11 for PWM
int LED_PINS[] = {R,G,B};       //PIN # for each color
int LED_VALS[] = {0,0,0};       //PWM values for each LED

int lightMode = MODES[0];          //Current light show mode
int tLastWake = 0;                 //Time last mode change was initiated (used to determine how long till sleep)
unsigned long tAwake = 0;          //Time unit has been awake
unsigned long tOperating = 0;      //Time unit has been performing the current lightShow
unsigned long tLastModeChange = 0;

void setup()
{
  pinMode(PIN_TILT, INPUT);
  pinMode(PIN_INDICATOR_LED, OUTPUT);

  Serial.begin(9600);

  //Ensure the onboard LED is off                                    
  //digitalWrite(13,LOW);

  MCUSR = 0;
  wdt_disable();  // disable watchdog on boot
  
  preSleep();  
  postSleep();  
}

void loop() {
  if ( fWatchDog == 1 ) {
    fWatchDog = 0;
    digitalWrite(PIN_INDICATOR_LED, HIGH);
    int nextState = checkWDTWakeup();
    digitalWrite(PIN_INDICATOR_LED, LOW);

    Serial.print("nextState is ");
    Serial.println(nextState);
      
    switch ( nextState ) {
      case -1:
        wdt_disable();  // disable watchdog then go to idle sleep mode
        sleepNowTilt();
      case 0:  //Remain in the same state
        sleepNowWDT();
        break;
      case 1:  //Wake up
        wdt_disable();  // disable watchdog when running    
        postSleep();
        break;
    }
  }
  else {
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
  
    boolean activity = checkMode();
    
    if (!activity)
      checkSleep();
  }
}

void wakeUpTilt() {        // here the interrupt is handled after wakeup
  // execute code here after wake-up before returning to the loop() function
  // timers and code using timers (serial.print and more...) will not work here.
  // we don't really need to execute any special functions here, since we
  // just want the thing to wake up
  Serial.println("wakeUpNow");  
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
  
  if ( !isTiltActive )
    return -1;
  
  return 0;
}

//Housekeeping before sleeping
void preSleep() {
  digitalWrite(PIN_INDICATOR_LED,LOW);
}

void postSleep() {
  unsigned long n = millis();
  tLastWake = n;
  tLastModeChange = n;
  digitalWrite(PIN_INDICATOR_LED,LOW);
}

void sleepNowTimer()         // here we put the arduino to watchdog timer sleep mode because the run time passed a certain time.
{
  delay(100);     // this delay is needed, the sleep function may provoke a serial error otherwise!!

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

  setupWatchdog(7);  
  
  sleepNowWDT();
}

//****************************************************************
// 0=16ms, 1=32ms,2=64ms,3=128ms,4=250ms,5=500ms
// 6=1 sec,7=2 sec, 8=4 sec, 9= 8sec
void setupWatchdog(int ii) {
  byte bb;
  int ww;
  if (ii > 9 ) ii=9;
  bb=ii & 7;
  if (ii > 7) bb|= (1<<5);
  bb|= (1<<WDCE);
  ww=bb;
  Serial.println(ww);


  MCUSR &= ~(1<<WDRF);
  // start timed sequence
  WDTCSR |= (1<<WDCE) | (1<<WDE);
  // set new watchdog timeout value
  WDTCSR = bb;
  WDTCSR |= _BV(WDIE);
}

void sleepNowWDT() {
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

void sleepNowTilt()         // here we put the arduino to idle sleep mode because the tilt switch was de-activated
{
  delay(100);     // this delay is needed, the sleep function may provoke a serial error otherwise!!
  
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

  //cli();            // enable interrupts
  attachInterrupt(0,wakeUpTilt, LOW); // use interrupt 0 (pin 2) and run function
                                     // wakeUpNow when pin 2 gets LOW 
  preSleep();
  
  sleep_mode();            // here the device is actually put to sleep!!
                           // THE PROGRAM CONTINUES FROM HERE AFTER WAKING UP
  postSleep();
  
  sleep_disable();         // first thing after waking from sleep:
                           // disable sleep...
  detachInterrupt(0);      // disables interrupt 0 on pin 2 so the 
                           // wakeUpNow code will not be executed 
                           // during normal running time.
  //sei();          // disable interrupts after waking up from sleep                          
}

boolean checkMode() {
  int distance = calcAveragedIRDistanceAsync();
  
  if ( distance > 0 ) {
//    Serial.print("Distance is: ");
//    Serial.println(distance);
        
    if ( distance < DIST_MODE_CHG ) {
      unsigned long n = millis();
//      Serial.print("tDistanceModeBegin is: ");
//      Serial.println(tDistanceModeBegin);
      
      if ( tDistanceModeBegin == -1 ) {
        tDistanceModeBegin = n;
//        Serial.print("tDistanceModeBegin set to: ");
//        Serial.println(tDistanceModeBegin);
      }

      int tHolding = n - tDistanceModeBegin;
//      Serial.print("tHolding is: ");
//      Serial.println(tHolding);
        
      digitalWrite(PIN_INDICATOR_LED, flashState(tHolding, 300));
      
      if ( tHolding > T_MODE_HOLD ) {
        //Switch modes
        if ( lightMode == MODE_MAX )
          lightMode = 0;
        else
          lightMode++;
                
        tLastModeChange = millis();
        tDistanceModeBegin = -1;
        
        Serial.print("lightMode changed to ");
        Serial.println(lightMode);  
      }
//      else {
//        Serial.print("lightMode stays at ");
//        Serial.println(lightMode);  
//      }

      return true;
    }
    else {
      tDistanceModeBegin = -1;
      digitalWrite(PIN_INDICATOR_LED, LOW);
    }
    
    return false;
  }
}

boolean flashState(unsigned long tDelta, int period) {
  int state = tDelta / period;
  return state % 2;
}

int calcAveragedIRDistanceAsync() {
  return (int)calcIRDistance(true, DIST_N_SAMPLE);
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
  float rawDistance = 65*pow(volts, -1.10);          // worked out from graph 65 = theretical distance / (1/Volts)S - luckylarry.co.uk

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

int fadeAdj = 5;
void procLightShow()  {       
  //Calc the PWM value for each LED
  if ( LED_VALS[0] >= 255 && fadeAdj > 0 ) {
//    Serial.println("Switch fadeAdj down");
    fadeAdj *= -1;  
  }
  if ( LED_VALS[0] <= 0 && fadeAdj < 0 ) {
//    Serial.println("Switch fadeAdj up");
    fadeAdj *= -1;  
  }
  
  for (int i=0; i<3; i++) {
      LED_VALS[i] += fadeAdj;
  } 

  //Set the PWM value for each LED      
  for (int i=0; i<3; i++) {
      analogWrite(LED_PINS[i], LED_VALS[i]);         
  } 
  
//  Serial.print("LED_VALS[0] = ");
//  Serial.println(LED_VALS[0]);
}

void checkSleepSerial() {
  // Check for serial input for sleep the serial input
  if (Serial.available()) {
    int val = Serial.read();
    if (val == 'S') {
      Serial.println("Serial: Entering WDT sleep mode");
      sleepNowTimer();     //Go into watchdog sleep mode
    }
    if (val == 'A') {
      Serial.println("Hola Caracola"); // classic dummy message
    }
  }
}

void checkSleepTimer() {
   // check if it should go to sleep because of time
  if (tOperating >= T_AWAKE_MAX) {
      Serial.print("Operating for ");
      Serial.println(tOperating);
      Serial.println("TIMER: Entering WDT sleep mode");
      sleepNowTimer();     //Go into watchdog sleep mode
  }
}

boolean isTiltActive() {
  /***************Production mode****************/
  //HIGH == tilt switch indicates we should sleep
  //return digitalRead(PIN_TILT) == HIGH;


  /***************Test mode****************/
  //LOW == pushbutton switch indicates we should go to sleep
  return digitalRead(PIN_TILT) == LOW;
}

void checkSleepTilt() {
  if ( isTiltActive() ) {
      Serial.println("TILT: Entering idle sleep mode");
      sleepNowTilt();     // sleep function called here
  }
}

void checkSleep() {
  //checkSleepSerial();
  checkSleepTilt();
  checkSleepTimer(); 
}

