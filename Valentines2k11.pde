#include <avr/sleep.h>

int DIST_SENS_MIN = 19;
int DIST_MODE_CHG = DIST_SENS_MIN * 2;
int N_DIST_SAMPLE = 250;

int T_AWAKE_MAX = 500000;   //Total time to stay awake, milliseconds
int T_MODE_HOLD = 2000;      //milliseconds

int MODE_A = 0;
int MODE_B = 1;
int MODE_C = 2;
int MODE_MAX = 2;
int MODES[] = { MODE_A, MODE_B, MODE_C };

int IRpin = 1;                  // analog pin for reading the IR sensor
int wakePin = 2;                // digital pin used for waking up
int indLED = 12;                //digital pin used for LED indicator

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
  pinMode(wakePin, INPUT);
  pinMode(indLED, OUTPUT);

  Serial.begin(9600);

  //Ensure the onboard LED is off                                    
  //digitalWrite(13,LOW);
  
  preSleep();  
  postSleep();  
}

void loop2() {
  unsigned long n = millis();
  Serial.print("N is ");
  Serial.println(n);
}

void loop() {
  unsigned long n = millis();
  tAwake = n - tLastWake;
  tOperating = n - tLastModeChange;
  
  // display information about the counter
//  Serial.print("N is ");
//  Serial.println(n);
//  Serial.print("tLastWake is ");
//  Serial.println(tLastWake);
//  Serial.print("tLastModeChange is ");
//  Serial.println(tLastModeChange);
//  Serial.print("Awake for ");
//  Serial.println(tAwake);
//  Serial.print("Operating for ");
//  Serial.println(tOperating);

  checkMode();

  procLightShow();
  
  checkSleep();
}

//Housekeeping before sleeping
void preSleep() {
  digitalWrite(13,LOW);
  digitalWrite(indLED,HIGH);
}

void wakeUpNow() {        // here the interrupt is handled after wakeup
  // execute code here after wake-up before returning to the loop() function
  // timers and code using timers (serial.print and more...) will not work here.
  // we don't really need to execute any special functions here, since we
  // just want the thing to wake up
  Serial.println("wakeUpNow");  
}

void postSleep() {
  unsigned long n = millis();
  tLastWake = n;
  tLastModeChange = n;
  digitalWrite(indLED,LOW);
}

void sleepNow()         // here we put the arduino to sleep
{
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

  attachInterrupt(0,wakeUpNow, LOW); // use interrupt 0 (pin 2) and run function
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
}

unsigned long tDistanceModeBegin = -1;
void checkMode() {
  int distance = (int)calcIRDistance();
  
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
        
      digitalWrite(indLED, flashState(tHolding, 300));
      
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
    }
    else {
      tDistanceModeBegin = -1;
      digitalWrite(indLED, LOW);
    }
  }
}

boolean flashState(unsigned long tDelta, int period) {
  int state = tDelta / period;
  return state % 2;
}

//Variables for running distance calculation
double distanceSampleSum = 0.0;
int distanceSampleCount = 0;
float calcIRDistance() {
  
  float volts = analogRead(IRpin)*0.0048828125;   // value from sensor * (5/1024) - if running 3.3.volts then change 5 to 3.3
  float rawDistance = 65*pow(volts, -1.10);          // worked out from graph 65 = theretical distance / (1/Volts)S - luckylarry.co.uk
  
//  Serial.print("rawDistance is: ");
//  Serial.println(rawDistance);

  distanceSampleSum += rawDistance;
  distanceSampleCount++;
  
//  Serial.print("distanceSampleSum: ");
//  Serial.println(distanceSampleSum);
  
  if (distanceSampleCount >= N_DIST_SAMPLE ) {    
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

void checkSleep() {
  // compute the serial input
  if (Serial.available()) {
    int val = Serial.read();
    if (val == 'S') {
      Serial.println("Serial: Entering Sleep mode");
      delay(100);     // this delay is needed, the sleep 
                      //function will provoke a Serial error otherwise!!
      sleepNow();     // sleep function called here
    }
    if (val == 'A') {
      Serial.println("Hola Caracola"); // classic dummy message
    }
  }

  // check if it should go to sleep because of time
  if (tOperating >= T_AWAKE_MAX) {
      Serial.println("Timer: Entering Sleep mode");
      delay(100);     // this delay is needed, the sleep 
                      //function will provoke a Serial error otherwise!!
      sleepNow();     // sleep function called here
  }
}

