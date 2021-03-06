/*
 *  Original code by Bob Hartmann and Dave Vondle for http://labs.ideo.com March 2010
 *  This is free software. You can redistribute it and/or modify it under
 *  the terms of Creative Commons Attribution 3.0 United States License. 
 *  To view a copy of this license, visit http://creativecommons.org/licenses/by/3.0/us/ 
 *  or send a letter to Creative Commons, 171 Second Street, Suite 300, San Francisco, California, 94105, USA.
 *
*/

// LEDController Class Source
#include "LEDController.h"

//i2c constructor
 LEDController::LEDController(int i2cAddressIn, 
                  int ledAddressIn, 
                  boolean updateIn,
                  int actionIn,
                  int analogHighValueIn,
                  int analogMidValueIn,
                  int analogLowValueIn,
                  int analogValueIn,
                  int rampUpValueIn,
                  int rampDownValueIn,
                  int rampUpDelayIn,
                  int rampDownDelayIn,
                  boolean blinkStateIn,
                  int blinkTimeHighIn,
                  int blinkTimeLowIn,
                  unsigned long previousTimeIn
                  )
 {
  LEDSetAttributes(i2cAddressIn, ledAddressIn, updateIn, actionIn, analogHighValueIn, analogMidValueIn, analogLowValueIn, analogValueIn, rampUpValueIn, 
                   rampDownValueIn, rampUpDelayIn, rampDownDelayIn, 
                   blinkStateIn, blinkTimeHighIn, 
                   blinkTimeLowIn, previousTimeIn
                   );
  usingPWM = false;
 }
//PWM constructor
 LEDController::LEDController(int ioPinIn,
                  boolean updateIn,
                  int actionIn,
                  int analogHighValueIn,
                  int analogMidValueIn,
                  int analogLowValueIn,
                  int analogValueIn,
                  int rampUpValueIn,
                  int rampDownValueIn,
                  int rampUpDelayIn,
                  int rampDownDelayIn,
                  boolean blinkStateIn,
                  int blinkTimeHighIn,
                  int blinkTimeLowIn,
                  unsigned long previousTimeIn
                  )
 {
  LEDSetAttributes(ioPinIn, updateIn, actionIn, analogHighValueIn, analogMidValueIn, analogLowValueIn, analogValueIn, rampUpValueIn, 
                   rampDownValueIn, rampUpDelayIn, rampDownDelayIn, 
                   blinkStateIn, blinkTimeHighIn, 
                   blinkTimeLowIn, previousTimeIn
                   );
  usingPWM = true;
 }
 //i2c attribute setter
 void LEDController::LEDSetAttributes(int i2cAddressIn, int ledAddressIn, boolean updateIn, int actionIn, int analogHighValueIn, int analogMidValueIn, int analogLowValueIn, 
                                     int analogValueIn, int rampUpValueIn, int rampDownValueIn, int rampUpDelayIn, int rampDownDelayIn,
                                     boolean blinkStateIn, int blinkTimeHighIn, int blinkTimeLowIn, 
                                     unsigned long previousTimeIn
                                     )
 {
  i2cAddress = i2cAddressIn;
  ledAddress = ledAddressIn;
  update = updateIn;
  action = actionIn;
  analogHighValue = analogHighValueIn;
  analogMidValue = analogMidValueIn;
  analogLowValue = analogLowValueIn;
  analogValue = analogValueIn;
  rampUpValue = rampUpValueIn;
  rampDownValue = rampDownValueIn;
  rampUpDelay = rampUpDelayIn;
  rampDownDelay = rampDownDelayIn;
  blinkState = blinkStateIn;
  blinkTimeHigh = blinkTimeHighIn;
  blinkTimeLow = blinkTimeLowIn;
  previousTime = previousTimeIn;
 }
  //PWM attribute setter
 void LEDController::LEDSetAttributes(int ioPinIn, boolean updateIn, int actionIn, int analogHighValueIn, int analogMidValueIn, int analogLowValueIn, 
                                     int analogValueIn, int rampUpValueIn, int rampDownValueIn, int rampUpDelayIn, int rampDownDelayIn,
                                     boolean blinkStateIn, int blinkTimeHighIn, int blinkTimeLowIn, 
                                     unsigned long previousTimeIn
                                     )
 {
  ioPin = ioPinIn;
  update = updateIn;
  action = actionIn;
  analogHighValue = analogHighValueIn;
  analogMidValue = analogMidValueIn;
  analogLowValue = analogLowValueIn;
  analogValue = analogValueIn;
  rampUpValue = rampUpValueIn;
  rampDownValue = rampDownValueIn;
  rampUpDelay = rampUpDelayIn;
  rampDownDelay = rampDownDelayIn;
  blinkState = blinkStateIn;
  blinkTimeHigh = blinkTimeHighIn;
  blinkTimeLow = blinkTimeLowIn;
  previousTime = previousTimeIn;
}

 void LEDController::takeAction(){
     
   unsigned long thisTime;
   unsigned long deltaTime;
     
   if(update){
     thisTime = millis();
     deltaTime = thisTime - previousTime;
     
     switch(action){
       
       case TURN_HIGH:    
             setLEDBrightness(analogHighValue);

         break;
       case TURN_MID:
             setLEDBrightness(analogMidValue);

         break;
       case TURN_LOW:
             setLEDBrightness(analogLowValue);

         break;
       
       case TURN_OFF:
             setLEDBrightness(0);

         break;
       
       case BLINK:
 
          if((blinkState)&&(deltaTime >= blinkTimeHigh)){
              analogWrite(ioPin, analogLowValue);
              blinkState = !blinkState;
              previousTime = thisTime;
          }
          else
          if((!blinkState)&&(deltaTime >= blinkTimeLow)){
              analogWrite(ioPin, analogHighValue);
              blinkState = !blinkState;
              previousTime = thisTime;
          }
         

         break;
       case RAMP_HIGH:
 
         if(deltaTime >= rampUpDelay)
         {
          
           analogValue = analogValue + rampUpValue;
           if(analogValue >= analogHighValue){
             analogValue = analogHighValue;
           }
             previousTime = thisTime;
         }
         
         setLEDBrightness(analogValue);
         
         break;
        
       case RAMP_LOW:
          if(deltaTime >= rampDownDelay)
         {
          
           analogValue = analogValue - rampDownValue;
           if(analogValue <= analogLowValue){
             analogValue = analogLowValue;
           }
             previousTime = thisTime;
         }
         
         setLEDBrightness(analogValue);
         break;
         
       case RAMP_CONT_HIGH_LOW:
    
          if(blinkState){
            if(deltaTime >= rampUpDelay){
              analogValue = analogValue + rampUpValue;
               if(analogValue >= analogHighValue){
                 analogValue = analogHighValue;
               blinkState = !blinkState;
               }
               previousTime = thisTime;
           }
          }
          else{
            if(deltaTime >= rampDownDelay){
                analogValue = analogValue - rampUpValue;
                 if(analogValue <= analogLowValue){
                   analogValue = analogLowValue;
                   blinkState = !blinkState;
                 }
             previousTime = thisTime;
           }
          }
         setLEDBrightness(analogValue);
         break;
       
       

         
      case RAMP_CONT_HIGH_MID:
       
          if(blinkState){
            if(deltaTime >= rampUpDelay){
              analogValue = analogValue + rampUpValue;
               if(analogValue >= analogHighValue){
                 analogValue = analogHighValue;
               blinkState = !blinkState;
               }
               previousTime = thisTime;
           }
          }
          else{
            if(deltaTime >= rampDownDelay){
                analogValue = analogValue - rampUpValue;
                 if(analogValue <= analogMidValue){
                   analogValue = analogMidValue;
                   blinkState = !blinkState;
                 }
             previousTime = thisTime;
           }
          }
          setLEDBrightness(analogValue);
         break;  
     } // End of switch
    
  } //End of if   

 } // End of takeAction
 
    void LEDController::setLEDBrightness(int ledValue){
      if(usingPWM){
        analogWrite(ioPin, ledValue);
      }else{
        if((i2cAddress < 0x1C) || (i2cAddress > 0x1D))
          return;
        
        if((ledAddress < 1) || (ledAddress > 18))
          return;
          
        if((ledValue < 0) ||(ledValue > 255))
          return;
  
        
        Wire.beginTransmission((byte)i2cAddress);    // LED controller I2C address, the first byte sent
        Wire.send((byte)ledAddress); // The second byte
        Wire.send((byte)(ledValue/4));  // The third byte, the level of desired current, set it to half way.  
                                        //divide by 4 because 0-255 is how the arduino PWM works, and this wants 0-63
        Wire.endTransmission();  // Send the data

      }
    }

