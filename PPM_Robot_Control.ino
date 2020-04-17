/*This program puts the servo values into an array,
 reagrdless of channel number, polarity, ppm frame length, etc...
 You can even change these while scanning!*/
 
#include "ZumoMotors.h"
#include <Pixy2.h>

Pixy2 pixy;
ZumoMotors motors;

#define PPM_Pin 5  //this must be 2 or 3 on older Arduinos. On Uno Wifi Rev2 It can be any pin.
volatile int ppm[16];  //array for storing up to 16 servo signals

byte state = HIGH;
int waitingForSyncCount = 0;
volatile bool readingAvailable = false;

//Pixy Cam Servo pan and tilt.
int panValue = 500;
int tiltValue = 500;
unsigned long lastMicrosCount = 0;

// We skip readings near the end of Micros, and reset to waiting for Sync.
#define SKIPREADINGS 4294967295 - 4000

const int MAXPULSE = 1900;
const int MINPULSE = 1100;
const int MIDPULSE = 1500;
const int TOLERANCEPULSEM = 13;

#define THRUST 0 // is thrust, 1900 is full,  980 is Disarmed (Left top Index Switch)
#define ROLL 1   // is Roll 1900 is left
#define PITCH 2  // is PITCH 1900 is Forward
#define YAW 3    // is YAW 1900 is left
#define MODE 4  // is Flight modes, 1900 is 0, 1500 is 1, 1100 is 2
#define FLAPS 5  // is Flaps, 1900 is 0, 1500 is 1, 1100 is 2
#define NOTUSED 6 // is
#define AUX 7    // is Aux


void setup()
{
  for (int i =0; i < 16; i++) {
    ppm[i] = 0;
  }
  
  Serial.begin(115200);
  Serial.println("ready");

  //timer0_overflow_count = 4294967295 - 20000;
  
  #if defined(servoOut)
  for(byte i=0; sizeof(servo)-1; i++) pinMode(servo[i], OUTPUT);
  #endif
 #if defined(ARDUINO_ARCH_SAMD)
   pinMode(PPM_Pin, INPUT_PULLUP);
   attachInterrupt(PPM_Pin, read_ppm, RISING);
#else
  pinMode(PPM_Pin, INPUT);
  attachInterrupt(digitalPinToInterrupt(PPM_Pin), read_ppm, RISING); //PPM_Pin - 2
#endif

  pinMode(LED_BUILTIN, OUTPUT);

 //reverse both motors.
  motors.flipLeftMotor(true);
  motors.flipRightMotor(true);

  pixy.init();
  pixy.changeProg("video");
}

void loop()
{

  if (readingAvailable) {
    readingAvailable = false;
  }

  digitalWrite(LED_BUILTIN, state);

  #if defined(DEBUG)
  int count = 0;
  while(ppm[count] != 0){  //print out the servo values
    Serial.print(ppm[count]);
    Serial.print("  ");
    count++;
  }
  #endif

  int leftThrustValue = ppm[THRUST] - MIDPULSE;
  int rightThrustValue = leftThrustValue;
  bool disarmed = false;

  static int mode = 0;
  static int modeOld = 0;

  if (isWithinZeroTolerance(ppm[MODE] - MAXPULSE)) {
    disarmed = true;
    mode = 0;
  } else if (isWithinZeroTolerance(ppm[MODE] - MIDPULSE)) { 
     //armed, no extra action here
     mode = 1;
  } else if (isWithinZeroTolerance(ppm[MODE] - MINPULSE)) { 
    // mode = 2 color conneected components
    mode = 2;
  }

  static uint8_t lampToggle = 0;
  
  if (modeOld != mode) {
    //state changes here
    if (mode == 2) {
      pixy.changeProg("color_connected_components");
    } else {
      pixy.changeProg("video");
    }

    if (mode == 1) {
        if (lampToggle == 0) {
          lampToggle = 1;
        } else {
          lampToggle = 0;
        }
        
      pixy.setLamp(lampToggle, lampToggle);
    }
  } // if modeOld != mode
  
  
  if (!disarmed && isWithinMidTolerance(ppm[THRUST]) && isWithinMidTolerance(ppm[YAW])) {
    //Serial.print("STOPPED ");
    leftThrustValue = 0;
    rightThrustValue = 0;
  } else if (ppm[THRUST] < 1000) {
    //Serial.println("DISARMED ");
    panValue = 500;
    tiltValue = 500;
    pixy.setServos(panValue, tiltValue);
    disarmed = true;
    rightThrustValue = 0;
    leftThrustValue = 0;
  }
//  else if (leftThrustValue > 0) {
//    Serial.print("Forward ");
//  } else if (leftThrustValue < 0) {
//    Serial.print("Reverse ");
//  }

  if (!disarmed) {
    //get the centered Yaw of 0. Midpulse is 1500 or so.
    int yawValue = ppm[YAW] - MIDPULSE;

    //apply tolerence to it.
    if (isWithinMidTolerance(ppm[YAW])) {
      //do nothing
    } else if (yawValue > 0) {
      //scale left and right motor speeds
      leftThrustValue -= yawValue;
      rightThrustValue += yawValue;
    } else if (yawValue < 0) {
      leftThrustValue -= yawValue;
      rightThrustValue += yawValue;
    }

    unsigned long microCount = micros();
    unsigned long counterMs = microCount - lastMicrosCount;

    if (counterMs > 3000) {
      lastMicrosCount = microCount;
      if (!isWithinMidTolerance(ppm[PITCH])) {
         int pitch = (ppm[PITCH] - 1500) / 30;

         tiltValue += pitch;
         limitValueTo1K(&tiltValue);
         pixy.setServos(panValue, tiltValue);
      } 
      
      if (!isWithinMidTolerance(ppm[ROLL])) {
        int pan = (ppm[ROLL] - 1500) / 30;
        panValue += pan;
        limitValueTo1K(&panValue);
        pixy.setServos(panValue, tiltValue);
      }

      Serial.print(tiltValue);
      Serial.print("\t");
      Serial.println(panValue);
    }// if (counterMs > 35)
  } // if !disarmed


  if (isWithinZeroTolerance(leftThrustValue) && isWithinZeroTolerance(rightThrustValue)) {
    motors.setSpeeds(0, 0);
  } else {
    motors.setSpeeds(leftThrustValue, rightThrustValue);
  }
  
  state = !state;
  modeOld = mode;
}

void limitValueTo1K(int *inValue) {
  if (*inValue < 0) { 
    *inValue = 0;
  } else if (*inValue > 1000) {
    *inValue = 1000;
  }
}

void read_ppm(){  //leave this alone
  
  static byte channel = -1; //wait for sync mode by default
  static unsigned long last_micros = 0;

  unsigned long microCount = micros();
  unsigned long counterMs = microCount - last_micros;

  if (microCount > SKIPREADINGS) {
    waitingForSyncCount += 1;
    channel = -1; //wait for sync mode
    last_micros = microCount;
    readingAvailable = false;
    return;
  }

  //sync pulses over 2100us *multiplier
  // also wait till we are out of waiting for sync mode
  if(counterMs > 2100 || channel == -1) {
    channel = 0;
    
    readingAvailable = true;
    
    #if defined(DEBUG)
    Serial.print("PPM Frame Len: ");
    Serial.println(microCount - last_micros);
    #endif
  } 
  else
  //ya this won't work, the disarm command sends this to below 1000... which is fine for throttle.
  if (counterMs > 970) {  //servo values between 710us and 2420us will end up here
    ppm[channel] = counterMs;

    #if defined(DEBUG)
    Serial.print(ppm[channel]);
    Serial.print("  ");
    #endif

  // perhaps one more else statement to increment the channel. We don't want to miss our sync because a time was off.
    channel++;
  } else {
    channel++; //if any reading was lower than 1000
  }

  last_micros = microCount;
}

//if a reading is withing the tolerance range of MID +/- TOLERANCEPULSEM to MIDPULSE (+/-10 to 1500) returns true
bool isWithinMidTolerance(int reading) {
  //MIDPULSE
  //TOLERANCEPULSEM
  if (reading > MIDPULSE + TOLERANCEPULSEM) {
    return false;
  } else if (reading < MIDPULSE - TOLERANCEPULSEM) {
    return false;
  }

  return true;
}

// Use this to send in a reading minus MID, MAX or MIN for Modes for example.
//Tests to see if the 'reading' is around 0 plus or minus the TOLERANCE amount.
bool isWithinZeroTolerance(int reading) {
  //MIDPULSE
  //TOLERANCEPULSEM
  if (reading > TOLERANCEPULSEM) {
    return false;
  } else if (reading < -1 * TOLERANCEPULSEM) {
    return false;
  }

  return true;
}
