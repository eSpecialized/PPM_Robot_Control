/*This program puts the servo values into an array,
 reagrdless of channel number, polarity, ppm frame length, etc...
 You can even change these while scanning!*/
#include "ZumoMotors.h"


ZumoMotors motors;

#define PPM_Pin 5  //this must be 2 or 3 on older Arduinos. On Uno Wifi Rev2 It can be any pin.
volatile int ppm[16];  //array for storing up to 16 servo signals
byte servo[] = {1,4,5,6,7,8,9,10};  //pin number of servo output
//#define servoOut  //comment this if you don't want servo output, it also appears to dead lock the program.
//#define DEBUG
byte state = HIGH;
int waitingForSyncCount = 0;

// We skip readings near the end of Micros, and reset to waiting for Sync.
#define SKIPREADINGS 4294967295 - 4000

//extern unsigned long timer0_overflow_count;

const int MAXPULSE = 1900;
const int MINPULSE = 1100;
const int MIDPULSE = 1500;
const int TOLERANCEPULSEM = 13;

#define THRUST 0 // is thrust, 1900 is full,  980 is Disarmed (Left top Index Switch)
#define ROLL 1   // is Roll 1900 is left
#define PITCH 2  // is PITCH 1900 is Forward
#define YAW 3    // is YAW 1900 is left
#define MODES 4  // is Flight modes, 1900 is 0, 1500 is 1, 1100 is 2
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

  motors.flipLeftMotor(true);
  motors.flipRightMotor(true);



}

void loop()
{
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
  
  if (isWithinMidTolerance(ppm[THRUST]) && isWithinMidTolerance(ppm[YAW])) {
    //Serial.print("STOPPED ");
    leftThrustValue = 0;
    rightThrustValue = 0;
  } else if (ppm[THRUST] < 1000) {
    //Serial.println("DISARMED ");
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
    int yawValue = ppm[YAW] - MIDPULSE;
    if (isWithinMidTolerance(ppm[YAW])) {
      //Serial.println(" ");
    } else if (yawValue > 0) {
      //scale left and right motor speeds
      leftThrustValue -= yawValue;
      rightThrustValue += yawValue;
      //Serial.print(" + Rotate Left ");
    } else if (yawValue < 0) {
      //Serial.print(" + Rotate Right ");
      leftThrustValue -= yawValue;
      rightThrustValue += yawValue;
    }
  }

//  Serial.print(leftThrustValue);
//  Serial.print("  ");
//  Serial.println(rightThrustValue);
  if (isWithinZeroTolerance(leftThrustValue) && isWithinZeroTolerance(rightThrustValue)) {
    motors.setSpeeds(0, 0);
  } else {
    motors.setSpeeds(leftThrustValue, rightThrustValue);
  }
  
  //Serial.print("  Sync:");
  //Serial.println(waitingForSyncCount);
  
  //delay(500);  //you can even use delays!!!
  state = !state;
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
    return;
  }

  //sync pulses over 2100us *multiplier
  // also wait till we are out of waiting for sync mode
  if(counterMs > 2100 || channel == -1) {
    channel = 0;

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
