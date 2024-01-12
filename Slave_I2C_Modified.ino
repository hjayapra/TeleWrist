//I2C Slave
//Sending:  Force Sensor
//Receiving: Roll, Pitch, Yaw, Potentiometer
//===================
// Using I2C to send and receive structs between two Arduinos
//   SDA is the data connection and SCL is the clock connection
//   On an Uno  SDA is A4 and SCL is A5
//   On an Mega SDA is 20 and SCL is 21
//   GNDs must also be connected
//===================
        // data to be sent and received
struct I2cRxStruct {
    int potentiometer;      // 2 bytes
    int roll;               // 2
    int pitch;              // 2
    int yaw;                // 2
    int initialHeading;     // 2
    byte padding[10];       // 10
                            //------
                            // 18
};
I2cRxStruct rxData;
bool newRxData = false;
        // I2C control stuff
#include <Wire.h>
const byte thisAddress = 9; // these need to be swapped for the other Arduino
const byte otherAddress = 8;
//=================================
//Setting up servos:
#include <Servo.h>
Servo gripperServo;
Servo rollServo;
Servo pitchServo;
Servo yawServo;
#define GRIP_SER 6
#define ROLL_SER 10
#define PITCH_SER 11
#define YAW_SER 9
//=================================
//Setting up force sensor:
int forceSensorPin = A0;
int forceSensorVal = 0;
//=================================
//Setting up Haptic Driver
#include <Adafruit_Sensor.h>
#include "Adafruit_DRV2605.h"
Adafruit_DRV2605 drv;
void setup() {
    Serial.begin(115200);
    Serial.println("\nStarting I2C SlaveRespond demo\n");
        // set up I2C
    Wire.begin(thisAddress); // join i2c bus
    Wire.onReceive(receiveEvent); // register function to be called when a message arrives
    gripperServo.attach(GRIP_SER); //attach gripper servo to pin to open and close gripper
    rollServo.attach(ROLL_SER); //attaches ther servo on pin to the roll servo object
    pitchServo.attach(PITCH_SER); //attaches the servo on pin to the pitch servo object
    yawServo.attach(YAW_SER);
    //Zeroing servos
    int RPYzeroPos = map(90, 0, 180, 0, 130);
    int gripperZeroPos = 105;
    rollServo.write(RPYzeroPos);
    pitchServo.write(RPYzeroPos);
    yawServo.write(RPYzeroPos);
    gripperServo.write(gripperZeroPos);
    // set up haptic driver
    //Haptic Driver setup
    if (! drv.begin())
    {
    Serial.println("Could not find DRV2605");
    while (1) delay(10);
    }
    drv.selectLibrary(1);
    //I2C trigger by sending 'go' command
    //default, internal trigger when sending GO command
    drv.setMode(DRV2605_MODE_INTTRIG);
}
//============
void loop() {
        // this bit checks if a message has been received
    if (newRxData == true) {
      //Use received data to move servos
      //Need to check if this needs to happen in above loop or outside
        moveRPYServos();
        moveGripperServo();
        showNewData();
        newRxData = false;
      forceSensorVal = analogRead(forceSensorPin);
      setHapticEffect(forceSensorVal);
    }
}
//=============
void showNewData() {
    Serial.print("This just in    ");
    Serial.print(rxData.potentiometer);
    Serial.print(' ');
    Serial.print(rxData.roll);
    Serial.print(' ');
    Serial.print(rxData.pitch);
    Serial.print(' ');
    Serial.print(rxData.yaw);
    Serial.print(' ');
    Serial.println(rxData.initialHeading);

}
//============
        // this function is called by the Wire library when a message is received
void receiveEvent(int numBytesReceived) {
    if (newRxData == false) {
            // copy the data to rxData
        Wire.readBytes( (byte*) &rxData, numBytesReceived);
        newRxData = true;
    }
    else {
            // dump the data
        while(Wire.available() > 0) {
            byte c = Wire.read();
        }
    }
}
//===========
void moveRPYServos() {
  int roll = rxData.roll;
  int pitch = rxData.pitch;
  int yaw = rxData.yaw;
  //Moving servos
  rollServo.write(roll);
  pitchServo.write(pitch);
  yawServo.write(yaw);
}
//===========
void moveGripperServo() {
  int gripperVal = rxData.potentiometer;
  gripperServo.write(gripperVal);
}
void setHapticEffect(int forceSensorVal) {
  int flexiForceReading = forceSensorVal;
  Serial.println(flexiForceReading);
  int effect;
  //Set effect
  if (flexiForceReading < 50) {
    effect = 5;
  }
  else if (50 < flexiForceReading && flexiForceReading < 100) {
    effect = 4;
  }
  else if (100 < flexiForceReading && flexiForceReading < 200) {
    effect = 3;
  }
  else if (200 < flexiForceReading && flexiForceReading < 400) {
    effect = 2;
  }
  else if (flexiForceReading >= 400) {
    effect = 1;
  }
  //Print statements for testing
  if (effect == 1) {
    Serial.println(F("1 − Buzz 1 – 20%"));};
  if (effect == 2) {
    Serial.println(F("2 − Buzz 2 – 40%"));};
  if (effect == 3) {
    Serial.println(F("3 − Buzz 3 – 60%"));};
  if (effect == 4) {
    Serial.println(F("4 − Buzz 4 – 80%"));};
  if (effect == 5) {
    Serial.println(F("5 − Buzz 5 – 100%"));};
  //et the effect to play
  drv.setWaveform(0, effect);  // play effect
  drv.setWaveform(1, 0);       // end waveform
  // play the effect!
  drv.go();
  //Took out delay
}
