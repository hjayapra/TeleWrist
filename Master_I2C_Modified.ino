//I2C Master
//Sending: Receiving: Roll, Pitch, Yaw, Potentiometer
//Receiving: Force Sensor
//===================
// Using I2C to send and receive structs between two Arduinos
//   SDA is the data connection and SCL is the clock connection
//   On an Uno  SDA is A4 and SCL is A5
//   On an Mega SDA is 20 and SCL is 21
//   GNDs must also be connected
//===================
        // data to be sent and received
struct I2cTxStruct {
    int potentiometer;      // 2 bytes
    int roll;               // 2
    int pitch;              // 2
    int yaw;                // 2
    int initialHeading;     // 2
    byte padding[10];       // 10
                            //------
                            // 18
};
I2cTxStruct txData = {0, 0, 0, 0, 0};
bool newTxData = false;
        // I2C control
#include <Wire.h>
const byte thisAddress = 8; // these need to be swapped for the other Arduino
const byte otherAddress = 9;
        // timing variables
unsigned long prevUpdateTime = 0;
unsigned long updateInterval = 50;
//=================================
//Setting up potentiometer
int potentiometerPin = A0;
#define SWITCH_PIN 2
bool isStart = false;
bool lastState = HIGH; 
//=================================
//Setting up IMU
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>
/* Set the delay between fresh samples */
#define BNO055_SAMPLERATE_DELAY_MS (100) //#TODO:Check whether this can be 0
// Check I2C device address and correct line below (by default address is 0x29 or 0x28)
//                                   id, address
Adafruit_BNO055 bno = Adafruit_BNO055(55, 0x28);
//=================================
//Setting up IMU reading processing
/* Board layout:
         +----------+
         |         *| RST   PITCH  ROLL  HEADING
     ADR |*        *| SCL
     INT |*        *| SDA     ^            /->
     PS1 |*        *| GND     |            |
     PS0 |*        *| 3VO     Y    Z-->    \-X
         |         *| VIN
         +----------+
  */
struct IMUValues {
  int roll;
  int pitch;
  int yaw;
};
bool haveInitializedHeading = false;
bool headingPosnSet = false;
int INITIAL_HEADING = 0;
int correctedX_0 = 0; //x if yaw = zero was along the axis of the wrist
int correctedX_90 = 0; //x if yaw = 90 was along the axis of the wrist
//=================================
//Setting up IMU calibration
uint8_t sys, gyro, accel, mag = 0;
void displaySensorDetails(void) //#TODO: check delay
{
  sensor_t sensor;
  bno.getSensor(&sensor);
  Serial.println("------------------------------------");
  Serial.print  ("Sensor:       "); Serial.println(sensor.name);
  Serial.print  ("Driver Ver:   "); Serial.println(sensor.version);
  Serial.print  ("Unique ID:    "); Serial.println(sensor.sensor_id);
  Serial.print  ("Max Value:    "); Serial.print(sensor.max_value); Serial.println(" xxx");
  Serial.print  ("Min Value:    "); Serial.print(sensor.min_value); Serial.println(" xxx");
  Serial.print  ("Resolution:   "); Serial.print(sensor.resolution); Serial.println(" xxx");
  Serial.println("------------------------------------");
  Serial.println("");
  delay(500);
}
//=================================
void setup() {
    Serial.begin(115200);
    pinMode(SWITCH_PIN, INPUT_PULLUP); 
    Serial.println("\nStarting I2C Master demo\n");
        // set up I2C
    Wire.begin(thisAddress); // join i2c bus
    //~ Wire.onReceive(receiveEvent); // register function to be called when a message arrives
        //initializing IMU
    Serial.println("Orientation Sensor Test"); Serial.println("");
    if(!bno.begin())
  {
    /* There was a problem detecting the BNO055 ... check your connections */
    Serial.print("Ooops, no BNO055 detected ... Check your wiring or I2C ADDR!");
    while(1);
  }
  delay(1000);
    /* Use external crystal for better accuracy */
  bno.setExtCrystalUse(true);
  /* Display some basic information on this sensor */
  displaySensorDetails();
}
//============
void loop() {
  int buttonState = digitalRead(SWITCH_PIN);
  //Setting up conditions to begin transmitting data
    //Checking if heading has been set
  //Serial.println("Entered Loop");
  if (buttonState == LOW && lastState == HIGH) {
  isStart = !isStart; 
  Serial.print("Button pressed. isStart is now: ");
  Serial.println(isStart);
  }
  lastState = buttonState; 

  if (isStart == false){
    headingPosnSet = true;
    //Serial.println("Heading Position Set!");
  }
  if (isStart == true) {
    headingPosnSet = false; 
  }
  bno.getCalibration(&sys, &gyro, &accel, &mag); //#TODO: move into calibration function
  Serial.print(F("IMU Calibration: "));
  Serial.print(sys, DEC);
  Serial.print(F(" "));
  Serial.print(gyro, DEC);
  Serial.print(F(" "));
  Serial.print(accel, DEC);
  Serial.print(F(" "));
  Serial.println(mag, DEC);
  int initialHeading = getHeading();
  if(mag == 3 && haveInitializedHeading == false && headingPosnSet == true){
    INITIAL_HEADING = initialHeading;
    Serial.print(INITIAL_HEADING);
    txData.initialHeading = INITIAL_HEADING;
    haveInitializedHeading = true;
  }
        // this function updates the data in txData
  if(gyro == 3 && mag == 3 && haveInitializedHeading){
    updateDataToSend();
  }
        // this function sends the data if one is ready to be sent
    transmitData();
}
//============
void updateDataToSend() {
  //Send potentiometer, roll, pitch, yaw
  int potentiometerReading;
  int roll;
  int pitch;
  int yaw;
    if (millis() - prevUpdateTime >= updateInterval) {
        prevUpdateTime = millis();
        if (newTxData == false) { // ensure previous message has been sent
        potentiometerReading = getPotentiometerReading();
        IMUValues imuReadings = getIMUReadings();
        txData.potentiometer = potentiometerReading;
        txData.roll = imuReadings.roll;
        txData.pitch = imuReadings.pitch;
        txData.yaw = imuReadings.yaw;
        newTxData = true;
        }
    }
}
//============
void transmitData() {
    if (newTxData == true) {
        Wire.beginTransmission(otherAddress);
        Wire.write((byte*) &txData, sizeof(txData));
        Wire.endTransmission();    // this is what actually sends the data
            // for demo show the data that as been sent
        // Serial.print("Sent ");
        // Serial.print(txData.potentiometer);
        // Serial.print(' ');
        // Serial.print(txData.roll);
        // Serial.print(' ');
        // Serial.print(txData.pitch);
        // Serial.print(' ');
        // Serial.println(txData.yaw);
        newTxData = false;
    }
}
//=============
int getHeading(){
  sensors_event_t event;
  bno.getEvent(&event);
  //Getting x, y, and z values
  int initialHeading = int(event.orientation.x);
  return initialHeading;
}
IMUValues getIMUReadings(){
  IMUValues imuReadings;
  int x;
  int y;
  int z;
  int mappedZ;
  int mappedY;
  int mappedX;
  int roll;
  int pitch;
  int yaw;
  sensors_event_t event;
  bno.getEvent(&event);
  //Getting x, y, and z values
  x = int(event.orientation.x);
  y = int(event.orientation.y);
  z = int(event.orientation.z);
  Serial.print(F("Original IMU Orientation: "));
   Serial.print(x);
   Serial.print(F(" "));
   Serial.print(y);
   Serial.print(F(" "));
   Serial.println(z);
  // Serial.println(F(""));
  //Map values to roll, pitch, and yaw
  //roll
  if (-180 < z && z < -90){
    z = -90;
    mappedZ = map(z, -90, 90, 0, 180);
    roll = map(mappedZ, 0, 180, 0, 130);
    // Serial.print("     Condition -180 -> -90 : ");
    // Serial.print("Roll: ");
    // Serial.println(roll);
  }
  else if (90 < z && z < 180){
     z = 90;
     mappedZ = map(z, -90, 90, 0, 180);
     roll = map(mappedZ, 0, 180, 0, 130);
    //  Serial.print("     Condition 90 -> 180 : ");
    //  Serial.print("Roll: ");
    //  Serial.println(roll);
   }
  //  else if (z == 180){
  //   z = 180;
  //   roll = map(z, 0, 180, 0, 130);
  //  }
  //  else if (z == -180){
  //   z = 0;
  //   roll = map(z, 0, 180, 0, 130);
  //  }
  else {
     mappedZ = map(z, -90, 90, 0, 180);
     roll = map(mappedZ, 0, 180, 0, 130);
    //  Serial.print("     Condition -90 -> 90 : ");
    //  Serial.print("Roll: ");
    //  Serial.println(roll);
  }
  //pitch
  mappedY = map(y, 90, -90, 0, 180);
  pitch = map(mappedY, 0, 180, 0, 130);
  //yaw
  correctedX_0 = x - INITIAL_HEADING;
  if(correctedX_0 < 0){
    correctedX_0 = correctedX_0 + 360;
  }
  correctedX_90 = correctedX_0 + 90 ;
  if(correctedX_90 > 360)
  {
    correctedX_90 = correctedX_90 - 360;
  }
  mappedX = map(correctedX_90, 180, 0, 0, 180);
  yaw = map(mappedX, 0, 180, 0, 130);
  imuReadings.roll = roll;
  imuReadings.yaw = yaw;
  imuReadings.pitch = pitch;
  return imuReadings;
}
int getPotentiometerReading(){
  int potentiometerReading = analogRead(potentiometerPin);
  return map(potentiometerReading, 157, 26, 105, 54); //scale it to use it with the servo (value between 0 and 20)
}