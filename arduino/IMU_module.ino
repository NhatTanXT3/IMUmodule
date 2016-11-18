//#include <I2Cdev.h>
//#include<Wire.h>
//#include "myIMU.h"
#include "Communication.h"
#include "myMPU6050.h"

/*===================================
    Define hardware
  ====================================*/
//int intPin = 2;
myMPU6050 myIMU;
int ledTest = 13;
unsigned char ledState = 0;



/*=====================================
             timer variables
  =====================================*/
unsigned long currentMicros;
unsigned long previousMicros_100Hz = 0;        // will store last time it was called for 100Hz timer
const long interval_100Hz = 10000;           // interval at which to do 100Hz task(micros)

unsigned long previousMicros_50Hz = 0;        // will store last time it was called for 50Hz timer
const long interval_50Hz = 20000;           // interval at which to do 50Hz task (micros)

unsigned long previousMicros_200Hz = 0;       // will store last time it was called for 200Hz timer
const long interval_200Hz = 5000;           // interval at which to do 200Hz task (micros)

float sampling_time_second = 0;
unsigned long pre_sampling_time_micro = 0;


void setup() {
  Serial.begin(115200);
  Serial.println(" begin config");
  pinMode(ledTest, OUTPUT); // pin for testing
  myIMU.init_module();
  Serial.println(" Calib gyro ...");
  while (myIMU.Calib_Gyro());
  Serial.println(" Calib accelerometer ...");
  while(myIMU.Calib_Accelerometer_Amplitude());
}

void loop() {
  serialEvent();
  currentMicros = micros();
  if (currentMicros - previousMicros_200Hz >= interval_200Hz)
  {
    previousMicros_200Hz = currentMicros;
    //your code begin from here

  }

  if (currentMicros - previousMicros_100Hz >= interval_100Hz)
  {
    previousMicros_100Hz = currentMicros;
    //your code begin from here
    if (flag.display)
    {
      display();
    }

  }

  if (currentMicros - previousMicros_50Hz >= interval_50Hz)
  {
    previousMicros_50Hz = currentMicros;
    //your code begin from here
    ledState ^= 1;
    digitalWrite(ledTest, ledState);

  }

  if (myIMU.flag_MPU6050_INTpin)
  {
    myIMU.flag_MPU6050_INTpin = 0;
    //your code begin from here
    sampling_time_second = (float)(currentMicros - pre_sampling_time_micro) / 1000000.0;
    pre_sampling_time_micro = currentMicros;
    myIMU.sensorUpdate();
    myIMU.sensorProcess(sampling_time_second);
  }

}

void display() {
  //your code start from here
  Serial.write(CN_1);
  Serial.print(myIMU.roll);
  Serial.write(CN_2);
  Serial.print(myIMU.pitch);
//  Serial.write(CN_3);
//  Serial.print(myIMU.yaw_gyro);
  Serial.write(CN_4);
  Serial.print(myIMU.accX_linear);
  Serial.write(CN_5);
  Serial.print(myIMU.accY_linear);
  Serial.write(CN_6);
  Serial.println(myIMU.accZ_linear);
//  Serial.write(CN_7);
//  Serial.println(myIMU.accX);
//  Serial.write(CN_8);
//  Serial.println(myIMU.accY);
//  Serial.write(CN_9);
//  Serial.println(myIMU.accZ);



}

void serialEvent() {
  while (Serial.available()) {
    // get the new byte:
    char inChar = (char)Serial.read();
    // add it to the inputString:
    command[command_index] = inChar;
    command_index++;

    // if the incoming character is a newline, set a flag
    // so the main loop can do something about it:
    if (inChar == '\n') {
      stringComplete = true;
    }
  }

  if (stringComplete) {
    Serial.print(command);
    switch (command[0])
    {
      case DISPLAY_ON_:
        flag.display = 1;
        break;
      case DISPLAY_OFF_:
        flag.display = 0;
        break;
      default:
        break;
    }
    memset(command, 0, sizeof(command));
    command_index = 0;
    // clear the string:
    stringComplete = false;
  }
}
