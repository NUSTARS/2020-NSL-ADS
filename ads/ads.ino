#include <string>
#include "sensors.h"

using namespace nustars;

byte value = 0;
Accelerometer* accelerometer = NULL;
Altimeter* altimeter = NULL;

void setup(){
  Serial.begin(9600);
  accelerometer = new Accelerometer;
  altimeter = new Altimeter;
}

void loop(){
  Serial.println("Hello World...");
  delay(1000);

  char* msg = new char[500] {0};
  accelerometer->tick();
  altimeter->tick();

  //check for BNO connection
  Wire.beginTransmission(BNO055_ADDRESS_A);
  Wire.send(0x00);
  Wire.endTransmission();
  Wire.requestFrom(BNO055_ADDRESS_A, (byte)1);
  value = Wire.receive();

  sprintf(msg, "%d,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f", millis(), accelerometer->getOrientation(0), accelerometer->getOrientation(1), accelerometer->getOrientation(2),
          accelerometer->getAcceleration(0), accelerometer->getAcceleration(1), accelerometer->getAcceleration(2),
          accelerometer->getGyro(0), accelerometer->getGyro(1), accelerometer->getGyro(2),
          altimeter->getPressure(), altimeter->getAltitude());

  Serial.println(msg);


}
