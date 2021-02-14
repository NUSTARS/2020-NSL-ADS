#include <Wire.h>
#include <Servo.h>
#include <SPI.h>
#include <Adafruit_Sensor.h>
#include "Adafruit_BMP3XX.h"
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>

#define BMP_SCK 13
#define BMP_MISO 12
#define BMP_MOSI 11
#define BMP_CS 10

#define SENSOR_SAMPLERATE_DELAY_MS (50)

#define SEALEVELPRESSURE_HPA (1013.25)

#define apogee_dt (0.5)

Adafruit_BMP3XX bmp; // I2C

Adafruit_BNO055 bno = Adafruit_BNO055();

Servo myservo;

int iter = 0;
double ground = 0;
double init_acc = 0;
int average_number_over = 5;
int pos = 180;
const int init_pos = 180;
const int end_pos = 120;
const int angle_change = (init_pos - end_pos)/4;
bool launched = false;
bool descent = false;
static volatile int eint = 0;
int ki = 0;
int kp = 0;
int kd = 0;
double fin_area = 0;
double rocket_area = 0.0134;
double mass = 16.78;

double target_apogee = 1515;
double expected_apogee = 0;

volatile double filtered_acceleration = 0;
volatile double filtered_new_velocity = 0;
volatile double filtered_old_velocity = 0;
volatile double filtered_new_altitude = 0;
volatile double filtered_old_altitude = 0;
volatile double sensor_acceleration = 0;
volatile double sensor_altitude = 0;
const double sensor_dt = SENSOR_SAMPLERATE_DELAY_MS/1000;

const double wgt_alt = 0.7;
const double wgt_acc = 0.5;
const double wgt_vel = 0.7;

/*****************************************************************************************/
/*****************************************************************************************/
/*                                    helper functions                                   */
/*****************************************************************************************/
/*****************************************************************************************/

// BNO basic info
void displayBNODetails(void)
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

// BNO calibration status
void displayBNOCalStatus(void)
{
  /* Get the four calibration values (0..3) */
  /* Any sensor data reporting 0 should be ignored, */
  /* 3 means 'fully calibrated" */
  uint8_t system, gyro, accel, mag;
  system = gyro = accel = mag = 0;
  bno.getCalibration(&system, &gyro, &accel, &mag);

  /* The data should be ignored until the system calibration is > 0 */
  Serial.print("\t");
  if (!system)
  {
    Serial.print("! ");
  }

  /* Display the individual values */
  Serial.print("Sys:");
  Serial.print(system, DEC);
  Serial.print(" G:");
  Serial.print(gyro, DEC);
  Serial.print(" A:");
  Serial.print(accel, DEC);
  Serial.print(" M:");
  Serial.print(mag, DEC);
}

// basic info about BNO
void displayBNOStatus(void)
{
  /* Get the system status values (mostly for debugging purposes) */
  uint8_t system_status, self_test_results, system_error;
  system_status = self_test_results = system_error = 0;
  bno.getSystemStatus(&system_status, &self_test_results, &system_error);

  /* Display the results in the Serial Monitor */
  Serial.println("");
  Serial.print("System Status: 0x");
  Serial.println(system_status, HEX);
  Serial.print("Self Test:     0x");
  Serial.println(self_test_results, HEX);
  Serial.print("System Error:  0x");
  Serial.println(system_error, HEX);
  Serial.println("");
  delay(500);
}

void startTimer(void)
{
  Serial.println("Waiting 5 seconds until motor burnout");
  
  delay(4500);

  Serial.println("Motor burnout; initiating ADS feedback loop");
  launched = true;
}

double calculateApogee(double alt, double velocity, double fin_area, double rocket_area, double mass) {
  double apogee = alt;
  while (velocity > 0){
    apogee = apogee + velocity * apogee_dt;
    velocity = velocity - (9.8 + (1.225*velocity*velocity)/(2*mass)*(rocket_area*0.45 + fin_area*1.15))*apogee_dt;
  }
  return apogee;
}

double velocityFilter() {
  double new_vel = filtered_new_velocity + sensor_dt * (wgt_vel * filtered_acceleration) + sensor_dt * (1 - wgt_vel) * (filtered_new_altitude - filtered_old_altitude);
  return new_vel;
}

double altitudeFilter() {
  double new_alt = wgt_alt * (filtered_new_altitude + sensor_dt * filtered_new_velocity) + (1 - wgt_alt) * sensor_altitude;
  return new_alt;
}

double accelerationFilter() {
  double new_acc = wgt_acc*(filtered_acceleration + (filtered_new_velocity - filtered_old_velocity) * sensor_dt) + (1 - wgt_acc) * sensor_acceleration;
  return new_acc;
}

void finFeedback() {
  if (expected_apogee > target_apogee) {
    moveMotor(pos, pos - angle_change);
  } else if (expected_apogee < target_apogee) {
    moveMotor(pos + angle_change, pos);
  }
  fin_area = (init_pos - pos)/(init_pos - end_pos) * 0.0027;
}

void moveMotor(int from, int to) {
  if (from > to) {
    if (to < end_pos) {return;}
    for(int i = from; i >= to; i -= 1) // goes from from degrees to to degrees 
    {                                  // in steps of 1 degree 
      myservo.write(i);              // tell servo to go to position in variable 'i' 
      delay(30);                       // waits 15ms for the servo to reach the position 
    } 
  } else {
    if (from > init_pos) {return;}
    for(int i = to; i <= from; i += 1) // goes from to degrees to from degrees 
    {                                  // in steps of 1 degree 
      myservo.write(i);              // tell servo to go to position in variable 'i' 
      delay(30);                       // waits 15ms for the servo to reach the position 
    } 
  }
  pos = to;
}
/*****************************************************************************************/
/*****************************************************************************************/
/*                                         setup                                         */
/*****************************************************************************************/
/*****************************************************************************************/
void setup() {
  
  Serial.begin(9600);

  myservo.attach(9);
  myservo.write(pos);
  
  Serial.println("Data Collection for Orientation, Acceleration, and Altitude");
  Serial.println("");

  /* Initialise the sensors */
  
  // bno055 -----------------------------------------------------------------------
  if(!bno.begin())
  {
    Serial.print("Ooops, no BNO055 detected ... Check your wiring or I2C ADDR!");
    while(1);
  }

  displayBNODetails();
  displayBNOCalStatus();
  displayBNOStatus();

  bno.setExtCrystalUse(true);

  // bmp388 -----------------------------------------------------------------------
  if (!bmp.begin()) {
    Serial.println("Could not find a valid BMP3 sensor, check wiring!");
    while (1);
  }

  // Set up oversampling and filter initialization
  bmp.setTemperatureOversampling(BMP3_OVERSAMPLING_8X);
  bmp.setPressureOversampling(BMP3_OVERSAMPLING_4X);
  bmp.setIIRFilterCoeff(BMP3_IIR_FILTER_COEFF_3);
  Serial.println("Setup-------------------------------");

  bmp.readAltitude(SEALEVELPRESSURE_HPA);
  delay(3000);
  for (int i=0; i<average_number_over; i++) {
    ground += bmp.readAltitude(SEALEVELPRESSURE_HPA);
  }
  ground = ground/average_number_over;
  Serial.println("Ground altitude= ");
  Serial.print(ground);
  Serial.println("");

  delay(1000);
}

/*****************************************************************************************/
/*****************************************************************************************/
/*                                         main                                          */
/*****************************************************************************************/
/*****************************************************************************************/
void loop() {

  imu::Vector<3> euler = bno.getVector(Adafruit_BNO055::VECTOR_ACCELEROMETER);

  int bleh = 1;
  while(1) {
    init_acc += euler.z();
    bleh++;
    if (bleh == average_number_over) {
      init_acc = init_acc/average_number_over;
      break;
    }
  }

  if (! bmp.performReading()) {
    Serial.println("Failed to perform BMP reading :(");
    return;
  }

  Serial.print("Time elapsed: ");
  Serial.print(millis());
  Serial.print("\tIterations: ");
  Serial.print(iter);
  Serial.println("");
  Serial.println("-----------------------------------------------------------");

  // Acceleration
  Serial.print("Acceleration:");
  Serial.println("");
  Serial.print("X: ");
  Serial.print(euler.x());
  Serial.print("\tY: ");
  Serial.print(euler.y());
  Serial.print("\tZ: ");
  double vertacc = euler.z();//init_acc;
  vertacc -= init_acc;
  Serial.print(euler.z());
  Serial.println("");
  
  // Altitude
  Serial.print("Approx. Altitude: ");
  double alt = bmp.readAltitude(SEALEVELPRESSURE_HPA) - ground;
  Serial.print(alt);
  Serial.print(" m");
  Serial.println("\t");

  Serial.println("");
  Serial.println("");

  sensor_acceleration = vertacc;
  sensor_altitude = alt;

  if (sensor_altitude > 5 && launched == false) {
    startTimer();
  }
  if (filtered_old_altitude - filtered_new_altitude > 1 && descent == false) {
    moveMotor(pos, init_pos);
    descent = true;
  }

  filtered_old_velocity = filtered_new_velocity;
  filtered_new_velocity = velocityFilter();

  filtered_acceleration = accelerationFilter();
  filtered_old_altitude = filtered_new_altitude;
  filtered_new_altitude = altitudeFilter();

  expected_apogee = calculateApogee(filtered_new_altitude, filtered_new_velocity, fin_area, rocket_area, mass);
  finFeedback();

  Serial.print("Filtered Acceleration = ");
  Serial.println(filtered_acceleration);
  Serial.print("Filtered Altitude = ");
  Serial.println(filtered_new_altitude);
  Serial.print("Filtered velocity = ");
  Serial.println(filtered_new_velocity);
  Serial.print("Expected apogee = ");
  Serial.println(expected_apogee);
  Serial.print("Error (Expected - Target) = ");
  Serial.println(expected_apogee - target_apogee);
  Serial.print("Motor position at ");
  Serial.println(pos);
  
  iter++;
 
  delay(SENSOR_SAMPLERATE_DELAY_MS);
}
