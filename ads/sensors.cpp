#include "sensors.h"
#define I2C 19

namespace nustars {
	Accelerometer::Accelerometer() {
        bno = Adafruit_BNO055(I2C); //I2C address, probably.
        orientation = new float[3];
        gyro = new float[3];
        acc = new float[3];
        if (!bno.begin()) {
            Serial.print(
                    "Ooops, no BNO055 detected ... Check your wiring or I2C ADDR!"); //TODO: Learn to throw an exception
            while (1);
        }
        bno.setExtCrystalUse(true);
    }

    void Accelerometer::tick() {
        imu::Vector<3> euler = bno.getVector(Adafruit_BNO055::VECTOR_EULER);
        orientation[0] = euler.x();
        orientation[1] = euler.y();
        orientation[2] = euler.z();
        euler = bno.getVector(Adafruit_BNO055::VECTOR_GYROSCOPE);
        gyro[0] = euler.x();
        gyro[1] = euler.y();
        gyro[2] = euler.z();
        euler = bno.getVector(Adafruit_BNO055::VECTOR_ACCELEROMETER);
        acc[0] = euler.x();
        acc[1] = euler.y();
        acc[2] = euler.z();
    }

    void Accelerometer::reconnect() {
      bno.begin();
    }

    float Accelerometer::getOrientation(int axis) {
        return orientation[axis];
    }

    float Accelerometer::getGyro(int axis) {
        return gyro[axis];
    }

    float Accelerometer::getAcceleration(int axis) {
        return acc[axis];
    }

	Altimeter::Altimeter() {
        if (!bme.begin()) {
            Serial.println("Could not find a valid BMP280 sensor, check wiring!");
            while (1);
        }
        setBaseAlt();
    }

    void Altimeter::setBaseAlt() {
        const int NUM_BASE_SAMPLES = 10;
        //Set altitude at ground
        baseAlt = 0;
        for (int i = 0; i < NUM_BASE_SAMPLES; i++) {
            baseAlt += bme.readAltitude(1000);
        }
        baseAlt = baseAlt / NUM_BASE_SAMPLES; //average readings
    }

    void Altimeter::tick() {
        bme.performReading();
        temp = bme.temperature;
        pressure = bme.pressure;
        alt = bme.readAltitude(SEALEVELPRESSURE_HPA);
    }

    float Altimeter::getTemp() {
        return temp;
    }

    float Altimeter::getAltitude() {
        return alt;
    }

    float Altimeter::getPressure() {
        return pressure;
    }

}




