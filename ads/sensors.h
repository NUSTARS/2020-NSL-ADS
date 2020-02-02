#ifndef _DEF_BNO
#define _DEF_BNO
#include <Adafruit_BNO055.h>
#endif
#include <Adafruit_BMP3XX.h>
#include <vector>
#define SEALEVELPRESSURE_HPA (1013.25)

namespace nustars {
    static const int X_AXIS = 0;
    static const int Y_AXIS = 1;
    static const int Z_AXIS = 2;

    class Sensor {
        virtual void tick();
    };

    /**
     * Adafruit BNO, the accelerometer
     */
    class Accelerometer: public Sensor {
    private:
        Adafruit_BNO055 bno;
        float* orientation;
        float* acc;
        float* gyro;
    public:
        Accelerometer();
        void tick() override;
        void reconnect();
        float getOrientation(int axis);
        float getAcceleration(int axis);
        float getGyro(int axis);
        std::vector<float> getVals();
    };

    /**
     * Adafruit BME, the altimeter
     */
    class Altimeter: public Sensor {
    private:
        Adafruit_BMP3XX bme;
        float temp, pressure, alt, baseAlt;
        void setBaseAlt();
    public:
        Altimeter();
        void tick() override;
        float getTemp();
        float getPressure();
        float getAltitude();
    };

}