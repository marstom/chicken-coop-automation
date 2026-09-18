/*
Example usage:


selectSensor(SensorId::Intake);
float t1 = am2320.readTemperature();

selectSensor(SensorId::Room);
float t2 = am2320.readTemperature();

selectSensor(SensorId::Exhaust);
float t3 = am2320.readTemperature();

*/
#include <Arduino.h>
#include <Wire.h>
#include "my_am_2320_tca9548a.h"
#include "my_am2320.h"
namespace my_am2320
{

    void init_tca9548a(const int sdaPin, const int sclPin){
        init(sdaPin, sclPin);
    }

    void selectSensor(SensorId sensor)
    {
        uint8_t channel = static_cast<uint8_t>(sensor);

        Wire.beginTransmission(0x70);
        Wire.write(1 << channel);
        Wire.endTransmission();
    }

    float measure_temperature_from_sensor_x(SensorId sensor_id)
    {
        selectSensor(sensor_id);
        delay(10); // Wait for the sensor to be ready
        return measure_temperature();
    }

    float measure_humidity_from_sensor_x(SensorId sensor_id)
    {
        selectSensor(sensor_id);
        delay(10); // Wait for the sensor to be ready
        return measure_humidity();
    }

}
