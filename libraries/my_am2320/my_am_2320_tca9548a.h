/*
Example usage:


selectSensor(SensorId::Intake);
float t1 = am2320.readTemperature();

selectSensor(SensorId::Room);
float t2 = am2320.readTemperature();

selectSensor(SensorId::Exhaust);
float t3 = am2320.readTemperature();

*/
#pragma once
#include <Arduino.h>
namespace my_am2320
{
    enum class SensorId : uint8_t
    {
        Intake = 0,
        Exhaust = 1,
        Room = 2,
        Outside = 3
    };

    void init_tca9548a(const int sdaPin, const int sclPin);
    void selectSensor(SensorId sensor);
    float measure_temperature_from_sensor_x(SensorId sensor_id);
    float measure_humidity_from_sensor_x(SensorId sensor_id);

}
