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
#include "Adafruit_Sensor.h"
#include "Adafruit_AM2320.h"

// Adafruit_AM2320 am2320 = Adafruit_AM2320(&Wire, -1, -1);
// Adafruit_AM2320 am2320_1 = Adafruit_AM2320(&Wire, -1, -1);
// Adafruit_AM2320 am2320_2 = Adafruit_AM2320(&Wire, -1, -1);

namespace my_am2320
{

    uint8_t muxAddr = 0x70;
    Adafruit_AM2320 list_of_sensors[4];

    // TODO

    void init_wire(const int sdaPin, const int sclPin)
    {
        Wire.end(); 

        // delay(1000);
        Wire.begin(sdaPin, sclPin, 500000);
        delay(1000);
    }
    void init_tca9548a_sensor(SensorId sensor_id)
    {
        uint8_t sensor_id_num = static_cast<uint8_t>(sensor_id);
        selectSensor(sensor_id);
        list_of_sensors[sensor_id_num] = Adafruit_AM2320(&Wire, -1, -1);
        if (!list_of_sensors[sensor_id_num].begin())
        {
            Serial.println("AM2320 not found");
        }
    }

    void selectSensor(SensorId sensor)
    {
        uint8_t channel = static_cast<uint8_t>(sensor);
        Wire.beginTransmission(muxAddr);
        Wire.write(1 << channel);
        // delay(250); // Wait for the sensor to be ready
        Wire.endTransmission();
    }

    float measure_temperature_from_sensor_x(SensorId sensor_id)
    {
        uint8_t sensor_id_num = static_cast<uint8_t>(sensor_id);
        selectSensor(sensor_id);
        delay(1000); // Wait for the sensor to be ready
        // TODO need dynamically select that sensor
        return list_of_sensors[sensor_id_num].readTemperature();
    }

    float measure_humidity_from_sensor_x(SensorId sensor_id)
    {
        uint8_t sensor_id_num = static_cast<uint8_t>(sensor_id);
        selectSensor(sensor_id);
        delay(1000); // Wait for the sensor to be ready
        return list_of_sensors[sensor_id_num].readHumidity();
    }

}
