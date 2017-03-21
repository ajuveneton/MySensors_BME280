/*

Copyright (c) 2017, Arnaud Juveneton
All rights reserved.

Redistribution and use in source and binary forms, with or without
modification, are permitted provided that the following conditions are met:

- Redistributions of source code must retain the above copyright notice,
  this list of conditions and the following disclaimer.

- Redistributions in binary form must reproduce the above copyright
  notice, this list of conditions and the following disclaimer in the
  documentation and/or other materials provided with the distribution.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN 
CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) 
ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF 
THE POSSIBILITY OF SUCH DAMAGE.

This software is using library BME280_MOD-1022 written by Embedded Adventures:
Copyright (c) 2015, Embedded Adventures
All rights reserved.
www.embeddedadventures.com

*/


// BME MySensors weather multi-sensor
// Written originaly by Arnaud Juveneton

// Enable debug prints to serial monitor
#define MY_DEBUG 

// Enable and select radio type attached
#define MY_RADIO_NRF24
//#define MY_RADIO_RFM69

#include <MyConfig.h> // ???
#include <MySensors.h> // MySensors library
#include <SPI.h> // SPI library
#include <Wire.h> // I2C library

// BME280 libraries and variables
// Bosch BME280 Embedded Adventures MOD-1022 weather multi-sensor Arduino code
// Written originally by Embedded Adventures
// https://github.com/embeddedadventures/BME280
#include <BME280_MOD-1022.h> // BME280 sensor library

#define BARO_CHILD 0
#define TEMP_CHILD 1
#define HUM_CHILD 2

const float ALTITUDE = 290; // Altitude of the sensor. Needed to calculate barometric pressure at sea level

// Sleep time between reads (in ms)
const unsigned long SLEEP_TIME = 2000; 

float lastPressure = -1;
float lastTemp = -1;
float lastHum = -1;

boolean metric;

MyMessage tempMsg(TEMP_CHILD, V_TEMP);
MyMessage humMsg(HUM_CHILD, V_HUM);
MyMessage pressureMsg(BARO_CHILD, V_PRESSURE);

void setup() 
{
  Wire.begin();
  Serial.begin(115200);

  // need to read the compensation parameters
  BME280.readCompensationParams();
  
  // Need to turn on 1x oversampling, default is os_skipped, which means it doesn't measure anything
  BME280.writeOversamplingPressure(os1x);  // 1x over sampling (ie, just one sample)
  BME280.writeOversamplingTemperature(os1x);
  BME280.writeOversamplingHumidity(os1x);
}

void presentation()
{
  // Send the sketch version information to the gateway and Controller
  sendSketchInfo("BME280 Sensor", "0.1");

  // Register sensors to gw (they will be created as child devices)
  present(BARO_CHILD, S_BARO);
  present(TEMP_CHILD, S_TEMP);
  present(HUM_CHILD, S_HUM);

}

void loop() 
{
   // example of a forced sample.  After taking the measurement the chip goes back to sleep
  BME280.writeMode(smForced);
  while (BME280.isMeasuring())
  {    
  }
   
  // read out the data - must do this before calling the getxxxxx routines
  BME280.readMeasurements();

 float temperature = BME280.getTemperatureMostAccurate();                    // must get temp first
  float humidity = BME280.getHumidityMostAccurate();
  float pressure_local = BME280.getPressureMostAccurate();                    // Get pressure at current location
  float pressure = pressure_local/pow((1.0 - ( ALTITUDE / 44330.0 )), 5.255); // Adjust to sea level pressure using user altitude
      
  Serial.println();
  Serial.print("Temperature = ");
  Serial.print(temperature);
  Serial.println(metric ? " °C" : " °F");
  Serial.print("Humidity = ");
  Serial.print(humidity);
  Serial.println(" %");
  Serial.print("Pressure = ");
  Serial.print(pressure);
  Serial.println(" hPa"); 

  if (temperature != lastTemp) 
  {
    send(tempMsg.set(temperature, 1));
    lastTemp = temperature;
  }


  if (humidity != lastHum) 
  {
    send(humMsg.set(humidity, 1));
    lastHum = humidity;
  }

  if (pressure != lastPressure) 
  {
    send(pressureMsg.set(pressure, 2));
    lastPressure = pressure;
  }
    
  delay(SLEEP_TIME); // Sleep time until next measurement

}
