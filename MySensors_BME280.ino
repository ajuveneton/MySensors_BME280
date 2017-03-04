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

#include <BME280_MOD-1022.h>
#include <Wire.h>

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

void loop() 
{
   // example of a forced sample.  After taking the measurement the chip goes back to sleep
  BME280.writeMode(smForced);
  while (BME280.isMeasuring())
  {    
  }
   
  // read out the data - must do this before calling the getxxxxx routines
  BME280.readMeasurements();
  Serial.print("Temp= ");
  Serial.println(BME280.getTemperature());  // must get temp first
  Serial.print("Humidity= ");
  Serial.println(BME280.getHumidity());
  Serial.print("Pressure= ");
  Serial.println(BME280.getPressure());
  Serial.println();
  
  delay(2000); 

}
