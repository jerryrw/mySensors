/**
 * The MySensors Arduino library handles the wireless radio link and protocol
 * between your home built sensors/actuators and HA controller of choice.
 * The sensors forms a self healing radio network with optional repeaters. Each
 * repeater and gateway builds a routing tables in EEPROM which keeps track of the
 * network topology allowing messages to be routed to nodes.
 *
 * Created by Henrik Ekblad <henrik.ekblad@mysensors.org>
 * Copyright (C) 2013-2015 Sensnology AB
 * Full contributor list: https://github.com/mysensors/Arduino/graphs/contributors
 *
 * Documentation: http://www.mysensors.org
 * Support Forum: http://forum.mysensors.org
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * version 2 as published by the Free Software Foundation.
 *
 *******************************
 *
 * REVISION HISTORY
 * Version 1.0: Henrik EKblad
 * Version 1.1 - 2016-07-20: Converted to MySensors v2.0 and added various improvements - Torben Woltjen (mozzbozz)
 * 2016-03-13 - Slightly altered to allow for updated Adafruit DHT libraries and for acting as a repeater - jerryrw
 * 
 * DESCRIPTION
 * This sketch provides an example of how to implement a humidity/temperature
 * sensor using a DHT11/DHT-22.
 *  
 * For more information, please visit:
 * http://www.mysensors.org/build/humidity
 * 
 */

#define MY_DEBUG  // Enable debug prints

#define MY_RADIO_NRF24  // Enable and select radio type attached 
#define MY_RF24_PA_LEVEL RF24_PA_MAX  //RF_PA_LOW
#define MY_RF24_CHANNEL 125
#define MY_RF24_DATARATE RF24_250KBPS //RF24_1MBPS or RF24_2MBPS 
#define MY_REPEATER_FEATURE

#include <SPI.h>
#include <MySensors.h>  
#include <DHT.h>

#define DHT_DATA_PIN 3  // Set this to the pin you connected the DHT's data pin to
#define DHTTYPE DHT11
#define SENSOR_TEMP_OFFSET 0  // Set this offset if the sensor has a permanent small offset to the real temperatures

// Sleep time between sensor updates (in milliseconds)
// Must be >1000ms for DHT22 and >2000ms for DHT11
// static const uint64_t UPDATE_INTERVAL = 2500;
// -- not usefull when operating as a repeater; use a different method to track time

// Force sending an update of the temperature after n sensor reads, so a controller showing the
// timestamp of the last update doesn't show something like 3 hours in the unlikely case, that
// the value didn't change since;
// i.e. the sensor would force sending an update every UPDATE_INTERVAL*FORCE_UPDATE_N_READS [ms]
static const uint8_t FORCE_UPDATE_N_READS = 10;

#define CHILD_ID_HUM 0
#define CHILD_ID_TEMP 1

float lastTemp;
float lastHum;
uint8_t nNoUpdatesTemp;
uint8_t nNoUpdatesHum;
bool metric = false;
unsigned long t0; //update interval counter
const unsigned long tUpdate = 30000;  //update interval 30 seconds

MyMessage msgHum(CHILD_ID_HUM, V_HUM);
MyMessage msgTemp(CHILD_ID_TEMP, V_TEMP);
DHT dht (DHT_DATA_PIN, DHTTYPE);

void presentation()  
{ 
  sendSketchInfo("TempAndHum", "1.0a");  // Send the sketch version information to the gateway
  // Register all sensors to gw (they will be created as child devices)
  present(CHILD_ID_HUM, S_HUM);
  present(CHILD_ID_TEMP, S_TEMP);
}

void setup()
{
  Serial.begin (115200);  // for debugging 
  dht.begin();  // start the dht library
  delay (2000); // give the sensor some time to stabilize
  t0 = millis();  // start the update timing
}

void loop()      
{  
  if ((millis()-t0)>tUpdate) { //send updates for temp/humidity every tUpdate interval
    // Get Temperature section
    float temperature = dht.readTemperature(true); // Get temperature from DHT library (true is for non metric)
    if (isnan(temperature)) {
      Serial.println("Failed reading temperature from DHT!");
    } else if (temperature != lastTemp || nNoUpdatesTemp == FORCE_UPDATE_N_READS) {  // Only send temperature if changed or every n times
      lastTemp = temperature;  // update our saved temp
      nNoUpdatesTemp = 0;  // reset no updates counter
      temperature += SENSOR_TEMP_OFFSET; // if it needs and offset
      send(msgTemp.set(temperature, 1)); // send our message to the gateway
      #ifdef MY_DEBUG Serial.print("T: "); Serial.println(temperature); 
      #endif
    } else {
      nNoUpdatesTemp++;  // Increase no update counter if the temperature stayed the same
    }
    // Get Humidity Section
    float humidity = dht.readHumidity();  // Get humidity from DHT library
    if (isnan(humidity)) {
      Serial.println("Failed reading humidity from DHT");
    } else if (humidity != lastHum || nNoUpdatesHum == FORCE_UPDATE_N_READS) {  // Only send humidity if it changed or every n times
      lastHum = humidity;  // update our saved humidity
      nNoUpdatesHum = 0;  // Reset no updates counter
      send(msgHum.set(humidity, 1));  // send our message to the gateway
      #ifdef MY_DEBUG Serial.print("H: "); Serial.println(humidity); 
      #endif
    } else {
      nNoUpdatesHum++;  // Increase no update counter if the humidity stayed the same
    }
    t0 = millis();  // reset the time interval counter
  }  //end if millis()-t0
  // cant sleep if we are also a repeater
  //sleep(UPDATE_INTERVAL); // Sleep for a while to save energy
}
