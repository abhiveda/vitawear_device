/*
Arduino-MAX30100 oximetry / heart rate integrated sensor library
Copyright (C) 2016  OXullo Intersecans <x@brainrapers.org>

This program is free software: you can redistribute it and/or modify
it under the terms of the GNU General Public License as published by
the Free Software Foundation, either version 3 of the License, or
(at your option) any later version.

This program is distributed in the hope that it will be useful,
but WITHOUT ANY WARRANTY; without even the implied warranty of
MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
GNU General Public License for more details.

You should have received a copy of the GNU General Public License
along with this program.  If not, see <http://www.gnu.org/licenses/>.
*/

// This example must be used in conjunction with the Processing sketch located
// in extras/rolling_graph

// WIFI
#include "WiFiEsp.h"
// Emulate Serial3 on pins 6/7 if not present
#ifndef HAVE_HWSERIAL1
#include "SoftwareSerial.h"
SoftwareSerial Serial3(6, 7); // RX, TX
#endif
char ssid[] = "ANIRUDH";            // your network SSID (name)
char pass[] = "9538417298";        // your network password
int status = WL_IDLE_STATUS;     // the Wifi radio's status
//int tdata ;
char server[] = "covidhackathon.tk";
String postUrl="/testdevice.php?";

float tempC=10.0;
float heartrate=99999.0;
float lastheartrate=99999.0;
float lastspo2 = 99999.0;
float spo2=99999.0;

WiFiEspClient client;

// VARIABLES NEEDED FOR MYLTITHREADING
unsigned long currentMillis = 0;
const int timeInterval = 1000; // number of millisecs between reports
const int poxInterval = 1000; //Time throughout which pox gets updated.
byte intervalState = LOW; 
byte lastintervalState = LOW;
unsigned long previousIntervalMillis = 0;
unsigned long previousPoxMillis = 0;

#include <Wire.h>
#include "MAX30100_PulseOximeter.h"

#define REPORTING_PERIOD_MS     5000

// PulseOximeter is the higher level interface to the sensor
// it offers:
//  * beat detection reporting
//  * heart rate calculation
//  * SpO2 (oxidation level) calculation
PulseOximeter pox;

uint32_t tsLastReport = 0;

// Callback (registered below) fired when a pulse is detected
void onBeatDetected()
{
    Serial.println("B:1");
}

void setup()
{
    Serial.begin(115200);

    Serial3.begin(115200);
  // initialize ESP module
  WiFi.init(&Serial3);

    // Initialize the PulseOximeter instance and register a beat-detected callback
    // The parameter passed to the begin() method changes the samples flow that
    // the library spews to the serial.
    // Options:
    //  * PULSEOXIMETER_DEBUGGINGMODE_PULSEDETECT : filtered samples and beat detection threshold
    //  * PULSEOXIMETER_DEBUGGINGMODE_RAW_VALUES : sampled values coming from the sensor, with no processing
    //  * PULSEOXIMETER_DEBUGGINGMODE_AC_VALUES : sampled values after the DC removal filter

    // Initialize the PulseOximeter instance
    // Failures are generally due to an improper I2C wiring, missing power supply
    // or wrong target chip
    if (!pox.begin()) {
        Serial.println("ERROR: Failed to initialize pulse oximeter");
        for(;;);
    }
    
    pox.setIRLedCurrent(MAX30100_LED_CURR_7_6MA);
    pox.setOnBeatDetectedCallback(onBeatDetected);

     if (WiFi.status() == WL_NO_SHIELD) {
    Serial.println("WiFi shield not present");
    // don't continue
    while (true);
  }

  // if (WiFi.status() == WL_CONNECTED){
  //   WiFi.stop();
  // }

  // attempt to connect to WiFi network
  while ( status != WL_CONNECTED) {
    Serial.print("Attempting to connect to WPA SSID: ");
    Serial.println(ssid);
    // Connect to WPA/WPA2 network
    status = WiFi.begin(ssid, pass);
  }

  Serial.println("You're connected to the network");
  // Scheduler.startLoop(poxLoop);

}

void loop()
{   

    currentMillis = millis();
    // Make sure to call update as fast as possible
    // pox.update();
    update_pox();
    update_intervalState();
    update_server();
    // Asynchronously dump heart rate and oxidation levels to the serial
    // For both, a value of 0 means "invalid"
    
}

// void connect_wifi(){
//      if (WiFi.status() == WL_NO_SHIELD) {
//     Serial.println("WiFi shield not present");
//     // don't continue
//     while (true);
//   }

//   // if (WiFi.status() == WL_CONNECTED){
//   //   WiFi.stop();
//   // }

//   // attempt to connect to WiFi network
//   while ( status != WL_CONNECTED) {
//     Serial.print("Attempting to connect to WPA SSID: ");
//     Serial.println(ssid);
//     // Connect to WPA/WPA2 network
//     status = WiFi.begin(ssid, pass);
//   }

//   Serial.println("You're connected to the network");
//   // Scheduler.startLoop(poxLoop);
// }



  // void send_dataToServer(){

  //   while (client.available()){
  //       char c = client.read();
  //       Serial.write(c);
  //   }


  //   Serial.println();
  //   client.stop();
  //   Serial.println("Connecting to server..");

  //   if(client.connect(server,80)){
  //       Serial.println("Connected to server");
  //       String sendMessage = "GET "+postUrl+"a="+tempC+"&b="+heartrate+"&c="+spo2+" HTTP/1.1";
  //       Serial.println(sendMessage);
  //       client.println(sendMessage);
  //       client.println("Host: covidhackathon.tk");
  //       client.println("Connection: close");
  //       client.println();
  //       tsLastReport = millis();
  //   }
  //   else {
  //       Serial.println("Connection failed");
  //   }

  // }

void update_pox(){
    int count = 0;
    previousPoxMillis = millis();
    while(millis() - previousPoxMillis <= poxInterval or heartrate==0 or spo2==0){
    pox.update();
    heartrate = pox.getHeartRate();
    spo2 = pox.getSpO2();
    print(count);
    print("H: ");
    print(heartrate);
    print("  S: ");
    println(spo2);
    count++;
    }
    previousPoxMillis += poxInterval;
}

void update_intervalState(){
    if (intervalState == LOW) {
          // if the state is off, we must wait for the interval to expire before turning it on
    if (currentMillis - previousIntervalMillis >= timeInterval) {
          // time is up, so change the state to HIGH
       intervalState = HIGH;
          // and save the time when we made the change
       previousIntervalMillis += timeInterval;
          // NOTE: The previous line could alternatively be
          //              previousIntervalMillislis = currentMillis
          //        which is the style used in the BlinkWithoutDelay example sketch
          //        Adding on the interval is a better way to ensure that succesive periods are identical

    }
  }
  else {  // i.e. if intervalState is HIGH
  
          // if the Led is on, we must wait for the duration to expire before turning it off
    if (currentMillis - previousIntervalMillis >= timeInterval) {
          // time is up, so change the state to LOW
       intervalState = LOW;
          // and save the time when we made the change
       previousIntervalMillis += timeInterval;
    } 
  }
}

void update_server(){
    if(intervalState!=lastintervalState){
        Serial.print("Heart rate:");
        Serial.print(heartrate);
        
        Serial.print("bpm / SpO2:");
        Serial.print(spo2);

        Serial.println("%");

        client.stop();
    Serial.println("Connecting to server..");

    if(client.connect(server,80)){
        Serial.println("Connected to server");
        String sendMessage = "GET "+postUrl+"a="+tempC+"&b="+heartrate+"&c="+spo2+" HTTP/1.1";
        Serial.println(sendMessage);
        client.println(sendMessage);
        client.println("Host: covidhackathon.tk");
        client.println("Connection: close");
        client.println();
        heartrate = 0.0;
        spo2 = 0.0;
        }
    else {
        Serial.println("Connection failed");
        }
    }
}