
#define BLYNK_PRINT Serial


#include <ESP8266_Lib.h>
#include <BlynkSimpleEsp8266.h>
#include <Wire.h>
#include "MAX30100_PulseOximeter.h"


// You should get Auth Token in the Blynk App.
// Go to the Project Settings (nut icon).
char auth[] = "YourAuthToken";

// Your WiFi credentials.
// Set password to "" for open networks.
char ssid[] = "YourNetworkName";
char pass[] = "YourPassword";

// Hardware Serial on Mega, Leonardo, Micro...
#define EspSerial Serial3

// or Software Serial on Uno, Nano...
//#include <SoftwareSerial.h>
//SoftwareSerial EspSerial(2, 3); // RX, TX

// Your ESP8266 baud rate:
#define ESP8266_BAUD 115200

#define REPORTING_PERIOD_MS 1000

ESP8266 wifi(&EspSerial);
BlynkTimer timer;

void setup()
{
  // Debug console
  Serial.begin(9600);

  delay(10);

  // Set ESP8266 baud rate
  EspSerial.begin(ESP8266_BAUD);
  delay(10);

  Blynk.begin(auth, wifi, ssid, pass);

  if(!pox.begin()){
    Serial.println("Failed POX initialization");
    for(;;);
  }
else{
    Serial.println("POX initialization Success");
    pox.setOnBeatDetectedCallback(onBeatDetected);
}
timer.setInterval(1000L, sensorDataSend);
}

void loop()
{
  pox.update();
  Blynk.run();
  // if(millis() - tsLastReport > REPORTING_PERIOD_MS){
  //   bpm = pox.getHeartRate();
  //   spo2 = pox.getSpO2();
  //   Serial.print("H: ");
  //   Serial.println(bpm);
  //   Serial.print("SpO2: ");
  //   Serial.println(spo2);
  //   Blynk.virtualWrite(V7, bpm);
  //   Blynk.virtualWrite(v8, spo2);
  //   tsLastReport = millis();
  // }
}

void onBeatDetected(){
    Serial.println("Beat!");
}

void sensorDataSend(){
    bpm = pox.getHeartRate();
    spo2 = pox.getSpO2();
    Blynk.virtualWrite(V1, bpm);
    Blynk.virtualWrite(V2, spo2);
    Serial.print("H: ");
    Serial.println(bpm);
    Serial.print("S: ");
    Serial.println(spo2);
}
