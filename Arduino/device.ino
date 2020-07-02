// HEADER FILES
// #include <Scheduler.h>

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

WiFiEspClient client;

// OXIMETER
#include <Wire.h>
#include "MAX30100_PulseOximeter.h"
#define REPORTING_PERIOD_MS     5000
 
PulseOximeter pox;
uint32_t tsLastReport = 0;
float heartrate=0.0;
float spo2=0.0;

// TEMP SENSOR
#include <OneWire.h>
#include <DallasTemperature.h>
// Data wire is plugged into port 2 on the Arduino
#define ONE_WIRE_BUS 2

// Setup a oneWire instance to communicate with any OneWire devices (not just Maxim/Dallas temperature ICs)
OneWire oneWire(ONE_WIRE_BUS);

// Pass our oneWire reference to Dallas Temperature. 
DallasTemperature sensors(&oneWire);
float tempC = 0.0;


void setup()
{
  // initialize serial for debugging
  Serial.begin(115200);

  // TEMP
  sensors.begin();

  // OXIMETER
  if (!pox.begin()) {
        Serial.println("FAILED");
        for(;;);
    } else {
        Serial.println("SUCCESS");
    }
     pox.setIRLedCurrent(MAX30100_LED_CURR_7_6MA);
 
    // Register a callback for the beat detection
    pox.setOnBeatDetectedCallback(onBeatDetected);


    // WIFI
    Serial3.begin(115200);
  // initialize ESP module
  WiFi.init(&Serial3);

  // check for the presence of the shield
  if (WiFi.status() == WL_NO_SHIELD) {
    Serial.println("WiFi shield not present");
    // don't continue
    while (true);
  }

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

  void loop{
  	if(millis() - tsLastReport > REPORTING_PERIOD_MS) {
  		pox_getdata();
  		temp_getdata();
  		send_dataToServer();
  	}
  }

  void pox_getdata(){
  	Serial.print("Requesting data from Oximeter...");
  	pox.update();
  	delay(0);
  	heartrate = pox.getHeartRate();
  	spo2 = pox.getSpO2();
  	Serial.println("heartrate = ");
  	Serial.println(heartrate);  	
  	Serial.println("SpO2 = ");
  	Serial.println(spo2);
  }

  // void poxLoop(){
  // 	pox.update();
  // 	heartrate = pox.getHeartRate();
  // 	spo2 = pox.getSpO2();
  // 	Serial.println("heartrate = ");
  // 	Serial.println(heartrate);
  // 	Serial.println("SpO2 = ");
  // 	Serial.println(spo2);
  // 	yield();
  // }

  void temp_getdata(){
  	Serial.print("Requesting temperatures...");
  	sensors.requestTemperatures(); // Send the command to get temperatures
  	Serial.println("DONE");
  	tempC = sensors.getTempCByIndex(0);
  	if(tempC != DEVICE_DISCONNECTED_C) 
	  {
	    Serial.print("Temperature for the device 1 (index 0) is: ");
	    Serial.println(tempC);
	  } 
	else
	  {
	    Serial.println("Error: Could not read temperature data");
	  }
  }

  void send_dataToServer(){

  	while (client.available()){
  		char c = client.read();
  		Serial.write(c);
  	}


  	Serial.println();
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
	    tsLastReport = millis();
  	}
  	else {
  		Serial.println("Connection failed");
  	}

  }


  void onBeatDetected()
{
    Serial.println("Beat!");
}