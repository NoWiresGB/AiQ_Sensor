/*
  Written by: James moverley
  Description: esp airQuality monitor using DHT11/SDS011/MQ135
	NB: optimised for hackbox power ninja
	
  libraries:
    pubsub        https://github.com/knolleary/pubsubclient
    DHTesp        https://github.com/beegee-tokyo/DHTesp
    SdsDustSensor https://github.com/lewapek/sds-dust-sensors-arduino-library

*/

#include <ESP8266WiFi.h> 
#include <PubSubClient.h>
#include <SdsDustSensor.h>
#include <ESP8266mDNS.h>
#include <DHTesp.h>

#include "settings.h"

// delay between readings (ms)
int sds_init_delay = 30000;

// wifi settings
// imported from settings.h

// MQTT settings
// set mqtt_server to IP to override mDNS autodiscovery 
String mqtt_server = "";
int mqtt_port;

String mqtt_baseId = "airQ_";
String mqtt_clientId = "";
String mqtt_topic = "";
String mqtt_base_topic = "sensors/airQ";
String msg = "";

// wifi setup
WiFiClient espClient;
PubSubClient client(espClient);

// led pins
int ledPin_green = 02;
int ledPin_blue = 02;

// dht11 pin
int dht11_pin = 14;
DHTesp dht;

// sds011 bits
int sds_rxPin = 13;
int sds_txPin = 15;
SdsDustSensor sds(sds_rxPin, sds_txPin);

//---- functions -------------------------------------------------------------------
//----------------------------------------------------------------------------------



//----------------------------------------------------------------------------------
// ## mqtt stuff
//----------------------------------------------------------------------------------

// mqtt sleep to include regular client.loop calls (keep things alive whilst waiting)
//----------------------------------------------------------------------------------
void mqtt_sleep(int sleep_length){
  long sleep_start = millis();
  while (millis() - sleep_start < sleep_length) {
         delay(250); 
         client.loop();
  }
}

void mqtt_reconnect() {
    Serial.print("[MQTT] Attempting MQTT connection: ");
    while (!client.connected()) {
      Serial.print(".");
      if (client.connect(mqtt_clientId.c_str())) {
        Serial.println(" OK");
        client.loop();
        // Once connected, publish an announcement...
        String connected_msg = mqtt_clientId + " joined";
        client.publish("clientJoin", connected_msg.c_str());
        client.loop();
        // ... and resubscribe - need to handle SUBACK!
        //client.subscribe("inTopic");
        //client.loop();
      } else {
        Serial.print(" FAIL, rc=");
        Serial.print(client.state());
        Serial.println(" retry in 5 seconds");
        // Wait 5 seconds before retrying
        mqtt_sleep(5000);
      }
    }
}

//----------------------------------------------------------------------------------
void mqtt_callback(char* topic, byte* payload, unsigned int length) {
  Serial.print("[MQTT] Message arrived [");
  Serial.print(topic);
  Serial.print("] ");
  for (int i = 0; i < length; i++) {
    Serial.print((char)payload[i]);
  }
  Serial.println();

  // Switch on the LED if an 1 was received as first character
  if ((char)payload[0] == '1') {
    digitalWrite(ledPin_green, LOW);   // Turn the LED on (Note that LOW is the voltage level
    // but actually the LED is on; this is because
    // it is acive low on the ESP-01)
  } else {
    digitalWrite(ledPin_green, HIGH);  // Turn the LED off by making the voltage HIGH
  }

}

//----------------------------------------------------------------------------------
void init_mqtt() {
  // mDNS to discover MQTT server

  if (mqtt_server.length() == 0 )
	if (!MDNS.begin("ESP")) {
		Serial.println("[mDNS] Error setting up mDNS");
	} else {
		Serial.println("[mDNS] Using Setup - Sending Query");
		int n = MDNS.queryService("mqtt", "tcp");
		if (n == 0) {
			Serial.println("[mDNS] No service found");
		} else {
			// at least one MQTT service is found
			// ip no and port of the first one is MDNS.IP(0) and MDNS.port(0)
			mqtt_server = MDNS.IP(0).toString();
			mqtt_port = MDNS.port(0);
			Serial.print("[mDNS] Service discovered: ");
		}
	}
	
  Serial.print("[MQTT] IUsing server :");
  Serial.print(mqtt_server);
  Serial.print(":");
  Serial.println(mqtt_port);
        
  // can only setup clientID and topic once WiFi is up
  mqtt_clientId = WiFi.macAddress();
  mqtt_topic = mqtt_base_topic + "/" + mqtt_clientId + "/";
  
  // mqtt setup (setup unqiue client id from mac
  Serial.print("(MQTT] clientId: ");
  Serial.println(mqtt_clientId);
  Serial.print("[MQTT] topic: ");
  Serial.println(mqtt_topic);
  client.setServer(mqtt_server.c_str(), mqtt_port);
  client.setCallback(mqtt_callback);
  //nb mqtt connection is handled later..
}




//----------------------------------------------------------------------------------
// ## wifi stuff
//----------------------------------------------------------------------------------

//----------------------------------------------------------------------------------
void init_wifi() {
  //##################################
  // connect to wifi
  Serial.println("[WIFI] initialising");
  Serial.print("[WIFI] MAC: ");
  Serial.println(WiFi.macAddress());
  Serial.print("[WIFI] SSID:");
  Serial.println(ssid);

  Serial.print("[WIFI] Attaching: ");
  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  Serial.println("");
  Serial.print("[WIFI] connected IP: ");
  Serial.println(WiFi.localIP());
  Serial.println("[WIFI] Setup complete");
}

//----------------------------------------------------------------------------------
// ## sds011 stuff
//----------------------------------------------------------------------------------

//Correction algorythm thanks to help of Zbyszek Kiliański (Krakow Zdroj)
// https://github.com/piotrkpaul/esp8266-sds011/blob/master/sds011_nodemcu/sds011_nodemcu.ino
//----------------------------------------------------------------------------------
float normalizePM25(float pm25, float humidity){
  return pm25/(1.0+0.48756*pow((humidity/100.0), 8.60068));
}

//----------------------------------------------------------------------------------
float normalizePM10(float pm10, float humidity){
  return pm10/(1.0+0.81559*pow((humidity/100.0), 5.83411));
}

//----------------------------------------------------------------------------------
float calculatePolutionPM25(float pm25){
  return pm25*100/25;
}
//----------------------------------------------------------------------------------
float calculatePolutionPM10(float pm10){
  return pm10*100/50;
}

//----------------------------------------------------------------------------------
// ## DHT sensor init
//----------------------------------------------------------------------------------
void init_dhtSensor(){
  Serial.println("[SENSOR] DHT: initiliasing");
  pinMode(dht11_pin, INPUT);
  dht.setup(dht11_pin, DHTesp::DHT11); //
  Serial.println("[SENSOR] DHT: Setup finished");
}

//----------------------------------------------------------------------------------
// ## LED stuff
//----------------------------------------------------------------------------------

// led init pins
//----------------------------------------------------------------------------------
void init_LEDs () {
  // setup status led(s)
  Serial.print("[LEDS] initialising");
  pinMode(ledPin_green, OUTPUT);
  digitalWrite(ledPin_green, HIGH);
  pinMode(ledPin_blue, OUTPUT);
  digitalWrite(ledPin_blue, HIGH);
  Serial.print("[LEDS] setup complete");
}

// led on and off functions
//----------------------------------------------------------------------------------
void ledOn() {
  digitalWrite(ledPin_blue, LOW);
} // endfunc

// led on and off functions
//----------------------------------------------------------------------------------
void ledOff() {
  digitalWrite(ledPin_blue, HIGH);
} // endfunc

// led blink function
//----------------------------------------------------------------------------------
void ledBlink(int duration_ms) {
  ledOn();
  mqtt_sleep(duration_ms);
  ledOff();
} // endfunc


//----------------------------------------------------------------------------------
// ## SETUP hook function
// # NB: for our power managed ESP node, we will remote power on/off so all activity 
//   during setup phase..
//----------------------------------------------------------------------------------
void setup() {

  // start serial
  Serial.begin(115200);
  
  //##################################
  Serial.println();
  Serial.println("================================"); 
  Serial.println("[SETUP] Starting");
  Serial.println("--------------------------------"); 

  // setup LEDS
  init_LEDs();
  
  // setup DHT11
  init_dhtSensor();
  
 
  // #1 fire up sds 1st
  
  // setup sds011 sensor
  Serial.println("[SENSOR] SDS: starting");
  sds.begin(); 
 
  // note millis, so we can calculate how long to delay till sds finish warm up so we can take reading
  long sdsStarted = millis();
 
  // start sds reading...
  sds.wakeup();
  
  // setup WIFI
  init_wifi();

  // mDNS MQTT setup
  init_mqtt();
 
  // now wait for SDS to finish warmup
  long sdsRemain = millis() - sdsStarted;
  mqtt_sleep(sdsRemain);
 
 // == make sds reading
  PmResult pm = sds.queryPm();
  if (pm.isOk()) {
    Serial.print("[SENSOR] SDS:{OK} ");
    Serial.print("PM2.5 = ");
    Serial.print(pm.pm25);
    Serial.print(", PM10 = ");
    Serial.println(pm.pm10);

    // if you want to just print the measured values, you can use toString() method as well
    //Serial.println(pm.toString());
  } else {
    Serial.print("[SENSOR] SDS:{ERROR} ");
    Serial.print("Could not read values from sensor, reason: ");
    Serial.println(pm.statusToString());
  }
  
  // == DHT reading - just after SDS reading to ensure improved accuracy.
  float humidity = dht.getHumidity();
  float temperature = dht.getTemperature();
  float heatIndex = dht.computeHeatIndex(temperature, humidity, false);
  
  Serial.print("[SENSOR] DHT: ");
  Serial.print("{" + String(dht.getStatusString()) + "}");
  Serial.print(" H:");
  Serial.print(humidity, 1);
  Serial.print(" T:");
  Serial.print(temperature, 1);
  Serial.print(" HI:");
  Serial.println(heatIndex, 1);
 

  // make adjusted pollution calculation of sds011 vs humidity
  float norm_p25 = normalizePM25(pm.pm25, humidity);
  float norm_p10 = normalizePM10(pm.pm10, humidity);
  float cpol_p25 = calculatePolutionPM25(norm_p25);
  float cpol_p10 = calculatePolutionPM10(norm_p10);
  Serial.println("[SENSOR] CONCL: PM2.5: " + String(norm_p25) + " (" + String(cpol_p25) + "% normy)");
  Serial.println("[SENSOR] CONCL: PM10: " + String(norm_p10) + " (" + String(cpol_p10) + "% normy)");
  
  // end of loop code below
  // -------------------------------------
  WorkingStateResult state = sds.sleep();
  if (state.isWorking()) {
    Serial.println("[SENSOR] ERROR: Problem with sleeping the sensor.");
  } else {
    Serial.println("[SENSOR] SDS: sleeping");
  }

  // build mqtt topic strings for push
  String mqtt_temp = mqtt_topic + "TEMP";
  String mqtt_humd = mqtt_topic + "HUMDITY";
  String mqtt_hind = mqtt_topic + "HEATIND";
  String mqtt_p25n = mqtt_topic + "PM25_N";
  String mqtt_p25c = mqtt_topic + "PM25_C";
  String mqtt_p10n = mqtt_topic + "PM10_N";
  String mqtt_p10c = mqtt_topic + "PM10_C";
  String mqtt_batt = mqtt_topic + "BATTERY";
  String mqtt_rssi = mqtt_topic + "RSSI";
  //String mqtt_pwr  = mqtt_topic + "PWR";
	
  // do mqtt publishes
  ledBlink(500);
 
  Serial.print("[MQTT] Checking connection: ");
  if (!client.loop()) {
      Serial.println("FAIL");
      mqtt_reconnect();
  } else {
      Serial.println("OK");
      Serial.print("[MQTT] making data publishes: "); 
  
      client.publish(mqtt_temp.c_str(), String(temperature,1).c_str());
      client.publish(mqtt_humd.c_str(), String(humidity,1).c_str());
      client.publish(mqtt_hind.c_str(), String(heatIndex,1).c_str());
      client.publish(mqtt_p25n.c_str(), String(norm_p25,2).c_str());
      client.publish(mqtt_p25c.c_str(), String(cpol_p25,2).c_str());
      client.publish(mqtt_p10n.c_str(), String(norm_p10,2).c_str());
      client.publish(mqtt_p10c.c_str(), String(cpol_p10,2).c_str());
      //client.publish(mqtt_pwr.c_str(), String(vcc_in,2).c_str() );
      Serial.println("done");
   }

  Serial.println("Disconnecting MQTT");
  client.disconnect(); 
  
  
  Serial.println("--------------------------------"); 
  Serial.println("[SETUP] FINISHED");
  Serial.println("================================"); 
}


//---- loop ------------------------------------------------------------------------
void loop() {
  
} // <========= loop end
