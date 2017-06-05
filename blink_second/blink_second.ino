#define BLYNK_DEBUG           // Comment this out to disable debug and save space
#define BLYNK_PRINT Serial    // Comment this out to disable prints and save space
#include <ESP8266WiFi.h>
#include <DNSServer.h>
#include <ESP8266WebServer.h>
#include <WiFiManager.h>          //https://github.com/tzapu/WiFiManager
#include <OneWire.h>
#include <DallasTemperature.h>

//for LED status
#include <Ticker.h>
Ticker ticker;

#include <ArduinoJson.h>          //https://github.com/bblanchon/ArduinoJson

char blynk_token[34] = "e15197dc67a14f879799e4c9ce20338cc";

bool shouldSaveConfig = false; //flag for saving data

#include <BlynkSimpleEsp8266.h>
#include <SimpleTimer.h>
SimpleTimer timer;

#define ONE_WIRE_BUS 14  // DS18B20 pin
#define POMP_POWERED_FEEDBACK 15
#define POMP_POWER_SWITCH 13
OneWire oneWire(ONE_WIRE_BUS);
DallasTemperature DS18B20(&oneWire);
BlynkTimer blynkTimer; // Create a Timer object called "timer"! 

void tickLed()
{
  //toggle state
  int state = digitalRead(BUILTIN_LED);  // get the current state of GPIO1 pin
  digitalWrite(BUILTIN_LED, !state);     // set pin to the opposite state
}

void saveConfigCallback () {  //callback notifying us of the need to save config
  Serial.println("Should save config");
  shouldSaveConfig = true;
  ticker.attach(0.2, tickLed);  // led toggle faster
}

void setup()
{
  delay(3000);
  Serial.begin(115200);
  Serial.println();

  //set led pin as output
  pinMode(BUILTIN_LED, OUTPUT);
  pinMode(POMP_POWER_SWITCH, OUTPUT);
  pinMode(POMP_POWERED_FEEDBACK, INPUT);
  
  // start ticker with 0.5 because we start in AP mode and try to connect
  ticker.attach(0.1, tickLed);


  Serial.println("Starting...");    //read configuration from FS json

  DS18B20.begin();

  // The extra parameters to be configured (can be either global or just in the setup)
  // After connecting, parameter.getValue() will get you the configured value
  // id/name placeholder/prompt default length
  WiFiManagerParameter custom_blynk_token("blynk", "blynk token", blynk_token, 33);   // was 32 length
  
  Serial.println(blynk_token);

  //WiFiManager
  //Local intialization. Once its business is done, there is no need to keep it around
  WiFiManager wifiManager;

  wifiManager.setSaveConfigCallback(saveConfigCallback);   //set config save notify callback

  //set static ip
  // this is for connecting to Office router not GargoyleTest but it can be changed in AP mode at 192.168.4.1
  //wifiManager.setSTAStaticIPConfig(IPAddress(192,168,10,111), IPAddress(192,168,10,90), IPAddress(255,255,255,0));
  
  wifiManager.addParameter(&custom_blynk_token);   //add all your parameters here

  //wifiManager.resetSettings();  //reset settings - for testing

  //set minimu quality of signal so it ignores AP's under that quality
  //defaults to 8%
  //wifiManager.setMinimumSignalQuality();
  
  //sets timeout until configuration portal gets turned off
  //useful to make it all retry or go to sleep, in seconds
  wifiManager.setTimeout(600);   // 10 minutes to enter data and then Wemos resets to try again.

  //fetches ssid and pass and tries to connect, if it does not connect it starts an access point with the specified name
  //and goes into a blocking loop awaiting configuration
  if (!wifiManager.autoConnect("ESPAutoConfig")) {
    Serial.println("Failed to connect and hit timeout");
    delay(3000);
    //reset and try again, or maybe put it to deep sleep
    ESP.reset();
    delay(5000);
  }
  Serial.println("Connected Smart Home System :)");   //if you get here you have connected to the WiFi
  ticker.detach();
  //turn LED off
  digitalWrite(BUILTIN_LED, HIGH);

  strcpy(blynk_token, custom_blynk_token.getValue());    //read updated parameters

  if (shouldSaveConfig) {      //save the custom parameters to FS
    Serial.println("saving config");
    DynamicJsonBuffer jsonBuffer;
    JsonObject& json = jsonBuffer.createObject();
    json["blynk_token"] = blynk_token;

    json.printTo(Serial);
    //json.printTo(configFile);
    //configFile.close();
    //end save
  }

  Serial.println("local ip");
  Serial.println(WiFi.localIP());
  
  Blynk.config(blynk_token);
  Blynk.connect();

  blynkTimer.setInterval(1000L, blynkPush);

}

double readTemperature() {
  float temp;
  DS18B20.requestTemperatures(); 
  temp = DS18B20.getTempCByIndex(0);
  Serial.print("Temperature: ");
  Serial.println(temp);
  return temp;  
}

// This function will run every time Blynk connection is established
BLYNK_CONNECTED() {
    // Request Blynk server to re-send latest values for all pins
    Blynk.syncAll();
}

//BLYNK_READ(V2) // Widget in the app READs Virtal Pin V5 with the certain frequency
void blynkPush()
{
  Blynk.virtualWrite(V2, readTemperature());
  Blynk.virtualWrite(V1, digitalRead(POMP_POWERED_FEEDBACK)*255);
  tickLed();
}

BLYNK_WRITE(V0) {
  int ledState = param.asInt();
  digitalWrite(POMP_POWER_SWITCH, ledState);
}

  void loop()
{
  Blynk.run(); // Initiates Blynk
  timer.run(); // Initiates SimpleTimer  
  blynkTimer.run();
}
