#define BLYNK_TEMPLATE_ID "TMPLyiiIu_zF"
#define BLYNK_DEVICE_NAME "Klimat Info"
#define BLYNK_FIRMWARE_VERSION        "0.3.4"
#define BLYNK_PRINT Serial
//#define BLYNK_DEBUG
#define APP_DEBUG


// Uncomment your board, or configure a custom board in Settings.h
#define USE_WEMOS_D1_MINI

#include "BlynkEdgent.h"

#include <ArduinoJson.h>          //https://github.com/bblanchon/ArduinoJson

#include <DHT.h>
#include <Adafruit_Sensor.h>

#include <Wire.h>
#include <Adafruit_BMP085.h>      //http://www.instructables.com/id/Adding-the-BMP180-to-the-ESP8266/

#include <CommandParser.h>
typedef CommandParser<16, 4, 30, 32, 64> MyCommandParser;

#define DHTPIN D3

#define DHTTYPE DHT22   // DHT 22  (AM2302)

DHT dht(DHTPIN, DHTTYPE);
Adafruit_BMP085 bmp;

MyCommandParser parser;

boolean bmpPresent = false;
boolean noDhtReported = false;
boolean macAddrReported = false;
int dhtReadErrorCount = 0;
boolean pracaCWUSwitch = false;
boolean pracaCWUFeedback = false;
double temperaturaCWU = 0.0;

//WidgetLED led1(V7);

int ticksToRestart = 60;

BlynkTimer blynkTimer; // Create a Timer object called "timer"! 


void setup() {
  delay(3000);
  // put your setup code here, to run once:
  Serial.begin(115200);
  Serial.println();
  Serial.println("Starting...");    //read configuration from FS json

  Serial.println("MAC ADDRESS:");
  Serial.println(WiFi.macAddress());

  parser.registerCommand("|CMD|--CWU_RUN_STATE", "i", &cmd_cwu_on_feedback_process);
  parser.registerCommand("|CMD|--CWU_TEMPERATURE", "d", &cmd_cwu_temp_process);
    
  
  BlynkEdgent.begin();
  
  Blynk.logEvent("started_info");
  //Blynk.logEvent("started_info", String("MAC:")+WiFi.macAddress());
  dht.begin();

  //SDA, SCL
  Wire.pins(D1, D2);
  Wire.begin(D1, D2);
  if (!bmp.begin()) {
    Serial.println("No BMP180 / BMP085 found");
    Blynk.logEvent("no_bmp_sensor");
  } else {
    bmpPresent = true;
  }

  blynkTimer.setInterval(2500L, blynkPush);
}

//// This function will run every time Blynk connection is established
void BlynkOnConnected() {
    ticksToRestart = 60;
    // Request Blynk server to re-send latest values for all pins
    Blynk.syncAll();
}

void blynkPush()
{
  if(!Blynk.connected()){
    --ticksToRestart;
  }
  
  if(ticksToRestart <=0){
    ESP.restart();
  }
  
  float h = dht.readHumidity();
  // Read temperature as Celsius
  float t = dht.readTemperature();
  
  if (isnan(h) || isnan(t)) {
    Serial.println("Failed to read from DHT sensor!");
    if(!noDhtReported) {
      Blynk.logEvent("no_dht_sensor");
      noDhtReported = true;
    }
    
    dhtReadErrorCount += 1;
    if(dhtReadErrorCount >= 10) {
      Blynk.virtualWrite(V1, -127);
      Blynk.virtualWrite(V2, -127);
      dhtReadErrorCount = 0;     
    }    
  } else {
    Blynk.virtualWrite(V1, t);
    Blynk.virtualWrite(V2, h); 
    double dp = dewPoint(t,h);
    Blynk.virtualWrite(V4, dp); 
    dhtReadErrorCount = 0;  
    Serial.println("Temp: "+String(t)+" Humidity: "+String(h)+".");
  }

  if(bmpPresent) {
    float p = bmp.readPressure()/100;
    Serial.println("Pressure: " + String(p));
    Blynk.virtualWrite(V3, p);
  } 

  Blynk.virtualWrite(V5, temperaturaCWU);
  Blynk.virtualWrite(V7, pracaCWUFeedback);

  Serial.println(String("|CMD|--CWU_ON ")+pracaCWUSwitch);

}

double dewPoint(double celsius, double humidity)
{
  // (1) Saturation Vapor Pressure = ESGG(T)
  double RATIO = 373.15 / (273.15 + celsius);
  double RHS = -7.90298 * (RATIO - 1);
  RHS += 5.02808 * log10(RATIO);
  RHS += -1.3816e-7 * (pow(10, (11.344 * (1 - 1/RATIO ))) - 1) ;
  RHS += 8.1328e-3 * (pow(10, (-3.49149 * (RATIO - 1))) - 1) ;
  RHS += log10(1013.246);

        // factor -3 is to adjust units - Vapor Pressure SVP * humidity
  double VP = pow(10, RHS - 3) * humidity;

        // (2) DEWPOINT = F(Vapor Pressure)
  double T = log(VP/0.61078);   // temp var
  return (241.88 * T) / (17.558 - T);
}
  
void loop()
{
  blynkTimer.run();
  BlynkEdgent.run();
  
  if (Serial.available()) {
    char line[128];
    size_t lineLength = Serial.readBytesUntil('\n', line, 127);
    if(lineLength > 0){
      line[lineLength-1] = '\0'; //to get rid of \r sent before \n
    } else {
      line[lineLength] = '\0';
    }
    if(line[0] == '|'){
      char response[MyCommandParser::MAX_RESPONSE_SIZE];
      Serial.println(String("Got and will process:")+line);
      parser.processCommand(line, response);
      Serial.println(response);
    }
  }
  
}

void cmd_cwu_on_feedback_process(MyCommandParser::Argument *args, char *response) {
  //Serial.println(String("setting pracaCWUFeedback to: ")+args[0].asInt64);
  pracaCWUFeedback = args[0].asInt64;
  strlcpy(response, "success", MyCommandParser::MAX_RESPONSE_SIZE);
}

void cmd_cwu_temp_process(MyCommandParser::Argument *args, char *response) {
  //Serial.println(String("setting temperaturaCWU to: ")+args[0].asDouble);
  temperaturaCWU = args[0].asDouble;
  strlcpy(response, "success", MyCommandParser::MAX_RESPONSE_SIZE);
}

BLYNK_WRITE(V6)
{
    pracaCWUSwitch = param.asInt();
}
