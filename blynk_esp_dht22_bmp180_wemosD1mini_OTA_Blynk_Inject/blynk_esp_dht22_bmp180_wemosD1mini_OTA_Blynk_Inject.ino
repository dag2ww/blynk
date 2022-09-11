#define BLYNK_TEMPLATE_ID "TMPLyiiIu_zF"
#define BLYNK_DEVICE_NAME "Klimat Info"
#define BLYNK_FIRMWARE_VERSION        "0.2.5"
#define BLYNK_PRINT Serial
//#define BLYNK_DEBUG
#define APP_DEBUG


// Uncomment your board, or configure a custom board in Settings.h
//#define USE_SPARKFUN_BLYNK_BOARD
//#define USE_NODE_MCU_BOARD
//#define USE_WITTY_CLOUD_BOARD
#define USE_WEMOS_D1_MINI

#include "BlynkEdgent.h"

//######### ESP NOW COMMS ############
//#include<ESP8266WiFi.h>
#include<espnow.h>

#define MY_ROLE         ESP_NOW_ROLE_COMBO              // set the role of this device: CONTROLLER, SLAVE, COMBO
#define RECEIVER_ROLE   ESP_NOW_ROLE_COMBO              // set the role of the receiver
#define WIFI_CHANNEL    11

#define MY_NAME         "GATE_NODE" //BC:DD:C2:24:BB:47

uint8_t receiverAddress[] = {0xBC, 0xDD, 0xC2, 0x24, 0xBB, 0x47};   // SENSOR MAC
//broadcast
//uint8_t receiverAddress[] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};   // SENSOR MAC

struct __attribute__((packed)) dataPacket {
  int sensor1;
  int sensor2;
  float sensor3;
};

void transmissionComplete(uint8_t *receiver_mac, uint8_t transmissionStatus) {
  if(transmissionStatus == 0) {
    Serial.println("ESP-NOW Data sent successfully");
  } else {
    Serial.print("ESP-NOW Error code: ");
    Serial.println(transmissionStatus);
  }
}

void dataReceived(uint8_t *senderMac, uint8_t *data, uint8_t dataLength) {
  char macStr[18];
  dataPacket packet;  

  snprintf(macStr, sizeof(macStr), "%02x:%02x:%02x:%02x:%02x:%02x", senderMac[0], senderMac[1], senderMac[2], senderMac[3], senderMac[4], senderMac[5]);

  Serial.println();
  Serial.print("ESP-NOW Received data from: ");
  Serial.println(macStr);
  
  memcpy(&packet, data, sizeof(packet));
  
  Serial.print("sensor1: ");
  Serial.println(packet.sensor1);
  Serial.print("sensor2: ");
  Serial.println(packet.sensor2);
  Serial.print("sensor3: ");
  Serial.println(packet.sensor3);
}

//######### ESP NOW COMMS END ############

//for LED status
//#include <Ticker.h>

#include <ArduinoJson.h>          //https://github.com/bblanchon/ArduinoJson

#include <DHT.h>
#include <Adafruit_Sensor.h>

#include <Wire.h>
#include <Adafruit_BMP085.h>      //http://www.instructables.com/id/Adding-the-BMP180-to-the-ESP8266/


#define DHTPIN D3

//Ticker ticker;

// Uncomment whatever type you're using!
//#define DHTTYPE DHT11   // DHT 11 
#define DHTTYPE DHT22   // DHT 22  (AM2302)
//#define DHTTYPE DHT21   // DHT 21 (AM2301)

// Connect pin 1 (on the left) of the sensor to +5V
// NOTE: If using a board with 3.3V logic like an Arduino Due connect pin 1
// to 3.3V instead of 5V!
// Connect pin 2 of the sensor to whatever your DHTPIN is
// Connect pin 4 (on the right) of the sensor to GROUND
// Connect a 10K resistor from pin 2 (data) to pin 1 (power) of the sensor

// Initialize DHT sensor for normal 16mhz Arduino
//DHT dht(DHTPIN, DHTTYPE);
// NOTE: For working with a faster chip, like an Arduino Due or Teensy, you
// might need to increase the threshold for cycle counts considered a 1 or 0.
// You can do this by passing a 3rd parameter for this threshold.  It's a bit
// of fiddling to find the right value, but in general the faster the CPU the
// higher the value.  The default for a 16mhz AVR is a value of 6.  For an
// Arduino Due that runs at 84mhz a value of 30 works.
// Example to initialize DHT sensor for Arduino Due:
DHT dht(DHTPIN, DHTTYPE);
Adafruit_BMP085 bmp;

boolean bmpPresent = false;
boolean noDhtReported = false;
boolean macAddrReported = false;
int dhtReadErrorCount = 0;
boolean pracaCWUSwitch = false;
boolean pracaCWUFeedback = false;
double temperaturaCWU = 0.0;

BlynkTimer timer;

int ticksToRestart = 60;

BlynkTimer blynkTimer; // Create a Timer object called "timer"! 

void tickLed()
{
  //toggle state
  int state = digitalRead(BUILTIN_LED);  // get the current state of GPIO1 pin
  digitalWrite(BUILTIN_LED, !state);     // set pin to the opposite state

  dataPacket packet;
  
  packet.sensor1 = 789;
  packet.sensor2 = 1011;
  packet.sensor3 = 6.28;

  esp_now_send(receiverAddress, (uint8_t *) &packet, sizeof(packet));
  
}


void setup() {
  delay(3000);
  // put your setup code here, to run once:
  Serial.begin(115200);
  Serial.println();
  Serial.println("Starting...");    //read configuration from FS json
  
  //WiFi.mode(WIFI_STA);
  
  BlynkEdgent.begin();

  Serial.println("local ip");
  Serial.println(WiFi.localIP());
  Serial.println("MAC ADDRESS:");
  Serial.println(WiFi.macAddress());
  Blynk.logEvent("started_info", String("MAC:")+WiFi.macAddress());
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

  timer.setInterval(1000L, tickLed);

  if(esp_now_init() != 0) {
    Serial.println("ESP-NOW initialization failed");
    return;
  }

  esp_now_set_self_role(MY_ROLE);   
  esp_now_register_send_cb(transmissionComplete);         // this function will get called once all data is sent
  esp_now_register_recv_cb(dataReceived);               // this function will get called whenever we receive data
  esp_now_add_peer(receiverAddress, RECEIVER_ROLE, WIFI_CHANNEL, NULL, 0);

  Serial.println("ESP-NOW Initialized.");

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

  if(!macAddrReported){
    Blynk.logEvent("started_info");
    Blynk.logEvent("started_info", String("MAC:")+WiFi.macAddress());
    macAddrReported = true;
  }
  
  //t = ((int) (t * 10) / 10.0);
  //h = ((int) (h * 10) / 10.0);
  if (isnan(h) || isnan(t)) {
    Serial.println("Failed to read from DHT sensor!");
    if(!noDhtReported) {
      Blynk.logEvent("no_dht_sensor"+String("MAC:")+WiFi.macAddress());
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
  //Blynk.run(); // Initiates Blynk
  blynkTimer.run();
  timer.run(); // Initiates SimpleTimer  
  BlynkEdgent.run();
}

BLYNK_WRITE(V5)
{
    Blynk.virtualWrite(V5, temperaturaCWU);
}
BLYNK_WRITE(V6)
{
    pracaCWUSwitch = param.asInt();
}
BLYNK_WRITE(V7)
{
    Blynk.virtualWrite(V7, pracaCWUFeedback);
}
