/****************************************************************************************************************************************************
 *  TITLE: ESP-NOW Getting Started Examples
 *
 *  By Frenoy Osburn
 *  YouTube Video: https://youtu.be/_cNAsTB5JpM
 ****************************************************************************************************************************************************/

 /********************************************************************************************************************
  * Please make sure that you install the board support package for the ESP8266 boards.
  * You will need to add the following URL to your Arduino preferences.
  * Boards Manager URL: http://arduino.esp8266.com/stable/package_esp8266com_index.json
 ********************************************************************************************************************/
 
 /********************************************************************************************************************
 *  Board Settings:
 *  Board: "WeMos D1 R1 or Mini"
 *  Upload Speed: "921600"
 *  CPU Frequency: "80MHz"
 *  Flash Size: "4MB (FS:@MB OTA:~1019KB)"
 *  Debug Port: "Disabled"
 *  Debug Level: "None"
 *  VTables: "Flash"
 *  IwIP Variant: "v2 Lower Memory"
 *  Exception: "Legacy (new can return nullptr)"
 *  Erase Flash: "Only Sketch"
 *  SSL Support: "All SSL ciphers (most compatible)"
 *  COM Port: Depends *On Your System*
 *********************************************************************************************************************/
#include<ESP8266WiFi.h>
#include<espnow.h>
#include <CommandParser.h>

#define MY_ROLE         ESP_NOW_ROLE_COMBO              // set the role of this device: CONTROLLER, SLAVE, COMBO
#define RECEIVER_ROLE   ESP_NOW_ROLE_COMBO              // set the role of the receiver
#define WIFI_CHANNEL    1

#define MY_NAME         "ESP_GATE_CO_NODE"

//see https://github.com/Uberi/Arduino-CommandParser
typedef CommandParser<16, 4, 30, 32, 64> MyCommandParser;
bool cwu_pom_on = false;
MyCommandParser parser;

//D1 MINI CWU CTRL MAC:        {0xBC, 0xDD, 0xC2, 0x24, 0xBB, 0x47};  
//D1 MINI PRO GATE CO-MOD MAC: 60:01:94:1C:29:FD
//D1 MINI GATE MAC:            60:01:94:1D:4A:61

//uint8_t receiverAddress[] = {0xBC, 0xDD, 0xC2, 0x24, 0xBB, 0x47}; //D1 MINI CWU CTRL 
//Just broadcast
uint8_t receiverAddress[] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF}; //D1 MINI CWU CTRL 

struct __attribute__((packed)) recDataPacket {
  bool cwu_pomp_is_running_feedback;
  float cwu_temperature;
};

struct __attribute__((packed)) sendDataPacket {
  bool cwu_pomp_on;
};

void transmissionComplete(uint8_t *receiver_mac, uint8_t transmissionStatus) {
  if(transmissionStatus == 0) {
    Serial.println("Data to SENSOR(s) sent successfully");
  } else {
    Serial.println(String("Sending data to SENSOR(s) failed with error code: ")+transmissionStatus);
  }
}

void dataReceived(uint8_t *senderMac, uint8_t *data, uint8_t dataLength) {
  char macStr[18];
  recDataPacket packet;  

  snprintf(macStr, sizeof(macStr), "%02x:%02x:%02x:%02x:%02x:%02x", senderMac[0], senderMac[1], senderMac[2], senderMac[3], senderMac[4], senderMac[5]);

  Serial.println();
  Serial.println(String("Received data from: ")+macStr);
  
  memcpy(&packet, data, sizeof(packet));

  Serial.println(String("cwu_pomp_is_running_feedback: ")+packet.cwu_pomp_is_running_feedback);
  Serial.println(String("cwu_temperature: ")+packet.cwu_temperature);


  Serial.println(String("|CMD|--CWU_RUN_STATE ")+packet.cwu_pomp_is_running_feedback);
  Serial.println(String("|CMD|--CWU_TEMPERATURE ")+packet.cwu_temperature);

}
 
void setup() {
  Serial.begin(115200);     // initialize serial port

  Serial.println();
  Serial.println("MAC ADDRESS:");
  Serial.println(WiFi.macAddress());
  Serial.println();

  parser.registerCommand("|CMD|--CWU_ON", "i", &cmd_cwu_on_process);
    
  Serial.println(String(MY_NAME)+"...initializing...");

  WiFi.mode(WIFI_STA);
  WiFi.disconnect();        // we do not want to connect to a WiFi network

  if(esp_now_init() != 0) {
    Serial.println("ESP-NOW initialization failed");
    return;
  }

  esp_now_set_self_role(MY_ROLE);   
  esp_now_register_send_cb(transmissionComplete);         // this function will get called once all data is sent
  esp_now_register_recv_cb(dataReceived);               // this function will get called whenever we receive data
  esp_now_add_peer(receiverAddress, RECEIVER_ROLE, WIFI_CHANNEL, NULL, 0);

  Serial.println("ESP-NOW comms Initialized.");
}

void loop() {
  sendDataPacket packet;  

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

  packet.cwu_pomp_on = cwu_pom_on;

  esp_now_send(receiverAddress, (uint8_t *) &packet, sizeof(packet));

  delay(1000);
}

void cmd_cwu_on_process(MyCommandParser::Argument *args, char *response) {
  Serial.println(String("setting cwu_pomp_on to: ")+args[0].asInt64);
  cwu_pom_on = args[0].asInt64;
  strlcpy(response, "success", MyCommandParser::MAX_RESPONSE_SIZE);
}
