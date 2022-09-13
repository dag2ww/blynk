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

#define MY_ROLE         ESP_NOW_ROLE_COMBO              // set the role of this device: CONTROLLER, SLAVE, COMBO
#define RECEIVER_ROLE   ESP_NOW_ROLE_COMBO              // set the role of the receiver
#define WIFI_CHANNEL    1

//#define MY_NAME         "GATE_NODE"
#define MY_NAME         "CWU_CTRL_NODE"

//D1 MINI CWU CTRL MAC:        {0xBC, 0xDD, 0xC2, 0x24, 0xBB, 0x47};  
//D1 MINI PRO GATE CO-MOD MAC: 60:01:94:1C:29:FD
//D1 MINI GATE MAC:            60:01:94:1D:4A:61

//uint8_t receiverAddress[] = {0x60, 0x01, 0x94, 0x1C, 0x29, 0xFD}; //D1 MINI PRO GATE CO-MOD  
//Just broadcast
uint8_t receiverAddress[] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF}; //D1 MINI PRO GATE CO-MOD 
bool cwu_praca_read = false;
bool cwu_praca_write = false;

struct __attribute__((packed)) sendDataPacket {
  bool cwu_pomp_is_running_feedback;
  float cwu_temperature;
};

struct __attribute__((packed)) recDataPacket {
  bool cwu_pomp_on;
};

void transmissionComplete(uint8_t *receiver_mac, uint8_t transmissionStatus) {
  if(transmissionStatus == 0) {
    Serial.println("Data sent successfully");
  } else {
    Serial.print("Error code: ");
    Serial.println(transmissionStatus);
  }
}

void dataReceived(uint8_t *senderMac, uint8_t *data, uint8_t dataLength) {
  char macStr[18];
  recDataPacket packet;  

  snprintf(macStr, sizeof(macStr), "%02x:%02x:%02x:%02x:%02x:%02x", senderMac[0], senderMac[1], senderMac[2], senderMac[3], senderMac[4], senderMac[5]);

  Serial.println();
  Serial.print("Received data from: ");
  Serial.println(macStr);
  
  memcpy(&packet, data, sizeof(packet));
  
  Serial.print("cwu_pomp_on: ");
  Serial.println(packet.cwu_pomp_on);
  cwu_praca_write = packet.cwu_pomp_on;
}
 
void setup() {
  Serial.begin(115200);     // initialize serial port

  Serial.println();
  Serial.println();
  Serial.println();
  Serial.println("MAC ADDRESS:");
  Serial.println(WiFi.macAddress());
  Serial.println();
    
  Serial.print(MY_NAME);
  Serial.println("...initializing...");

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

  Serial.println("Initialized.");
}

void loop() {
  sendDataPacket packet;
  //TODO read pomp pin and temp stats
  cwu_praca_read = cwu_praca_write;
  packet.cwu_pomp_is_running_feedback = cwu_praca_read;
  packet.cwu_temperature = 7.28;

  esp_now_send(receiverAddress, (uint8_t *) &packet, sizeof(packet));

  delay(1000);
}
