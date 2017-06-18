
//The AVR hardware clears the global interrupt flag in SREG before entering an interrupt vector. Thus, normally interrupts will remain disabled inside the handler until the handler exits, where the RETI instruction (that is emitted by the compiler as part of the normal function epilogue for an interrupt handler) will eventually re-enable further interrupts. For that reason, interrupt handlers normally do not nest. For most interrupt handlers, this is the desired behaviour, for some it is even required in order to prevent infinitely recursive interrupts (like UART interrupts, or level-triggered external interrupts). In rare circumstances though it might be desired to re-enable the global interrupt flag as early as possible in the interrupt handler, in order to not defer any other interrupt more than absolutely needed. This could be done using an sei() instruction right at the beginning of the interrupt handler, but this still leaves few instructions inside the compiler-generated function prologue to run with global interrupts disabled.

/**
 *
 * Pins:
 * Hardware SPI:
 * MISO -> 12
 * MOSI -> 11
 * SCK -> 13
 *
 * Configurable:
 * CE -> 10
 * CSN -> 9
 *
 */


#include <SPI.h>
#include <Keypad.h>
#include "RF24.h"
#include "nRF24L01.h"

#define ON_OFF_CONTEROLLER
#define Password_Lenght 7 // Give enough room for six chars + NULL char
//PINS SETUP
// from SPI.h
// MOSI: pin 11
// MISO: pin 12
// SCK: pin 13       
#define TONE_PIN 7

#define TEMP_PACKET             3
#define TEMP_PACKET_POWER_BOX   10
#define POWER_ON_PACKET         11
#define ANSW_NONE 0
#define NO_MORE_ITEMS 255
#define REQUEST_ON_TRIGGER 198
#define REQUEST_ON_TRIGGER_INPUT 199
#define REQUEST_ON_TRIGGER_FINAL 200

//to test if our receiver is working and getting sth
unsigned long lastAnyRadioResponse = 0;

unsigned long previousMillisLed1=0;
unsigned long previousMillisLed2=0;
unsigned long previousMillisTemp=0;
unsigned long previousMillisTXRX=0;

//button handling
int lastButtonState;
unsigned long last_send_interval=0;
unsigned long lastButtonDebounceTime;
int lastButtonStateReading;
unsigned long requestOnTriggerToken;
boolean waitForConversion = false;
boolean receivedRequest = false;
float temp;
byte msgType = 0;
// temperature read related:
byte ti;
byte tdata[12];
byte taddr[8];
// the follow variables is a long because the time, measured in miliseconds,
// will quickly become a bigger number than can be stored in an int.
unsigned long intervalLED1 = 1000;           // interval at which to blink (milliseconds)
unsigned long intervalLED2 = 1000;           // interval at which to blink (milliseconds)
unsigned long intervalBeep = 6000;
unsigned long previousMillisCircuit_1;
unsigned long intervalCircuit_1;
unsigned long previousMillisCircuit_2;
unsigned long intervalCircuit_2;
int answerType = ANSW_NONE;

// Set up nRF24L01 radio on SPI bus plus pins ce = 10  & csn = 9
RF24 radio(10,9);

// Radio pipe addresses for the 2 nodes to communicate.
//Tx,Rx
const uint64_t InfoToSensorStationPipes[2] = { 0xF0F0F0F0E5LL, 0xF0F0F0F0D6LL};
const uint64_t SensorToInfoStationPipes[3] = { 0xF0F0F0F0D6LL, 0xF0F0F0F0E5LL};

const uint64_t InfoToPowerStationPipes[2] = { 0xF0F0F0F0C7LL, 0xF0F0F0F0B8LL};
const uint64_t PowerToInfoStationPipes[3] = { 0xF0F0F0F0B8LL, 0xF0F0F0F0C7LL};



char Data[Password_Lenght]; // 6 is the number of chars it can hold + the null char = 7
char Master[Password_Lenght] = "123456"; 
byte data_count = 0, master_count = 0;
bool Pass_is_good;
char customKey;

const byte ROWS = 4;
const byte COLS = 3;
char keys[ROWS][COLS] = {
  {'1','2','3'},
  {'4','5','6'},
  {'7','8','9'},
  {'*','0','#'}
};

byte rowPins[ROWS] = {
  2,3,4,5}; //connect to the row pinouts of the keypad
byte colPins[COLS] = {
  10,9,8}; //connect to the column pinouts of the keypad

Keypad keypad( makeKeymap(keys), rowPins, colPins, ROWS, COLS); //initialize an instance of class NewKeypad 



void setup() {

  pinMode(TONE_PIN, OUTPUT);
  keypad.addEventListener(keypadEvent);
   
  //
  // Setup and configure rf radio
  //
  radio.begin();
  // optionally, increase the delay between retries & # of retries
  radio.setRetries(15,15);
  //setDataRate()  RF24_2MBPS, RF24_250KBPS
  radio.setDataRate(RF24_250KBPS);
  // radio.setPALevel( RF24_PA_MAX );
  // optionally, reduce the payload size.  seems to
  // improve reliability
  radio.setPayloadSize(8);
  radio.setChannel(85);  
  
  radio.openWritingPipe(InfoToPowerStationPipes[0]);
  radio.openReadingPipe(1,InfoToPowerStationPipes[1]);
  
  radio.startListening();
  //radio.printDetails();
}

void loop() {
  handleSendReceiveRF24();
  handleUserButton();
  //if no message from radio since 1 minute - allow reset
  //if(millis() - lastAnyRadioResponse < 60000){
  //  wdt_reset();
  //}  
}

 void handleSendReceiveRF24(){
  byte data[radio.getPayloadSize()];

  //RECEIVE 
  // If a rf package is available - process it
  if(radio.available()){

     boolean readingDone = false;  
     
     while(!readingDone){
       readingDone = radio.read( &data, radio.getPayloadSize() );
       msgType = data[0];
       if( msgType == REQUEST_ON_TRIGGER_INPUT) {
         //requestOnTriggerToken = (unsigned long)(data[4] << 24) | (data[3] << 16) | (data[2] << 8) | data[1];
         requestOnTriggerToken = 0;
         requestOnTriggerToken = data[4];
         requestOnTriggerToken = (requestOnTriggerToken <<8) | data[3];
         requestOnTriggerToken = (requestOnTriggerToken <<8) | data[2];
         requestOnTriggerToken = (requestOnTriggerToken <<8) | data[1];
         receivedRequest = true;  
     }
     }

     lastAnyRadioResponse = millis();
  } 
  
  //SEND
  
  if ( receivedRequest ) {
    if(msgType == REQUEST_ON_TRIGGER_INPUT){
      msgType = ANSW_NONE;
      data[0] = REQUEST_ON_TRIGGER_FINAL;
      requestOnTriggerToken = (requestOnTriggerToken >> 3) + 15;
        data[1] = requestOnTriggerToken & 0xFF;
                 data[2] = (requestOnTriggerToken >> 8) & 0xFF;
         data[3] = (requestOnTriggerToken >> 16) & 0xFF;
         data[4] = (requestOnTriggerToken >> 24) & 0xFF;
    } else {
    data[0] = REQUEST_ON_TRIGGER;
    }
    receivedRequest = false;
    // Delay just a little bit to let the other unit
    // make the transition to receiver
    delay(40);

    //do the transmission
    //printf("Now sending: %d...",data[0]);
    // First, stop listening so we can talk.
    radio.stopListening();

    bool ok = radio.write( data, radio.getPayloadSize() );
    // Now, continue listening
    radio.startListening();
    
    //if (ok)
    //  printf("ACK");
    //else
    //  printf("NO_ACK.\n\r");
    //if(ledsMode == LEDS_MODE_TXRX){
    //   digitalWrite(LED2_PIN, LOW);     
    // }
    delay(10);
  }
}

void handleUserButton(){
  receivedRequest = false;

  customKey = keypad.getKey();
  
  if (customKey) // makes sure a key is actually pressed, equal to (customKey != NO_KEY)
  {
    Data[data_count] = customKey; // store char into data array
    data_count++; // increment data array by 1 to store new char, also keep track of the number of chars entered
  }

}

// Taking care of some special events.
void keypadEvent(KeypadEvent key){
    tone(TONE_PIN, 1000, 100); //pin, freq, duration
    switch (keypad.getState()){
    case PRESSED:
        if (key == '#') {
          if(data_count == Password_Lenght-1) // if the array index is equal to the number of expected chars, compare data to master
          {
            if(!strcmp(Data, Master)) {// equal to (strcmp(Data, Master) == 0)
              receivedRequest = true;
            }
          }
        }
        break;

    case RELEASED:
        if (key == '*') {
            clearData();
        }
        break;

    case HOLD:
        if (key == '*') {
           
        }
        break;
    }
}


void clearData()
{
  while(data_count !=0)
  {   // This can be used for any array size, 
    Data[data_count--] = 0; //clear array for new data
  }
  return;
}



