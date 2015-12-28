// Remote node for RFM12B, with AM2302 and BMP180
// Board v.1.10
//
// ATMEL ATMEGA8 & 168 / ARDUINO
//
//                  +-\/-+
//            PC6  1|    |28  PC5 (AI 5)
//      (D 0) PD0  2|    |27  PC4 (AI 4)
//      (D 1) PD1  3|    |26  PC3 (AI 3)
//      (D 2) PD2  4|    |25  PC2 (AI 2)
// PWM+ (D 3) PD3  5|    |24  PC1 (AI 1)
//      (D 4) PD4  6|    |23  PC0 (AI 0)
//            VCC  7|    |22  GND
//            GND  8|    |21  AREF
//            PB6  9|    |20  AVCC
//            PB7 10|    |19  PB5 (D 13)
// PWM+ (D 5) PD5 11|    |18  PB4 (D 12)
// PWM+ (D 6) PD6 12|    |17  PB3 (D 11) PWM
//      (D 7) PD7 13|    |16  PB2 (D 10) PWM
//      (D 8) PB0 14|    |15  PB1 (D 9)  PWM
//                  +----+
#include <OneWire.h>
#include <RFM12B.h>
#include <avr/eeprom.h> // Global configuration for in chip EEPROM
#define VERSION 100

#include <RFM12B.h>
// You will need to initialize the radio by telling it what ID it has and what network it's on
// The NodeID takes values from 1-127, 0 is reserved for sending broadcast messages (send to all nodes)
// The Network ID takes values from 0-255
// By default the SPI-SS line used is D10 on Atmega328. You can change it by calling .SetCS(pin) where pin can be {8,9,10}
#define NODEID    2 //network ID used for this unit
#define GATEWAYID 1
#define NETWORKID 255 //the network ID we are on
#define FREQUENCY RF12_868MHZ
#define ACK_TIME  50
//encryption is OPTIONAL
//to enable encryption you will need to:
// - provide a 16-byte encryption KEY (same on all nodes that talk encrypted)
// - to call .Encrypt(KEY) to start encrypting
// - to stop encrypting call .Encrypt(NULL)
uint8_t KEY[] = "ABCDABCDABCDABCD";
// Need an instance of the Radio Module
RFM12B radio;

#include <SFE_BMP180.h>
#include <Wire.h>
SFE_BMP180 pressure;
#define ALTITUDE 240.0 

#include "DHT.h"
#define DHTPIN 5        // what pin we're connected to
DHT dht;

int8_t   i;
uint8_t  pos;
uint8_t  version = 0;
long     previousMillis = 0;
long     tempMillis = 0;
uint8_t  msg[13];
uint8_t  *msg_p;
float    temperature;
float    input;

struct config_t {
  uint16_t version;
  char     reg[21];
} conf; 

// Float conversion 
union u_tag {
    byte  b[4]; 
    float fval;
} u;

// wait a few milliseconds for proper ACK to me, return true if indeed received
static bool waitForAck(byte theNodeID) {
  long now = millis();
  while (millis() - now <= ACK_TIME) {
    if (radio.ACKReceived(theNodeID))
      return true;
  }
  return false;
}

void send_conf(){ // Registration
  pos = 0;
  do { 
    pos++; 
    radio.Send(GATEWAYID, conf.reg, sizeof(conf.reg), true);
  } while (!waitForAck(GATEWAYID) && pos < 10);
}

void setDefault(){
  conf.version = VERSION;
  conf.reg[0]  = 'R';       // Registration
  conf.reg[1]  = 'S';       // Sensor
  conf.reg[2]  = 'T';       // Temperature
  conf.reg[3]  = 0;         // Local address
  conf.reg[4]  = B00011110; // Default setting, group=16, disabled
  conf.reg[5]  = 'S';       // Sensor
  conf.reg[6]  = 'H';       // Humidity
  conf.reg[7]  = 0;         // Local address
  conf.reg[8]  = B00011110; // Default setting, group=16, disabled
  conf.reg[9]  = 'S';       // Sensor
  conf.reg[10] = 'T';       // Temperature
  conf.reg[11] = 1;         // Local address
  conf.reg[12] = B00011110; // Default setting, group=16, disabled
  conf.reg[13] = 'S';       // Sensor
  conf.reg[14] = 'P';       // Pressure
  conf.reg[15] = 1;         // Local address
  conf.reg[16] = B00011110; // Default setting, group=16, disabled
  conf.reg[17] = 'S';       // Sensor
  conf.reg[18] = 'I';       // Pressure
  conf.reg[19] = 2;         // Local address
  conf.reg[20] = B00011110; // Default setting, group=16, disabled
}

void setup() {
  radio.Initialize(NODEID, FREQUENCY, NETWORKID, 0);
  radio.Encrypt((byte*)KEY);
  
  eeprom_read_block((void*)&conf, (void*)0, sizeof(conf)); // Read current configuration
  if (conf.version != VERSION) setDefault();
  
  delay(200);
  send_conf(); 
  pressure.begin();
  dht.setup(DHTPIN);
 
  tempMillis = millis();
}

void loop() {
  // Look for incomming transmissions
  if (radio.ReceiveComplete() && radio.CRCPass()) {
    if ((char)radio.Data[0] == 'C') {
      if (radio.Data[1]==1) send_conf(); // Registration
    }
    if ((char)radio.Data[0]=='R') { // Registration
      // Replace part of conf string with new paramters.
      pos = 0;
      do { pos++; }
      while (((conf.reg[pos] != radio.Data[1]) ||
              (conf.reg[pos+1] != radio.Data[2]) ||
              (conf.reg[pos+2] != radio.Data[3])) && (pos < sizeof(conf)-3));
      if (pos < sizeof(conf)-3) {
        conf.reg[pos+3] = radio.Data[4];
        if (radio.ACKRequested()) radio.SendACK(); delay(200);
        // Save it to EEPROM
        conf.version = VERSION;
        eeprom_update_block((const void*)&conf, (void*)0, sizeof(conf)); // Save current configuration
        send_conf(); // Send it back for reregistration
      }
    }
    if ((char)radio.Data[0]=='I') { // Input
      u.b[0] = radio.Data[2]; u.b[1] = radio.Data[3]; u.b[2] = radio.Data[4]; u.b[3] = radio.Data[5];
      // check for our local address
      if (radio.Data[1] == 2 ) { input = u.fval; }
    }
    if (radio.ACKRequested()) radio.SendACK(); 
  }
   
  // Temperature readings
  if ((millis() - tempMillis) > 60000) {
    tempMillis = millis();
    msg_p = msg;   
    *msg_p++ = 'S';
    u.fval = input;
    *msg_p++ = 'I';
    *msg_p++ = 2;
    *msg_p++ = u.b[0]; *msg_p++ = u.b[1]; *msg_p++ = u.b[2]; *msg_p++ = u.b[3];
    radio.Send(GATEWAYID, msg, 7, false); delay(1000);

    msg_p = msg;   
    *msg_p++ = 'S';
    u.fval = dht.getTemperature();
    *msg_p++ = 'T';
    *msg_p++ = 0;
    *msg_p++ = u.b[0]; *msg_p++ = u.b[1]; *msg_p++ = u.b[2]; *msg_p++ = u.b[3];
    u.fval = dht.getHumidity();
    *msg_p++ = 'H';
    *msg_p++ = 0;
    *msg_p++ = u.b[0]; *msg_p++ = u.b[1]; *msg_p++ = u.b[2]; *msg_p++ = u.b[3];
    radio.Send(GATEWAYID, msg, sizeof(msg), false); delay(1000);
    
    msg_p = msg;   
    *msg_p++ = 'S';
    i = pressure.startTemperature();
    if (i != 0) {
      delay(i);
      i = pressure.getTemperature(temperature);
      if (i != 0) {
        u.fval = temperature;
        *msg_p++ = 'T';
        *msg_p++ = 1;
        *msg_p++ = u.b[0]; *msg_p++ = u.b[1]; *msg_p++ = u.b[2]; *msg_p++ = u.b[3];
        
        i = pressure.startPressure(3);
        if (i != 0) {
          delay(i);
          i = pressure.getPressure(u.fval, temperature);
          if (i != 0) {
            *msg_p++ = 'P';
            *msg_p++ = 1;
            *msg_p++ = u.b[0]; *msg_p++ = u.b[1]; *msg_p++ = u.b[2]; *msg_p++ = u.b[3];
            radio.Send(GATEWAYID, msg, sizeof(msg), false);
          }
        }
      }
    }
  }
  
}
