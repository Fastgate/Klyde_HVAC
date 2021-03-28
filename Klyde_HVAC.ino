#include <SPI.h>
#include <FlexCAN.h>
#include <arduinoIO.h>


#include "Binary.h"
#include "Serial.h"
#include "Hvac.h"

#include "Can.h"

  /////////////
 // HELPERS //
/////////////

#define min(X, Y)  ((X) < (Y) ? (X) : (Y))
#define max(X, Y)  ((X) > (Y) ? (X) : (Y))


  /////////////////////
 // MMI DEFINITIONS //
/////////////////////

//SerialDataPacket<uint8_t> _MmiAndroidPacket(0x73, 0x72);

  ////////////////////////////
 // CAR MODULE DEFINITIONS //
////////////////////////////

Hvac hvac;



  ////////////////////////
 // SERIAL DEFINITIONS //
////////////////////////

SerialPacket statusInitSuccess(0x61, 0x01);
SerialPacket statusInitError(0x65, 0x01);
SerialDataPacket<unsigned long> baudRateChange(0x65, 0x01);

SerialReader serialReader(128);


  /////////////////////////
 // CAN BUS DEFINITIONS //
/////////////////////////

CanSniffer canSniffer;
Obd2Helper obd2;



  //////////////////
 // SKETCH SETUP //
//////////////////

void setup() {

  delay(1000);  
  Serial.begin(115200);
  delay(500);
  statusInitSuccess.serialize(Serial);

  Can0.begin(500000);


  SPI.begin();

}


  /////////////////
 // SKETCH LOOP //
/////////////////

void loop() {
  

  hvac.update();
  updateCan();
  
}


  //////////////////
 // SERIAL EVENT //
//////////////////

void serialEvent() {
  serialReader.read(Serial, readSerial);
}

void readSerial(uint8_t type, uint8_t id, BinaryBuffer *payloadBuffer) {
  switch (type) {
    case 0x61:
      switch (id) {
        case 0x0a: // start sniffer
          canSniffer.toggle(true);
          break;
        case 0x0b: // stop sniffer
          canSniffer.toggle(false);
          break;
        case 0x72: { // set baud rate
            BinaryData::LongResult result = payloadBuffer->readLong();
            if (result.state == BinaryData::OK) {
              baudRateChange.payload(htonl(result.data));
              baudRateChange.serialize(Serial);
              Serial.flush();
              Serial.end();
              Serial.begin(result.data);
              while (Serial.available()) {
                Serial.read();
              }
            }
            break;
          }
      }
      break;
    case 0x63:
      hvac.write(id, payloadBuffer);
      break;
  }
}


  ///////////////////
 // MMI FUNCTIONS //
///////////////////




  ///////////////////////
 // CAN BUS FUNCTIONS //
///////////////////////

void updateCan() { 
  CAN_message_t canMessage;
 
  while (Can0.available()) {
    Can0.read(canMessage);
    canSniffer.update(canMessage);
    

    
  }
  
  //Serial.println(FLDoorSensor.getState());
}
