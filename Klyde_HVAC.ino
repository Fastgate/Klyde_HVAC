#include <SPI.h>
//#include <FlexCAN.h>
//#include <FlexCAN_T4.h>
#include <arduinoIO.h>


#include "Binary.h"
#include "Serial.h"
#include "Hvac.h"

//#include "Can.h"

  /////////////
 // HELPERS //
/////////////

#define min(X, Y)  ((X) < (Y) ? (X) : (Y))
#define max(X, Y)  ((X) > (Y) ? (X) : (Y))

//FlexCAN_T4<CAN1, RX_SIZE_256, TX_SIZE_16> can1;
//CAN_message_t canMessage;

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

//CanSniffer canSniffer;
//Obd2Helper obd2;



  //////////////////
 // SKETCH SETUP //
//////////////////

void setup() {

  
  
  Serial.begin(115200);
  
  //while(!Serial && millis()<4000){
   //}
  delay(2000);
  statusInitSuccess.serialize(Serial);

  //Can0.begin(500000);
  //can1.begin();
  //can1.setBaudRate(500000);


  SPI.begin();

  //Keyboard.begin();

}


  /////////////////
 // SKETCH LOOP //
/////////////////

void loop() {
  

  hvac.update();
  //updateCan();
  
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
       //case 0x0a: // start sniffer
          //canSniffer.toggle(true);
          //break;
        //case 0x0b: // stop sniffer
          //canSniffer.toggle(false);
          //break;
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

//void updateCan() { 
  //CAN_message_t canMessage;
 
  
    //can1.read(canMessage);
    //canSniffer.update(canMessage);
    

    
  
  
  //Serial.println(FLDoorSensor.getState());
//}
