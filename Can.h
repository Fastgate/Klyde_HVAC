#ifndef CAN_H
#define CAN_H

#include <arduinoIO.h>
#include <FlexCAN.h>
#include "Serial.h"

struct CanData {
  union {
    unsigned char data[4];
    BitFieldMember<0, 32> canId;
  } metaData;
  uint8_t rxBuf[8] = { 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 };
};

class CanInput : public Input {
  public:
    CanInput(uint32_t canId, uint8_t byteNumber, uint8_t bitMask) {
      this->canId       = canId;
      this->byteNumber  = byteNumber;
      this->bitMask     = bitMask;
    }
    virtual boolean getState() { 
      return this->state;
    }
    void update(CAN_message_t canMessage) {
      if (canMessage.id == this->canId) {
        this->state = canMessage.len >= this->byteNumber + 1 && (canMessage.buf[this->byteNumber] & this->bitMask) == this->bitMask;
      }
    }
  private:
    uint32_t canId      = 0;
    uint8_t byteNumber  = 0;
    uint8_t bitMask     = B00000000;
    
    boolean state       = false;
};

class Obd2Helper {
  public:
  void sendRequest(uint8_t mode, uint8_t pid) {
     CAN_message_t requestMessage;
     requestMessage.id = 0x7DF;
     requestMessage.buf[0] = 2;
     requestMessage.buf[1] = mode;
     requestMessage.buf[2] = pid;
     Can0.write(requestMessage);
  }
};

class CanSniffer {
  public:
    CanSniffer() {
      this->canSnifferPacket = new SerialDataPacket<CanData>(0x62, 0x6d);
    }
    void toggle(boolean state) {
      this->isActive = state;
    }
    void update(CAN_message_t canMessage) {
      if (!this->isActive) {
        return; 
      }

      this->canSnifferPacket->payload()->metaData.canId = canMessage.id;
      for (uint8_t i = 0; i < 8; i++) {
        uint8_t data = 0x00;
        if (i < canMessage.len) {
           data = canMessage.buf[i];
        }
        this->canSnifferPacket->payload()->rxBuf[i] = data;
      }
      this->canSnifferPacket->serialize(Serial);
    }
  private:
    SerialDataPacket<CanData> *canSnifferPacket;
    boolean isActive = false;
};

#endif
