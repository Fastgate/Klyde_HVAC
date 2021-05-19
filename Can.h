#ifndef CAN_H
#define CAN_H

#include <arduinoIO.h>
//#include <FlexCAN.h>
#include <carduinotest.h>
#include <FlexCAN_T4.h>

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
    void updateCan(const CAN_message_t &message) {
      if (message.id == this->canId) {
        this->state = message.len >= this->byteNumber + 1 && (message.buf[this->byteNumber] & this->bitMask) == this->bitMask;
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

class Can {
public:
    Can(Stream * serial) {
        this->sourceCan = new FlexCAN_T4<CAN2, RX_SIZE_256, TX_SIZE_16>();
        this->targetCan = new FlexCAN_T4<CAN1, RX_SIZE_256, TX_SIZE_16>();
        this->serial = serial;
    }
    ~Can() {
        //delete this->sourceCan;
        //delete this->targetCan;
        for (int i = 0; i < this->canIdCount; i++) {
            this->canIds[i] = 0;
        }
    }
    boolean setup(uint32_t sourceBaudRate, uint32_t targetBaudRate) {
        this->sourceCan->begin();
        this->sourceCan->setBaudRate(sourceBaudRate);
        this->targetCan->begin();
        this->targetCan->setBaudRate(targetBaudRate);
        
        this->isInitialized = true;
        return this->isInitialized;
    }


    void startSniffer() {
        this->isSniffing = true;
    }

    void stopSniffer() {
        this->isSniffing = false;
    }

    bool addCanId(uint32_t canId) {
        this->removeCanId(canId);
      
        if (this->canIdCount < 50) {
            this->canIds[this->canIdCount] = canId;
            this->canIdCount++;
            return true;
        }

        return false;
    }

    void removeCanId(uint32_t canId) {
        for (int index = 0; index < this->canIdCount; index++) {
            if (this->canIds[index] == canId) {
                for (int moveIndex = index; moveIndex < this->canIdCount - 1; moveIndex++) {
                    this->canIds[moveIndex] = this->canIds[moveIndex + 1];
                }
                this->canIdCount--;
            }
        }
    }

    void update(void (*canCallback)(const CAN_message_t &message)) {
        if (!this->isInitialized) {
            canNotInitializedError.serialize(this->serial, 1000);
            return;
        }
        if (this->sourceCan->read(this->msg)) {
          /*if (this->msg.id == 0x0551) 
          {
           //Serial.println(this->msg.buf[5], HEX);
          }*/
          
          if (this->isSniffing) {
            this->sniff(this->msg);
          }

          for (uint8_t i = 0; i < this->canIdCount; i++) {
            if (this->canIds[i] == this->msg.id) {
            //Serial.println(this->msg.id, HEX);
            this->write(this->msg);
            //return;
          }
          }

          canCallback(this->msg);
        }
    }

    template<uint8_t BYTE_INDEX, uint8_t BIT_MASK, uint8_t COMPARE_VALUE>
    static bool readFlag(uint8_t * data) {
        return (data[BYTE_INDEX] & BIT_MASK) == COMPARE_VALUE;
    }
    template<uint8_t BYTE_INDEX, uint8_t BIT_MASK>
    static inline bool readFlag(uint8_t * data) {
        return Can::readFlag<BYTE_INDEX, BIT_MASK, BIT_MASK>(data);
    }

    void write(const CAN_message_t &message) {
        if (!this->isInitialized) {
            canNotInitializedError.serialize(this->serial, 1000);
            return;
        }

        this->targetCan->write(message);
    }
private:
    FlexCAN_T4<CAN2, RX_SIZE_256, TX_SIZE_16> * sourceCan;
    FlexCAN_T4<CAN1, RX_SIZE_256, TX_SIZE_16> * targetCan;
    CAN_message_t msg;

    Stream * serial;
    uint32_t canIds[50];
    uint8_t canIdCount = 0;

    boolean isInitialized = false;                          
    boolean isSniffing = false;

    
    void sniff(const CAN_message_t &message) {
        uint32_t flippedCanId = htonl(message.id);
        this->serial->write("{");
        this->serial->write(0x62);
        this->serial->write(0x6d);
        this->serial->write(message.len + 0x04);
        this->serial->write((byte*)&flippedCanId, sizeof(message.id));
        for (uint8_t i = 0; i < message.len; i++) {
            this->serial->write(message.buf[i]);
        }
        this->serial->write("}");
    }
};

#endif
