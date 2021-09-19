#include <SPI.h>
#include <FlexCAN.h>
#include <arduinoIO.h>
#include <arduinoMmi.h>

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

SerialDataPacket<uint8_t> _MmiAndroidPacket(0x73, 0x72);


// Buttons
Mmi mmi(&Serial3, SERIAL_8N2_RXINV, 16, 17, 2);
MmiButton *mmiBigWheelButton    = mmi.createButton(0x01);
MmiButton *mmiMediaButton       = mmi.createButton(0x02);
MmiButton *mmiNameButton        = mmi.createButton(0x03);
MmiButton *mmiTelButton         = mmi.createButton(0x04);
MmiButton *mmiNavButton         = mmi.createButton(0x05);
MmiButton *mmiInfoButton        = mmi.createButton(0x06);
MmiButton *mmiCarButton         = mmi.createButton(0x07);
MmiButton *mmiSetupButton       = mmi.createButton(0x08);
MmiButton *mmiTopLeftButton     = mmi.createButton(0x0A);
MmiButton *mmiBottomLeftButton  = mmi.createButton(0x0B);
MmiButton *mmiPreviousButton    = mmi.createButton(0x0C);
MmiButton *mmiTopRightButton    = mmi.createButton(0x0D);
MmiButton *mmiBottomRightButton = mmi.createButton(0x0E);
MmiButton *mmiReturnButton      = mmi.createButton(0x0F);
MmiButton *mmiNextButton        = mmi.createButton(0x10);
MmiButton *mmiRadioButton       = mmi.createButton(0x18);
MmiButton *mmiSmallWheelButton  = mmi.createButton(0x38);

// Wheels
MmiWheel *mmiSmallWheel = mmi.createWheel(0x40);
MmiWheel *mmiBigWheel   = mmi.createWheel(0x50);

// Lights
MmiLight mmiMediaLight(0x02, &mmi);
MmiLight mmiNameLight(0x03, &mmi);
MmiLight mmiTelLight(0x04, &mmi);
MmiLight mmiNavLight(0x05, &mmi);
MmiLight mmiInfoLight(0x06, &mmi);
MmiLight mmiCarLight(0x07, &mmi);
MmiLight mmiSetupLight(0x08, &mmi);
MmiLight mmiTopLeftLight(0x0A, &mmi);
MmiLight mmiBottomLeftLight(0x0B, &mmi);
MmiLight mmiTopRightLight(0x0D, &mmi);
MmiLight mmiBottomRightLight(0x0E, &mmi);
MmiLight mmiRadioLight(0x18, &mmi);
MmiLight mmiAllOff (0xFF, &mmi);


  //////////////////////////////
 // ILLUMINATION DEFINITIONS //
//////////////////////////////


Button illuminationDimUpButton(new DigitalInput(45, 20, LOW, INPUT), 0);
Button illuminationDimDownButton(new DigitalInput(46, 20, LOW, INPUT), 0);

uint8_t desiredIlluminationLevel = 0xFF / 2;
uint8_t illuminationLevel = 0x00;
bool illuminationState = false;



CanInput brakeSensor(0x06F1, 4, B01000000);
CanInput keySensor(0x0358, 0, B00000001);

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

CanInput handbrakeSensor    (0x06F1, 4, B00010000);
CanInput headlightSensor    (0x060D, 0, B00000110);
CanInput runningLightSensor (0x060D, 0, B00000100);
CanInput frontFogLight      (0x060D, 1, B00000001);
CanInput ignitionAcc        (0x060D, 1, B00000010);
CanInput ignitionOn         (0x060D, 1, B00000110);
CanInput cruiseControl      (0x0233, 3, B00000010);



  //////////////////
 // SKETCH SETUP //
//////////////////

void setup() {

  delay(3000);  
  Serial.begin(115200);
  delay(500);
  statusInitSuccess.serialize(Serial);

  Keyboard.begin();

  Can0.begin(500000);

  //obd2.sendRequest(8, 23);

  SPI.begin();

}


  /////////////////
 // SKETCH LOOP //
/////////////////

void loop() {
  updateCan();
  
  updateMmi();
  
  updateIllumination();

  hvac.update();

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

void updateMmi() {
  mmi.update(mmiEvent);
  
  
  if (mmiNavButton->wasPressedTimes(1)) {
     mmiAllOff.off();
     mmiNavLight.on();
     _MmiAndroidPacket.payload(1);
     _MmiAndroidPacket.serialize(Serial);
  }
  if (mmiInfoButton->wasPressedTimes(1)) {
     mmiAllOff.off();
     mmiInfoLight.on();
     _MmiAndroidPacket.payload(2);
     _MmiAndroidPacket.serialize(Serial);
  }
  if (mmiInfoButton->wasPressedFor(1000)) {
     mmiAllOff.off();
     mmiInfoLight.on();
     _MmiAndroidPacket.payload(42);
     _MmiAndroidPacket.serialize(Serial);
  }
  if (mmiCarButton->wasPressedTimes(1)) {
     mmiAllOff.off();
     mmiCarLight.on();
     _MmiAndroidPacket.payload(3);
     _MmiAndroidPacket.serialize(Serial);
  }
  if (mmiSetupButton->wasPressedTimes(1)) {
     mmiAllOff.off();
     mmiSetupLight.on();   
  }
  if (mmiSetupButton->wasHeldFor(500)) {
     mmiAllOff.off();
     _MmiAndroidPacket.payload(4);
     _MmiAndroidPacket.serialize(Serial);
  }
  if (mmiRadioButton->wasPressedTimes(1)) {
     mmiAllOff.off();
     mmiRadioLight.on();
  }
  if (mmiMediaButton->wasPressedTimes(1)) {
      mmiAllOff.off();
      mmiMediaLight.on();
     _MmiAndroidPacket.payload(6);
     _MmiAndroidPacket.serialize(Serial);
  }
  if (mmiMediaButton->wasPressedTimes(1) && mmiSetupButton->wasPressedTimes(1)) {
      mmiAllOff.off();
      mmiMediaLight.on();
     _MmiAndroidPacket.payload(20);
     _MmiAndroidPacket.serialize(Serial);
    }
  if (mmiNameButton->wasPressedTimes(1)) {
      mmiAllOff.off();
      mmiNameLight.on();
     _MmiAndroidPacket.payload(7);
     _MmiAndroidPacket.serialize(Serial);
  }
  if (mmiTelButton->wasPressedTimes(1)) {
     mmiAllOff.off();
     mmiTelLight.on();
  }
  if (mmiTopLeftButton->wasPressedTimes(1)) {
     //mmiAllOff.off();
     mmiTopLeftLight.toggle();
  }
  if (mmiTopRightButton->wasPressedTimes(1)) {
     mmiAllOff.off();
     //mmiTopRightLight.toggle();
      //Keyboard.press(KEY_HOME); 
     //Keyboard.release(KEY_HOME); 
     //Keyboard.press(KEY_HOME); 
     //Keyboard.release(KEY_HOME); 
  }
  if (mmiBottomLeftButton->wasPressedTimes(1)) {
     mmiBottomLeftLight.toggle();
  }
  if (mmiBottomRightButton->wasPressedTimes(1)) {
     mmiBottomRightLight.toggle();
  }
  if (mmiPreviousButton->wasPressedTimes(1)) {
     //Keyboard.press(KEY_MEDIA_PREV_TRACK);
     //Keyboard.release(KEY_MEDIA_PREV_TRACK);
  }
  if (mmiReturnButton->wasHeldFor(500))  {
     //Keyboard.press(KEY_BACKSPACE); 
     //Keyboard.release(KEY_BACKSPACE);
     //Keyboard.release(KEY_LEFT_ALT);
  }

  if (mmiReturnButton->wasPressedFor(1000))  {
     //Keyboard.press(KEY_DELETE); 
     //Keyboard.release(KEY_DELETE);
     //Keyboard.release(KEY_LEFT_ALT);
  }
  if (mmiReturnButton->wasPressedTimes(1)) {
     //Keyboard.press(KEY_ESC);
     //Keyboard.release(KEY_ESC);
     //Keyboard.release(KEY_LEFT_ALT);
  }
  if (mmiNextButton->wasPressedTimes(1)) {
     //Keyboard.press(KEY_MEDIA_NEXT_TRACK);
     //Keyboard.release(KEY_MEDIA_NEXT_TRACK);
  }
  if (mmiNextButton->wasPressedTimes(2)) {
     //Keyboard.press(KEY_MEDIA_PLAY_SKIP);
     //Keyboard.release(KEY_MEDIA_PLAY_SKIP);
  }

  if (mmiBigWheel->wasTurned()) {
    if (mmiBigWheel->getAmount() < 0) {
      //Keyboard.press(MODIFIERKEY_SHIFT);
      //Keyboard.press(KEY_TAB);
      //Keyboard.release(KEY_TAB);
      //Keyboard.release(MODIFIERKEY_SHIFT);
    }
    else {
      //Keyboard.press(KEY_TAB);
      //Keyboard.release(KEY_TAB);
    }
  }
 // ********************************************************************
   if (mmiBigWheelButton->wasHeldFor(2000)) {
    if (mmiBigWheel->wasTurned()) {
      if (mmiBigWheel->getAmount() < 0) {
         mmi.setIllumination(illuminationLevel);
         mmi.setHighlightLevel(max(0x46, illuminationLevel));
      }
      else {
        mmi.setIllumination(illuminationLevel);
        mmi.setHighlightLevel(max(0x46, illuminationLevel));
      }
    }
   }
 // ********************************************************************
  if (mmiBigWheelButton->wasPressedTimes(1)) {
      //Keyboard.press(KEY_MEDIA_PLAY_PAUSE);
      //Keyboard.press(KEY_ENTER);
      //Keyboard.release(KEY_MEDIA_PLAY_PAUSE);
      //Keyboard.release(KEY_ENTER);
      //Keyboard.release(KEY_LEFT_ALT);
  }
  if (mmiBigWheelButton-> wasPressedFor(1000)) {
     mmiAllOff.off();
     //Keyboard.press(KEY_LEFT_ALT);
     //Keyboard.press(KEY_TAB);
     //Keyboard.release(KEY_TAB);
     
    
  }

  if (mmiSmallWheel->wasTurned()) {
    if (mmiSmallWheel->getAmount() < 0) {
      //Keyboard.press(KEY_MEDIA_VOLUME_DEC);
      //Keyboard.release(KEY_MEDIA_VOLUME_DEC);
    }
    else {
      //Keyboard.press(KEY_MEDIA_VOLUME_INC);
      //Keyboard.release(KEY_MEDIA_VOLUME_INC);
    }
  }
  if (mmiSmallWheelButton->wasPressedTimes(1)) {
     //Keyboard.press(KEY_MEDIA_MUTE);
     //Keyboard.release(KEY_MEDIA_MUTE);
  }
  if (mmiSmallWheelButton->wasHeldFor(1000)) {
     mmiAllOff.off();
     mmi.shutdown();
  }else{
    //Keyboard.press(KEY_SYSTEM_WAKE_UP);  
    //Keyboard.release(KEY_SYSTEM_WAKE_UP);
    }
}

void mmiEvent(uint8_t code) {
  if (code == 0xff || code == 0x38) {
    mmi.enableKeys();
  }
  else if (code == 0x35) {
    mmi.setIllumination(illuminationLevel);
    mmi.setHighlightLevel(0x99);
  }
}


  ////////////////////////////
 // ILLUMINATION FUNCTIONS //
////////////////////////////

void updateIllumination() {
  illuminationDimUpButton.update();
  illuminationDimDownButton.update();

  if (illuminationDimUpButton.isPressed() || illuminationDimUpButton.wasHeldFor(500, 200)) {
    desiredIlluminationLevel = min(255, (desiredIlluminationLevel + 0xFF / 16));
  }
  if (illuminationDimDownButton.isPressed() || illuminationDimDownButton.wasHeldFor(500, 200)) {
    desiredIlluminationLevel = max(46, (desiredIlluminationLevel - 0xFF / 16));
  }

  changeIllumination(runningLightSensor.getState(), desiredIlluminationLevel);
}

void changeIllumination(bool newState, uint8_t newLevel) {
  newLevel = newState ? newLevel : 0x00;
  if (illuminationState != newState || illuminationLevel != newLevel) {
    illuminationState = newState;
    illuminationLevel = newLevel;

    Serial.printf("Illumination %d.\r\n", illuminationLevel);
    
    mmi.setIllumination(illuminationLevel);
    mmi.setHighlightLevel(max(0x46, illuminationLevel));
  }
}

  ///////////////////////
 // CAN BUS FUNCTIONS //
///////////////////////

void updateCan() { 
  CAN_message_t canMessage;
 
  while (Can0.available()) {
    Can0.read(canMessage);
    canSniffer.update(canMessage);
    
    handbrakeSensor.update(canMessage);
    headlightSensor.update(canMessage);
    runningLightSensor.update(canMessage);
    frontFogLight.update(canMessage);
    ignitionAcc.update(canMessage);
    ignitionOn.update(canMessage);  
  }
}
