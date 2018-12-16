/*
 * Sketch: octo32_ttn_basic-test
 * Demo sketch for Octopus32 by G. Burger, 2018
 * Initial code
 * 2018-12 G. Burger: Blinks Neopixels, scans I2C bus and connects to TTN network
 * Changelog
 * 2018-12 t-pi: Neopixels disabled, as LoRa was not working correctly
 */
#include <lmic.h>
#include <hal/hal.h>
#include <Wire.h>
#include <Adafruit_NeoPixel.h>

// Adafruit_NeoPixel pixels = Adafruit_NeoPixel(2,21,NEO_GRBW + NEO_KHZ800);
// LoraWAN Copyright (c) 2015 Thomas Telkamp and Matthijs Kooijman
// https://github.com/matthijskooijman/arduino-lmic
// -------- LoRa PinMapping FeatherWing Octopus
const lmic_pinmap lmic_pins = {  
  .nss = 14,                            // Connected to pin D
  .rxtx = LMIC_UNUSED_PIN,             // For placeholder only, Do not connected on RFM92/RFM95
  .rst = LMIC_UNUSED_PIN,              // Needed on RFM92/RFM95? (probably not) D0/GPIO16 
  .dio = {
    33, 33, LMIC_UNUSED_PIN         }
};

#include "access_keys.h"
// include file for The Things Network keys
//static const u1_t PROGMEM DEVEUI[8]= { 0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00 }; // lsb
//void os_getDevEui (u1_t* buf) { 
//  memcpy_P(buf, DEVEUI, 8);
//}
//
//static const u1_t PROGMEM APPEUI[8]={ 0x00,0x00,0x00,0x00,0x00,0x00,0xB3,0x70 }; // lsb
//void os_getArtEui (u1_t* buf) { 
//  memcpy_P(buf, APPEUI, 8);
//}
//
//static const u1_t PROGMEM APPKEY[16]={ 0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00 }; // MSB !!
//void os_getDevKey (u1_t* buf) {  
//  memcpy_P(buf, APPKEY, 16);
//};
//
volatile int LoRaWAN_Tx_Ready      = 0; // Merker für ACK 

int LoRaWAN_Rx_Payload = 0 ;
// -------- LoRa Event 
void onEvent (ev_t ev) { 
  Serial.print(os_getTime());
  Serial.print(": ");
  switch(ev) {
  case EV_SCAN_TIMEOUT:
    Serial.println(F("EV_SCAN_TIMEOUT"));
    break;
  case EV_BEACON_FOUND:
    Serial.println(F("EV_BEACON_FOUND"));
    break;
  case EV_BEACON_MISSED:
    Serial.println(F("EV_BEACON_MISSED"));
    break;
  case EV_BEACON_TRACKED:
    Serial.println(F("EV_BEACON_TRACKED"));
    break;
  case EV_JOINING:
    Serial.println(F("EV_JOINING"));
    break;
  case EV_JOINED:
    Serial.println(F("EV_JOINED"));
    // Disable link check validation (automatically enabled
    // during join, but not supported by TTN at this time).
    LMIC_setLinkCheckMode(0);
    break;
  case EV_RFU1:
    Serial.println(F("EV_RFU1"));
    break;
  case EV_JOIN_FAILED:
    Serial.println(F("EV_JOIN_FAILED"));
    break;
  case EV_REJOIN_FAILED:
    Serial.println(F("EV_REJOIN_FAILED"));
    break;
    break;
  case EV_TXCOMPLETE:
    Serial.println(F("EV_TXCOMPLETE (includes waiting for RX windows)"));
    if (LMIC.txrxFlags & TXRX_ACK)
      Serial.println(F("Received ack"));
    if (LMIC.dataLen) {
      Serial.println(F("Received "));
      Serial.println(LMIC.dataLen);
      Serial.println(F(" bytes of payload"));
      LoRaWAN_Rx_Payload = 0; 
      for (int i = 0;i<LMIC.dataLen;i++) { 
        Serial.println(LMIC.frame[i+ LMIC.dataBeg],HEX);
        LoRaWAN_Rx_Payload = 256*LoRaWAN_Rx_Payload+LMIC.frame[i+ LMIC.dataBeg];
      }
    }
    LoRaWAN_Tx_Ready = 1;
    // Schedule next transmission
    //os_setTimedCallback(&sendjob, os_getTime()+sec2osticks(TX_INTERVAL), do_send);
    break;
  case EV_LOST_TSYNC:
    Serial.println(F("EV_LOST_TSYNC"));
    break;
  case EV_RESET:
    Serial.println(F("EV_RESET"));
    break;
  case EV_RXCOMPLETE:
    // data received in ping slot
    Serial.println(F("EV_RXCOMPLETE"));
    break;
  case EV_LINK_DEAD:
    Serial.println(F("EV_LINK_DEAD"));
    break;
  case EV_LINK_ALIVE:
    Serial.println(F("EV_LINK_ALIVE"));
    break;
  default:
    Serial.println(F("Unknown event"));
    break;
  }
}


void setup(){ // Einmalige Initialisierung

  Serial.begin(115200);
  Wire.begin();
  byte error, address;
  int nDevices;
 
  Serial.println("Scanning...");
 
  nDevices = 0;
  for(address = 1; address < 127; address++ ) 
  {
    // The i2c_scanner uses the return value of
    // the Write.endTransmisstion to see if
    // a device did acknowledge to the address.
    Wire.beginTransmission(address);
    error = Wire.endTransmission();
 
    if (error == 0)
    {
      Serial.print("I2C device found at address 0x");
      if (address<16) 
        Serial.print("0");
      Serial.print(address,HEX);
      Serial.println("  !");
 
      nDevices++;
    }
    else if (error==4) 
    {
      Serial.print("Unknown error at address 0x");
      if (address<16) 
        Serial.print("0");
      Serial.println(address,HEX);
    }    
  }
  if (nDevices == 0)
    Serial.println("No I2C devices found\n");
  else
    Serial.println("done\n");
 
  delay(5000);           // wait 5 seconds for next scan

  
//  pixels.begin();//-------------- Initialisierung Neopixel
//  delay(1);
//  pixels.show();
//  pixels.setPixelColor(0,0,0,0); // alle aus
//  pixels.setPixelColor(1,0,0,0);
//  pixels.show();                 // und anzeigen

  // -- Initialisiere LoraWAN 
  os_init();             // LMIC LoraWAN
  LMIC_reset();          // Reset the MAC state 
  LMIC.txpow = 27;       // Maximum TX power 
  LMIC.datarate=DR_SF12; // Long Range
  LMIC.rps = updr2rps(LMIC.datarate);

//  pixels.setPixelColor(0,40,0,0);
//  pixels.show();
//  pixels.setPixelColor(1,40,0,0);
//  pixels.show();
//  delay( 1000 );
//
//  pixels.setPixelColor(0,0,30,0);
//  pixels.show();
//  pixels.setPixelColor(1,0,30,0);
//  pixels.show();
//  delay( 1000 );
//
//  pixels.setPixelColor(0,0,0,30);
//  pixels.show();
//  pixels.setPixelColor(1,0,0,30);
//  pixels.show();
//  delay(1000);
//
//  pixels.setPixelColor(0,0,0,0); // alle aus
//  pixels.setPixelColor(1,0,0,0);
//  pixels.show();                 // und anzeigen

}

int blinky = 0;

void loop() { // Kontinuierliche Wiederholung 

  if (blinky == 0) blinky = 1; else blinky = 0;
//  pixels.setPixelColor(blinky,0,0,40);
//  pixels.show();                 // und anzeigen

  { //Block------------------------------ sende Daten an TTN  
    int port = 10;
    static uint8_t mydata[2];
    int wert=round(47.1*10);
    mydata[0] = wert >> 8; 
    mydata[1] = wert & 0xFF;
    // Check if there is not a current TX/RX job running
    //if (LMIC.opmode & OP_TXRXPEND) {
    if (LMIC.opmode & (1 << 7)) { 
      Serial.println(F("OP_TXRXPEND, not sending"));
    } 
    else {
      // Prepare upstream data transmission at the next possible time.
      LoRaWAN_Tx_Ready = 0;                                 // Merker für ACK
      LMIC_setTxData2(port, mydata, sizeof(mydata)-1, 0);   // Sende         
      Serial.println(F("Packet queued"));
      while(LoRaWAN_Tx_Ready==0) {
        yield();
        os_runloop_once();
      };  // Warte bis gesendet
    }
  } // Blockende
//  pixels.setPixelColor(blinky,0,0,0);
//  pixels.show();                 // und anzeigen
}
