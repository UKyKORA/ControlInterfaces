/*
 * Firmware for UKRMT KATS Testing Remote
 * Matt Ruffner 
 * Damien Lawhorn
 * 
 * March 2019
 * http://github.com/ukrmt/testingremote
 * http://github.com/ukrmt/marsbotcontrol
 */

#include <Arduino.h>
#include <U8g2lib.h> //OLED
#ifdef U8X8_HAVE_HW_SPI
#include <SPI.h>
#endif
#ifdef U8X8_HAVE_HW_I2C
#include <Wire.h>
#endif
#include <WiFi.h>
#include <WiFiMulti.h>
#include <WiFiClientSecure.h>
#include <WebSocketsClient.h>
#include <Adafruit_ADS1015.h>
#include <NeoPixelAnimator.h>
#include <NeoPixelBrightnessBus.h>
#include <NeoPixelBus.h>
#include <ArduinoJson.h>
#include "dictionary.h"

#define DigDwn 33
#define DigUp 27
#define OLEDUp 12
#define OLEDDwn 21
#define LED 5
#define SLIDER_M1 A3
#define SLIDER_M2 A2
#define SLIDER_M3 A1
#define SLIDER_M4 A0
#define SLIDER_EXC A5
#define SLIDER_OFL A4
#define BATTERY A13

#define UI_DISPLAY  1000
#define UI_LED      1000
#define UI_CONTROL  200
#define UI_WS       250
unsigned long lastDisplayUpdate = 0;
unsigned long lastLEDUpdate = 0;
unsigned long lastControlUpdate = 0;
unsigned long lastWSUpdate = 0;


WiFiMulti WiFiMulti;
WebSocketsClient webSocket;
U8G2_SH1106_128X64_NONAME_F_HW_I2C u8g2(U8G2_R0, /* reset=*/ U8X8_PIN_NONE);
NeoPixelBus<NeoRgbFeature, Neo800KbpsMethod> strip(1, LED);

RgbColor red(128,0,0);
RgbColor green(0,128,0);

Adafruit_ADS1115 ads;

#define USE_SERIAL Serial

// TODO: implement different display modes
int displayMode = 0;
int wsConnected = 0;

DynamicJsonDocument doc(1024);
DeserializationError error = deserializeJson(doc, DICTIONARY);



void hexdump(const void *mem, uint32_t len, uint8_t cols = 16) {
  const uint8_t* src = (const uint8_t*) mem;
  USE_SERIAL.printf("\n[HEXDUMP] Address: 0x%08X len: 0x%X (%d)", (ptrdiff_t)src, len, len);
  for(uint32_t i = 0; i < len; i++) {
    if(i % cols == 0) {
      USE_SERIAL.printf("\n[0x%08X] 0x%08X: ", (ptrdiff_t)src, i);
    }
    USE_SERIAL.printf("%02X ", *src);
    src++;
  }
  USE_SERIAL.printf("\n");
}

void webSocketEvent(WStype_t type, uint8_t * payload, size_t length) {

  switch(type) {
    case WStype_DISCONNECTED:
      USE_SERIAL.printf("[WSc] Disconnected!\n");
      wsConnected = 0;
      break;
    case WStype_CONNECTED:
      USE_SERIAL.printf("[WSc] Connected to url: %s\n", payload);
      wsConnected = 1;
      // send message to server when Connected
      //webSocket.sendTXT("subscribe comms.sent");
      break;
    case WStype_TEXT:
      USE_SERIAL.printf("[WSc] get text: %s\n", payload);

      // send message to server
      //webSocket.sendTXT("message here");
      //webSocket.sendTXT("message here");
      //object["id"] = 'contro.motor1speed';
      
      break;
    case WStype_BIN:
      USE_SERIAL.printf("[WSc] get binary length: %u\n", length);
      hexdump(payload, length);

      // send data to server
      // webSocket.sendBIN(payload, length);
      break;
    case WStype_ERROR:      
    case WStype_FRAGMENT_TEXT_START:
    case WStype_FRAGMENT_BIN_START:
    case WStype_FRAGMENT:
    case WStype_FRAGMENT_FIN:
      break;
  }

}

void setup() {

  strip.Begin();
  strip.Show(); // Initialize all pixels to 'off'
 
  // USE_SERIAL.begin(921600);
  USE_SERIAL.begin(115200);

  //OLED Begin -- blank
  u8g2.begin();
  u8g2_prepare();
  u8g2.clearBuffer();
  u8g2.drawStr( 10, 10, "Booting...");
  u8g2.sendBuffer();

  //Serial.setDebugOutput(true);
  USE_SERIAL.setDebugOutput(true);

  USE_SERIAL.println();
  USE_SERIAL.println();
  USE_SERIAL.println();

  for(uint8_t t = 1; t > 0; t--) {
    USE_SERIAL.printf("[SETUP] BOOT WAIT %d...\n", t);
    USE_SERIAL.flush();
    delay(1000);
  }

  WiFiMulti.addAP("UKRMT", "trashcan");

  //WiFi.disconnect();
  while(WiFiMulti.run() != WL_CONNECTED) {
    delay(100);
  }

  // server address, port and URL
  webSocket.begin("192.168.1.150",1234, "/");

  // event handler
  webSocket.onEvent(webSocketEvent);

  // try ever 5000 again if connection has failed
  webSocket.setReconnectInterval(5000);

  // initialize i2c adc
  ads.begin();

  pinMode(LED, OUTPUT);
  pinMode(DigDwn, INPUT_PULLUP);
  pinMode(DigUp, INPUT_PULLUP);
  pinMode(OLEDDwn, INPUT_PULLUP);
  pinMode(OLEDUp, INPUT_PULLUP);
  pinMode(BATTERY, INPUT);
  pinMode(SLIDER_M1, INPUT);
  pinMode(SLIDER_M2, INPUT);
  pinMode(SLIDER_M3, INPUT);
  pinMode(SLIDER_M4, INPUT);
  pinMode(SLIDER_EXC, INPUT);
  pinMode(SLIDER_OFL, INPUT);
}

void u8g2_prepare(void) {
  u8g2.setFont(u8g2_font_6x10_tf);
  u8g2.setFontRefHeightExtendedText();
  u8g2.setDrawColor(1);
  u8g2.setFontPosTop();
  u8g2.setFontDirection(0);
}

void updateDisplay() {
  // TODO: switch display mode
  u8g2.clearBuffer();          // clear the internal memory
  if( wsConnected ){
    u8g2.drawStr(10,10,"Connected"); // write something to the internal memory
  } else {
    u8g2.drawStr(10,10,"Disconnected");
  }
  float battery = analogRead(BATTERY)/4095.0*2*3.3;
  String message = "Batt: " + String(battery);
  u8g2.drawStr(10,20,message.c_str());
  u8g2.sendBuffer();          // transfer internal memory to the display
}

void updateLED() {
  // TODO: switch LED state
  if( wsConnected ){
    strip.SetPixelColor(0, green); // Moderately bright green color.
    strip.Show();
  } else {
    strip.SetPixelColor(0, red); // Moderately bright red color.
    strip.Show();
  }
}

void updateControlValues() {
// old config  
//  doc["control.motor1speed"] = ads.readADC_SingleEnded(1) >> 2;
//  delay(2);
//  doc["control.motor2speed"] = ads.readADC_SingleEnded(0) >> 2;
//  doc["control.motor3speed"] = analogRead(A2);
//  doc["control.motor4speed"] = analogRead(A3);

  // new config
  doc["control.motor1speed"] = analogRead(A2);
  doc["control.motor2speed"] = doc["control.motor1speed"];
  doc["control.motor3speed"] = analogRead(A3);
  doc["control.motor4speed"] = doc["control.motor3speed"];
  
  doc["control.offloadmotorspeed"] = ads.readADC_SingleEnded(1) >> 2;
  delay(2);
  doc["control.digmotorspeed"] = ads.readADC_SingleEnded(0) >> 2;
  delay(2);
  
  doc["control.digarmspeed"] = (digitalRead(DigDwn)==LOW) ? 0 : ((digitalRead(DigUp)==LOW) ? 4095 : 2000); 
  doc["control.raisearmspeed"] = ads.readADC_SingleEnded(2) >> 2;
}

void sendState() {
  if( wsConnected ){
    String message;
    serializeJson(doc, message);
    webSocket.sendTXT(message);
  } else {
    String message;
    serializeJson(doc, message);
    Serial.println(message);
  }
}

void loop() {
  unsigned long n = millis();

  if( n-lastDisplayUpdate > UI_DISPLAY ){
    updateDisplay();
    lastDisplayUpdate = n;
  } 

  if( n-lastLEDUpdate > UI_LED ){
    updateLED();
    lastLEDUpdate = n;
  } 

  if( n-lastControlUpdate > UI_CONTROL ){
    updateControlValues();
    lastControlUpdate = n;
  } 

  if( n-lastWSUpdate > UI_WS ){
    sendState();
    lastWSUpdate = n;
  } 
  
  webSocket.loop();
  
}
