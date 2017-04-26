/****************************************************************
  Hardware Connections:

  IMPORTANT: The APDS-9960 can only accept 3.3V!

  Arduino Pin  APDS-9960 Board  Function

  3.3V         VCC              Power
  GND          GND              Ground
  A4           SDA              I2C Data
  A5           SCL              I2C Clock

  Resources:
  Include Wire.h and SparkFun_APDS-9960.h

  Distributed as-is; no warranty is given.
****************************************************************/
#include <Wire.h>
#include <SparkFun_APDS9960.h>
#include <Arduino.h>
#include <ESP8266WiFi.h>
#include <WebSocketsServer.h>
#include <Hash.h>

SparkFun_APDS9960 apds = SparkFun_APDS9960();
WebSocketsServer webSocket = WebSocketsServer(81);

uint16_t ambient_light = 0;
uint16_t red_light = 0;
uint16_t green_light = 0;
uint16_t blue_light = 0;
boolean clientConnected;


typedef struct {
  volatile uint16_t ambient;
  volatile uint16_t red;
  volatile uint16_t green;
  volatile uint16_t blue;
} DataPacket;

void webSocketEvent(uint8_t num, WStype_t type, uint8_t * payload, size_t lenght) {
  switch (type) {
    case WStype_DISCONNECTED:
      {
        clientConnected = false;
        Serial.println(F("Disconnected"));
      }
      break;
    case WStype_CONNECTED:
      {
        clientConnected = true;
        Serial.println(F("Connected"));
        webSocket.sendTXT(num, "Connected");
      }
      break;
  }
}

void setup() {
  Serial.begin(9600);
  Serial.println();
  Serial.println(F("--------------------------------"));
  Serial.println(F("APDS-9960 - ColorSensor"));
  Serial.println(F("--------------------------------"));

  WiFi.begin("}RooT{", "123456789");

  while (WiFi.status() != WL_CONNECTED) {
    delay(100);
  }
  Serial.println(WiFi.localIP());
  webSocket.begin();
  webSocket.onEvent(webSocketEvent);

  if ( apds.init() ) {
    Serial.println(F("APDS-9960 initialization complete"));
  } else {
    Serial.println(F("Something went wrong during APDS-9960 init!"));
  }
  if ( apds.enableLightSensor(false) ) {
    Serial.println(F("Light sensor is now running"));
  } else {
    Serial.println(F("Something went wrong during light sensor init!"));
  }
  delay(500);
}

void loop() {

  // Read the light levels (ambient, red, green, blue)
  if (  !apds.readAmbientLight(ambient_light) ||
        !apds.readRedLight(red_light) ||
        !apds.readGreenLight(green_light) ||
        !apds.readBlueLight(blue_light) ) {
    Serial.println("Error reading light values");
  } else {
    if (clientConnected) {
      DataPacket dataPackage;
      dataPackage.ambient = ambient_light;
      dataPackage.red = red_light;
      dataPackage.green = green_light;
      dataPackage.blue = blue_light;
      webSocket.sendBIN(0, (const uint8_t *) &dataPackage, sizeof(dataPackage));
    }
    Serial.print("Ambient: ");
    Serial.print(ambient_light);
    Serial.print(" Red: ");
    Serial.print(red_light);
    Serial.print(" Green: ");
    Serial.print(green_light);
    Serial.print(" Blue: ");
    Serial.println(blue_light);
  }
  webSocket.loop();
}
