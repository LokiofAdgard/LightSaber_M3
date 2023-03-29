#include <Arduino.h>
#include <FastLED.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <Wire.h>
#include <SoftwareSerial.h>
#include <DFRobotDFPlayerMini.h>

namespace LIGHTS{
  const char LED_PIN = 13;
  const char NUM_LEDS = 60;
}



void setup() {
  pinMode(LIGHTS::LED_PIN, OUTPUT);
}

void loop() {
  digitalWrite(LIGHTS::LED_PIN, HIGH);
  delay(1000);
  digitalWrite(LIGHTS::LED_PIN, LOW);
  delay(1000);
}