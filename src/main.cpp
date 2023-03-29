#include <Arduino.h>
#include <FastLED.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <Wire.h>
#include <SoftwareSerial.h>
#include <DFRobotDFPlayerMini.h>




namespace LG{   // LIGHTS=====================================================================================================
  const char LED_PIN = 7;
  const char NUM_LEDS = 60;

  CRGB leds[NUM_LEDS];


  void init_lights(){
    FastLED.addLeds<WS2812, LED_PIN, GRB>(leds, NUM_LEDS);
  }
}








namespace SN{   //SOUNDS=====================================================================================================
  static const uint8_t PIN_MP3_TX = 2; // Connects to module's RX 
  static const uint8_t PIN_MP3_RX = 3; // Connects to module's TX 

  SoftwareSerial softwareSerial(PIN_MP3_RX, PIN_MP3_TX);
  DFRobotDFPlayerMini player;


  void init_sounds(){
    softwareSerial.begin(9600);
    if (player.begin(softwareSerial)) {
    Serial.println("OK");

      // Set volume to maximum (0 to 30).
      //player.volume(15);
      // Play the first MP3 file on the SD card
      //player.play(7);
      
    } else {
      Serial.println("Connecting to DFPlayer Mini failed!");
    }
  }
}







namespace GY{   //GYROSCPOE=====================================================================================================
  const char gyro_sens = 10;
  const char low_s = 20;
  const char med_s = 40;

  Adafruit_MPU6050 mpu;


  void init_gyro(){
    while (!Serial)
      delay(10); // will pause Zero, Leonardo, etc until serial console opens

    Serial.println("Adafruit MPU6050 test!");
    if (!mpu.begin()) {
      Serial.println("Failed to find MPU6050 chip");
      while (1) {
        delay(10);
      }
    }
    Serial.println("MPU6050 Found!");
    mpu.setAccelerometerRange(MPU6050_RANGE_8_G);
    mpu.setGyroRange(MPU6050_RANGE_500_DEG);
    mpu.setFilterBandwidth(MPU6050_BAND_21_HZ);
    Serial.println("");
    delay(100);
  }

  int gyroRead(){
    sensors_event_t a, g, temp;
    mpu.getEvent(&a, &g, &temp);

    float v = gyro_sens*sqrt(sq(g.gyro.x) + sq(g.gyro.y) + sq(g.gyro.z));
    return(v);
  }
}






namespace FN{   //FUNCTIONS=====================================================================================================
  const char ign_pin = 13;
  const char hit_leave = 10;
  bool ON = false;


  void ignite(){
    ON = true;
    SN::player.volume(25);
    SN::player.play(3);
    //delay(800);
    for(int i = 0;i<LG::NUM_LEDS;i++){
      LG::leds[i] = CRGB(255, 0, 0);
      FastLED.show();
      delay(10); 
    }
    delay(800);
    SN::player.loop(2);
  }

  void retract(){
    ON = false;
    SN::player.volume(25);
    SN::player.disableLoop();
    SN::player.play(1);
    delay(600);
    for(int i = LG::NUM_LEDS;i>=0;i--){
      LG::leds[i] = CRGB(0, 0, 0);
      FastLED.show();
      delay(10); 
    }
  }

  void restore(){
    for(int i = 0;i<LG::NUM_LEDS;i++){
      LG::leds[i] = CRGB(255, 0, 0);
      FastLED.show();
    }
  }

  void hit(){
    Serial.println("hit");
    for(int i = 40;i<LG::NUM_LEDS;i++){
      LG::leds[i] = CRGB(255, 255, 255);
      FastLED.show();
      delay(10); 
    }

    delay(500);
    int r = 0;
    while((r<hit_leave) && (digitalRead(ign_pin)==HIGH)){
      r = GY::gyroRead();
      Serial.println("hit");
      delay(500);// consider removing
    }
    restore();
  }

  void swingState(){
    int v = GY::gyroRead();
    if(v<GY::low_s) {
      //set propotional vol (<30)
      Serial.println("hum");
      SN::player.volume((10*v/GY::low_s)+20);
    }
    else if(v<(GY::low_s + GY::med_s)){
      //swing sound
      Serial.println("swing");
      SN::player.disableLoop();
      SN::player.volume(25);
      SN::player.play(5);
      
      delay(600);
      SN::player.loop(2);
    }
    else FN::hit();
  }
}



void setup() {
  Serial.begin(9600);
  pinMode(FN::ign_pin,INPUT);

  GY::init_gyro();
  LG::init_lights();
  SN::init_sounds(); 
}

void loop() {
  if((digitalRead(FN::ign_pin)==HIGH)&&(FN::ON==false)) FN::ignite();
  else if((digitalRead(FN::ign_pin)!=HIGH)&&(FN::ON==true)) FN::retract();
  else if((digitalRead(FN::ign_pin)==HIGH)&&(FN::ON==true)){
    FN::swingState();
    delay(100);
  }
}