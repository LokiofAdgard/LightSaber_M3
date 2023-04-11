#include <Arduino.h>
#include <FastLED.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <Wire.h>
#include <SoftwareSerial.h>
#include <DFRobotDFPlayerMini.h>
#include <EEPROM.h>




namespace PF{   // LIGHTS=====================================================================================================
  const unsigned char nProfiles = 3;
  unsigned char profile = 0;


  void profile_increment(){
    profile++;
    if (profile == nProfiles) profile = 0;
    Serial.println("Profile : " + String(profile));
  }
}




namespace LG{   // LIGHTS=====================================================================================================
  const char LED_PIN = 7;
  const char LED_PIN_BACKUP = 6;
  const char NUM_LEDS = 60;
  unsigned char color = 0;
  char nColors = 3;
  char ign_delay[] = {10, 5, 50};
  char ret_delay[] = {10, 5, 50};

  CRGB leds[NUM_LEDS];


  void init_lights(){
    FastLED.addLeds<WS2812, LED_PIN, GRB>(leds, NUM_LEDS);
    Serial.println("Lights Initialized");
  }

  void set_color(unsigned char c, unsigned char p){
    switch (c)    {
    case 0:
      LG::leds[p] = CRGB(255, 0, 0);
      break;
    case 1:
      LG::leds[p] = CRGB(0, 255, 0);
      break;
    case 2:
      LG::leds[p] = CRGB(0, 0, 255);
      break;

    default:
      break;
    }
  }

  void color_increment(){
    color++;
    if(color == nColors) color = 0;
    Serial.println("Color : " + String(color));
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
      Serial.println("Sound Initialized");

      // Set volume to maximum (0 to 30).
      //player.volume(15);
      // Play the first MP3 file on the SD card
      //player.play(7);
      
    } else {
      Serial.println("Sound init Failed------");
    }
  }
}







namespace GY{   //GYROSCPOE=====================================================================================================
  const char gyro_sens = 10;
  const char low_s = 20;
  const char med_s = 40;

  Adafruit_MPU6050 mpu;


  void init_gyro(){
    while (!Serial) delay(10); // will pause Zero, Leonardo, etc until serial console opens
    if (!mpu.begin()) {
      Serial.println("Gyro init Failed------");
      while (1) {
        delay(10);
      }
    }
    Serial.println("Gyro Initialized");
    mpu.setAccelerometerRange(MPU6050_RANGE_8_G);
    mpu.setGyroRange(MPU6050_RANGE_500_DEG);
    mpu.setFilterBandwidth(MPU6050_BAND_21_HZ);
    delay(100);
  }

  int gyroRead(){
    sensors_event_t a, g, temp;
    mpu.getEvent(&a, &g, &temp);

    float v = gyro_sens*sqrt(sq(g.gyro.x) + sq(g.gyro.y) + sq(g.gyro.z));
    return(v);
  }
}






namespace ST{   //STORAGE============================================================================
  const unsigned char color_addr = 0;
  const unsigned char profile_addr = 1;
  unsigned char temp;


  char init_color(){
    if((temp = EEPROM.read(color_addr))<LG::nColors){
      Serial.println("Color Initialized : " + String(temp));
      return (temp);
    } else {
      Serial.println("Color init Failed------");
      return 0;
    }
  }

  char init_profile(){
    if((temp = EEPROM.read(profile_addr))<PF::nProfiles){
      Serial.println("Profile Initialized : " + String(temp));
      return (temp);
    } else {
      Serial.println("Profile init Failed------");
      return 0;
    }
  }

  void save_color(){
    // if(EEPROM.read(color_addr) != LG::color) EEPROM.write(color_addr, LG::color);
    EEPROM.update(color_addr, LG::color);
  }

  void save_profile(){
    // if(EEPROM.read(profile_addr) != PF::profile) EEPROM.write(profile_addr, PF::profile);
    EEPROM.update(profile_addr, PF::profile);
  }
}






namespace BT{   //BATTERY=====================================================================================================
  const char btry_pin = A0;
  const char battery_pin_backup = A1;
  const char btry_cutoff = 32;   //3.2 V
  const char btry_low = 34;      //3.4 V
  const char btry_medium = 37;   //3.7 V
  const char btry_good = 40;     //4.0 V

  int temp;

  unsigned char get_battery(){
    temp = analogRead(btry_pin);
    return (temp*5/102.3);
  }

  void init_battery(){
    temp = get_battery();
    if (temp <= btry_cutoff){
      Serial.println("Battery Too Low");
      // Add Sound
      while (1) delay(10000);      
    }
    else if(temp <= btry_low) Serial.println("Battery : LOW");
    else if(temp <= btry_medium) Serial.println("Battery : MEDIUM");
    else if(temp <= btry_good) Serial.println("Battery : GOOD");
    else Serial.println("Battery : FULL");
  }
}





namespace FN{   //FUNCTIONS=====================================================================================================
  const char ign_pin = 10;
  const char mode_pin = 9;
  const char hit_leave = 10;
  const char indicator1 = 13;
  const char indicator2 = 12;
  bool ON = false;
  int v;
  unsigned long press_time = 0;
  unsigned long last_retract = 0;

  
  void init_fn(){
    pinMode(ign_pin, INPUT);
    pinMode(mode_pin, INPUT);
    pinMode(BT::btry_pin, INPUT);
    pinMode(indicator1, OUTPUT);
    pinMode(indicator2, OUTPUT);

    //SET BACKUP PINS AS INPUTS
    pinMode(LG::LED_PIN_BACKUP, INPUT);
    pinMode(BT::battery_pin_backup, INPUT);

    //SAFTEY
    pinMode(0, INPUT);

    Serial.println("Controls Initialized");
  }

  void ignite(){
    Serial.println("Ignite");
    ON = true;
    SN::player.volume(25);
    SN::player.playFolder(PF::profile + 1, 1);
    //delay(800);
    for(int i = 0;i<LG::NUM_LEDS;i++){
      LG::set_color(LG::color,i);
      FastLED.show();
      delay(LG::ign_delay[PF::profile]); 
    }
    delay(800);
    SN::player.playFolder(PF::profile + 1, 3);
    SN::player.enableLoop();
  }

  void retract(){
    Serial.println("Retract");
    ON = false;
    SN::player.volume(25);
    SN::player.disableLoop();
    SN::player.playFolder(PF::profile + 1, 2);
    delay(600);
    for(int i = LG::NUM_LEDS;i>=0;i--){
      LG::leds[i] = CRGB(0, 0, 0);
      FastLED.show();
      delay(LG::ret_delay[PF::profile]); 
    }
    SN::player.pause();
    last_retract = millis(); // for sleep
  }

  void restore(){
    for(int i = 0;i<LG::NUM_LEDS;i++){
      LG::set_color(LG::color,i);
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
    v = GY::gyroRead();
    if(v<GY::low_s) {
      //set propotional vol (<30)
      //Serial.println("hum");
      SN::player.volume((10*v/GY::low_s)+20);
    }
    else if(v<(GY::low_s + GY::med_s)){
      Serial.println("Swing");
      SN::player.disableLoop();
      SN::player.volume(25);
      SN::player.playFolder(PF::profile + 1, 4);
      
      delay(600);
      SN::player.playFolder(PF::profile + 1, 3);
      SN::player.enableLoop();
    }
    else FN::hit();
  }

  char mode_button(){
    if (digitalRead(mode_pin) == HIGH){
      Serial.println("Mode Button Pressed");
      press_time = millis();
      while (digitalRead(mode_pin) == HIGH){
        if((((millis()-press_time)/500)%2 == 0) && ((millis()-press_time)>500)) digitalWrite(indicator1, HIGH);
        else digitalWrite(indicator1, LOW);
      }
      press_time = millis() - press_time;
      digitalWrite(indicator1, LOW);
      Serial.println(press_time);

      if (press_time<300) return 1;
      else if(press_time<1000) return 2;
      else if(press_time<2000) return 3;
      else if(press_time<3000) return 4;
      else return 0;  
    }
    return 0;
  }

  void default_settings(){
    Serial.println("Default Settings");
    LG::color = 0;
    PF::profile = 0;
    ST::save_color();
    ST::save_profile();
  }

  void battery_check(bool full){
    v = BT::get_battery();
    if (v <= BT::btry_cutoff){
      Serial.println("Battery Too Low");
      // Add Sound
      if(FN::ON) FN::retract();
      while (1) delay(10000);      
    }
    if(!full) return;

    if(v <= BT::btry_low){ Serial.println("Battery : LOW"); /*Add Sound*/}
    else if(v <= BT::btry_medium){ Serial.println("Battery : MEDIUM"); /*Add Sound*/}
    else if(v <= BT::btry_good){ Serial.println("Battery : GOOD"); /*Add Sound*/}
    else{ Serial.println("Battery : FULL"); /*Add Sound*/}
  }

  void sleeper(unsigned char m){ //minutes
    if(millis()>(last_retract + 60000*m)){
      Serial.println("Going to Sleep");
      while (1) delay(1000000);      
    }
  }
}







void setup() {
  Serial.begin(9600);
  Serial.println("\n=================================================");
  
  BT::init_battery();
  LG::init_lights();
  SN::init_sounds(); 
  GY::init_gyro();
  FN::init_fn();
  LG::color = ST::init_color();
  PF::profile = ST::init_profile();

  // SN::player.playFolder(PF::profile + 1, 2);
}

void loop() {

  if(digitalRead(FN::ign_pin)==HIGH){
    if(FN::ON==false) FN::ignite();
    else{
      FN::swingState();
      FN::battery_check(false);
      delay(100);
    }
  }
  else{
    if(FN::ON==true){
      FN::retract();
      ST::save_color();
      ST::save_profile();
    } else FN::sleeper(10);

    switch (FN::mode_button()){
    case 1:
      LG::color_increment();
      break;
    case 2:
      PF::profile_increment();
      break;
    case 3:
      FN::default_settings();
      break;
    case 4:
      FN::battery_check(true);
      break;
    default:
      break;
    }

    delay(10);
  }
}