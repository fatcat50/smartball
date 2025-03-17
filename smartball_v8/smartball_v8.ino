/**/
//Filename: Ball_Programm_Structure_v7_Gyro.ino
//Author: Luis Ritter (Modifiziert für Gyroskop & Sanfte Übergänge)
//Description: Ball Programm mit Gyroskop-basiertem Feedback
/**/

// --- Includes ---
#include <ArduinoBLE.h>
#include <Arduino_LSM9DS1.h>
#include <Adafruit_NeoPixel.h>
#include "nRF52_MBED_PWM.h"

// --- Defines ---
enum state {Bluetooth, Sensor, Ton, LED, Akku, OffMode};
state currentState = Bluetooth;

// BLE
#define SERVICE_UUID         "4fafc201-1fb5-459e-8fcc-c5c9c331914b"
#define CHARACTERISTIC_UUID  "beb5483e-36e1-4688-b7f5-ea07361b26a8" 
#define CHARACTERISTIC2_UUID "4a78b8dd-a43d-46cf-9270-f6b750a717c8" 

BLEService pService(SERVICE_UUID);
BLEUnsignedIntCharacteristic xCharacteristic(CHARACTERISTIC_UUID, BLEBroadcast | BLERead | BLEWrite);
BLEUnsignedIntCharacteristic yCharacteristic(CHARACTERISTIC2_UUID, BLEBroadcast | BLERead | BLEWrite);

// LED
#define NUMLEDs 60 // Einmal definiert
Adafruit_NeoPixel LEDs(NUMLEDs, A0, NEO_GRB + NEO_KHZ800);
Adafruit_NeoPixel LEDs1(NUMLEDs, A2, NEO_GRB + NEO_KHZ800);
Adafruit_NeoPixel LEDs2(NUMLEDs, A6, NEO_GRB + NEO_KHZ800);

// Tone
#define BUZZERS 3
uint32_t buzPins[BUZZERS] = {D2, D4, D6};
mbed::PwmOut* pwm[BUZZERS] = {nullptr};

// Gyroskop & Glättung
float gyrx, gyry, gyrz;
float smoothedGyro = 0.0f;
const float GYRO_SMOOTHING = 0.2f; // Je niedriger, desto glatter
const float MAX_GYRO = 1000.0f;     // Maximale erwartete Drehrate (°/s)

// --- Variablen ---
// (Unverändert aus Originalcode, außer accsum → gyroValue)
float gyroValue = 0.0f;
unsigned long lastUpdate = 0;

// --- Setup ---
void setup() {
  BLE.begin();
  BLE.setLocalName("Klingelball");
  pService.addCharacteristic(xCharacteristic);
  BLE.addService(pService);
  xCharacteristic.setValue(0);
  BLE.advertise();

  IMU.begin();
  IMU.setGyroFS(3); // ±2000°/s Range

  for(int i=0; i<BUZZERS; i++){
    pwm[i] = new mbed::PwmOut(digitalPinToPinName(buzPins[i]));
  }
}

// --- Loop ---
void loop() {
  switch(currentState){
    case Bluetooth: handleBluetooth(); break;
    case Sensor:    readGyro();       break;
    case Ton:       updateTone();     break;
    case LED:       updateLEDs();     break;
    case Akku:      checkBattery();   break;
    case OffMode:   enterLowPower();  break;
  }
}

// --- Gyroskop-Lesung mit Glättung ---
void readGyro() {
  if(IMU.gyroAvailable()){
    IMU.readGyro(gyrx, gyry, gyrz);
    float rawGyro = sqrt(gyrx*gyrx + gyry*gyry + gyrz*gyrz);
    smoothedGyro = (1.0-GYRO_SMOOTHING)*smoothedGyro + GYRO_SMOOTHING*rawGyro;
    
    gyroValue = constrain(smoothedGyro / MAX_GYRO, 0.0f, 1.0f);
    currentState = Ton;
  }
}

// --- Sanfte Ton-Übergänge ---
void updateTone() {
  if(millis() - lastUpdate < 50) return; // 20Hz Update
  
  float freq = FreqStill + (FreqMov - FreqStill) * gyroValue;
  float volume = Volume * gyroValue;
  
  for(int i=0; i<BUZZERS; i++){
    setPWM(pwm[i], buzPins[i], freq, volume);
  }
  
  lastUpdate = millis();
  currentState = LED;
}

// --- Sanfte LED-Übergänge ---
void updateLEDs() {
  uint8_t r = LEDRedStill + (LEDRedMov - LEDRedStill) * gyroValue;
  uint8_t g = LEDGreenStill + (LEDGreenMov - LEDGreenStill) * gyroValue;
  uint8_t b = LEDBlueStill + (LEDBlueMov - LEDBlueStill) * gyroValue;
  
  for(int i=0; i<NUMLEDs; i++){
    LEDs.setPixelColor(i, LEDs.Color(r, g, b));
    LEDs1.setPixelColor(i, LEDs.Color(r, g, b));
    LEDs2.setPixelColor(i, LEDs.Color(r, g, b));
  }
  LEDs.show();
  LEDs1.show();
  LEDs2.show();
  
  currentState = Bluetooth;
}

// --- Vereinfachte BLE-Handling-Funktion ---
void handleBluetooth() {
  BLEDevice central = BLE.central();
  if(central && central.connected()){
    // BLE-Datenverarbeitung hier (wie im Originalcode)
    currentState = Sensor;
  } else {
    currentState = OffMode;
  }
}

// --- Restliche Funktionen analog zum Originalcode (angepasst für Gyro) ---
// (checkBattery(), enterLowPower(), etc.)