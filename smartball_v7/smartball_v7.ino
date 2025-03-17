/**/
// Filename: Ball_Programm_Structure_v7_Gyro.ino
// Author: Luis Ritter
// Created On: 13. March 2025
// Description: Ball Program with Gyroscope Input for Arduino Nano 33 BLE
/**/

// Includes
#include <ArduinoBLE.h>
#include <math.h>
#include <Arduino.h>
#include <Arduino_LSM9DS1.h>
#include <Adafruit_NeoPixel.h>
#include "nRF52_MBED_PWM.h"

// Defines
#define SERVICE_UUID         "4fafc201-1fb5-459e-8fcc-c5c9c331914b"
#define CHARACTERISTIC_UUID  "beb5483e-36e1-4688-b7f5-ea07361b26a8"
#define CHARACTERISTIC2_UUID "4a78b8dd-a43d-46cf-9270-f6b750a717c8"

#define LEDPIN_DI A0
#define NUMLEDs 60

#define AkkuRead_Pin A4

// BLE Service & Characteristics
BLEService pService(SERVICE_UUID);
BLEUnsignedIntCharacteristic xCharacteristic(CHARACTERISTIC_UUID, BLEBroadcast | BLERead | BLEWrite);
BLEUnsignedIntCharacteristic yCharacteristic(CHARACTERISTIC2_UUID, BLEBroadcast | BLERead | BLEWrite);

// LED Setup
Adafruit_NeoPixel LEDs(NUMLEDs, LEDPIN_DI, NEO_GRB + NEO_KHZ800);

// Buzzer Setup
uint32_t buz  = D2;
mbed::PwmOut* pwm = NULL;

// Sensor Variables
float gyroX, gyroY, gyroZ, gyroSum;
unsigned long gyroTime = 0;

// State Machine
enum state {Bluetooth, Sensor, Ton, LED, Akku, OffMode};
state currentState = Bluetooth;

void setup() {
    BLE.begin();
    BLE.setLocalName("Klingelball");
    BLE.setDeviceName("Klingelball");
    BLE.setAdvertisedService(pService);
    pService.addCharacteristic(xCharacteristic);
    pService.addCharacteristic(yCharacteristic);
    BLE.addService(pService);
    BLE.advertise();
    
    if (!IMU.begin()) {
        while (1);
    }
    
    LEDs.begin();
    pinMode(buz, OUTPUT);
    pinMode(AkkuRead_Pin, INPUT);
}

void loop() {
    switch (currentState) {
        case Bluetooth:
            handleBluetooth();
            currentState = Sensor;
            break;
        case Sensor:
            readGyro();
            currentState = Ton;
            break;
        case Ton:
            handleTone();
            currentState = LED;
            break;
        case LED:
            handleLED();
            currentState = Akku;
            break;
        case Akku:
            handleBattery();
            currentState = Bluetooth;
            break;
        case OffMode:
            handleOffMode();
            break;
    }
}

void handleBluetooth() {
    if (BLE.connected()) {
        xCharacteristic.setValue(1);
    }
}

void readGyro() {
    if (IMU.gyroscopeAvailable()) {
        IMU.readGyroscope(gyroX, gyroY, gyroZ);
        gyroSum = (fabs(gyroX) + fabs(gyroY) + fabs(gyroZ)) / 3;
        gyroTime = millis();
    }
}

void handleTone() {
    int frequency = map(gyroSum, 0, 200, 200, 2000);
    //setPWM(pwm, buz, frequency, 100);
}

void handleLED() {
    int brightness = map(gyroSum, 0, 200, 50, 255);
    LEDs.setBrightness(brightness);
    LEDs.fill(LEDs.Color(0, 255, 0));
    LEDs.show();
}

void handleBattery() {
    int Akkuvalue = analogRead(AkkuRead_Pin);
    yCharacteristic.setValue(Akkuvalue);
}

void handleOffMode() {
    setPWM(pwm, buz, 0, 0);
    LEDs.clear();
    LEDs.show();
}
