#include <Adafruit_NeoPixel.h>
#include <ArduinoBLE.h>
#include <Arduino_LSM9DS1.h>

#include "nRF52_MBED_PWM.h"

// BLE Einstellungen
#define SERVICE_UUID "4fafc201-1fb5-459e-8fcc-c5c9c331914b"
#define CHARACTERISTIC_UUID "beb5483e-36e1-4688-b7f5-ea07361b26a8"

// LED Einstellungen
#define LED_PIN A0
#define NUM_LEDS 60
Adafruit_NeoPixel leds(NUM_LEDS, LED_PIN, NEO_GRB + NEO_KHZ800);

// Buzzer Pins
const uint8_t buzzerPins[] = { D2, D4, D6 };

// Ton Variablen
float freq = 800.0f;
int period = 1000 * 0.5;

mbed::PwmOut* pwm = NULL;
mbed::PwmOut* pwm2 = NULL;
mbed::PwmOut* pwm3 = NULL;

struct {
  // Farben
  uint8_t stillColor[3] = { 0, 0, 255 };   // Blau
  uint8_t movingColor[3] = { 255, 0, 0 };  // Rot

  // Frequenzen
  float stillFreq = 30.0;
  float maxFreq = 80.0;
  float volume = 80.0;

  // Schwellwerte
  float threshold = 30;  // Mindestrotation für Bewegungserkennung
  float ledBlend = 400;
  float maxRotation = 1000.0;  // Maximal erwartete Rotation
  float hysteresis = 10.0;

  // Allgemein
  uint8_t brightness = 100;
} settings;

// Globale Variablen
BLEService bleService(SERVICE_UUID);
BLEUnsignedIntCharacteristic bleCharacteristic(CHARACTERISTIC_UUID,
                                               BLEWrite | BLERead);
float gxBias = 0, gyBias = 0, gzBias = 0;
unsigned long lastSensorUpdate = 0;
const unsigned long SENSOR_INTERVAL = 20;  // 50Hz Update

unsigned long getSafeMillis() {
  static unsigned long last = 0;
  unsigned long current = millis();
  if (current < last) last = 0;  // Handle 50-Tage-Überlauf
  return current;
}

void calibrateIMU() {
  const int samples = 300;
  Serial.println("Kalibriere IMU...");

  for (int i = 0; i < samples; i++) {
    float x, y, z;
    if (IMU.gyroscopeAvailable()) {
      IMU.readGyroscope(x, y, z);
      gxBias += x;
      gyBias += y;
      gzBias += z;
    }
    delay(10);
  }
  gxBias /= samples;
  gyBias /= samples;
  gzBias /= samples;

  Serial.print("Bias-Kompensation: ");
  Serial.print(gxBias);
  Serial.print(", ");
  Serial.print(gyBias);
  Serial.print(", ");
  Serial.println(gzBias);
}

bool parseBluetoothData(unsigned long xCharact) {

  byte aByteAr[3] = { 0, 0, 0 };
  byte bByte = 0;
  byte cByte = 0;
  byte dByte = 0;
  unsigned int pruefziffer = 0;
  uint8_t aByteZ = xCharact;

  for (int i = 0; i <= 2; i++) {
    aByteAr[i] = aByteZ % 10;
    aByteZ = aByteZ / 10;
  }
  bByte = (xCharact >> 8);
  cByte = (xCharact >> 16);
  dByte = (xCharact >> 24);

  // Prüfziffer berechnen
  pruefziffer = 0;
  pruefziffer += bByte * 1;
  pruefziffer += cByte * 3;
  pruefziffer += dByte * 1;

  pruefziffer = 9 - (pruefziffer % 10);

  if (aByteAr[0] == pruefziffer) {

    switch ((aByteAr[2] * 10 + aByteAr[1])) {
      default:

        break;
      case 0:

        if (bByte == 0) {
          settings.stillColor[0] = 0;
          settings.stillColor[1] = 0;
          settings.stillColor[2] = 0;
          settings.movingColor[0] = 0;
          settings.movingColor[1] = 0;
          settings.movingColor[2] = 0;
          settings.stillFreq = 0;
          settings.maxFreq = 0;
          settings.volume = 0;
        }

        break;
      case 1:

        settings.volume = bByte;
        settings.stillFreq = cByte;
        settings.maxFreq = dByte;
        Serial.println(settings.maxFreq);

        break;
      case 2:

        // settings.Beep = bByte;  // on/off

        break;
      case 3:

        settings.stillColor[0] = bByte;
        settings.stillColor[1] = cByte;
        settings.stillColor[2] = dByte;

        break;
      case 4:

        settings.movingColor[0] = bByte;
        settings.movingColor[1] = cByte;
        settings.movingColor[2] = dByte;

        break;
      case 5:

        // Brightness = bByte;
        // LEDFlashing = cByte;
        // LEDFlashFreq = dByte;

        break;
    }

    bleCharacteristic.setValue(1);
    return true;

  } else {
    bleCharacteristic.setValue(2);
    return false;
  }
}

void updateBLE() {
  BLEDevice central = BLE.central();
  if (central && central.connected()) {
    if (bleCharacteristic.written()) {
      unsigned long data = bleCharacteristic.value();
      if (data > 3) {  // Originalbedingung
        bool success = parseBluetoothData(data);
        bleCharacteristic.setValue(success ? 1
                                           : 2);  // Feedback an Client
      }
    }
  }
}

void updatePWM(float frequency, float volume) {
  float duty = constrain(volume, 0, 100) / 100.0f;

  float period = 1.0f / frequency;
  pwm->period(period);
  pwm->write(duty);
  pwm2->period(period);
  pwm2->write(duty);
  pwm3->period(period);
  pwm3->write(duty);
}

void stopPWM() {
  pwm->write(0.0f);
  pwm2->write(0.0f);
  pwm3->write(0.0f);
}

float mapFloat(float x, float in_min, float in_max, float out_min,
               float out_max) {
  return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

void setup() {
  Serial.begin(9600);

  if (!BLE.begin()) {
    Serial.println("BLE Initialisierung fehlgeschlagen!");
    while (1)
      ;
  }
  BLE.setLocalName("Klingelball");
  BLE.setAdvertisedService(bleService);
  bleService.addCharacteristic(bleCharacteristic);
  BLE.addService(bleService);
  BLE.advertise();

  // IMU Initialisierung
  if (!IMU.begin()) {
    Serial.println("IMU Initialisierung fehlgeschlagen!");
    while (1)
      ;
  }
  calibrateIMU();

  // LED Initialisierung
  leds.begin();
  leds.setBrightness(settings.brightness);
  leds.fill(leds.Color(settings.stillColor[0], settings.stillColor[1],
                       settings.stillColor[2]));
  leds.show();

  // Buzzer Initialisierung
  for (auto pin : buzzerPins) {
    pinMode(pin, OUTPUT);
  }

  pwm = new mbed::PwmOut(digitalPinToPinName(D2));
  pwm2 = new mbed::PwmOut(digitalPinToPinName(D4));
  pwm3 = new mbed::PwmOut(digitalPinToPinName(D6));

  pwm->period_us(period);
  pwm->write(0.0f);
  pwm2->period_us(period);
  pwm2->write(0.0f);
  pwm3->period_us(period);
  pwm3->write(0.0f);

  Serial.println("Ready");
}

void loop() {
  static float rotation = 0.0f;
  static unsigned long lastBeep = 0;

  updateBLE();

  if (getSafeMillis() - lastSensorUpdate >= SENSOR_INTERVAL) {
    lastSensorUpdate = getSafeMillis();

    // IMU-Daten verarbeiten
    float x, y, z;
    if (IMU.gyroscopeAvailable()) {
      IMU.readGyroscope(x, y, z);

      // Bias-Kompensation
      x -= gxBias;
      y -= gyBias;
      z -= gzBias;

      rotation = sqrt(x * x + y * y + z * z);

      // Zustandsupdate mit Hysterese
      if (rotation > (settings.threshold + settings.hysteresis)) {
        float freq =
          mapFloat(rotation, settings.threshold, settings.maxRotation,
                   settings.stillFreq, settings.maxFreq / 2);

        float duty = settings.volume / 10;

        if (freq <= settings.stillFreq) {
          freq = settings.stillFreq;
        }
        updatePWM(freq * 20, duty);

      } else if (rotation < (settings.threshold - settings.hysteresis)) {
        if (millis() - lastBeep >= 1000) {
          static unsigned long beepStart = 0;
          static bool beepInProgress = false;

          if (!beepInProgress) {
            updatePWM(settings.stillFreq * 20,
                      settings.volume / 10);
            beepStart = millis();
            beepInProgress = true;
          } else if (millis() - beepStart >= 500) {  // 500ms Ton an
            stopPWM();
            beepInProgress = false;
            lastBeep = millis();
          }
        }
      }

      // LED Update
      float blend =
        constrain((rotation - settings.threshold) / (settings.ledBlend - settings.threshold),
                  0.0, 1.0);
      uint8_t r =
        settings.stillColor[0] + (settings.movingColor[0] - settings.stillColor[0]) * blend;
      uint8_t g =
        settings.stillColor[1] + (settings.movingColor[1] - settings.stillColor[1]) * blend;
      uint8_t b =
        settings.stillColor[2] + (settings.movingColor[2] - settings.stillColor[2]) * blend;

      leds.fill(leds.Color(r, g, b));
      leds.show();
    }
  }

  //Serial.print("Rotation: ");
  //Serial.println(rotation);

  delay(10);
}