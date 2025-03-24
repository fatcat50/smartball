#include <Adafruit_NeoPixel.h>
#include <ArduinoBLE.h>
#include <Arduino_LSM9DS1.h>

// BLE Einstellungen
#define SERVICE_UUID "4fafc201-1fb5-459e-8fcc-c5c9c331914b"
#define CHARACTERISTIC_UUID "beb5483e-36e1-4688-b7f5-ea07361b26a8"

// LED Einstellungen
#define LED_PIN A0
#define NUM_LEDS 60
Adafruit_NeoPixel leds(NUM_LEDS, LED_PIN, NEO_GRB + NEO_KHZ800);

// Buzzer Pins
const uint8_t buzzerPins[] = { D2, D4, D6 };

// Einstellungen
struct {
  // Farben
  uint8_t stillColor[3] = { 0, 0, 255 };   // Blau
  uint8_t movingColor[3] = { 255, 0, 0 };  // Rot

  // Frequenzen
  float stillFreq = 500.0;  // 500Hz im Stand
  float maxFreq = 3000.0;   // 3000Hz bei Maximalgeschwindigkeit

  // Schwellwerte
  float threshold = 50.0;     // Mindestrotation für Bewegungserkennung
  float maxRotation = 200.0;  // Maximal erwartete Rotation
  float hysteresis = 5.0;     // Hystereseband

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

/*void parseBluetoothData(unsigned long data) {
    // Datenformat: 0xAARRGGBB (AA=Adressbyte, RRGGBB=Farbwert)
    settings.movingColor[0] = (data >> 16) & 0xFF;  // Rot
    settings.movingColor[1] = (data >> 8) & 0xFF;   // Grün
    settings.movingColor[2] = data & 0xFF;          // Blau
    Serial.print("Neue Farbe: R");
    Serial.print(settings.movingColor[0]);
    Serial.print(" G");
    Serial.print(settings.movingColor[1]);
    Serial.print(" B");
    Serial.println(settings.movingColor[2]);
}*/

void parseBluetoothData(unsigned long data) {
  uint8_t addr = (data >> 24) & 0xFF;

  if (addr == 0x01) {
    // moving color
    settings.movingColor[0] = (data >> 16) & 0xFF;
    settings.movingColor[1] = (data >> 8) & 0xFF;
    settings.movingColor[2] = data & 0xFF;
  } else if (addr == 0x02) {
    // still color
    settings.stillColor[0] = (data >> 16) & 0xFF;
    settings.stillColor[1] = (data >> 8) & 0xFF;
    settings.stillColor[2] = data & 0xFF;
  }
}

void updateBLE() {
  BLEDevice central = BLE.central();
  if (central && central.connected()) {
    if (bleCharacteristic.written()) {
      parseBluetoothData(bleCharacteristic.value());
    }
  }
}

void setup() {
  Serial.begin(9600);
  // while(!Serial);

  // BLE Initialisierung
  if (!BLE.begin()) {
    Serial.println("BLE Initialisierung fehlgeschlagen!");
    while (1)
      ;
  }
  BLE.setLocalName("SmartBall");
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

  Serial.println("System bereit");
}

void loop() {
  static bool isMoving = false;
  static unsigned long lastBeep = 0;
  static bool beepState = false;

  updateBLE();

  if (getSafeMillis() - lastSensorUpdate >= SENSOR_INTERVAL) {
    lastSensorUpdate = getSafeMillis();

    float x, y, z;
    if (IMU.gyroscopeAvailable()) {
      IMU.readGyroscope(x, y, z);

      // Bias-Kompensation
      x -= gxBias;
      y -= gyBias;
      z -= gzBias;

      float rotation = sqrt(x * x + y * y + z * z);

      // Zustandsupdate mit Hysterese
      if (rotation > (settings.threshold + settings.hysteresis)) {
        isMoving = true;
      } else if (rotation < (settings.threshold - settings.hysteresis)) {
        isMoving = false;
      }

      // LED Update
      float blend =
        constrain((rotation - settings.threshold) / (settings.maxRotation - settings.threshold),
                  0.0, 1.0);
      uint8_t r =
        settings.stillColor[0] + (settings.movingColor[0] - settings.stillColor[0]) * blend;
      uint8_t g =
        settings.stillColor[1] + (settings.movingColor[1] - settings.stillColor[1]) * blend;
      uint8_t b =
        settings.stillColor[2] + (settings.movingColor[2] - settings.stillColor[2]) * blend;

      leds.fill(leds.Color(r, g, b));
      leds.show();

      // Buzzer Control
      if (isMoving) {
        float freq = constrain(
          map(rotation, settings.threshold, settings.maxRotation,
              settings.stillFreq, settings.maxFreq),
          settings.stillFreq, settings.maxFreq);

        for (auto pin : buzzerPins) {
          // tone(pin, freq);
          Serial.println(freq);
        }
      } else {
        for (auto pin : buzzerPins) {
          noTone(pin);
        }

        // 1s Piepintervall
        if (getSafeMillis() - lastBeep > 1000) {
          beepState = !beepState;
          for (auto pin : buzzerPins) {
            // digitalWrite(pin, beepState);
            Serial.println("Beep");
          }
          lastBeep = getSafeMillis();
        }
      }

      // Debug-Ausgabe
      Serial.print("Rotation: ");
      Serial.print(rotation);
      Serial.print(" rad/s | Zustand: ");
      Serial.println(isMoving ? "Bewegung" : "Ruhe");
    }
  }
}