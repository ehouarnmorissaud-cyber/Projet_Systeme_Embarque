#include <Wire.h>
#include <RTClib.h>
#include <SoftwareSerial.h>
#include <TinyGPS++.h>
#include <SPI.h>
#include <SD.h>
#include <ChainableLED.h>
#include <BME280I2C.h>

#define PIN_LED_1 5
#define PIN_LED_2 6
#define PIN_BTN_V 2
#define PIN_BTN_R 3
#define PIN_LUM A0
#define PIN_CS 4

RTC_DS3231 rtc;
SoftwareSerial gpsSerial(8, 9);
TinyGPSPlus gps;
BME280I2C bme;
ChainableLED leds(PIN_LED_1, PIN_LED_2, 1);

uint8_t mode = 0, modePrev = 0, modeLED = 0;
volatile bool btnR = false, btnV = false;
bool sdOK = false, rtcOK = false, bmeOK = false;

float temp = 20.0, hum = 50.0, press = 1013.25, lat = 0, lon = 0;
uint16_t lum = 0;
int Y = 2026;
byte M = 1, D = 1, h = 0, m = 0, s = 0;

// LED
void gererLED(byte r1, byte g1, byte b1, byte r2, byte g2, byte b2, int d1, int d2) {
  unsigned long t = millis() % (d1 + d2);
  if (t < d1) leds.setColorRGB(0, r1, g1, b1);
  else leds.setColorRGB(0, r2, g2, b2);
}

void setLED(byte md) {
  modeLED = md;
  if (md == 1) leds.setColorRGB(0, 0, 255, 0);
  else if (md == 3) leds.setColorRGB(0, 0, 0, 255);
  else if (md == 4) leds.setColorRGB(0, 255, 165, 0);
  else leds.setColorRGB(0, 0, 0, 0);
}

void updateLED() {
  if (modeLED == 5) gererLED(255, 0, 0, 0, 0, 255, 500, 500);
  else if (modeLED == 6) gererLED(255, 0, 0, 255, 255, 0, 500, 500);
  else if (modeLED == 7) gererLED(255, 0, 0, 0, 255, 0, 500, 500);
  else if (modeLED == 9) gererLED(255, 0, 0, 255, 255, 255, 500, 500);
  else if (modeLED == 10) gererLED(255, 0, 0, 255, 255, 255, 333, 667);
}

// ISR
void ISR_R() { btnR = true; }
void ISR_V() { btnV = true; }

// RTC
void updateTime() {
  if (!rtcOK) return;
  DateTime now = rtc.now();
  Y = now.year(); M = now.month(); D = now.day();
  h = now.hour(); m = now.minute(); s = now.second();
  
  Serial.print(F("Heure: "));
  if (h < 10) Serial.print('0');
  Serial.print(h); Serial.print(':');
  if (m < 10) Serial.print('0');
  Serial.print(m); Serial.print(':');
  if (s < 10) Serial.print('0');
  Serial.println(s);
}

// Capteurs
void updateSensors() {
  lum = analogRead(PIN_LUM);
  if (!bmeOK) return;
  
  float t, hh, p;
  bme.read(p, t, hh, BME280::TempUnit_Celsius, BME280::PresUnit_Pa);
  
  if (!isnan(t) && t >= -40 && t <= 85) temp = t;
  if (!isnan(hh) && hh >= 0 && hh <= 100) hum = hh;
  if (!isnan(p)) press = p / 100.0;
}

// GPS
void getGPS() {
  unsigned long t0 = millis();
  while (millis() - t0 < 5000) {
    while (gpsSerial.available()) {
      if (gps.encode(gpsSerial.read()) && gps.location.isValid()) {
        lat = gps.location.lat();
        lon = gps.location.lng();
        Serial.print(F("GPS Position: ")); Serial.print(lat, 6);
        Serial.print(F(", ")); Serial.println(lon, 6);
        return;
      }
    }
  }
  Serial.println(F("GPS Timeout - Pas de signal"));
  lat = lon = 0;
}

// SD
void saveSD() {
  if (!sdOK) { Serial.println(F("Carte SD absente")); return; }
  
  char nom[20];
  sprintf(nom, "%04d%02d%02d.LOG", Y, M, D);
  
  File f = SD.open(nom, FILE_WRITE);
  if (!f) {
    Serial.println(F("Erreur ouverture fichier"));
    if (SD.begin(PIN_CS)) f = SD.open(nom, FILE_WRITE);
    if (!f) { modeLED = 9; return; }
  }
  
  f.print(Y); f.print('-');
  if (M < 10) f.print('0'); f.print(M); f.print('-');
  if (D < 10) f.print('0'); f.print(D); f.print(' ');
  if (h < 10) f.print('0'); f.print(h); f.print(':');
  if (m < 10) f.print('0'); f.print(m); f.print(':');
  if (s < 10) f.print('0'); f.print(s);
  
  f.print(F("|T:")); f.print(temp, 1);
  f.print(F("|H:")); f.print(hum, 1);
  f.print(F("|P:")); f.print(press, 1);
  f.print(F("|L:")); f.print(lum);
  
  if (lat != 0 || lon != 0) {
    f.print(F("|GPS:")); f.print(lat, 6);
    f.print(','); f.print(lon, 6);
  }
  
  f.println();
  f.close();
  Serial.print(F("Sauvegarde dans: ")); Serial.println(nom);
}

// Boutons
void checkButtons() {
  if (btnR) {
    btnR = false;
    if (digitalRead(PIN_BTN_R) == LOW) {
      unsigned long t0 = millis();
      while (digitalRead(PIN_BTN_R) == LOW && millis() - t0 < 5000);
      if (millis() - t0 >= 5000) {
        if (mode == 0 || mode == 2) {
          modePrev = mode; mode = 3; setLED(4);
          Serial.println(F(">>> MODE MAINTENANCE"));
        } else if (mode == 3) {
          mode = modePrev; setLED(mode == 0 ? 1 : 3);
          Serial.println(mode == 0 ? F(">>> MODE STANDARD") : F(">>> MODE ECONOMIQUE"));
        }
      }
    }
  }
  
  if (btnV) {
    btnV = false;
    if (digitalRead(PIN_BTN_V) == LOW) {
      unsigned long t0 = millis();
      while (digitalRead(PIN_BTN_V) == LOW && millis() - t0 < 5000);
      if (millis() - t0 >= 5000) {
        if (mode == 0) { mode = 2; setLED(3); Serial.println(F(">>> MODE ECONOMIQUE")); }
        else if (mode == 2) { mode = 0; setLED(1); Serial.println(F(">>> MODE STANDARD")); }
      }
    }
  }
}

// Modes
void modeStandard() {
  setLED(1);
  Serial.println(F("\n===== MODE STANDARD ====="));
  updateTime();
  updateSensors();
  getGPS();
  saveSD();
  
  Serial.print(F("Temperature: ")); Serial.print(temp, 1); Serial.println(F(" C"));
  Serial.print(F("Humidite: ")); Serial.print(hum, 1); Serial.println(F(" %"));
  Serial.print(F("Pression: ")); Serial.print(press, 1); Serial.println(F(" hPa"));
  Serial.print(F("Luminosite: ")); Serial.println(lum);
  
  delay(10000);
}

void modeEco() {
  setLED(3);
  Serial.println(F("\n===== MODE ECONOMIQUE ====="));
  updateTime();
  updateSensors();
  
  // GPS une mesure sur deux
  static bool gpsOn = false;
  gpsOn = !gpsOn;
  if (gpsOn) {
    getGPS();
  } else {
    Serial.println(F("GPS desactive (economie)"));
    lat = 0;
    lon = 0;
  }
  
  saveSD();
  Serial.print(F("Temperature: ")); Serial.print(temp, 1); Serial.println(F(" C"));
  Serial.print(F("Humidite: ")); Serial.print(hum, 1); Serial.println(F(" %"));
  
  delay(20000);  
}

void modeMaint() {
  setLED(4);
  updateSensors();
  Serial.println(F("\n===== MODE MAINTENANCE ====="));
  Serial.print(F("Temperature: ")); Serial.print(temp, 1); Serial.println(F(" C"));
  Serial.print(F("Humidite: ")); Serial.print(hum, 1); Serial.println(F(" %"));
  Serial.print(F("Pression: ")); Serial.print(press, 1); Serial.println(F(" hPa"));
  Serial.print(F("Luminosite: ")); Serial.println(lum);
  delay(2000);
}

void setup() {
  Serial.begin(9600);
  gpsSerial.begin(9600);
  
  pinMode(PIN_BTN_R, INPUT_PULLUP);
  pinMode(PIN_BTN_V, INPUT_PULLUP);
  pinMode(PIN_LUM, INPUT);
  pinMode(PIN_CS, OUTPUT);
  
  Wire.begin();
  Serial.println(F("\n=METEO=\n"));
  
  // RTC
  Serial.print(F("RTC..."));
  if (rtc.begin()) {
    rtcOK = true;
    rtc.disable32K();
    rtc.clearAlarm(1);
    rtc.clearAlarm(2);
    rtc.writeSqwPinMode(DS3231_OFF);
    rtc.adjust(DateTime(F(__DATE__), F(__TIME__)));
    Serial.println(F("OK"));
  } else {
    Serial.println(F("ERR"));
    modeLED = 5;
  }
  
  // BME280
  Serial.print(F("BME..."));
  if (bme.begin()) {
    bmeOK = true;
    Serial.println(F("OK"));
  } else {
    BME280I2C::Settings st(BME280::OSR_X1, BME280::OSR_X1, BME280::OSR_X1,
      BME280::Mode_Forced, BME280::StandbyTime_1000ms,
      BME280::Filter_Off, BME280::SpiEnable_False, 0x77);
    BME280I2C bme2(st);
    if (bme2.begin()) {
      bme = bme2; bmeOK = true;
      Serial.println(F("OK(77)"));
    } else {
      Serial.println(F("ERR"));
      modeLED = 7;
    }
  }
  
  // SD
  Serial.print(F("SD..."));
  digitalWrite(PIN_CS, HIGH);
  delay(50);
  
  if (SD.begin(PIN_CS)) {
    sdOK = true;
    Serial.println(F("OK"));
  } else {
    Serial.println(F("ERR"));
    modeLED = 10;
  }
  
  Serial.println(F("=PRET=\n"));
  mode = 0;
  setLED(1);
  
  attachInterrupt(digitalPinToInterrupt(PIN_BTN_R), ISR_R, FALLING);
  attachInterrupt(digitalPinToInterrupt(PIN_BTN_V), ISR_V, FALLING);
}

void loop() {
  checkButtons();
  updateLED();
  
  if (mode == 0) modeStandard();
  else if (mode == 2) modeEco();
  else if (mode == 3) modeMaint();
}
