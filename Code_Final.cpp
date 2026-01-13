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
#define PIN_SD 4

RTC_DS3231 rtc;
SoftwareSerial gpsSerial(8, 9);
TinyGPSPlus gps;
BME280I2C bme;
ChainableLED leds(PIN_LED_1, PIN_LED_2, 1);

byte mode = 0, modePrev = 0;
volatile bool btnR = false, btnV = false;
bool sdOK = true;

float temp, hum, pres;
int lum;
float lat, lon;
int an, mo, jo, hr, mn, sc;

byte LUMIN_LOW = 255;
int LUMIN_HIGH = 768;
char MIN_TEMP = -10, MAX_TEMP = 60;
int PRES_MIN = 850, PRES_MAX = 1080;
byte LOG_INT = 10;
int FILE_MAX = 4096;

unsigned long lastLog = 0;
byte ledM = 0;

// LED
void gererLED(byte r1, byte g1, byte b1, byte r2, byte g2, byte b2, int d1, int d2) {
  unsigned long t = millis() % (d1 + d2);
  if (t < d1) leds.setColorRGB(0, r1, g1, b1);
  else leds.setColorRGB(0, r2, g2, b2);
}

void setLED(byte m) {
  ledM = m;
  if (m == 1) leds.setColorRGB(0, 0, 255, 0);
  else if (m == 2) leds.setColorRGB(0, 255, 255, 0);
  else if (m == 3) leds.setColorRGB(0, 0, 0, 255);
  else if (m == 4) leds.setColorRGB(0, 255, 165, 0);
  else leds.setColorRGB(0, 0, 0, 0);
}

void updateLED() {
  if (ledM == 5) gererLED(255, 0, 0, 0, 0, 255, 500, 500);
  else if (ledM == 6) gererLED(255, 0, 0, 255, 255, 0, 500, 500);
  else if (ledM == 7) gererLED(255, 0, 0, 0, 255, 0, 500, 500);
  else if (ledM == 8) gererLED(255, 0, 0, 0, 255, 0, 333, 667);
  else if (ledM == 10) gererLED(255, 0, 0, 255, 255, 255, 333, 667);
}

// ISR
void ISR_R() { btnR = true; }
void ISR_V() { btnV = true; }

// Capteurs
void updateTime() {
  if (!rtc.begin()) { ledM = 5; return; }
  DateTime now = rtc.now();
  an = now.year(); mo = now.month(); jo = now.day();
  hr = now.hour(); mn = now.minute(); sc = now.second();
}

void updateSensors() {
  float t, h, p;
  bme.read(p, t, h, BME280::TempUnit_Celsius, BME280::PresUnit_Pa);
  if (isnan(t) || isnan(h) || isnan(p)) { ledM = 7; return; }
  if (t < -40 || t > 85 || h < 0 || h > 100) { ledM = 8; return; }
  temp = t; hum = h; pres = p;
  lum = analogRead(PIN_LUM);
}

void getGPS() {
  unsigned long start = millis();
  while (millis() - start < 5000) {
    while (gpsSerial.available()) gps.encode(gpsSerial.read());
    if (gps.location.isValid()) {
      lat = gps.location.lat();
      lon = gps.location.lng();
      return;
    }
  }
  lat = lon = 0;
  ledM = 6;
}

// GPS détaillé pour mode maintenance
void afficherGPS() {
  while (gpsSerial.available() > 0) {
    gps.encode(gpsSerial.read());
    if (gps.location.isUpdated()) {
      Serial.print(F("L:")); Serial.print(gps.location.lat(), 6);
      Serial.print(F(" ")); Serial.println(gps.location.lng(), 6);
      if (gps.altitude.isValid()) {
        Serial.print(F("A:")); Serial.println(gps.altitude.meters());
      }
      if (gps.speed.isValid()) {
        Serial.print(F("V:")); Serial.println(gps.speed.kmph());
      }
      if (gps.satellites.isValid()) {
        Serial.print(F("S:")); Serial.println(gps.satellites.value());
      }
    }
  }
}

// Sauvegarde
void saveData() {
  if (!sdOK) { ledM = 10; return; }
  char fn[13];
  int idx = 1;
  File f;

  while (true) {
    sprintf(fn, "D_%04d.LOG", idx);
    if (!SD.exists(fn)) {
      f = SD.open(fn, FILE_WRITE);
      if (f) f.println(F("an,mo,jo,hr,mn,sc,temp,hum,pres,lum,lat,lon"));
      break;
    }
    f = SD.open(fn, FILE_WRITE);
    if (f && f.size() < FILE_MAX) break;
    if (f) f.close();
    idx++;
    if (idx > 9999) { ledM = 10; return; }
  }

  if (!f) { ledM = 10; return; }

  f.print(an); f.print(',');
  f.print(mo); f.print(',');
  f.print(jo); f.print(',');
  f.print(hr); f.print(',');
  f.print(mn); f.print(',');
  f.print(sc); f.print(',');
  f.print(temp, 2); f.print(',');
  f.print(hum, 1); f.print(',');
  f.print(pres, 2); f.print(',');
  f.print(lum); f.print(',');
  f.print(lat, 6); f.print(',');
  f.println(lon, 6);
  f.close();
}

void checkButtons() {
  if (btnR) {
    btnR = false;
    if (digitalRead(PIN_BTN_R) == LOW) {
      unsigned long t0 = millis();
      while (digitalRead(PIN_BTN_R) == LOW && millis() - t0 < 5000);
      if (millis() - t0 >= 5000) {
        if (mode == 0 || mode == 2) {
          modePrev = mode; mode = 3; setLED(4);
        } else if (mode == 3) {
          mode = modePrev; setLED(mode == 0 ? 1 : 3);
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
        if (mode == 0) { mode = 2; setLED(3); }
        else if (mode == 2) { mode = 0; setLED(1); }
      }
    }
  }
}

// Modes
void modeStd() {
  setLED(1);
  updateTime();
  updateSensors();
  getGPS();
  saveData();
}

void modeCfg() {
  setLED(2);
  static unsigned long lastAct = millis();
  if (Serial.available()) {
    lastAct = millis();
    while (Serial.available()) Serial.read();
  }
  if (millis() - lastAct > 1800000) mode = 0;
}

void modeEco() {
  setLED(3);
  updateTime();
  updateSensors();
  static bool gpsOn = false;
  gpsOn = !gpsOn;
  if (gpsOn) getGPS();
  saveData();
}

void modeMaint() {
  setLED(4);
  updateSensors();
  Serial.print(F("T:")); Serial.print(temp);
  Serial.print(F(" H:")); Serial.print(hum);
  Serial.print(F(" P:")); Serial.print(pres);
  Serial.print(F(" L:")); Serial.println(lum);
  afficherGPS();
  delay(100);
}

void setup() {
  Serial.begin(9600); 
  gpsSerial.begin(9600);

  pinMode(PIN_BTN_R, INPUT_PULLUP);
  pinMode(PIN_BTN_V, INPUT_PULLUP);
  pinMode(PIN_LUM, INPUT);

  attachInterrupt(digitalPinToInterrupt(PIN_BTN_R), ISR_R, FALLING);
  attachInterrupt(digitalPinToInterrupt(PIN_BTN_V), ISR_V, FALLING);

  mode = (digitalRead(PIN_BTN_R) == LOW) ? 1 : 0;
  setLED(mode == 1 ? 2 : 1);

  if (!rtc.begin()) ledM = 5;
  if (!bme.begin()) ledM = 7;
  sdOK = SD.begin(PIN_SD);
  if (!sdOK) ledM = 10;

  // Test GPS simplifié
  Serial.print(F("GPS..."));
  unsigned long t = millis();
  while (millis() - t < 5000) {
    while (gpsSerial.available()) {
      gps.encode(gpsSerial.read());
      if (gps.location.isValid()) {
        Serial.println(F("OK"));
        Serial.print(gps.location.lat(), 6);
        Serial.print(F(","));
        Serial.println(gps.location.lng(), 6);
        goto gps_ok;
      }
    }
  }
  Serial.println(F("?"));
  gps_ok:
  Serial.println(F("Pret"));
}

void loop() {
  checkButtons();
  updateLED();
  unsigned long now = millis();

  if (mode == 0 && now - lastLog >= LOG_INT * 60000UL) {
    lastLog = now; modeStd();
  } else if (mode == 1) {
    modeCfg();
  } else if (mode == 2 && now - lastLog >= LOG_INT * 2 * 60000UL) {
    lastLog = now; modeEco();
  } else if (mode == 3) {
    modeMaint();
  }

  delay(10);
}
