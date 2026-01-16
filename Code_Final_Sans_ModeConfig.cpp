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
File fichier;

// Modes
uint8_t mode = 0, modePrev = 0, modeLED = 0;
volatile bool btnR = false, btnV = false;
bool sdOK = false, rtcOK = false, bmeOK = false;

// Coordonnées GPS
double latitude = NAN, longitude = NAN;

// Repère temporel (Date & heure, minute, seconde)
int Y;
byte M, D, h, m, s;

// Valeur capteur
float temp, hum, press;

// Pour mode éco
static bool gpsOn = false;

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

// Affichage capteurs
void UseCapteurs() {
  updateTime();
  updateBME();
  
  Serial.print(F("Temperature: ")); Serial.print(temp); Serial.println(F(" C"));
  Serial.print(F("Humidite: ")); Serial.print(hum); Serial.println(F(" %"));
  Serial.print(F("Pression: ")); Serial.print(press); Serial.println(F(" Pa"));
  Serial.print(F("Luminosite: ")); Serial.print(getLum()); Serial.println(F(" %"));
}

// ISR
void ISR_R() { btnR = true; }
void ISR_V() { btnV = true; }

// RTC
void updateTime() {
  if (!rtcOK) {
    Serial.println(F("RTC non disponible"));
    return;
  }
  DateTime now = rtc.now();
  Y = now.year(); M = now.month(); D = now.day();
  h = now.hour(); m = now.minute(); s = now.second();
  
  Serial.print(F("Heure : "));
  if (h < 10) Serial.print('0');
  Serial.print(h); Serial.print(':');
  if (m < 10) Serial.print('0');
  Serial.print(m); Serial.print(':');
  if (s < 10) Serial.print('0');
  Serial.println(s);
}

// Capteurs
void updateBME() {
  if (bmeOK) {
    bme.read(press, temp, hum);
  }
}

float getLum() {
  return analogRead(PIN_LUM) * 100.0 / 1023.0;
}

// GPS
void getGPS() {
  unsigned long t0 = millis();
  latitude = NAN;
  longitude = NAN;
  
  while (millis() - t0 < 5000) {
    while (gpsSerial.available() > 0) {
      char c = gpsSerial.read();
      gps.encode(c);
      
      if (gps.location.isUpdated() && gps.location.isValid()) {
        latitude = gps.location.lat();
        longitude = gps.location.lng();
        Serial.print(F("GPS Position: ")); 
        Serial.print(latitude, 6);
        Serial.print(F(", ")); 
        Serial.println(longitude, 6);
        return;
      }
    }
  }
  
  Serial.println(F("GPS Timeout - Pas de signal"));
}

// SD
void saveSD() {
  if (!sdOK) {
    Serial.println(F("ERREUR: Carte SD absente"));
    modeLED = 10;
    return;
  }
  
  char nomFichier[20];
  sprintf(nomFichier, "%04d%02d%02d.LOG", Y, M, D);
  
  fichier = SD.open(nomFichier, FILE_WRITE);

  if (!fichier) {
    Serial.println(F("ERREUR ouverture fichier"));
    modeLED = 10;
    return;
  }

  fichier.print(F("TIME: "));
  if (h < 10) fichier.print('0');
  fichier.print(h); fichier.print(':');
  if (m < 10) fichier.print('0');
  fichier.print(m); fichier.print(':');
  if (s < 10) fichier.print('0');
  fichier.println(s);
  
  fichier.print(F("TEMP: ")); fichier.print(temp); fichier.println(F(" C"));
  fichier.print(F("HUM: ")); fichier.print(hum); fichier.println(F(" %"));
  fichier.print(F("PRESS: ")); fichier.print(press); fichier.println(F(" Pa"));
  fichier.print(F("LUM: ")); fichier.print(getLum()); fichier.println(F(" %"));

  if (!isnan(latitude) && !isnan(longitude)) {
    fichier.print(F("LAT: ")); fichier.println(latitude, 5);
    fichier.print(F("LONG: ")); fichier.println(longitude, 5);
  } else {
    fichier.println(F("GPS INVALIDE"));
  }

  fichier.println(F("====================="));
  fichier.close();

  Serial.println(F("Écriture finie"));
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
  getGPS();
  UseCapteurs();
  saveSD();
  delay(10000);
}

void modeEco() {
  setLED(3);
  Serial.println(F("\n===== MODE ECONOMIQUE ====="));
  
  gpsOn = !gpsOn;
  if (gpsOn) {
    getGPS();
  }

  UseCapteurs();
  saveSD();
  delay(20000);  
}

void modeMaint() {
  setLED(4);
  Serial.println(F("\n===== MODE MAINTENANCE ====="));
  UseCapteurs();
  delay(2000);
}

void setup() {
  Serial.begin(9600);
  Wire.begin();
  gpsSerial.begin(9600);
  
  pinMode(PIN_BTN_R, INPUT_PULLUP);
  pinMode(PIN_BTN_V, INPUT_PULLUP);
  pinMode(PIN_LUM, INPUT);
  pinMode(PIN_CS, OUTPUT);
  
  // RTC
  Serial.print(F("RTC..."));
  if (rtc.begin()) {
    rtcOK = true;
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
      bme = bme2; 
      bmeOK = true;
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
    Serial.println(F("ERREUR"));
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
