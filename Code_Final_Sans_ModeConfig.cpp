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
File fichier ;

uint8_t mode = 0, modePrev = 0, modeLED = 0;
volatile bool btnR = false, btnV = false;
bool sdOK = false, rtcOK = false, bmeOK = false;

long latitude = 0, longitude = 0;
uint16_t lum = 0;
int Y;
byte M , D , h , m , s;

//Valeur capteur
float temp, hum, press;

//Pour mode éco
static bool gpsOn = false;

//LED
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
  if (!rtcOK) {
    Serial.println(F("RTC non disponible"));
    return;
  }
  DateTime now = rtc.now();
  Y = now.year(); M = now.month(); D = now.day(); //jour précis
  h = now.hour(); m = now.minute(); s = now.second(); //heure précise
  
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
    bme.read(press, temp, hum); // ordre IMPORTANT : pression, température, humidité
  }
}

float getTemp(){
  return temp;
}

float getHumid(){
  return hum;
}

float getPress(){
  return press;
}

float getLum(){
  float l = analogRead(PIN_LUM);
  return l;
}


// GPS
void getGPS() {
  unsigned long t0 = millis();
  while (millis() - t0 < 5000) {
    while (gpsSerial.available()) {
      if (gps.encode(gpsSerial.read())) {
        latitude = gps.location.lat();
        longitude = gps.location.lng();
        Serial.print(F("GPS Position: ")); Serial.print(latitude, 6);
        Serial.print(F(", ")); Serial.println(longitude, 6);
        return;
      }
    }
  }
  Serial.println(F("GPS Timeout - Pas de signal"));
  latitude = longitude = 0;
}

// SD
void saveSD() {
  if (sdOK) {


    char nomFichier[25];
    sprintf(
      nomFichier,
      "%04d%02d%02d_%02d%02d%02d.LOG",
      Y,
      M,
      D,
      h,
      m,
      s
    );

    fichier = SD.open(nomFichier, FILE_WRITE);

    if (fichier){
      fichier.print("TEMP : "); fichier.println(getTemp());
      fichier.print("HUM : "); fichier.println(getHumid());
      fichier.print("PRESS : "); fichier.println(getPress());
      fichier.print("LUM : "); fichier.println(getLum());
      fichier.print("CO : "); fichier.println(longitude, latitude);
      fichier.close();
    }
  }
  else
  {
    Serial.println(F("ERREUR: Carte SD absente"));
    modeLED = 10;
  }
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
  getGPS();
  updateBME();
  saveSD();
  
  Serial.print(F("Temperature: ")); Serial.print(getTemp()); Serial.println(F(" C"));
  Serial.print(F("Humidite: ")); Serial.print(getHumid()); Serial.println(F(" %"));
  Serial.print(F("Pression: ")); Serial.print(getPress()); Serial.println(F(" Pa"));
  Serial.print(F("Luminosite: ")); Serial.println(getLum());
  
  delay(10000);
}

void modeEco() {
  setLED(3);
  Serial.println(F("\n===== MODE ECONOMIQUE ====="));
  updateTime();
  
  // GPS une mesure sur deux
  gpsOn = !gpsOn;
  if (gpsOn) {
    getGPS();
  }
  updateBME();
  saveSD();
  Serial.print(F("Temperature: ")); Serial.print(getTemp()); Serial.println(F(" C"));
  Serial.print(F("Humidite: ")); Serial.print(getHumid()); Serial.println(F(" %"));
  Serial.print(F("Pression: ")); Serial.print(getPress()); Serial.println(F(" Pa"));
  Serial.print(F("Luminosite: ")); Serial.println(getLum());
  
  delay(20000);  
}

void modeMaint() {
  setLED(4);
  updateBME();
  Serial.println(F("\n===== MODE MAINTENANCE ====="));
  Serial.print(F("Temperature: ")); Serial.print(getTemp()); Serial.println(F(" C"));
  Serial.print(F("Humidite: ")); Serial.print(getHumid()); Serial.println(F(" %"));
  Serial.print(F("Pression: ")); Serial.print(getPress()); Serial.println(F(" Pa"));
  Serial.print(F("Luminosite: ")); Serial.println(getLum());
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
