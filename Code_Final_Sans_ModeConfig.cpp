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
#define PIN_BOUTON_VERT 2
#define PIN_BOUTON_ROUGE 3
#define PIN_LUMIERE A0
#define PIN_SD_CS 4

RTC_DS3231 rtc;
SoftwareSerial gpsSerial(8, 9);
TinyGPSPlus gps;
BME280I2C bme;
ChainableLED leds(PIN_LED_1, PIN_LED_2, 1);

uint8_t mode_actuel = 0;
uint8_t mode_precedent = 0;
uint8_t modeLED = 0;

volatile bool bouton_rouge_presse = false;
volatile bool bouton_vert_presse = false;

bool carteSD_presente = true;
bool rtc_ok = false;
bool bme_ok = false;

// Capteurs
float temperature_air = 20.0;
float humidite = 50.0;
float pression_atmospherique = 1013.25;
uint16_t luminosite = 0;

// GPS
float latitude = 0;
float longitude = 0;

// RTC
int annee = 2026;
byte mois = 1, jour = 1, heure = 0, minute = 0, seconde = 0;


// ============================================================================
// LED
// ============================================================================

void gererLED(byte r1, byte g1, byte b1, byte r2, byte g2, byte b2, int d1, int d2) {
  unsigned long t = millis() % (d1 + d2);
  if (t < d1) leds.setColorRGB(0, r1, g1, b1);
  else leds.setColorRGB(0, r2, g2, b2);
}

void setLED(byte mode) {
  modeLED = mode;
  if (mode == 1) leds.setColorRGB(0, 0, 255, 0);        // VERT
  else if (mode == 3) leds.setColorRGB(0, 0, 0, 255);   // BLEU
  else if (mode == 4) leds.setColorRGB(0, 255, 165, 0); // ORANGE
  else leds.setColorRGB(0, 0, 0, 0);
}

void updateLED() {
  if (modeLED == 5) gererLED(255, 0, 0, 0, 0, 255, 500, 500);
  else if (modeLED == 6) gererLED(255, 0, 0, 255, 255, 0, 500, 500);
  else if (modeLED == 7) gererLED(255, 0, 0, 0, 255, 0, 500, 500);
  else if (modeLED == 8) gererLED(255, 0, 0, 0, 255, 0, 333, 667);
  else if (modeLED == 9) gererLED(255, 0, 0, 255, 255, 255, 500, 500);
  else if (modeLED == 10) gererLED(255, 0, 0, 255, 255, 255, 333, 667);
}

// ============================================================================
// ISR
// ============================================================================
void ISR_bouton_rouge() { bouton_rouge_presse = true; }
void ISR_bouton_vert() { bouton_vert_presse = true; }

// ============================================================================
// CAPTEURS
// ============================================================================

void updateDateTime() {
  if (!rtc_ok) {
    Serial.println(F("RTC non disponible"));
    // Utiliser millis() comme horloge de secours
    unsigned long sec = millis() / 1000;
    seconde = sec % 60;
    minute = (sec / 60) % 60;
    heure = (sec / 3600) % 24;
    return;
  }
  
  DateTime now = rtc.now();
  annee = now.year();
  mois = now.month();
  jour = now.day();
  heure = now.hour();
  minute = now.minute();
  seconde = now.second();
  
  // Affichage de l'horloge
  Serial.print(F("Date: "));
  if (jour < 10) Serial.print('0');
  Serial.print(jour);
  Serial.print('/');
  if (mois < 10) Serial.print('0');
  Serial.print(mois);
  Serial.print('/');
  Serial.print(annee);
  
  Serial.print(F(" Heure: "));
  if (heure < 10) Serial.print('0');
  Serial.print(heure);
  Serial.print(':');
  if (minute < 10) Serial.print('0');
  Serial.print(minute);
  Serial.print(':');
  if (seconde < 10) Serial.print('0');
  Serial.println(seconde);
}

void updateSensors() {
  // Luminosité (toujours disponible)
  luminosite = analogRead(PIN_LUMIERE);
  
  if (!bme_ok) {
    Serial.println(F("BME280 non disponible - valeurs par defaut"));
    return;
  }
  
  float t, h, p;
  bme.read(p, t, h, BME280::TempUnit_Celsius, BME280::PresUnit_Pa);
  
  if (isnan(t) || isnan(h) || isnan(p)) {
    Serial.println(F("Erreur lecture BME280"));
    return;
  }
  
  if (t < -40 || t > 85 || h < 0 || h > 100) {
    Serial.println(F("Donnees BME280 incoherentes"));
    return;
  }
  
  temperature_air = t;
  humidite = h;
  pression_atmospherique = p / 100.0; // Pa vers hPa
}

void getGPS() {
  unsigned long debut = millis();
  while (millis() - debut < 5000) {
    while (gpsSerial.available()) gps.encode(gpsSerial.read());
    if (gps.location.isValid()) {
      latitude = gps.location.lat();
      longitude = gps.location.lng();
      Serial.print(F("GPS: "));
      Serial.print(latitude, 6);
      Serial.print(F(", "));
      Serial.println(longitude, 6);
      return;
    }
  }
  Serial.println(F("GPS non disponible"));
  latitude = 0;
  longitude = 0;
}

void saveSD() {
  if (!carteSD_presente) {
    Serial.println(F("Carte SD absente"));
    modeLED = 10;
    return;
  }
  
  // Nom de fichier avec date complète
  char nomFichier[25];
  sprintf(nomFichier, "%04d%02d%02d.LOG", annee, mois, jour);
  
  File fichier = SD.open(nomFichier, FILE_WRITE);
  
  if (!fichier) {
    Serial.print(F("Erreur ouverture fichier: "));
    Serial.println(nomFichier);
    modeLED = 9;
    return;
  }
  
  // Enregistrement avec horodatage
  fichier.print(annee); fichier.print('-');
  if (mois < 10) fichier.print('0');
  fichier.print(mois); fichier.print('-');
  if (jour < 10) fichier.print('0');
  fichier.print(jour); fichier.print(' ');
  
  if (heure < 10) fichier.print('0');
  fichier.print(heure); fichier.print(':');
  if (minute < 10) fichier.print('0');
  fichier.print(minute); fichier.print(':');
  if (seconde < 10) fichier.print('0');
  fichier.print(seconde);
  fichier.print(F(" | "));
  
  fichier.print(F("TEMP:"));
  fichier.print(temperature_air, 1);
  fichier.print(F("C | HUM:"));
  fichier.print(humidite, 1);
  fichier.print(F("% | PRESS:"));
  fichier.print(pression_atmospherique, 1);
  fichier.print(F("hPa | LUM:"));
  fichier.print(luminosite);
  
  if (latitude != 0 || longitude != 0) {
    fichier.print(F(" | GPS:"));
    fichier.print(latitude, 6);
    fichier.print(',');
    fichier.print(longitude, 6);
  }
  
  fichier.println();
  fichier.close();
  
  Serial.print(F("Enregistre dans: "));
  Serial.println(nomFichier);
}

void checkButtons() {
  if (bouton_rouge_presse) {
    bouton_rouge_presse = false;
    if (digitalRead(PIN_BOUTON_ROUGE) == LOW) {
      unsigned long t0 = millis();
      while (digitalRead(PIN_BOUTON_ROUGE) == LOW && millis() - t0 < 5000);
      
      if (millis() - t0 >= 5000) {
        if (mode_actuel == 0 || mode_actuel == 2) {
          mode_precedent = mode_actuel;
          mode_actuel = 3;
          setLED(4);
          Serial.println(F("\n>>> MAINTENANCE"));
        } else if (mode_actuel == 3) {
          mode_actuel = mode_precedent;
          setLED(mode_actuel == 0 ? 1 : 3);
          Serial.println(mode_actuel == 0 ? F("\n>>> STANDARD") : F("\n>>> ECONOMIQUE"));
        }
      }
    }
  }

  if (bouton_vert_presse) {
    bouton_vert_presse = false;
    if (digitalRead(PIN_BOUTON_VERT) == LOW) {
      unsigned long t0 = millis();
      while (digitalRead(PIN_BOUTON_VERT) == LOW && millis() - t0 < 5000);
      
      if (millis() - t0 >= 5000) {
        if (mode_actuel == 0) {
          mode_actuel = 2;
          setLED(3);
          Serial.println(F("\n>>> ECONOMIQUE"));
        } else if (mode_actuel == 2) {
          mode_actuel = 0;
          setLED(1);
          Serial.println(F("\n>>> STANDARD"));
        }
      }
    }
  }
}

// ============================================================================
// MODES
// ============================================================================

void modeStandard() {
  setLED(1);
  Serial.println(F("\n=== MODE STANDARD ==="));
  updateDateTime();
  updateSensors();
  getGPS();
  saveSD();
  
  Serial.print(F("Temp: ")); Serial.print(temperature_air, 1); Serial.println(F(" C"));
  Serial.print(F("Hum: ")); Serial.print(humidite, 1); Serial.println(F(" %"));
  Serial.print(F("Pres: ")); Serial.print(pression_atmospherique, 1); Serial.println(F(" hPa"));
  Serial.print(F("Lum: ")); Serial.println(luminosite);
  
  delay(10000); // Attente 10 secondes
}

void modeEco() {
  setLED(3);
  Serial.println(F("\n=== MODE ECONOMIQUE ==="));
  updateDateTime();
  updateSensors();
  
  static bool gpsOn = false;
  gpsOn = !gpsOn;
  if (gpsOn) {
    getGPS();
  } else {
    Serial.println(F("GPS desactive"));
  }
  saveSD();
  
  Serial.print(F("Temp: ")); Serial.print(temperature_air, 1); Serial.println(F(" C"));
  Serial.print(F("Hum: ")); Serial.print(humidite, 1); Serial.println(F(" %"));
  
  delay(20000); // Attente 20 secondes
}

void modeMaint() {
  setLED(4);
  updateSensors();
  Serial.println(F("\n=== MAINTENANCE ==="));
  Serial.print(F("Temp: ")); Serial.print(temperature_air, 1); Serial.println(F(" C"));
  Serial.print(F("Hum: ")); Serial.print(humidite, 1); Serial.println(F(" %"));
  Serial.print(F("Pres: ")); Serial.print(pression_atmospherique, 1); Serial.println(F(" hPa"));
  Serial.print(F("Lum: ")); Serial.println(luminosite);
  delay(2000);
}

void setup() {
  Serial.begin(9600);
  Wire.begin();
  rtc.begin();
  bme.begin();
  
  gpsSerial.begin(9600);

  pinMode(PIN_BOUTON_ROUGE, INPUT_PULLUP);
  pinMode(PIN_BOUTON_VERT, INPUT_PULLUP);
  pinMode(PIN_LUMIERE, INPUT);
  pinMode(PIN_SD_CS, OUTPUT);

  Wire.begin();

  Serial.println(F("\n=== STATION METEO ===\n"));
  
  // RTC
  Serial.print(F("RTC... "));
  if (rtc.begin()) {
    rtc_ok = true;
    Serial.println(F("OK"));
    
    if (rtc.lostPower()) {
      Serial.println(F("RTC perdu - reglage..."));
      rtc.adjust(DateTime(F(__DATE__), F(__TIME__)));
    }
    
    DateTime now = rtc.now();
    Serial.print(F("  "));
    Serial.print(now.day()); Serial.print('/');
    Serial.print(now.month()); Serial.print('/');
    Serial.print(now.year()); Serial.print(' ');
    Serial.print(now.hour()); Serial.print(':');
    Serial.print(now.minute()); Serial.print(':');
    Serial.println(now.second());
  } else {
    Serial.println(F("ERREUR"));
    modeLED = 5;
  }
  
  // BME280
  Serial.print(F("BME280... "));
  if (bme.begin()) {
    bme_ok = true;
    Serial.println(F("OK (0x76)"));
  } else {
    // Essayer l'autre adresse
    BME280I2C::Settings settings(
      BME280::OSR_X1, BME280::OSR_X1, BME280::OSR_X1,
      BME280::Mode_Forced, BME280::StandbyTime_1000ms,
      BME280::Filter_Off, BME280::SpiEnable_False, 0x77
    );
    BME280I2C bme2(settings);
    
    if (bme2.begin()) {
      bme = bme2;
      bme_ok = true;
      Serial.println(F("OK (0x77)"));
    } else {
      Serial.println(F("ERREUR"));
      modeLED = 7;
    }
  }
  
  // Carte SD
  Serial.print(F("SD... "));
  carteSD_presente = SD.begin(PIN_SD_CS);
  if (carteSD_presente) {
    Serial.println(F("OK"));
  } else {
    Serial.println(F("ERREUR"));
    modeLED = 10;
  }
  
  Serial.println(F("\n=== PRET ===\n"));

  mode_actuel = 0;
  setLED(1);

  attachInterrupt(digitalPinToInterrupt(PIN_BOUTON_ROUGE), ISR_bouton_rouge, FALLING);
  attachInterrupt(digitalPinToInterrupt(PIN_BOUTON_VERT), ISR_bouton_vert, FALLING);
}

void loop() {
  checkButtons();
  updateLED();

  if (mode_actuel == 0) {
    modeStandard();
  } 
  else if (mode_actuel == 2) {
    modeEco();
  } 
  else if (mode_actuel == 3) {
    modeMaint();
  }
}
