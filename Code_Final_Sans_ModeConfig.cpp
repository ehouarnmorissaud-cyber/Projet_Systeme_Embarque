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

volatile bool bouton_rouge_presse = false;
volatile bool bouton_vert_presse = false;

bool carteSD_presente = true;

// Capteurs
int8_t temperature_air;
int8_t humidite;
int16_t pression_atmospherique;
uint16_t luminosite;

// GPS
float latitude;
float longitude;

// RTC
byte annee, mois, jour, heure, minute, seconde;


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
  if (modeLED == 5) gererLED(255, 0, 0, 0, 0, 255, 500, 500);      // ROUGE-BLEU
  else if (modeLED == 6) gererLED(255, 0, 0, 255, 255, 0, 500, 500);   // ROUGE-JAUNE
  else if (modeLED == 7) gererLED(255, 0, 0, 0, 255, 0, 500, 500);     // ROUGE-VERT
  else if (modeLED == 8) gererLED(255, 0, 0, 0, 255, 0, 333, 667);     // ROUGE-VERT LONG
  else if (modeLED == 9) gererLED(255, 0, 0, 255, 255, 255, 500, 500); // ROUGE-BLANC
  else if (modeLED == 10) gererLED(255, 0, 0, 255, 255, 255, 333, 667);// ROUGE-BLANC LONG
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
  if (!rtc.begin()) {
    Serial.println(F("ERREUR: RTC non detecte"));
    modeLED = 5; // ROUGE-BLEU
    return;
  }
  DateTime now = rtc.now();
  annee = now.year();
  mois = now.month();
  jour = now.day();
  heure = now.hour();
  minute = now.minute();
  seconde = now.second();
}

void updateSensors() {
  float t, h, p;
  bme.read(p, t, h, BME280::TempUnit_Celsius, BME280::PresUnit_Pa);
  
  if (isnan(t) || isnan(h) || isnan(p)) {
    Serial.println(F("ERREUR: Capteur BME280 inaccessible"));
    modeLED = 7; // ROUGE-VERT
    return;
  }
  
  if (t < -40 || t > 85 || h < 0 || h > 100) {
    Serial.println(F("ERREUR: Donnees capteur incoherentes"));
    Serial.print(F("  Temp: ")); Serial.print(t);
    Serial.print(F(" Hum: ")); Serial.println(h);
    modeLED = 8; // ROUGE-VERT LONG
    return;
  }
  
  temperature_air = t;
  humidite = h;
  pression_atmospherique = p;
  luminosite = analogRead(PIN_LUMIERE);
}

void getGPS() {
  unsigned long debut = millis();
  while (millis() - debut < 5000) {
    while (gpsSerial.available()) gps.encode(gpsSerial.read());
    if (gps.location.isValid()) {
      latitude = gps.location.lat();
      longitude = gps.location.lng();
      Serial.print(F("GPS OK: "));
      Serial.print(latitude, 6);
      Serial.print(F(", "));
      Serial.println(longitude, 6);
      return;
    }
  }
  Serial.println(F("ERREUR: GPS non disponible"));
  latitude = 0;
  longitude = 0;
  modeLED = 6; // ROUGE-JAUNE
}

void saveSD() {
  File fichier ;
  if (carteSD_presente) {
    char nomFichier[20];
    sprintf(
      nomFichier,
      "%02d%02d%02d_%02d%02d%02d.LOG",
      annee,
      mois,
      jour,
      heure,
      minute,
      seconde
    );

    fichier = SD.open( nomFichier, FILE_WRITE);

    if (fichier){
      fichier.print("TEMP : "); fichier.println(temperature_air);
      fichier.print("HUM : "); fichier.println(humidite);
      fichier.print("PRESS : "); fichier.println(pression_atmospherique);
      fichier.print("LUM : "); fichier.println(luminosite);
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
          Serial.println(F("\n>>> Mode MAINTENANCE active"));
        } else if (mode_actuel == 3) {
          mode_actuel = mode_precedent;
          setLED(mode_actuel == 0 ? 1 : 3);
          if (mode_actuel == 0) Serial.println(F("\n>>> Retour Mode STANDARD"));
          else Serial.println(F("\n>>> Retour Mode ECONOMIQUE"));
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
          Serial.println(F("\n>>> Mode ECONOMIQUE active"));
        } else if (mode_actuel == 2) {
          mode_actuel = 0;
          setLED(1);
          Serial.println(F("\n>>> Mode STANDARD active"));
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
  Serial.print(F("Temp: ")); Serial.print(temperature_air); Serial.println(F(" C"));
  Serial.print(F("Hum: ")); Serial.print(humidite); Serial.println(F(" %"));
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
    Serial.println(F("GPS desactive (economie)"));
  }
  saveSD();
}

void modeMaint() {
  setLED(4);
  updateSensors();
  Serial.println(F("\n=== MODE MAINTENANCE ==="));
  Serial.print(F("Temp: ")); Serial.print(temperature_air); Serial.println(F(" C"));
  Serial.print(F("Hum: ")); Serial.print(humidite); Serial.println(F(" %"));
  Serial.print(F("Pres: ")); Serial.print(pression_atmospherique); Serial.println(F(" Pa"));
  Serial.print(F("Lumi: ")); Serial.println(luminosite);
  delay(2000);
}



void setup() {
  Serial.begin(9600); 
  gpsSerial.begin(9600);

  pinMode(PIN_BOUTON_ROUGE, INPUT_PULLUP);
  pinMode(PIN_BOUTON_VERT, INPUT_PULLUP);
  pinMode(PIN_LUMIERE, INPUT);

  Serial.println(F("\n--- Verification composants ---"));
  
  if (!rtc.begin()) {
    Serial.println(F("ERREUR: RTC non detecte"));
    modeLED = 5;
  } else {
    Serial.println(F("OK: RTC"));
  }
  
  if (!bme.begin()) {
    Serial.println(F("ERREUR: BME280 non detecte"));
    modeLED = 7;
  } else {
    Serial.println(F("OK: BME280"));
  }
  
  carteSD_presente = SD.begin(PIN_SD_CS);
  if (!carteSD_presente) {
    Serial.println(F("ERREUR: Carte SD absente"));
    modeLED = 10;
  } else {
    Serial.println(F("OK: Carte SD"));
  }
  
  // Test GPS au démarrage
  Serial.print(F("Test GPS..."));
  unsigned long debut = millis();
  bool gpsOK = false;
  while (millis() - debut < 5000) {
    while (gpsSerial.available()) {
      gps.encode(gpsSerial.read());
      if (gps.location.isValid()) {
        gpsOK = true;
        break;
      }
    }
    if (gpsOK) break;
  }
  
  if (gpsOK) {
    Serial.println(F(" OK: GPS"));
  } else {
    Serial.println(F(" ATTENTION: GPS non disponible"));
  }
  
  Serial.println(F("--- Systeme pret ---\n"));

 

  attachInterrupt(digitalPinToInterrupt(PIN_BOUTON_ROUGE), ISR_bouton_rouge, FALLING);
  attachInterrupt(digitalPinToInterrupt(PIN_BOUTON_VERT), ISR_bouton_vert, FALLING);
  
}

void loop() {
  checkButtons();
  updateLED();

  unsigned long now = millis();

  if (mode_actuel == 0 ){
    modeStandard();
    } 
 
  else if (mode_actuel == 2) {
    modeEco();
    } 
  else if (mode_actuel == 3) {
    modeMaint();
  }
}
