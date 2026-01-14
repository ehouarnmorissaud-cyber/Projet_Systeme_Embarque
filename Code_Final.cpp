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
File fichier ;

byte mode_actuel = 0;
byte mode_precedent = 0;

volatile bool bouton_rouge_presse = false;
volatile bool bouton_vert_presse = false;

bool carteSD_presente = true;

// Capteurs
float temperature_air;
float humidite;
float pression_atmospherique;
int luminosite;

// GPS
float latitude;
float longitude;

// RTC
int annee;
byte mois, jour, heure, minute, seconde;

// Config réduite
bool LUMIN = 1;
byte LUMIN_LOW = 255;
int LUMIN_HIGH = 768;
bool TEMP_AIR = 1;
char MIN_TEMP_AIR = -10;
char MAX_TEMP_AIR = 60;
bool HYGR = 1;
int HYGR_MINT = 0;
int HYGR_MAXT = 50;
bool PRESSURE = 1;
int PRESSURE_MIN = 850;
int PRESSURE_MAX = 1080;
byte LOG_INTERVAL = 10;
int FILE_MAX_SIZE = 4096;
int TIMEOUT = 30;
byte version_logiciel = 2;

//Config réinitialisation
void resetConfiguration() {
  LUMIN = 1;
  LUMIN_LOW = 255;
  LUMIN_HIGH = 768;
  TEMP_AIR = 1;
  MIN_TEMP_AIR = -10;
  MAX_TEMP_AIR = 60;
  HYGR = 1;
  HYGR_MINT = 0;
  HYGR_MAXT = 50;
  PRESSURE = 1;
  PRESSURE_MIN = 850;
  PRESSURE_MAX = 1080;
  LOG_INTERVAL = 10;
  FILE_MAX_SIZE = 4096;
  TIMEOUT = 30;
}

unsigned long dernierLog = 0;
byte modeLED = 0; // 0=off, 1=vert, 2=jaune, 3=bleu, 4=orange, 5+=erreurs

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
  else if (mode == 2) leds.setColorRGB(0, 255, 255, 0); // JAUNE
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
  if (carteSD_presente) {
    char nomFichier[25];
    sprintf(
      nomFichier,
      "%04d%02d%02d_%02d%02d%02d.LOG",
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

void modeConfig() {
  setLED(2);
  static unsigned long lastAct = millis();

  if (Serial.available() != NULL) {
    lastAct = millis();
    Serial.println(F("Commande recue"));

    String ligne = Serial.readStringUntil('\n'); // Récupère la ligne rédigée par l'utilisateur
    ligne.trim(); // enlève \r et espaces

    byte indexEgal = ligne.indexOf('='); // Récupère l'index du "="

    if (indexEgal != -1) { // Si on ne trouve pas de "=", la valeur de indexEgal se met automatiquement à -1
      String commande = ligne.substring(0, indexEgal);
      int valeur = ligne.substring(indexEgal + 1).toInt();
      
      // Traitement des différentes commandes
      if (commande == "LUMIN" && (valeur == 0 || valeur == 1)) {
        LUMIN = valeur;
      }
      else if (commande == "LUMIN_LOW" && valeur >= 0 && valeur <= 1023) {
        if (valeur > LUMIN_HIGH) {
          Serial.println("Erreur : LUMIN_LOW ne peut pas être supérieur à LUMIN_HIGH");
        } 
        else {
          LUMIN_LOW = valeur;
        }
      }
      else if (commande == "LUMIN_HIGH" && valeur >= 0 && valeur <= 1023) {
        if (valeur < LUMIN_LOW)
        {
          Serial.println("Erreur : LUMIN_HIGH ne peut pas être inférieur à LUMIN_LOW");
        }
        else {
        LUMIN_HIGH = valeur;
        }
      }
      else if (commande == "TEMP_AIR" && (valeur == 0 || valeur == 1)) {
        TEMP_AIR = valeur;
      }
      else if (commande == "MIN_TEMP_AIR" && valeur >= -40 && valeur <= 85) {
        if (valeur > MAX_TEMP_AIR)
        {
          Serial.println("Erreur : MIN_TEMP_AIR ne peut pas être supérieur à MAX_TEMP_AIR");
        }
        else {
        MIN_TEMP_AIR = valeur;
        }
      }
      else if (commande == "MAX_TEMP_AIR" && valeur >= -40 && valeur <= 85) {
        if (valeur < MIN_TEMP_AIR)
        {
          Serial.println("Erreur : MAX_TEMP_AIR ne peut pas être inférieur à MIN_TEMP_AIR");
        }
        else {
        MAX_TEMP_AIR = valeur;
        }
      }
      else if (commande == "HYGR" && (valeur == 0 || valeur == 1)) {
        HYGR = valeur;
      }
      else if (commande == "HYGR_MINT" && valeur >= -40 && valeur <= 85) {
        if (valeur > HYGR_MAXT)
        {
          Serial.println("Erreur : HYGR_MINT ne peut pas être supérieur à HYGR_MAXT");
        }
        else {
        HYGR_MINT = valeur;
        }
      }
      else if (commande == "HYGR_MAXT" && valeur >= -40 && valeur <= 85) {
        if (valeur < HYGR_MINT)
        {
          Serial.println("Erreur : HYGR_MAXT ne peut pas être inférieur à HYGR_MINT");
        }
        else {
        HYGR_MAXT = valeur;
        }
      }
      else if (commande == "PRESSURE" && (valeur == 0 || valeur == 1)) {
        PRESSURE = valeur;
      }
      else if (commande == "PRESSURE_MIN" && valeur >= 300 && valeur <= 1100) {
        if (valeur > PRESSURE_MAX)
        {
          Serial.println("Erreur : PRESSURE_MIN ne peut pas être supérieur à PRESSURE_MAX");
        }
        else {  
        PRESSURE_MIN = valeur;
        }
      }
      else if (commande == "PRESSURE_MAX" && valeur >= 300 && valeur <= 1100) {
        if (valeur < PRESSURE_MIN)
        {
          Serial.println("Erreur : PRESSURE_MAX ne peut pas être inférieur à PRESSURE_MIN");
        }
        else {
        PRESSURE_MAX = valeur;
        }
      }
      else if (commande == "LOG_INTERVAL" && valeur > 0) {
        LOG_INTERVAL = valeur;
      }
      else if (commande == "FILE_MAX_SIZE" && valeur > 0) {
        FILE_MAX_SIZE = valeur;
      }
      else if (commande == "TIMEOUT" && valeur > 0) {
        TIMEOUT = valeur;
      }
      else {
        Serial.print("La valeur rentrée est n'est pas correcte pour la commande "); Serial.println(commande);
      }
    }
    else if (ligne == "VERSION"){
      Serial.print("Version du logiciel embarqué : "); 
      Serial.println(version_logiciel);
    }
    else if (ligne == "RESET"){
      Serial.println("Réinitialisation de l’ensemble des paramètres à leurs valeurs par défaut.");
      resetConfiguration();
    }
    else { // Si il n'y a eu aucun "=" dans la ligne, et que la valeur est donc à -1
      Serial.println("La commande saisie n'est pas valide, merci d'utiliser le format suivant :'COMMANDE=VALEUR'");
    }
  }
  if (millis() - lastAct > 30000) {
    mode_actuel = 0;
    Serial.println(F("\n>>> Timeout - Retour Mode STANDARD"));
  }
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

  if (digitalRead(PIN_BOUTON_ROUGE) == LOW) {
    mode_actuel = 1;
    setLED(2);
    Serial.println(F("\n>>> DEMARRAGE EN MODE CONFIGURATION"));
  } else {
    mode_actuel = 0;
    setLED(1);
    Serial.println(F("\n>>> DEMARRAGE EN MODE STANDARD"));
  }

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
  else if (mode_actuel == 1) {
    modeConfig();
    } 
  else if (mode_actuel == 2) {
    modeEco();
    } 
  else if (mode_actuel == 3) {
    modeMaint();
  }
}
