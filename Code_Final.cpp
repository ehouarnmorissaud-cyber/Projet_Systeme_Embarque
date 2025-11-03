#include <Wire.h>
#include <RTClib.h>
#include <SoftwareSerial.h>
#include <TinyGPS++.h>
#include <SPI.h>
#include <SD.h>
#include <Adafruit_BMP280.h>

// Définition des broches
#define PIN_LED 4            // LED multicolore (pilotée par PWM ou multiplexage)
#define PIN_BOUTON_VERT 2  
#define PIN_BOUTON_ROUGE 3 // Button d'interrruption
#define PIN_LUMIERE A0       // Capteur de lumière
#define PIN_PRESSION A1      // Capteur de pression analogique
#define PIN_SD_CS 10         // Chip Select de la carte SD

// Objets
RTC_DS3231 rtc;// Objet pour communiquer avec le module RTC DS3231
SoftwareSerial gpsSerial(6, 7); // branchement du GPS RX, TX 
TinyGPSPlus gps; // Objet pour décoder les données  module GPS
Adafruit_BMP280 bmp;// Capteur numérique de température et pression BMP280
File fichier;// Objet pour gérer les fichiers sur la carte SD

// Capteurs environnementaux
float pression_atmospherique;   // Pression mesurée par le capteur analogique
float luminosite;               // Intensité lumineuse détectée par le capteur de lumière
float temperature_air;          // Température de l’air ambiant mesurée par le capteur BMP280

// GPS
float latitude;                 // Position GPS en latitude
float longitude;                // Position GPS en longitude

// Horodatage RTC
int annee;                      // Année actuelle récupérée via le module RTC
int mois;                       // Mois actuel récupéré via le module RTC
int jour;                       // Jour actuel récupéré via le module RTC
int heure;                      // Heure actuelle récupérée via le module RTC
int minute;                     // Minutes actuelles récupérées via le module RTC
int seconde;                    // Secondes actuelles récupérées via le module RTC

// État du système
bool carteSD_presente;          // Indique si la carte SD est détectée et prête
bool bouton_appuye;             // État du bouton connecté à D2

// Paramètres (modifiable en mode configuration)

// Valeurs et structure de configuration (défauts basés sur la documentation)
// Luminosité
boolean LUMIN = 1;           // {0,1} activation du capteur de luminosité
int LUMIN_LOW = 255;    // {0-1023} seuil bas (ex: 200)
int LUMIN_HIGH = 768;   // {0-1023} seuil haut (ex: 700)
// Température air
boolean TEMP_AIR = 1;        // {0,1} activation capteur température de l'air
char MIN_TEMP_AIR = -10;  // {-40-85}
char MAX_TEMP_AIR = 60;   // {-40-85}

// Hygrométrie
boolean HYGR = 1;            // {0,1} activation capteur hygrométrie
char HYGR_MINT = 0;       // {-40-85} temperature en dessous de laquelle hygrometrie ignorée
char HYGR_MAXT = 50;      // {-40-85} temperature au dessus de laquelle hygrometrie ignorée

// Pression
boolean PRESSURE = 0;        // {0,1} activation capteur pression atmosphérique
int PRESSURE_MIN = 850;  // {300-1100} HPa seuil bas (ex: 850)
int PRESSURE_MAX = 1080; // {300-1100} HPa seuil haut (ex: 1080)

// Autres paramètres généraux
int LOG_INTERVAL = 10;  // minutes entre deux mesures (défaut 10)
int FILE_MAX_SIZE = 4096;// octets avant archivage
int TIMEOUT = 30;       // secondes timeout acquisition capteur

int version_logiciel = 1.0; // Version du logiciel embarqué

// Remise des valeurs de départ pour l'ensemble des paramètres (RESET en lode configuration)
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
// Mode de fonctionnement
byte mode_precedent;           // Mode précédemment utilisé avant le mode actuel
byte mode_actuel;               // Mode actif du système (0 = STANDARD, 1 = ECONOMIQUE, etc.)
// Met à jour les variables annee, mois, jour, heure, minute, seconde à partir du module RTC DS3231
void updateDateTime() {
  DateTime now = rtc.now();
  annee = now.year();
  mois = now.month();
  jour = now.day();
  heure = now.hour();
  minute = now.minute();
  seconde = now.second();
}

// Met à jour les variables temperature_air, luminosite et pression_atmospherique
// en lisant les capteurs analogiques et le capteur BMP280 (VMA335)
void updateSensors() {
  temperature_air = bmp.readTemperature();       // Température via BMP280
  luminosite = analogRead(PIN_LUMIERE);          // Lumière analogique
  pression_atmospherique = analogRead(PIN_PRESSION); // Pression analogique
}

// Lit les données GPS, met à jour latitude et longitude,
// et affiche la position dans le moniteur série si elle est valide.
// Si la position est invalide, met les coordonnées à 0 et signale une erreur par LED.
void getAndPrintGPSPosition() {
  while (gpsSerial.available()) {
    gps.encode(gpsSerial.read());
  }

  if (gps.location.isValid()) {
    latitude = gps.location.lat();
    longitude = gps.location.lng();

    Serial.print("GPS : ");
    Serial.print(latitude);
    Serial.print(", ");
    Serial.println(longitude);
  } else {
    latitude = 0.0;
    longitude = 0.0;
    Serial.println("GPS non disponible");
    setLEDColor("ROUGE_JAUNE"); // Erreur GPS
  }
}

// Ouvre le fichier LOG sur la carte SD et enregistre les données capteurs + GPS + horloge
// Si la carte est absente ou le fichier inaccessible, signale une erreur par LED
void saveDataToSD() {
  if (carteSD_presente) {
    fichier = SD.open("200531_0.LOG", FILE_WRITE);
    if (fichier) {
      fichier.print(annee); fichier.print(",");
      fichier.print(mois); fichier.print(",");
      fichier.print(jour); fichier.print(",");
      fichier.print(heure); fichier.print(",");
      fichier.print(minute); fichier.print(",");
      fichier.print(seconde); fichier.print(",");
      fichier.print(temperature_air); fichier.print(",");
      fichier.print(luminosite); fichier.print(",");
      fichier.print(pression_atmospherique); fichier.print(",");
      fichier.print(latitude); fichier.print(",");
      fichier.println(longitude);
      fichier.close();
    } else {
      setLEDColor("ROUGE_BLANC_LONG"); // Erreur écriture SD
    }
  } else {
    setLEDColor("ROUGE_BLANC"); // Carte SD absente ou pleine
  }
}
// PIN broche bouton rouge/vert et variables ISR 
 const byte PIN_BOUTON_ROUGE = 3; 
 const byte PIN_BOUTON_VERT = 2;
 volatile bool bouton_rouge_presse = false; 
 volatile bool bouton_vert_presse = false;

void setup() {
  // Initialisation des communications série
  Serial.begin(9600);          // Pour le moniteur série
  gpsSerial.begin(9600);       // Pour le module GPS Air530Z

  // Configuration des broches
  pinMode(PIN_LED, OUTPUT);           // LED multicolore
  pinMode(PIN_BOUTON, INPUT_PULLUP);  // Bouton avec résistance interne
  pinMode(PIN_BOUTON_ROUGE, INPUT_PULLUP); 
  pinMode(PIN_BOUTON_VERT,  INPUT_PULLUP);
  pinMode(PIN_LUMIERE, INPUT);        // Capteur analogique
  pinMode(PIN_PRESSION, INPUT);       // Capteur analogique

  // interruptions externes 
  attachInterrupts(digitalPinToInterrupt(PIN_BOUTON_ROUGE), bouton_rouge_presse, FALLING);
  attachInterrupts(digitalPinToInterrupt(PIN_BOUTON_VERT), bouton_vert_presse, FALLING);
  if (digitalRead(PIN_BOUTON_ROUGE) == LOW) {
    mode_actuel = 1
  } else {
    mode_actuel = 0 
  }
}

void initialisation_Interupt_externe() {
  if (boutonRougeFlag) {
    boutonRougeFlag = false;
    if (digitalRead(PIN_BOUTON_ROUGE) == LOW) {          
      unsigned long t0 = millis();
      while (digitalRead(PIN_BOUTON_ROUGE) == LOW) {
        if (millis() - t0 >= LONG_PRESS_MS) break;
        }
      if (millis() - t0 >= LONG_PRESS_MS) {
        if (mode_actuel == 0)        { mode_precedent = mode_actuel; mode_actuel = 3; }
        else if (mode_actuel == 2) { mode_precedent = mode_actuel; mode_actuel = 3;    }
        else if (mode_actuel == 3 && mode_precedent == 0){ mode_precedent = mode_actuel ; mode_actuel = 0 }
        else if (mode_actuel == 3 && mode_precedent == 2){ mode_precedent = mode_actuel ; mode_actuel = 2 }
      }
    }
  }  
  if (boutonVertFlag) {
    boutonVertFlag = false;
    if (digitalRead(PIN_BOUTON_VERT) == LOW) {
      unsigned long t0 = millis();
      while (digitalRead(PIN_BOUTON_VERT) == LOW) {
        if (millis() - t0 >= LONG_PRESS_MS) break;
      }
      if (millis() - t0 >= LONG_PRESS_MS) {
        if (mode_actuel == 0)        { mode_precedent = mode_actuel; mode_actuel = 2; }
        else if (mode_actuel == 2) { mode_precedent = mode_actuel; mode_actuel = 0;    }
      }
    }
  }
}
  
//interruption interne timer  
void initialisation_Interupt_timer() {
  noInterrupts();
  TCCR1A = 0; TCCR1B = 0; TCNT1 = 0;
  OCR1A  = 15624;                      // 16 MHz / 1024 (prescaler)
  TCCR1B |= (1 << WGM12);
  TCCR1B |= (1 << CS12) | (1 << CS10);
  TIMSK1 |= (1 << OCIE1A);
  interrupts();
}   

// Initialisation du module RTC
  if (!rtc.begin()) {
    Serial.println("Erreur : RTC non détecté !");
  } else {
    Serial.println("RTC prêt.");
  }
 // Initialisation du capteur BMP280
  if (!bmp.begin(0x76)) {
    Serial.println("Erreur : BMP280 non détecté !");
  } else {
    Serial.println("BMP280 prêt.");
  }
  // Initialisation de la carte SD
  if (!SD.begin(PIN_SD_CS)) {
    Serial.println("Carte SD absente ou erreur !");
    carteSD_presente = false;
  } else {
    Serial.println("Carte SD détectée.");
    carteSD_presente = true;
  }
 // Lecture de l’état du bouton au démarrage
  bouton_appuye = (digitalRead(PIN_BOUTON) == LOW);

  // Détermination du mode de fonctionnement
  if (bouton_appuye) {
    mode_actuel = 1; // Mode configuration ou économique
    Serial.println("Mode configuration activé.");
  } else {
    mode_actuel = 0; // Mode standard
    Serial.println("Mode standard activé.");
  }
  // Signal visuel de démarrage
  digitalWrite(PIN_LED, HIGH); // LED allumée pour indiquer que le système démarre
  delay(500);
  digitalWrite(PIN_LED, LOW);
}
void modeStandard() {
  Serial.println("Mode STANDARD actif");
  setLEDColor("VERT");

  updateDateTime();             // Lecture RTC
  updateSensors();              // Lecture capteurs
  getAndPrintGPSPosition();     // Lecture + affichage GPS
  saveDataToSD();               // Sauvegarde SD
}


void modeConfiguration() {
  Serial.println("Mode CONFIGURATION actif");
  // Interface UART pour modifier les paramètres
  // Pas d’acquisition
  // LED jaune continue
  setLEDColor("JAUNE");

  // Gestion du timeout de 30 min
  static unsigned long lastActivity = millis();
  if (Serial.available() != NULL) {
    lastActivity = millis(); // activité détectée
    String ligne = Serial.read(); // Récupère la ligne rédigée par l'utilisateur
    byte indexEgal = stringeOne.indexOf('='); // Récupère l'index du "="
    if (indexEgal != -1) { // Si on ne trouve pas de "=", la valeur de indexEgal se met automatiquement à -1
      String commande = stringeOne.substring(0, indexEgal);
      int valeur = stringeOne.substring(indexEgal + 1).toInt();
      
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
      Serial.print("Version du logiciel embarqué : "); Serial.println(version_logiciel);
    }
    else if (ligne == "RESET"){
      Serial.println("Réinitialisation de l’ensemble des paramètres à leurs valeurs par défaut.");
      resetConfiguration();
    }
    else { // Si il n'y a eu aucun "=" dans la ligne, et que la valeur est donc à -1
      Serial.println("La commande saisie n'est pas valide, merci d'utiliser le format suivant :'COMMANDE=VALEUR'");
    }

  }
    
    
  if (millis() - lastActivity > 1800000) { // 30 min
    mode_actuel = 0; // retour au mode standard
  }
}

void modeEconomique() {
  Serial.println("Mode ÉCONOMIQUE actif");
  setLEDColor("BLEU");

  updateDateTime();             // Lecture RTC

  // Lecture capteurs allégée
  temperature_air = bmp.readTemperature();
  luminosite = analogRead(PIN_LUMIERE);

  // GPS une mesure sur deux
  static bool gpsActif = false;
  gpsActif = !gpsActif;
  if (gpsActif) {
    getAndPrintGPSPosition();   // Lecture + affichage GPS
  }

  saveDataToSD();               // Sauvegarde SD
  delay(LOG_INTERVAL * 2 * 60000); // Temporisation
}

void modeMaintenance() {
  Serial.println("Mode MAINTENANCE actif");
  setLEDColor("ORANGE");

  // Affichage capteurs en direct
  Serial.print("Température : "); Serial.println(bmp.readTemperature());
  Serial.print("Lumière : "); Serial.println(analogRead(PIN_LUMIERE));
  Serial.print("Pression : "); Serial.println(analogRead(PIN_PRESSION));

  getAndPrintGPSPosition(); // Lecture + affichage GPS

  // Carte SD désactivée
}

// ========== Boucle principale ==========

void loop() {
  
initialisation_Interupt_externe();
initialisation_Interupt_timer();
  
  if (mode_actuel == 0) {
    modeStandard();
  } else if (mode_actuel == 1) {
    modeConfiguration();
  } else if (mode_actuel == 2) {
    modeEconomique();
  } else if (mode_actuel == 3) {
    modeMaintenance();
  }
}
