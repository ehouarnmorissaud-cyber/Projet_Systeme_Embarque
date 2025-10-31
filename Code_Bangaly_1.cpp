#include <Wire.h>
#include <RTClib.h>
#include <SoftwareSerial.h>
#include <TinyGPS++.h>
#include <SPI.h>
#include <SD.h>
#include <Adafruit_BMP280.h>


// Définition des broches
#define PIN_LED 4            // LED multicolore (pilotée par PWM ou multiplexage)
#define PIN_BOUTON 2         // Bouton d’interruption
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

// Mode de fonctionnement
byte mode_actuel;               // Mode actif du système (0 = STANDARD, 1 = ECONOMIQUE, etc.)
void setup() {
  // Initialisation des communications série
  Serial.begin(9600);          // Pour le moniteur série
  gpsSerial.begin(9600);       // Pour le module GPS Air530Z

  // Configuration des broches
  pinMode(PIN_LED, OUTPUT);           // LED multicolore
  pinMode(PIN_BOUTON, INPUT_PULLUP);  // Bouton avec résistance interne
  pinMode(PIN_LUMIERE, INPUT);        // Capteur analogique
  pinMode(PIN_PRESSION, INPUT);       // Capteur analogique
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
   // Lecture RTC
  DateTime now = rtc.now();
  annee = now.year(); mois = now.month(); jour = now.day();
  heure = now.hour(); minute = now.minute(); seconde = now.second();

  // Lecture capteurs
  luminosite = analogRead(PIN_LUMIERE);
  pression_atmospherique = analogRead(PIN_PRESSION);
  temperature_air = bmp.readTemperature();

  // Lecture GPS
  while (gpsSerial.available()) {
    gps.encode(gpsSerial.read());
  }
  if (gps.location.isValid()) {
    latitude = gps.location.lat();
    longitude = gps.location.lng();
  } else {
    latitude = 0.0;
    longitude = 0.0;
    setLEDColor("ROUGE_JAUNE"); // Erreur GPS
  }

  // Sauvegarde SD
  if (carteSD_presente) {
    fichier = SD.open("200531_0.LOG", FILE_WRITE);//ouvre un fichier sur la carte SD
    //Vérifie que le fichier a bien été ouvert sur la carte SD. 
    if (fichier){
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
      fichier.close();//Ferme proprement le fichier après l’écriture.
    } else {
      setLEDColor("ROUGE_BLANC_LONG"); // Erreur écriture SD
    }
  } else {
    setLEDColor("ROUGE_BLANC"); // Carte SD pleine ou absente
  }
}
}

void modeConfiguration() {
  Serial.println("Mode CONFIGURATION actif");
  // Interface UART pour modifier les paramètres
  // Pas d’acquisition
  // LED jaune continue
  setLEDColor("JAUNE");

  // Gestion du timeout de 30 min
  static unsigned long lastActivity = millis();
  if (Serial.available()) {
    lastActivity = millis(); // activité détectée
    handleUARTCommand();     // fonction à créer pour parser les commandes
  }
  if (millis() - lastActivity > 1800000) { // 30 min
    mode_actuel = 0; // retour au mode standard
  }
}

void modeEconomique() {
  Serial.println("Mode ÉCONOMIQUE actif");

  //  LED bleue continue pour signaler le mode
  setLEDColor("BLEU");

  //  Lecture RTC
  DateTime now = rtc.now();
  annee = now.year();
  mois = now.month();
  jour = now.day();
  heure = now.hour();
  minute = now.minute();
  seconde = now.second();

  //  Lecture capteurs allégée
  luminosite = analogRead(PIN_LUMIERE);
  temperature_air = bmp.readTemperature();

  //  GPS une mesure sur deux
  static bool gpsActif = false;
  gpsActif = !gpsActif; // alterne à chaque appel

  if (gpsActif) {
    while (gpsSerial.available()) {
      gps.encode(gpsSerial.read());
    }
    if (gps.location.isValid()) {
      latitude = gps.location.lat();
      longitude = gps.location.lng();
    } else {
      latitude = 0.0;
      longitude = 0.0;
      setLEDColor("ROUGE_JAUNE"); // Erreur GPS
    }
  }

  // Sauvegarde SD
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
      fichier.print(latitude); fichier.print(",");
      fichier.println(longitude);
      fichier.close();
    } else {
      setLEDColor("ROUGE_BLANC_LONG"); // Erreur écriture SD
    }
  } else {
    setLEDColor("ROUGE_BLANC"); // Carte SD absente ou pleine
  }

  //  Temporisation allongée : LOG_INTERVAL × 2
  delay(LOG_INTERVAL * 2 * 60000); // en millisecondes
}

void modeMaintenance() {
  Serial.println("Mode MAINTENANCE actif");

  // LED orange continue
  setLEDColor("ORANGE");

  // Affichage capteurs en direct
  Serial.print("Température : "); Serial.println(bmp.readTemperature());
  Serial.print("Lumière : "); Serial.println(analogRead(PIN_LUMIERE));
  Serial.print("Pression : "); Serial.println(analogRead(PIN_PRESSION));

  // GPS
  while (gpsSerial.available()) {
    gps.encode(gpsSerial.read());
  }
  if (gps.location.isValid()) {
    Serial.print("GPS : ");
    Serial.print(gps.location.lat()); Serial.print(", ");
    Serial.println(gps.location.lng());
  } else {
    Serial.println("GPS non disponible");
    setLEDColor("ROUGE_JAUNE");
  }

  // Carte SD désactivée 
}
