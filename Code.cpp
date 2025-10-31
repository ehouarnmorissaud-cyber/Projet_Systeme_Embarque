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
//Insérez ici le code correspondant
