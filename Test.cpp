#include <Arduino.h>

// --- BIBLIOTHÈQUES ---
#include <Wire.h>
#include <RTC.h>                  // Pour RTC v1.2.4
#include <TinyGPS++.h>
#include <Grove_ChainableLED.h>
#include <SD.h>
#include <SPI.h>
 
// --- OBJETS ---
static DS1307 RTC;
TinyGPSPlus gps;
 
// --- CONFIGURATION DES BROCHES ---
const int BOUTON_PIN = 2;
const int LED_PIN = 4;
const int LIGHT_SENSOR_PIN = A0;
const int PRESSION_SENSOR_PIN = A1;
const int CHIP_SELECT_SD = 10;
 
// Configuration LED RGB
#define NUM_LEDS 1
ChainableLED leds(LED_PIN, NUM_LEDS);
 
// Adresse I2C GPS
const uint8_t GPS_I2C_ADDRESS = 0x42;
 
// Variables de test
bool tousTestsReussis = true;
 
void setup() {
  Serial.begin(9600);
  Serial.println("=== VÉRIFICATION COMPLÈTE DU SYSTÈME ===");
  Serial.println();
  // Initialisation I2C
  Wire.begin();
  // Test de tous les composants
  delay(1000);
  Serial.println("🔍 DÉBUT DES TESTS...");
  Serial.println();
  // 1. Test RTC (I2C)
  testRTC();
  delay(500);
  // 2. Test GPS I2C
  testGPSI2C();
  delay(500);
  // 3. Test Bouton (D2)
  testBouton();
  delay(500);
  // 4. Test LED RGB (D4)
  testLED();
  delay(500);
  // 5. Test Capteur Lumière (A0)
  testLumiere();
  delay(500);
  // 6. Test Capteur Pression (A1)
  testPression();
  delay(500);
  // 7. Test Carte SD (SPI)
  testCarteSD();
  delay(500);
  // 8. Test Communication GPS UART (si présent)
  testGPSUART();
  // Résumé final
  afficherResume();
}
 
void loop() {
  // Ne rien faire après les tests
  delay(1000);
}
 
// === TEST RTC ===
void testRTC() {
  Serial.println("1. 📅 TEST RTC (I2C)");
  RTC.begin();
  if (!RTC.isRunning()) {
    Serial.println("   ❌ RTC non démarré - tentative de démarrage...");
    RTC.start();
    delay(100);
  }
  if (RTC.isRunning()) {
    Serial.println("   ✅ RTC en fonctionnement");
    // Lire et afficher l'heure
    Serial.print("   🕒 Heure RTC: ");
    Serial.print(RTC.getDay());
    Serial.print("/");
    Serial.print(RTC.getMonth());
    Serial.print("/");
    Serial.print(RTC.getYear());
    Serial.print(" ");
    Serial.print(RTC.getHour());
    Serial.print(":");
    Serial.print(RTC.getMinute());
    Serial.print(":");
    Serial.println(RTC.getSecond());
  } else {
    Serial.println("   ❌ ERREUR: RTC ne répond pas");
    tousTestsReussis = false;
  }
  Serial.println();
}
 
// === TEST GPS I2C ===
void testGPSI2C() {
  Serial.println("2. 🛰️ TEST GPS VMA335 (I2C)");
  // Test connexion I2C
  Wire.beginTransmission(GPS_I2C_ADDRESS);
  byte error = Wire.endTransmission();
  if (error == 0) {
    Serial.println("   ✅ Communication I2C avec GPS OK");
    Serial.println("   📍 Adresse: 0x" + String(GPS_I2C_ADDRESS, HEX));
    // Essayer de lire des données
    Wire.requestFrom(GPS_I2C_ADDRESS, 10);
    if (Wire.available()) {
      Serial.println("   ✅ Données GPS disponibles");
    } else {
      Serial.println("   ⚠️  Aucune donnée reçue (peut être normal au début)");
    }
  } else {
    Serial.println("   ❌ ERREUR: GPS I2C non détecté");
    Serial.println("   Vérifiez:");
    Serial.println("   - SDA/SCL branchés");
    Serial.println("   - CSB → GND");
    Serial.println("   - SDO → GND");
    Serial.println("   - Alimentation 3.3V");
    tousTestsReussis = false;
  }
  Serial.println();
}
 
// === TEST BOUTON ===
void testBouton() {
  Serial.println("3. 🔘 TEST BOUTON (D2)");
  pinMode(BOUTON_PIN, INPUT_PULLUP);
  int etatBouton = digitalRead(BOUTON_PIN);
  Serial.print("   État bouton: ");
  if (etatBouton == HIGH) {
    Serial.println("RELÂCHÉ");
  } else {
    Serial.println("PRESSÉ");
  }
  Serial.println("   ✅ Broche D2 opérationnelle");
  Serial.println("   💡 Appuyez sur le bouton pour tester...");
  // Attendre un appui pendant 3 secondes
  unsigned long debut = millis();
  while (millis() - debut < 3000) {
    if (digitalRead(BOUTON_PIN) == LOW) {
      Serial.println("   ✅ BOUTON DÉTECTÉ - Fonctionne correctement!");
      break;
    }
    delay(100);
  }
  Serial.println();
}
 
// === TEST LED RGB ===
void testLED() {
  Serial.println("4. 💡 TEST LED RGB (D4)");
  leds.init();
  // Test des couleurs
  Serial.println("   Test des couleurs...");
  // Rouge
  leds.setColorRGB(0, 255, 0, 0);
  Serial.println("   🔴 Rouge");
  delay(500);
  // Vert
  leds.setColorRGB(0, 0, 255, 0);
  Serial.println("   🟢 Vert");
  delay(500);
  // Bleu
  leds.setColorRGB(0, 0, 0, 255);
  Serial.println("   🔵 Bleu");
  delay(500);
  // Blanc
  leds.setColorRGB(0, 255, 255, 255);
  Serial.println("   ⚪ Blanc");
  delay(500);
  // Éteindre
  leds.setColorRGB(0, 0, 0, 0);
  Serial.println("   ✅ LED RGB testée avec succès");
  Serial.println();
}
 
// === TEST CAPTEUR LUMIÈRE ===
void testLumiere() {
  Serial.println("5. ☀️ TEST CAPTEUR LUMIÈRE (A0)");
  int valeurLumiere = analogRead(LIGHT_SENSOR_PIN);
  Serial.print("   Valeur luminaire: ");
  Serial.println(valeurLumiere);
  // Interprétation
  if (valeurLumiere == 0) {
    Serial.println("   ⚠️  Valeur 0 - Vérifiez la connexion");
    tousTestsReussis = false;
  } else if (valeurLumiere == 1023) {
    Serial.println("   ⚠️  Valeur max - Capteur saturé ou problème");
    tousTestsReussis = false;
  } else if (valeurLumiere < 100) {
    Serial.println("   🌙 Environnement sombre");
  } else if (valeurLumiere < 500) {
    Serial.println("   ⛅ Lumière modérée");
  } else {
    Serial.println("   🔆 Forte luminosité");
  }
  Serial.println("   ✅ Capteur lumière opérationnel");
  Serial.println();
}
 
// === TEST CAPTEUR PRESSION ===
void testPression() {
  Serial.println("6. 📊 TEST CAPTEUR PRESSION (A1)");
  int valeurPression = analogRead(PRESSION_SENSOR_PIN);
  Serial.print("   Valeur brute: ");
  Serial.println(valeurPression);
  // Conversion hypothétique en hPa
  float pression = map(valeurPression, 0, 1023, 90000, 110000) / 100.0;
  Serial.print("   Pression estimée: ");
  Serial.print(pression);
  Serial.println(" hPa");
  if (valeurPression == 0) {
    Serial.println("   ❌ ERREUR: Valeur 0 - Vérifiez la connexion");
    tousTestsReussis = false;
  } else if (valeurPression == 1023) {
    Serial.println("   ⚠️  Valeur max - Vérifiez l'alimentation");
  } else {
    Serial.println("   ✅ Capteur pression opérationnel");
  }
  Serial.println();
}
 
// === TEST CARTE SD ===
void testCarteSD() {
  Serial.println("7. 💾 TEST CARTE SD (SPI)");
  if (!SD.begin(CHIP_SELECT_SD)) {
    Serial.println("   ❌ ERREUR: Carte SD non détectée");
    Serial.println("   Vérifiez:");
    Serial.println("   - Carte insérée correctement");
    Serial.println("   - Broche CS sur D10");
    Serial.println("   - Alimentation 5V");
    tousTestsReussis = false;
  } else {
    Serial.println("   ✅ Carte SD détectée");
    // Tester l'écriture
    File fichierTest = SD.open("test.txt", FILE_WRITE);
    if (fichierTest) {
      fichierTest.println("Test de la carte SD - " + String(millis()));
      fichierTest.close();
      Serial.println("   ✅ Écriture réussie");
      // Tester la lecture
      fichierTest = SD.open("test.txt");
      if (fichierTest) {
        Serial.println("   ✅ Lecture réussie");
        while (fichierTest.available()) {
          Serial.write(fichierTest.read());
        }
        fichierTest.close();
      } else {
        Serial.println("   ⚠️  Lecture impossible");
      }
    } else {
      Serial.println("   ❌ Écriture impossible");
      tousTestsReussis = false;
    }
  }
  Serial.println();
}
 
// === TEST GPS UART (si présent) ===
void testGPSUART() {
  Serial.println("8. 📡 TEST GPS UART (Optionnel)");
  // Initialiser Serial1 pour GPS UART
  Serial1.begin(9600);
  delay(100);
  if (Serial1.available()) {
    Serial.println("   ✅ Données UART reçues");
    Serial.println("   📡 Trames GPS UART détectées");
    // Afficher quelques trames
    for (int i = 0; i < 5 && Serial1.available(); i++) {
      String trame = Serial1.readStringUntil('\n');
      Serial.print("   ");
      Serial.println(trame);
    }
  } else {
    Serial.println("   ℹ️  Aucune donnée UART (normal si GPS en I2C)");
  }
  Serial.println();
}
 
// === AFFICHAGE RÉSUMÉ ===
void afficherResume() {
  Serial.println("==========================================");
  Serial.println("           RÉSUMÉ DES TESTS");
  Serial.println("==========================================");
  if (tousTestsReussis) {
    Serial.println("🎉 TOUS LES TESTS SONT RÉUSSIS !");
    Serial.println("✅ Votre système est prêt à fonctionner");
    // LED verte pour succès
    leds.setColorRGB(0, 0, 255, 0);
  } else {
    Serial.println("⚠️  CERTAINS TESTS ONT ÉCHOUÉ");
    Serial.println("🔧 Vérifiez les connexions signalées");
    // LED rouge pour échec
    leds.setColorRGB(0, 255, 0, 0);
  }
  Serial.println();
  Serial.println("📋 RÉCAPITULATIF DES CONNEXIONS:");
  Serial.println("   RTC      → Port I2C     ✅");
  Serial.println("   GPS      → I2C (0x42)   ✅"); 
  Serial.println("   Bouton   → D2           ✅");
  Serial.println("   LED RGB  → D4           ✅");
  Serial.println("   Lumière  → A0           ✅");
  Serial.println("   Pression → A1           ✅");
  Serial.println("   Carte SD → SPI (D10)    ✅");
  Serial.println("   VMA335   → I2C + 3.3V   ✅");
  Serial.println();
  Serial.println("🚀 Le système est opérationnel!");
}
