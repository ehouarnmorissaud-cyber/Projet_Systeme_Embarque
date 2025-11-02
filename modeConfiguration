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

// ========== Fonction mode Configuration ==========

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
        Serial.print("LUMIN mis à jour : "); Serial.println(LUMIN);
      }
      else if (commande == "LUMIN_LOW" && valeur >= 0 && valeur <= 1023) {
        if (valeur > LUMIN_HIGH) {
          Serial.println("Erreur : LUMIN_LOW ne peut pas être supérieur à LUMIN_HIGH");
        } 
        else {
          LUMIN_LOW = valeur;
          Serial.print("LUMIN_LOW mis à jour : "); Serial.println(LUMIN_LOW);
        }
      }
      else if (commande == "LUMIN_HIGH" && valeur >= 0 && valeur <= 1023) {
        if (valeur < LUMIN_LOW)
        {
          Serial.println("Erreur : LUMIN_HIGH ne peut pas être inférieur à LUMIN_LOW");
        }
        else {
        LUMIN_HIGH = valeur;
        Serial.print("LUMIN_HIGH mis à jour : "); Serial.println(LUMIN_HIGH);
        }
      }
      else if (commande == "TEMP_AIR" && (valeur == 0 || valeur == 1)) {
        TEMP_AIR = valeur;
        Serial.print("TEMP_AIR mis à jour : "); Serial.println(TEMP_AIR);
      }
      else if (commande == "MIN_TEMP_AIR" && valeur >= -40 && valeur <= 85) {
        if (valeur > MAX_TEMP_AIR)
        {
          Serial.println("Erreur : MIN_TEMP_AIR ne peut pas être supérieur à MAX_TEMP_AIR");
        }
        else {
        MIN_TEMP_AIR = valeur;
        Serial.print("MIN_TEMP_AIR mis à jour : "); Serial.println(MIN_TEMP_AIR);
        }
      }
      else if (commande == "MAX_TEMP_AIR" && valeur >= -40 && valeur <= 85) {
        if (valeur < MIN_TEMP_AIR)
        {
          Serial.println("Erreur : MAX_TEMP_AIR ne peut pas être inférieur à MIN_TEMP_AIR");
        }
        else {
        MAX_TEMP_AIR = valeur;
        Serial.print("MAX_TEMP_AIR mis à jour : "); Serial.println(MAX_TEMP_AIR);
        }
      }
      else if (commande == "HYGR" && (valeur == 0 || valeur == 1)) {
        HYGR = valeur;
        Serial.print("HYGR mis à jour : "); Serial.println(HYGR);
      }
      else if (commande == "HYGR_MINT" && valeur >= -40 && valeur <= 85) {
        if (valeur > HYGR_MAXT)
        {
          Serial.println("Erreur : HYGR_MINT ne peut pas être supérieur à HYGR_MAXT");
        }
        else {
        HYGR_MINT = valeur;
        Serial.print("HYGR_MINT mis à jour : "); Serial.println(HYGR_MINT);
        }
      }
      else if (commande == "HYGR_MAXT" && valeur >= -40 && valeur <= 85) {
        if (valeur < HYGR_MINT)
        {
          Serial.println("Erreur : HYGR_MAXT ne peut pas être inférieur à HYGR_MINT");
        }
        else {
        HYGR_MAXT = valeur;
        Serial.print("HYGR_MAXT mis à jour : "); Serial.println(HYGR_MAXT);
        }
      }
      else if (commande == "PRESSURE" && (valeur == 0 || valeur == 1)) {
        PRESSURE = valeur;
        Serial.print("PRESSURE mis à jour : "); Serial.println(PRESSURE);
      }
      else if (commande == "PRESSURE_MIN" && valeur >= 300 && valeur <= 1100) {
        if (valeur > PRESSURE_MAX)
        {
          Serial.println("Erreur : PRESSURE_MIN ne peut pas être supérieur à PRESSURE_MAX");
        }
        else {  
        PRESSURE_MIN = valeur;
        Serial.print("PRESSURE_MIN mis à jour : "); Serial.println(PRESSURE_MIN);
        }
      }
      else if (commande == "PRESSURE_MAX" && valeur >= 300 && valeur <= 1100) {
        if (valeur < PRESSURE_MIN)
        {
          Serial.println("Erreur : PRESSURE_MAX ne peut pas être inférieur à PRESSURE_MIN");
        }
        else {
        PRESSURE_MAX = valeur;
        Serial.print("PRESSURE_MAX mis à jour : "); Serial.println(PRESSURE_MAX);
        }
      }
      else if (commande == "LOG_INTERVAL" && valeur > 0) {
        LOG_INTERVAL = valeur;
        Serial.print("LOG_INTERVAL mis à jour : "); Serial.println(LOG_INTERVAL);
      }
      else if (commande == "FILE_MAX_SIZE" && valeur > 0) {
        FILE_MAX_SIZE = valeur;
        Serial.print("FILE_MAX_SIZE mis à jour : "); Serial.println(FILE_MAX_SIZE);
      }
      else if (commande == "TIMEOUT" && valeur > 0) {
        TIMEOUT = valeur;
        Serial.print("TIMEOUT mis à jour : "); Serial.println(TIMEOUT);
      }
      else {
        Serial.print("La valeur rentrée est n'est pas correcte pour la commande "); Serial.println(commande);
      }
    }
    else { // Si il n'y a eu aucun "=" dans la ligne, et que la valeur est donc à -1
      Serial.println("La commande saisie n'est pas valide, merci d'utiliser le format suivant :'COMMANDE=VALEUR'");
    }

    }
    
    
  if (millis() - lastActivity > 1800000) { // 30 min
    mode_actuel = 0; // retour au mode standard
  }
}
