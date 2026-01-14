#include <Wire.h>
#include <RTClib.h>

RTC_DS3231 rtc;

void setup() {
  Serial.begin(9600);
  while (!Serial);

  Serial.println("=== RESET RTC DS3231 ===");

  Wire.begin();

  if (!rtc.begin()) {
    Serial.println("❌ RTC non détectée");
    while (1);
  }

  Serial.println("✅ RTC détectée");

  // Force le redémarrage de l'oscillateur
  rtc.disable32K();
  rtc.clearAlarm(1);
  rtc.clearAlarm(2);
  rtc.writeSqwPinMode(DS3231_OFF);

  // Réglage forcé de l'heure
  rtc.adjust(DateTime(F(__DATE__), F(__TIME__)));
  Serial.println("⏱ Heure réglée (compilation)");

}

void loop() {
  DateTime now = rtc.now();

  Serial.print("Heure : ");
  Serial.print(now.hour()); Serial.print(":");
  Serial.print(now.minute()); Serial.print(":");
  Serial.println(now.second());

  delay(1000);
}
