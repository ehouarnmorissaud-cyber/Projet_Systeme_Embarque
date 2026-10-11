# Worldwide Weather Watcher (W3)

> Prototype de station météo embarquée pour navires, projet de système embarqué pour l'**AIVM** (Agence Internationale pour la Vigilance Météorologique)

## Contexte

L'AIVM déploie des navires de surveillance équipés de stations météo chargées de mesurer les paramètres qui influencent la formation de cyclones et autres catastrophes naturelles. À long terme, les navires pourront échanger leurs données pour **prévoir les catastrophes naturelles**.

La station doit être **simple, efficace et pilotable par un membre d'équipage**, avec une documentation utilisateur dédiée.

## Technologies

C/C++ · Arduino · ATmega328 · I2C / SPI / UART · UML / SysML

## Matériel

**Microcontrôleur :** AVR ATmega328 (carte Arduino `[Uno / Nano]`)

| Composant | Rôle | Interface |
|-----------|------|-----------|
| Lecteur carte SD | Sauvegarde des mesures | SPI |
| Horloge RTC | Date et heure | I2C |
| LED RGB | État du système | 2-wire |
| 2 boutons poussoirs | Interaction avec l'utilisateur | Numérique |

**Capteurs**

| Mesure | Capteur | Interface |
|--------|---------|-----------|
| Pression atmosphérique | `[modèle]` | I2C / SPI |
| Température de l'air | `[modèle]` | I2C / SPI |
| Hygrométrie | `[modèle]` | I2C / SPI |
| Position | GPS `[modèle]` | UART |
| Luminosité | `[modèle]` | Analogique |

**Modules tiers (intégration ultérieure) :** température de l'eau (analogique), courant marin (I2C), vent (I2C), particules fines (2-wire).
 
 
## Technologies

C/C++ · Arduino · ATmega328 · I2C / SPI / UART · UML / SysML

---
*Projet pédagogique réalisé dans le cadre de [nom de l'école / bloc], année 2025-2026.*
