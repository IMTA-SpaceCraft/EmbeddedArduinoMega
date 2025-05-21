# Code Arduino pour la Mega principale

## Prérequis
Les bibliothèques Arduino suivantes sont nécessaires : 
```
#include <Wire.h>
#include <SD.h>
#include <SPI.h>
#include <Adafruit_Sensor.h>
#include "Adafruit_BMP3XX.h"
#include "Adafruit_SGP40.h"
#include "Adafruit_VEML7700.h"
#include <Adafruit_LSM9DS1.h>
```
## Architecture hardware
Le système embarqué est composé de plusieurs briques harwares élémentaires :
* Une carte Arduino Mega, alimentée par une pile 9V
* Un PCB dit "de routage", contenant une LED de fonctionnement, un hub d'alimentation pour les capteurs ainsi qu'un multiplexeur I2C
* Un PCB dit "de puissance", avec une LED de fonctionnement, qui permet les conversions de tension pour l'UART de la Kikiwi et la mesure de tension du panneau solaire
* Un montage de capteurs extérieurs (sur le bus I2C 1 du multiplexeur)
* Un second montage de capteurs extérieur, redondant avec le premier (bus I2C 7 du multiplexeur)
* Un montage de capteurs interne (bus I2C 0 du multiplexeur)

## Composants et datasheet
Les composants utilisés ainsi que les datasheet se trouvent sur le drive spacecraft :
Commande 1 : https://docs.google.com/spreadsheets/d/11y-STKDh9emp1uLKqlTc0yuigvIuqDNgb3BU6f3UHWU/edit?usp=drive_link

Commande 2 : https://docs.google.com/spreadsheets/d/1ghB6hktbWF-RtLrQcKJszQ0cAxbrcWrscFNlA4eMYhE/edit?usp=drive_link

## Architecture logicielle
Dans cette première version, par manque de temps, le code embarqué est séquentiel, et donc relativement lent. Une amélioration future serait d'introduire une logique par interruption sur microcontroleur type FEMTO, afin d'évoluer sur un RTOS permettant une période d'exécution d'une milliseconde.

Globalement, le pseudo-code est le suivant : 

```
Définir les capteurs (BMP, SGP40, LSM9DS1, VEML7700, AM2301B, etc.)

Initialiser les variables :
  - Xon, Xoff : commandes de communication Xon/Xoff
  - Temps, delai : gestion du temps
  - LogEntry : stocke les données à enregistrer
  - Trames : tableau pour découper les trames à envoyer

Initialiser les capteurs et la carte SD :
  - Vérifier la connexion à la carte SD
  - Initialiser chaque capteur (BMP3XX, SGP40, VEML7700, LSM9DS1)
  - Configurer les capteurs (résolutions, fréquences, etc.)
  - Gérer les buses I2C via le sélecteur (TCA9548A)

Boucle principale (`loop`) :

1. **Gestion du temps** :
   - Si le temps écoulé depuis la dernière mesure est supérieur au délai défini (`delai`), exécuter la lecture des capteurs.

2. **Sélectionner et lire les capteurs** :

   Pour chaque capteur (capteurs de température, pression, humidité, lumière, etc.) :
   - Lire les valeurs des capteurs (comme la pression du BMP388, les gaz du SGP40, la lumière du VEML7700, l'accélération du LSM9DS1, etc.)
   - Vérifier que la lecture est correcte, sinon afficher un message d'erreur.

3. **Collecte des données** :
   - Calculer et enregistrer les valeurs lues dans des variables (comme `pres_bmp0`, `temp_bmp0`, etc.).
   - Lire les valeurs analogiques (par exemple, la tension sur A0).

4. **Enregistrer les données dans un fichier sur la carte SD** :
   - Construire une chaîne `logEntry` avec les valeurs lues (horodatage + capteurs).
   - Ouvrir un fichier (`log_test.txt`) en mode ajout.
   - Si le fichier s'ouvre correctement, écrire la ligne dans le fichier et le fermer.

5. **Communication avec le Kikiwi via UART** :
   - Diviser la chaîne `logEntry` en trames de 48 caractères.
   - Attendre la commande `Xon` via `Serial1` pour envoyer une trame.
   - Envoyer chaque trame dans l'ordre via `Serial1`.
   - Répéter l'envoi des trames jusqu'à ce que toutes les trames soient envoyées.

6. **Réinitialiser l'index des trames** si toutes ont été envoyées.
```
