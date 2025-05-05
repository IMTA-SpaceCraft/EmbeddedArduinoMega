#include <Wire.h>
#include <SD.h>
#include <SPI.h>
#include <Adafruit_Sensor.h>
#include "Adafruit_BMP3XX.h"
#include "Adafruit_SGP40.h"
#include "Adafruit_VEML7700.h"
#include <Adafruit_LSM9DS1.h>  

// Définition des capteurs
Adafruit_BMP3XX bmp0; // Press intérieur
Adafruit_BMP3XX bmp1; // Press exté 1
Adafruit_BMP3XX bmp7; // Press exté 2
Adafruit_SGP40 sgp0; // Qair intérieur
Adafruit_SGP40 sgp1; // Qair exté 1
Adafruit_SGP40 sgp7; // Qair exté 2
Adafruit_VEML7700 veml; // Luxmetre
Adafruit_LSM9DS1 lsm = Adafruit_LSM9DS1(); // Accel 0

byte Xon = 17 ; // La valeur 17 est affectée à Xon conformément aux règles du protocle de communication Xon/Xoff.
unsigned int RX ; // Variable permettant la lecture du buffer de la carte Arduino reliée au Kikiwi via la PIN 2,
String Date = "" ; // Création d'une chaine de caractères stockant le date de la mesure
String Mesure = "" ; // Création d'une chaine de caractère pour stocker la mesure
unsigned int i = 0 ; // Indice pour incrémenter une boucle

byte Xoff = 19 ; // La valeur 19 est affectée à Xoff conformément aux règles du protocle de communication Xon/Xoff.

String E = "" ; // Chaine de caractères nommée émission destinée à écrire le buffer associé à la PIN 3
String C = "" ; // Chaine stockant le dernier caractère de chaque trame
unsigned long delai = 30000 ; // Durée souhaitée entre chaque mesure en millisecondes
unsigned long temps = 0 ; // Stockage du temps

boolean Flag = LOW ; // Booléen indiquant si le Kikiwi peut recevoir des caractères.

#define SEALEVELPRESSURE_HPA (1013.25)  // Pression atmosphérique de référence
#define AM2301B_ADDR 0x38  // Adresse I2C du capteur d'humidité
#define SD_CS_PIN 53  // Chip Select pour Arduino Mega

// Grandeurs associées aux capteurs
float pres_bmp0;
float temp_bmp0;
float alt_bmp0;
float pres_bmp1;
float temp_bmp1;
float alt_bmp1;
float pres_bmp7;
float temp_bmp7;
float alt_bmp7;
float lux;
float acc_x;
float acc_y;
float acc_z;
float mag_x;
float mag_y;
float mag_z;
float gyr_x;
float gyr_y;
float gyr_z;

String serial_input;
String trames[11];
String logEntry;

// Initialisation des capteurs
void setup() {
    Serial.begin(115200);
    Serial1.begin(115200) ; // La vitesse est programmée pour être compatible de celle du Kikiwi. Cette vitesse est imposée par le fabricant du Kikiwi
    while (!Serial) { delay(10); }  

    Wire.begin();
    
    // Initialisation SD
    if (!SD.begin(SD_CS_PIN)) {
        Serial.println("Carte SD non connectee !!");
        return;  // Arrêt si la carte SD ne fonctionne pas
    }

    selectI2CBus(0);
    
    // BMP388 0
    if (!bmp0.begin_I2C(0x76)) {
        Serial.println("Erreur BMP388 0! Ignore");
    }
    bmp0.setTemperatureOversampling(BMP3_OVERSAMPLING_8X);
    bmp0.setPressureOversampling(BMP3_OVERSAMPLING_4X);
    bmp0.setIIRFilterCoeff(BMP3_IIR_FILTER_COEFF_3);
    bmp0.setOutputDataRate(BMP3_ODR_50_HZ);

    // SGP40 0
    if (!sgp0.begin()) {
        Serial.println("Erreur SGP40 0! Ignore");
    }

    // VEML7700
    if (!veml.begin()) {
        Serial.println("Erreur VEML7700! Ignore");
    }

    // LSM9DS1
    if (!lsm.begin()) {
        Serial.println("Erreur LSM9DS1 ! Ignore");
    }
    
    setupLSM9DS1();

    selectI2CBus(1);

    // BMP388 1
    if (!bmp1.begin_I2C(0x76)) {
        Serial.println("Erreur BMP388 1! Ignore");
    }
    bmp1.setTemperatureOversampling(BMP3_OVERSAMPLING_8X);
    bmp1.setPressureOversampling(BMP3_OVERSAMPLING_4X);
    bmp1.setIIRFilterCoeff(BMP3_IIR_FILTER_COEFF_3);
    bmp1.setOutputDataRate(BMP3_ODR_50_HZ);

    // SGP40 1
    if (!sgp1.begin()) {
        Serial.println("Erreur SGP40 1! Ignore");
    }

    selectI2CBus(7);

    // BMP388 7
    if (!bmp7.begin_I2C(0x76)) {
        Serial.println("Erreur BMP388 7! Ignore");
    }
    bmp7.setTemperatureOversampling(BMP3_OVERSAMPLING_8X);
    bmp7.setPressureOversampling(BMP3_OVERSAMPLING_4X);
    bmp7.setIIRFilterCoeff(BMP3_IIR_FILTER_COEFF_3);
    bmp7.setOutputDataRate(BMP3_ODR_50_HZ);

    // SGP40 7
    if (!sgp7.begin()) {
        Serial.println("Erreur SGP40 7! Ignore");
    }

    delay(100);
}

bool listesEgales(String listeA[], String listeB[], int taille) {
    for (int i = 0; i < taille; i++) {
        if (listeA[i] != listeB[i]) {  // Comparaison directe des objets String
            return false;
        }
    }
    return true;
}

// Sélecteur de bus I2C
void selectI2CBus(uint8_t bus) {
    if (bus > 7) return; // Vérification du numéro de bus
    Wire.beginTransmission(0x70); // Adresse du TCA9548A
    Wire.write(1 << bus); // Active le bus choisi
    Wire.endTransmission();
}

// Calculer l'horodatage en heures, minutes, secondes
String horodatage(){
    unsigned long timeMillis = millis() / 1000;  // Convertir en secondes
    int hours = (timeMillis / 3600) % 24;
    int minutes = (timeMillis / 60) % 60;
    int seconds = timeMillis % 60;

    // Construire la chaîne d'horodatage formatée
    char timestamp[12]; 
    sprintf(timestamp, "%02dh%02dm%02ds", hours, minutes, seconds);

    return String(timestamp);
}

// Configuration du LSM9DS1
void setupLSM9DS1() {
    lsm.setupAccel(lsm.LSM9DS1_ACCELRANGE_2G, lsm.LSM9DS1_ACCELDATARATE_10HZ);
    lsm.setupMag(lsm.LSM9DS1_MAGGAIN_4GAUSS);
    lsm.setupGyro(lsm.LSM9DS1_GYROSCALE_245DPS);
}

// Lecture du capteur AM2301B (humidité + température)
bool readAM2301B(float &humidity, float &temperature) {
    Wire.beginTransmission(AM2301B_ADDR);
    Wire.write(0xAC);
    Wire.write(0x33);
    Wire.write(0x00);
    if (Wire.endTransmission() != 0) return false;

    delay(80);  
    Wire.requestFrom(AM2301B_ADDR, 7);
    if (Wire.available() != 7) return false;

    uint8_t hum_H = Wire.read();
    uint8_t hum_M = Wire.read();
    uint8_t hum_L = Wire.read();
    uint8_t temp_H = Wire.read();
    uint8_t temp_M = Wire.read();
    uint8_t temp_L = Wire.read();
    Wire.read();  // Ignorer CRC

    uint32_t rawHumidity = ((uint32_t)hum_H << 12) | ((uint32_t)hum_M << 4) | ((hum_L >> 4) & 0x0F);
    uint32_t rawTemperature = (((uint32_t)temp_H & 0x0F) << 16) | ((uint32_t)temp_M << 8) | temp_L;

    humidity = (rawHumidity * 1000.0) / 1048576.0;
    temperature = ((rawTemperature * 200.0) / 1048576.0) - 50.0;
    
    return true;
}

// Boucle principale
void loop() {

  if (millis() - temps > delai) {
    temps = millis();
  
    selectI2CBus(0);

    // Lecture AM2301B 0
    float humidity0, temperature0;
    if (readAM2301B(humidity0, temperature0)) {
        
    } else {
        Serial.println("Erreur de lecture AM2301B 0!");
    }

    // Lecture SGP40 0
    uint16_t sraw0 = sgp0.measureRaw(temperature0, humidity0);
    int32_t voc_index0 = sgp0.measureVocIndex(temperature0, humidity0);

    // Lecture BMP388 0
    if (bmp0.performReading()) {
        pres_bmp0 = bmp0.pressure / 100.0;
        temp_bmp0 = bmp0.temperature;
        alt_bmp0 = bmp0.readAltitude(SEALEVELPRESSURE_HPA);
    } else {
        Serial.println("Erreur de lecture BMP388 0!");
    }

    // Lecture VEML7700
    lux = veml.readLux(VEML_LUX_AUTO);

    // Lecture LSM9DS1
    lsm.read();
    sensors_event_t a, m, g, temp;
    lsm.getEvent(&a, &m, &g, &temp);

    acc_x = a.acceleration.x;
    acc_y = a.acceleration.y;
    acc_z = a.acceleration.z;
    
    mag_x = m.magnetic.x;
    mag_y = m.magnetic.y;
    mag_z = m.magnetic.z;

    gyr_x = g.gyro.x;
    gyr_y = g.gyro.y;
    gyr_z = g.gyro.z;

     // Lecture de la valeur analogique sur A0 (panneaux solaires)
    int valeurADC = analogRead(A0);
  
    // Conversion de la valeur ADC en tension (en utilisant 5V comme référence)
    float tension = valeurADC * ((12 * 5.0) / 1023.0);
  
    // Affichage de la tension sur le moniteur série

    selectI2CBus(1);

    // Lecture AM2301B 1
    float humidity1, temperature1;
    if (readAM2301B(humidity1, temperature1)) {

    } else {
        Serial.println("Erreur de lecture AM2301B 1!");
    }

    // Lecture SGP40 1
    uint16_t sraw1 = sgp1.measureRaw(temperature1, humidity1);
    int32_t voc_index1 = sgp1.measureVocIndex(temperature1, humidity1);

    // Lecture BMP388 1
    if (bmp1.performReading()) {
        pres_bmp1 = bmp1.pressure / 100.0;
        temp_bmp1 = bmp1.temperature;
        alt_bmp1 = bmp1.readAltitude(SEALEVELPRESSURE_HPA);
    } else {
        Serial.println("Erreur de lecture BMP388 1!");
    }

    selectI2CBus(7);

    // Lecture AM2301B 7
    float humidity7, temperature7;
    if (readAM2301B(humidity7, temperature7)) {

    } else {
        Serial.println("Erreur de lecture AM2301B 7!");
    }

    // Lecture SGP40 7
    uint16_t sraw7 = sgp7.measureRaw(temperature7, humidity7);
    int32_t voc_index7 = sgp7.measureVocIndex(temperature7, humidity7);

    // Lecture BMP388 7
    if (bmp7.performReading()) {
        pres_bmp7 = bmp7.pressure / 100.0;
        temp_bmp7 = bmp7.temperature;
        alt_bmp7 = bmp7.readAltitude(SEALEVELPRESSURE_HPA);
        
    } else {
        Serial.println("Erreur de lecture BMP388 7!");
    }

    // Construire la ligne à écrire (SD)
    logEntry = horodatage() + "," + String(humidity0) + "," + String(temperature0) + "," + String(sraw0) + "," + String(voc_index0) + "," + String(pres_bmp0) + "," + String(temp_bmp0) + "," + String(alt_bmp0) + "," + String(lux) + "," + String(acc_x) + "," + String(acc_y) + "," + String(acc_z) + "," + String(mag_x) + "," + String(mag_y) + "," + String(mag_z) + "," + String(gyr_x) + "," + String(gyr_y) + "," + String(gyr_z) + "," + String(tension) + "," + String(humidity1) + "," + String(temperature1) + "," + String(sraw1) + "," + String(voc_index1) + "," + String(pres_bmp1) + "," + String(temp_bmp1) + "," + String(alt_bmp1) + "," + String(humidity7) + "," + String(temperature7) + "," + String(sraw7) + "," + String(voc_index7) + "," + String(pres_bmp7) + "," + String(temp_bmp7) + "," + String(alt_bmp7) + ";\n\n";
    E = E + logEntry;
    // Envoie sur serial
    //Serial.println(logEntry);
    
    // Ouvrir le fichier en mode écriture
    File dataFile = SD.open("log_test.txt", O_WRITE | O_APPEND);
    
    if (!dataFile) {
          Serial.println("Erreur ouverture fichier !");
        return;  // Quitte la fonction si le fichier ne peut pas être ouvert
    }
    
    if (dataFile) {
        dataFile.println(logEntry);  // Écrire la ligne dans le fichier
        dataFile.close();  // Fermer le fichier pour enregistrer les modifications
    }

  } // FIN LECTURE CAPTEURS
  
    // TELECOM KIKIWI VIA UART PORT SERIE

    int index = 1;
    trames[0] = "TT";  // Trame vide pour annoncer l'arrivée des données
    
    for (int j = 0; j < logEntry.length(); j += 48) {
        trames[index++] = logEntry.substring(j, j + 48);
    }
    
    while (Serial1.available()) {
        RX = Serial1.read();
        if (RX == Xon) {
            Serial1.print(trames[i]);  // Envoi d'une seule trame
            Serial.println(trames[i]);  // Debug sur le PC
            i++;
            if (i >= 11) i = 0;  // Réinitialisation de l’index
        }
    }
} // FIN LOOP
