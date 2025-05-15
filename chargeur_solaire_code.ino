#include "TM1651.h"  // Inclusion de la bibliothèque pour contrôler le module indicateur de batterie à barres DFRobot (TM1651)

// === Définition des broches ===
#define CLK 10        // Broche horloge pour la communication avec le module TM1651
#define DIO 9         // Broche de données pour le module TM1651
#define cellPin A0    // Broche analogique utilisée pour mesurer la tension de la batterie
#define buttonPin 11  // Broche utilisée pour le bouton-poussoir d’activation/désactivation de l'afficheur

// === Création des objets et constantes ===
TM1651 batteryDisplay(CLK, DIO);  // Objet pour contrôler l’indicateur de batterie
const int tempPin = A1;           // Broche analogique pour lire la température via un capteur MCP9700

// === Variables globales ===
float voltage, temperature, batteryVoltage; // Stockage des mesures
int buttonState = 0;             // État actuel du bouton
int lastButtonState = 0;         // Dernier état du bouton
int displayState = 0;            // État de l’affichage batterie (0 = OFF, 1 = ON)

int lastLevel = -1;              // Dernier niveau de batterie affiché (pour éviter les mises à jour inutiles)

unsigned long lastUpdateTime = 0;               // Dernier moment de mise à jour des données
const unsigned long updateInterval = 1000;      // Intervalle d’actualisation (en millisecondes)

// === Variables pour l’anti-rebond du bouton ===
unsigned long lastDebounceTime = 0;             // Dernier moment de changement d’état du bouton
const unsigned long debounceDelay = 50;         // Délai pour éviter le rebond mécanique (50 ms)

void setup() {
  Serial.begin(9600);           // Initialisation du port série pour le débogage
  Serial1.begin(9600);          // Port série secondaire pour l’écran LCD RS232

  // === Configuration de l’écran LCD ===
  Serial1.write(0xFE); Serial1.write(0x53); Serial1.write(0x08); // Réglage de la luminosité (max)
  Serial1.write(0xFE); Serial1.write(0x51);                      // Efface l’écran LCD au démarrage

  // === Initialisation de l’indicateur de batterie ===
  batteryDisplay.init();              // Initialisation du module TM1651
  batteryDisplay.set(7);              // Réglage de la luminosité de l’afficheur
  batteryDisplay.frame(FRAME_OFF);    // Désactivation du cadre lumineux au départ
  batteryDisplay.displayLevel(0);     // Aucun niveau affiché initialement

  pinMode(cellPin, INPUT);            // Définir la broche de mesure batterie en entrée
  pinMode(buttonPin, INPUT);          // Définir la broche du bouton en entrée
}

void loop() {
  // === Lecture et anti-rebond du bouton ===
  int reading = digitalRead(buttonPin);         // Lecture de l’état du bouton
  if (reading != lastButtonState) {
    lastDebounceTime = millis();                // Mise à jour du temps si changement détecté
  }

  if ((millis() - lastDebounceTime) > debounceDelay) {
    if (reading != buttonState) {
      buttonState = reading;                    // Mise à jour de l’état stable du bouton
      if (buttonState == HIGH) {                // Si bouton pressé
        displayState = 1 - displayState;        // Inversion de l’état d’affichage (ON/OFF)

        if (displayState == 0) {                // Si affichage désactivé
          batteryDisplay.displayLevel(0);       // Éteindre les barres de niveau
          batteryDisplay.frame(FRAME_OFF);      // Masquer le cadre lumineux
          lastLevel = -1;                       // Forcer la mise à jour au prochain affichage
        }
      }
    }
  }
  lastButtonState = reading;                    // Mise à jour de l’état précédent du bouton

  // === Mise à jour de la température et de la tension chaque seconde ===
  if (millis() - lastUpdateTime > updateInterval) {
    lastUpdateTime = millis();

    // === Lecture de la température (capteur MCP9700) ===
    int tempValue = analogRead(tempPin);                // Lire la tension analogique du capteur
    voltage = tempValue * (5.0 / 1023.0);                // Convertir en tension (0–5 V)
    temperature = (voltage - 0.5) * 100;                 // Formule MCP9700 : 500 mV à 0 °C, 10 mV/°C

    // === Lecture de la tension batterie (A0) ===
    int batteryADC = analogRead(cellPin);               // Lire la tension analogique de la batterie
    batteryVoltage = batteryADC * (5.0 / 1023.0);       // Conversion de la valeur ADC en volts

    // === Affichage de la température sur le LCD ===
    Serial1.write(0xFE); Serial1.write(0x45); Serial1.write(0x00); // Positionner le curseur au début
    Serial1.print("Temperature: ");
    Serial1.print(temperature, 1);                       // Affiche la température avec 1 décimale
    Serial1.write(223);                                  // Symbole de degré °
    Serial1.print("C    ");                              // Espaces pour effacer d’anciens caractères
  }

  // === Mise à jour de l’indicateur de batterie si activé ===
  if (displayState == 1) {
    updateBatteryIndicator();
  }
}

// === Fonction pour mettre à jour les barres de l’indicateur de batterie ===
void updateBatteryIndicator() {
  int val = analogRead(cellPin);                         // Lire la valeur brute ADC
  float voltage = val * (5.0 / 1023.0);                  // Conversion en volts

  uint8_t level = 0;                                     // Niveau de batterie à afficher (0 à 6)

  // === Seuils réalistes pour batterie Li-ion 3.0–4.2 V ===
  if (voltage >= 4.1) level = 6;
  else if (voltage >= 3.9) level = 5;
  else if (voltage >= 3.7) level = 4;
  else if (voltage >= 3.5) level = 3;
  else if (voltage >= 3.3) level = 2;
  else if (voltage >= 3.1) level = 1;
  else level = 0;

  // Mettre à jour l’afficheur seulement si le niveau change
  if (level != lastLevel) {
    batteryDisplay.frame(FRAME_ON);               // Afficher le cadre lumineux
    batteryDisplay.displayLevel(level);           // Afficher les barres correspondantes
    lastLevel = level;                            // Enregistrer le niveau actuel
  }
}
