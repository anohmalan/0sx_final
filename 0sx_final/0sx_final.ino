#include <Arduino.h>
#include <U8g2lib.h>
#include <Wire.h>
#include <LCD_I2C.h>
#include <HCSR04.h>
#include <AccelStepper.h>
#include "PorteAutomatique.h"
#include "Alarm.h"
#include <WiFiEspAT.h>
#include <PubSubClient.h>

#define DEVICE_NAME "ProjetFinal"
// Broches pour UltraSon
#define TRIGGER_PIN 9
#define ECHO_PIN 10

const char ssid[] = "TechniquesInformatique-Etudiant";  // your network SSID (name)
const char pass[] = "shawi123";                         // your network password (use for WPA, or use as key for WEP)
// const char ssid[] = "Mywifi";  // your network SSID (name)
// const char pass[] = "123456789";


#define AT_BAUD_RATE 115200

int blinkRate = 500;

#define MQTT_PORT 1883
#define MQTT_USER "etdshawi"
#define MQTT_PASS "shawi123"
#define MQTT_ETD "8"

// Serveur MQTT du prof
const char* mqttServer = "216.128.180.194";

// Broches pour DEL RGB et buzzer
const int R_PIN = 2;
const int G_PIN = 3;
const int B_PIN = 4;
const int BUZZER_PIN = 5;

int currentRed = 0;
int currentGreen = 0;
int currentBlue = 0;

// Broches pour Moteur
#define MOTOR_INTERFACE_TYPE 4
#define IN_1 53
#define IN_2 51
#define IN_3 49
#define IN_4 47

// Initialisation du client MQTT
WiFiClient espClient;
PubSubClient client(espClient);

// Broches pour l'afficheur Matrix
#define CLK_PIN 30
#define DIN_PIN 34
#define CS_PIN 32

unsigned long previousTime = 0;
unsigned long lastLCDUpdate = 0;
unsigned long lastStartupDisplay = 0;
bool startupDisplayed = false;

unsigned long rateMatrix = 3000;

float distance;

int potentiometrePin = A0;
const int convertir = 101;
const int potentiometreMax = 1023;
int valeurDeDepart = 0;

String lastLine1 = "";
String lastLine2 = "";
bool lcdDirty = false;

bool motorIsActivate = false;

int pasParTour = 2038;
int angleFeme = 10;
int angleOuvert = 170;
int distanceOuverture = 30;
int distanceFermeture = 60;

// Objet Alarme
Alarm alarme(R_PIN, G_PIN, B_PIN, BUZZER_PIN, distance);

// Objet Porte
PorteAutomatique porte(IN_1, IN_2, IN_3, IN_4, distance);

LCD_I2C lcd(0x27, 16, 2);
HCSR04 hc(TRIGGER_PIN, ECHO_PIN);

// Objet U8G2
U8G2_MAX7219_8X8_F_4W_SW_SPI u8g2(
  U8G2_R0, CLK_PIN, DIN_PIN, CS_PIN,
  U8X8_PIN_NONE, U8X8_PIN_NONE);

static const unsigned char limiteError[] U8X8_PROGMEM = {
  0b00111100,
  0b01000010,
  0b10100001,
  0b10010001,
  0b10001001,
  0b10000101,
  0b01000010,
  0b00111100
};

static const unsigned char commandOk[] U8X8_PROGMEM = {
  0b00000000,
  0b10000000,
  0b01000000,
  0b00100000,
  0b00010001,
  0b00001010,
  0b00000100,
  0b00000000
};

static const unsigned char commandInconnu[] U8X8_PROGMEM = {
  0b10000001,
  0b01000010,
  0b00100100,
  0b00011000,
  0b00011000,
  0b00100100,
  0b01000010,
  0b10000001
};

void setup() {
  u8g2.begin();
  Serial.begin(9600);
  pinMode(TRIGGER_PIN, OUTPUT);
  pinMode(ECHO_PIN, INPUT);

  lcd.begin();
  lcd.backlight();
  updateStartupDisplay();

  porte.setPasParTour(pasParTour);
  porte.setAngleFerme(angleFeme);
  porte.setAngleOuvert(angleOuvert);
  porte.setDistanceOuverture(distanceOuverture);
  porte.setDistanceFermeture(distanceFermeture);

  lastStartupDisplay = millis();

  alarme.setColourA(255, 0, 0);    // Rouge
  alarme.setColourB(255, 255, 0);  // Bleu
  alarme.setVariationTiming(300);  // 300 ms entre les clignotements
  alarme.setDistance(15);          // Se déclenche si distance ≤ 15 cm
  alarme.setTimeout(3000);         // S’éteint après 3 secondes d'absence


  pinMode(LED_BUILTIN, OUTPUT);

  Serial.begin(115200);
  while (!Serial)
    ;

  Serial1.begin(AT_BAUD_RATE);
  WiFi.init(&Serial1);

  if (WiFi.status() == WL_NO_MODULE) {
    Serial.println();
    Serial.println("La communication avec le module WiFi a échoué!");
    errorState(2, 1);
  }

  WiFi.disconnect();  // pour effacer le chemin. non persistant

  WiFi.setPersistent();  // définir la connexion WiFi suivante comme persistante

  WiFi.endAP();  // pour désactiver le démarrage automatique persistant AP par défaut au démarrage

  Serial.println();
  Serial.print("Tentative de connexion à SSID: ");
  Serial.println(ssid);

  int status = WiFi.begin(ssid, pass);

  if (status == WL_CONNECTED) {
    Serial.println();
    Serial.println("Connecté au réseau WiFi.");
    printWifiStatus();
  } else {
    WiFi.disconnect();  // supprimer la connexion WiFi
    Serial.println();
    Serial.println("La connexion au réseau WiFi a échoué.");
    blinkRate = 100;
  }

  client.setServer(mqttServer, MQTT_PORT);
  client.setCallback(mqttEvent);

  if (!client.connect(DEVICE_NAME, MQTT_USER, MQTT_PASS)) {
    Serial.println("Incapable de se connecter sur le serveur MQTT");
    Serial.print("client.state : ");
    Serial.println(client.state());
  } else {
    Serial.println("Connecté sur le serveur MQTT");
  }
  client.subscribe("etd/8/#");
}

void updateStartupDisplay() {
  unsigned long rate = 2000;
  String da = "6334158         ";
  if (!startupDisplayed && millis() - lastStartupDisplay < rate) {
    lcd.setCursor(0, 0);
    lcd.print(da);
    lcd.setCursor(0, 1);
    lcd.print("Labo Final       ");
  } else if (!startupDisplayed) {
    lcd.clear();
    startupDisplayed = true;
  }
}

void updateDistance() {
  unsigned long rate = 50;
  unsigned long currentTime = millis();
  if (currentTime - previousTime >= rate) {
    distance = mesurerDistance();
    previousTime = currentTime;
  }
}

long mesurerDistance() {
  long duration;
  long distanceMesure = hc.dist();

  return distanceMesure;
}

void afficherLCD() {
  unsigned long rate = 500;
  String line1 = "Dist: " + String(distance) + " cm  ";
  String line2 = "Porte: " + String(porte.getEtatTexte());

  if (millis() - lastLCDUpdate >= rate) {
    lcd.setCursor(0, 0);
    lcd.print(line1);
    lcd.setCursor(0, 1);
    lcd.print(line2);
    lastLCDUpdate = millis();

    // Vérifie si le contenu a changé
    if (line1 != lastLine1 || line2 != lastLine2) {
      lcdDirty = true;
      lastLine1 = line1;
      lastLine2 = line2;
    }
  }
}

void limiteErreur() {
  u8g2.drawXBMP(0, 0, 8, 8, limiteError);  // Affiche le 🚫
  u8g2.sendBuffer();
}

void commandeOk() {

  u8g2.drawXBMP(0, 0, 8, 8, commandOk);  // Affiche le ✔️
  u8g2.sendBuffer();
}

void commandeInconnu() {
  u8g2.drawXBMP(0, 0, 8, 8, commandInconnu);  // Affiche le ❌

  u8g2.sendBuffer();
}

unsigned long uptime() {
  return millis();
}
long distanceForMqtt() {
  return mesurerDistance();
}
float angleMoteur() {
  return porte.getAngle();
}
// Fonction pour définir la couleur de la DEL RGB
// La couleur est définie par une chaîne hexadécimale de 6 caractères (ex: FF5733)
void SetLedColour(const char* hexColor) {
  // Assurez-vous que la chaîne hexColor commence par '#' et a une longueur de 7 caractères (ex: #FF5733)
  if (strlen(hexColor) == 6) {
    // Extraction des valeurs hexadécimales pour rouge, vert et bleu
    long number = strtol(hexColor, NULL, 16);  // Convertit hex à long

    int red = number >> 16;            // Décale de 16 bits pour obtenir le rouge
    int green = (number >> 8) & 0xFF;  // Décale de 8 bits et masque pour obtenir le vert
    int blue = number & 0xFF;          // Masque pour obtenir le bleu

    // Définissez les couleurs sur les broches de la DEL
    alarme.setColourB(red, green, blue);

    Serial.print("Couleur : ");
    Serial.print(red);
    Serial.print(", ");
    Serial.print(green);
    Serial.print(", ");
    Serial.println(blue);
  } else {
    // Gestion d'erreur si le format n'est pas correct
    Serial.println("Erreur: Format de couleur non valide.");
  }
}

// Gestions des topics après le préfixe "prof/"
void topicManagement(char* topic, byte* payload, unsigned int length) {
  // Assumer que le topic est déjà trimmé à "prof/"
  if (strcmp(topic, "color") == 0) {
    

    const char* hasthagPosition = strchr((char*)payload, '#');

    char colorStr[7];
    strncpy(colorStr, hasthagPosition + 1, sizeof(colorStr) - 1);
    colorStr[sizeof(colorStr) - 1] = '\0';
    SetLedColour(colorStr);
  }

  if (strcmp(topic, "motor") == 0) {

    const char* position = strchr((char*)payload, ':');
    char statusMotorStr[2];
    

    if (position[1] == '1') {
      Serial.println("ICI");
      porte.activer();
      motorIsActivate = true;
      Serial.println("État du moteur: allumé");
    } else {
       Serial.println("la");
      porte.desactiver();
      motorIsActivate = false;
      Serial.println("État du moteur: éteint");
    }
  }
  return;
}

// Gestion des messages reçues de la part du serveur MQTT
void mqttEvent(char* topic, byte* payload, unsigned int length) {
  const char* pretopic = "etd/8/";  // Configurer pour vos besoins
  const int pretopicLen = 6;        // Configurer pour vos besoins

  Serial.print("Message recu [");
  Serial.print(topic);
  Serial.print("] ");
  for (int i = 0; i < length; i++) {
    Serial.print((char)payload[i]);
  }
  Serial.println();

  if (strncmp(topic, pretopic, pretopicLen) != 0) {
    Serial.print("Sujet non reconnu!");
    return;
  }

  // Décale le pointeur pour ignorer "prof/"
  char* specificTopic = topic + pretopicLen;

  topicManagement(specificTopic, payload, length);
}

void analyserCommande(const String& tampon, String& commande, String& arg1, String& arg2) {
  commande = "";
  arg1 = "";
  arg2 = "";

  int firstSep = tampon.indexOf(';');
  int secondSep = tampon.indexOf(';', firstSep + 1);

  if (firstSep == -1) {
    // Pas de point-virgule, c'est peut-être "stop" ou autre commande sans paramètre
    commande = tampon;
    return;
  }

  // Extraire la commande
  commande = tampon.substring(0, firstSep);

  // Extraire arg1
  if (secondSep != -1) {
    arg1 = tampon.substring(firstSep + 1, secondSep);
    arg2 = tampon.substring(secondSep + 1);
  } else {
    // Il y a une seule valeur après la commande
    arg1 = tampon.substring(firstSep + 1);
  }
}

void serialEvent() {
  String tampon = Serial.readStringUntil('\n');

  String commande;
  String arg1, arg2;

  analyserCommande(tampon, commande, arg1, arg2);

  if (commande == "gDist") {
    Serial.println("PC : " + tampon);
    Serial.print("Arduino : ");
    Serial.println(distance);
  } else if (commande == "cfg") {
    if (arg1 == "lim_inf" && arg2.toFloat() < porte.getDistanceFermeture() && arg2.toFloat() >= 0) {
      porte.setDistanceOuverture(arg2.toFloat());
      Serial.println("PC : " + tampon);
      Serial.print("Arduino : Il configure la limite inférieure du moteur à " + arg2);
      Serial.println("cm");
      commandeOk();
    } else if (arg1 == "lim_sup" && arg2.toFloat() > porte.getDistanceOuverture()) {
      porte.setDistanceFermeture(arg2.toFloat());
      Serial.println("PC : " + tampon);
      Serial.print("Arduino : Il configure la limite supérieure du moteur à " + arg2);
      Serial.println("cm");
      commandeOk();
    } else if (arg1 == "alm") {
      alarme.setDistance(arg2.toFloat());
      Serial.println("PC : " + tampon);
      Serial.print("Arduino : Configure la distance de détection de l’alarme à " + arg2);
      Serial.println("cm");
      commandeOk();
    } else if (arg1 == "lim_inf" && arg2.toInt() >= porte.getDistanceFermeture()) {
      Serial.println("PC : " + tampon);
      Serial.println("Arduino : Erreur – Limite inférieure plus grande que limite supérieure");
      limiteErreur();
      delay(rateMatrix);
      u8g2.clearBuffer();
      u8g2.sendBuffer();
    } else {
      Serial.println("PC : " + tampon);
      limiteErreur();
      delay(rateMatrix);
      u8g2.clearBuffer();
      u8g2.sendBuffer();
    }
  } else if (commande == "alm") {
    if (arg1 == "on") {
      alarme.turnOn();
      Serial.println("Alarme activée");
      commandeOk();
    } else if (arg1 == "off") {
      alarme.turnOff();
      Serial.println("Alarme désactivée");
      while (commande != "alm" && arg1 != "on") {
        alarme.turnOff();
      }
      commandeOk();
    } else if (arg1 == "test") {
      alarme.test();
      Serial.println("Alarme en mode test");
      commandeOk();
    } else if (arg1 == "getEtat") {
      Serial.print("État de l'alarme : ");
      AlarmState etat = alarme.getState();
      switch (etat) {
        case OFF: Serial.println("OFF"); break;
        case WATCHING: Serial.println("WATCHING"); break;
        case ON: Serial.println("ON"); break;
        case TESTING: Serial.println("TESTING"); break;
        default: Serial.println("État inconnu"); break;
      }
      commandeOk();
    } else {
      limiteErreur();
    }
  } else {
    commandeInconnu();
    u8g2.clearBuffer();
    delay(rateMatrix);
    u8g2.clearBuffer();
    u8g2.sendBuffer();
  }
}

void printWifiStatus() {

  // imprime le SSID du réseau auquel vous êtes connecté:
  char ssid[33];
  WiFi.SSID(ssid);
  Serial.print("SSID: ");
  Serial.println(ssid);

  // imprime le BSSID du réseau auquel vous êtes connecté:
  uint8_t bssid[6];
  WiFi.BSSID(bssid);
  Serial.print("BSSID: ");
  printMacAddress(bssid);

  uint8_t mac[6];
  WiFi.macAddress(mac);
  Serial.print("MAC: ");
  printMacAddress(mac);

  // imprime l'adresse IP de votre carte:
  IPAddress ip = WiFi.localIP();
  Serial.print("Adresse IP: ");
  Serial.println(ip);

  // imprime la force du signal reçu:
  long rssi = WiFi.RSSI();
  Serial.print("force du signal (RSSI):");
  Serial.print(rssi);
  Serial.println(" dBm");
}

// Imprime l'adresse MAC
void printMacAddress(byte mac[]) {
  for (int i = 5; i >= 0; i--) {
    if (mac[i] < 16) {
      Serial.print("0");
    }
    Serial.print(mac[i], HEX);
    if (i > 0) {
      Serial.print(":");
    }
  }
  Serial.println();
}

void errorState(int codeA, int codeB) {
  const int rate = 100;
  const int pauseBetween = 500;
  const int pauseAfter = 1000;

  // On ne sort jamais de cette fonction
  while (true) {
    for (int i = 0; i < codeA; i++) {
      digitalWrite(LED_BUILTIN, HIGH);
      delay(rate);
      digitalWrite(LED_BUILTIN, LOW);
      delay(rate);
    }
    delay(pauseBetween);
    for (int i = 0; i < codeB; i++) {
      digitalWrite(LED_BUILTIN, HIGH);
      delay(rate);
      digitalWrite(LED_BUILTIN, LOW);
      delay(rate);
    }
    delay(pauseAfter);

    Serial.print("Erreur : ");
    Serial.print(codeA);
    Serial.print(".");
    Serial.println(codeB);
  }
}

void periodicTask() {
  static unsigned long lastTime = 0;
  static const unsigned int rate = 2500;
  int motor = (porte.getAngle() > 10) ? 1 : 0;

  if (millis() - lastTime < rate) return;
  lastTime = millis();

  char message[256]; 
  char szDist[8];
  char szAngle[8];
  char szUptime[12];
  char szPot[8];
  char szMotor[4];

  // Récupération des données
  float dist = distanceForMqtt();
  float angle = angleMoteur();
  bool moteur = motor;
  unsigned long up = uptime() / 1000;

  int potValue = map(analogRead(potentiometrePin), valeurDeDepart, potentiometreMax, valeurDeDepart, convertir);
  // Conversion en chaînes de caractères
  dtostrf(dist, 4, 1, szDist);     // ex: "12.3"
  dtostrf(angle, 4, 1, szAngle);   // ex: "90.0"
  dtostrf(potValue, 4, 0, szPot);  // ex: "52.1"
  sprintf(szMotor, "%d", moteur);  // "0" ou "1"
  sprintf(szUptime, "%lu", up);    // uptime en secondes

  // Construction du message JSON
  sprintf(message,
          "{\"number\":\"6334158\", \"name\":\"Rodrigue Brim\", \"uptime\":%s, \"dist\":%s, \"angle\":%s, \"pot\":%s}",
          szUptime,
          szDist,
          szAngle,
          szPot
         );

  Serial.print("Envoi MQTT : ");
  Serial.println(message);

  client.publish("etd/8/data", message);
}
void sendLCDUpdate() {
  char buffer[128];
  sprintf(buffer, "{\"line1\":\"%s\", \"line2\":\"%s\"}", lastLine1.c_str(), lastLine2.c_str());
  client.publish("etd/8/data", buffer);
  Serial.print("Envoi LCD : ");
  Serial.println(buffer);
}

bool reconnect() {
  bool result = client.connect(DEVICE_NAME, MQTT_USER, MQTT_PASS);
  if (!result) {
    Serial.println("Incapable de se connecter sur le serveur MQTT");
  }
  return result;
}

void loop() {
  updateDistance();
  porte.update();
  afficherLCD();
  alarme.update();
  static unsigned long lastBlink = 0;
  if (millis() - lastBlink >= blinkRate) {
    lastBlink = millis();
    digitalWrite(LED_BUILTIN, !digitalRead(LED_BUILTIN));
  }
  periodicTask();
  static unsigned long lastLCDSent = 0;
  if (lcdDirty && millis() - lastLCDSent >= 1100) {
    sendLCDUpdate();
    lastLCDSent = millis();
    lcdDirty = false;
  }

  client.loop();
}