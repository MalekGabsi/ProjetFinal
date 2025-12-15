#include <WiFi.h>
#include <PubSubClient.h>
#include <Wire.h>
#include "DFRobot_BloodOxygen_S.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

// =========================================================================
// 1. CONFIGURATION RÉSEAU ET MQTT
// =========================================================================

// Paramètres Wi-Fi 
 const char *WIFI_SSID = "iot";
const char *WIFI_PASSWORD = "iotisis;"; 
/* const char *WIFI_SSID = "IoT";
const char *WIFI_PASSWORD = "jGo2umnNdBHnHSsIVWZYPmatS4c6QrKg"; */

// Paramètres MQTT
//const char *MQTT_SERVER = "192.168.1.21";
 const char *MQTT_SERVER = "172.18.32.43";

const int MQTT_PORT = 1883;
const char *MQTT_CLIENT_ID = "ESP32Client_Master";

// Topics de souscription
const char *TOPIC_SYSTEM_STATE = "system/state";

// Topics de publication
const char *TOPIC_PUB_BUTTON1 = "esp32/master/button1";
const char *TOPIC_PUB_BUTTON2 = "esp32/master/button2";
const char *TOPIC_PUB_BUTTON3 = "esp32/master/button3";
const char *TOPIC_PUB_BIOMETRICS = "esp32/master/capteur/biometrie";
const long PUBLISH_INTERVAL_MS = 2000;

// =========================================================================
// 2. CONFIGURATION MATÉRIELLE (PINOUT)
// =========================================================================

// LEDs d'état
const int PIN_LED_RED = 27;   // Indique l'état MQTT / État OFF du système
const int PIN_LED_GREEN = 26; // Indique l'état de la connexion des capteurs / État ON du système

// Boutons (avec INPUT_PULLUP)
const int PIN_BUTTON1 = 25;
const int PIN_BUTTON2 = 13;
const int PIN_BUTTON3 = 5;

// I2C et Capteurs
const int I2C_SLAVE_ADDR = 0x08;
const int PIN_SDA = 21;
const int PIN_SCL = 22;
const int HR_SENSOR_I2C_ADDRESS = 0x57;

// =========================================================================
// 3. STRUCTURES ET VARIABLES GLOBALES D'ÉTAT
// =========================================================================

// Structure des données lues depuis l'esclave I2C
struct SlaveData_t
{
  int ky039Raw;
  int gsrRaw;
};
SlaveData_t slaveData;

// Variables de la bibliothèque MQTT/WiFi
WiFiClient espClient;
PubSubClient client(espClient);
long lastMsg = 0;

// Instance du capteur HR/SpO2
DFRobot_BloodOxygen_S_I2C hrSensor(&Wire, HR_SENSOR_I2C_ADDRESS);

// Variables d'état partagées (volatile car utilisées par loop() et les tâches FreeRTOS)
volatile bool systemStatus = false; // État logique ON/OFF via MQTT
volatile int bpmValue = -1;
volatile int spo2Value = -1;
volatile float tempCValue = -1;
volatile bool sensorsReady = false; // Indique si les lectures de capteurs sont valides

// Handles des tâches FreeRTOS
TaskHandle_t xTaskLedRedHandle = NULL;
TaskHandle_t xTaskLedGreenHandle = NULL;

// =========================================================================
// 4. PROTOTYPES DES FONCTIONS
// =========================================================================

void initializeWiFi();
void handleMqttMessage(char *topic, byte *message, unsigned int length);
void ensureMqttConnection();
SlaveData_t readSlaveData();

// Tâches FreeRTOS (Core 1)
void vTaskLedStatusMQTT(void *pvParameters);
void vTaskLedStatusSensors(void *pvParameters);

// =========================================================================
// 5. FONCTIONS ARDUINO PRINCIPALES
// =========================================================================

void setup()
{
  Serial.begin(115200);
  Serial.printf("\n--- Démarrage ESP32 Biometric Master ---\n");

  // Initialisation des broches
  pinMode(PIN_LED_RED, OUTPUT);
  pinMode(PIN_LED_GREEN, OUTPUT);
  pinMode(PIN_BUTTON1, INPUT_PULLUP);
  pinMode(PIN_BUTTON2, INPUT_PULLUP);
  pinMode(PIN_BUTTON3, INPUT_PULLUP);
  
  // Démarrer les tâches de gestion des LEDs sur le Core 1
  xTaskCreatePinnedToCore(
    vTaskLedStatusMQTT,
    "TaskLedRedStatus",
    2048,
    NULL,
    1,
    &xTaskLedRedHandle,
    1 
  );
  xTaskCreatePinnedToCore(
    vTaskLedStatusSensors,
    "TaskLedGreenStatus",
    2048,
    NULL,
    1,
    &xTaskLedGreenHandle,
    1 
  );

  initializeWiFi();

  // Initialisation I2C Master
  Wire.begin(PIN_SDA, PIN_SCL);
  Wire.setTimeout(500);

  // Initialisation du capteur SpO2/HR
  Serial.println("Initialisation capteur SpO2/HR...");
  if (!hrSensor.begin())
  {
    Serial.println("ERREUR CRITIQUE : capteur HR/SpO2 introuvable !");
    // Optionnel: Bloquer ici ou indiquer un échec fatal
    while (1) delay(1000);
  }
  hrSensor.sensorStartCollect();

  // Configuration MQTT
  client.setServer(MQTT_SERVER, MQTT_PORT);
  client.setKeepAlive(60);
  client.setCallback(handleMqttMessage);
}

void loop()
{
  // 1. Assurer la connexion MQTT
  ensureMqttConnection();
  client.loop(); // Traiter le trafic MQTT

  // 2. Tâche périodique (Lecture et Publication des données)
  if (millis() - lastMsg > PUBLISH_INTERVAL_MS)
  {
    lastMsg = millis();

    // Publier l'état des boutons
    // Note : Le polling des boutons est moins réactif que les interruptions,
    // mais suffisant pour cet intervalle.
    Serial.println("Publication état boutons...");
    Serial.printf("Button1: %s | Button2: %s | Button3: %s\n",
                  (digitalRead(PIN_BUTTON1) == LOW) ? "down" : "up",
                  (digitalRead(PIN_BUTTON2) == LOW) ? "down" : "up",
                  (digitalRead(PIN_BUTTON3) == LOW) ? "down" : "up");
    if (digitalRead(PIN_BUTTON1) == LOW) client.publish(TOPIC_PUB_BUTTON1, "down");
    else client.publish(TOPIC_PUB_BUTTON1, "up");

    if (digitalRead(PIN_BUTTON2) == LOW) client.publish(TOPIC_PUB_BUTTON2, "down");
    else client.publish(TOPIC_PUB_BUTTON2, "up");

    if (digitalRead(PIN_BUTTON3) == LOW) client.publish(TOPIC_PUB_BUTTON3, "down");
    else client.publish(TOPIC_PUB_BUTTON3, "up");

    // Lecture des capteurs
    hrSensor.getHeartbeatSPO2();
    bpmValue = hrSensor._sHeartbeatSPO2.Heartbeat;
    spo2Value = hrSensor._sHeartbeatSPO2.SPO2;
    tempCValue = hrSensor.getTemperature_C();
    
    // Mettre à jour l'état de validité des capteurs pour la LED Verte
    if (bpmValue > 0 && spo2Value > 0) {
      sensorsReady = true;
    } else {
      sensorsReady = false;
    }

    slaveData = readSlaveData();

    // Construction JSON (Utilisation des variables volatiles à ce point)
    String payload = "{";
    payload += "\"GSR_RAW\":" + String(slaveData.gsrRaw) + ",";
    payload += "\"BPM_RAW\":" + String(slaveData.ky039Raw) + ",";
    payload += "\"HR_OXI\":" + String(bpmValue) + ",";
    payload += "\"SpO2\":" + String(spo2Value) + ",";
    payload += "\"Temp\":" + String(tempCValue, 1);
    payload += "}";

    // Envoi MQTT
    client.publish(TOPIC_PUB_BIOMETRICS, payload.c_str());

    // Logs
    Serial.printf("HR: %d | SpO2: %d | Temp: %.1f°C | GSR: %d | KY039: %d\n", 
                  bpmValue, spo2Value, tempCValue, slaveData.gsrRaw, slaveData.ky039Raw);
  }
}

// =========================================================================
// 6. FONCTIONS DE CONNEXION ET DE GESTION MQTT
// =========================================================================

void initializeWiFi()
{
  Serial.print("Connecting to WiFi: ");
  Serial.println(WIFI_SSID);

  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);

  while (WiFi.status() != WL_CONNECTED)
  {
    delay(500);
    Serial.print(".");
  }

  Serial.println("\nWiFi connected.");
  Serial.print("IP Address: ");
  Serial.println(WiFi.localIP());
}

/**
 * @brief Assure la reconnexion au broker MQTT si nécessaire.
 */
void ensureMqttConnection()
{
  while (!client.connected())
  {
    Serial.print("Attempting MQTT connection... ");
    String clientId = String(MQTT_CLIENT_ID) + "-" + String(random(0xffff), HEX);

    if (client.connect(clientId.c_str()))
    {
      Serial.println("OK");
      // S'abonner aux topics de contrôle
      client.subscribe(TOPIC_SYSTEM_STATE);
    }
    else
    {
      Serial.print("Failed rc=");
      Serial.print(client.state());
      Serial.println(". Retrying in 5 seconds...");
      delay(5000);
    }
  }
}

/**
 * @brief Fonction de rappel (callback) appelée lors de la réception d'un message MQTT.
 */
void handleMqttMessage(char *topic, byte *message, unsigned int length)
{
  Serial.print("Message received on [");
  Serial.print(topic);
  Serial.print("]: ");
  
  // Convertir le message en String pour faciliter la comparaison
  String messageTemp;
  for (unsigned int i = 0; i < length; i++)
  {
    messageTemp += (char)message[i];
  }
  Serial.println(messageTemp);

  // Traitement du topic 'System/state'
  if (String(topic) == TOPIC_SYSTEM_STATE)
  {
    if (messageTemp == "on")
    {
      systemStatus = true;
      Serial.println("System state set to ON (Status LED Green controlled by vTaskLedStatusSensors).");
    }
    else if (messageTemp == "off")
    {
      systemStatus = false;
      Serial.println("System state set to OFF (Status LED Green controlled by vTaskLedStatusSensors).");
    }
  }
}

/**
 * @brief Lit les données binaires depuis l'esclave I2C.
 * @return SlaveData_t Structure contenant les données lues.
 */
SlaveData_t readSlaveData()
{
  SlaveData_t data = {-1, -1};
  int size = sizeof(SlaveData_t);

  Wire.requestFrom(I2C_SLAVE_ADDR, size);
  delay(10); 

  if (Wire.available() >= size)
  {
    Wire.readBytes((byte *)&data, size);
  }
  else
  {
    Serial.println("[I2C] Error: Data not received from slave.");
  }

  return data;
}

// =========================================================================
// 7. TÂCHES FREERTOS
// =========================================================================

/**
 * @brief Tâche pour la LED ROUGE (Indicateur MQTT / État OFF du système).
 * Fonctionnement:
 * - Clignote si MQTT déconnecté.
 * - Allumée si MQTT connecté MAIS systemStatus est FALSE.
 * - Éteinte si MQTT connecté ET systemStatus est TRUE.
 */
void vTaskLedStatusMQTT(void *pvParameters){
  Serial.printf("vTaskLedStatusMQTT started on core %d (Controls Red LED)\n", xPortGetCoreID());

  const TickType_t delay500 = 500 / portTICK_PERIOD_MS;
  const TickType_t delay100 = 100 / portTICK_PERIOD_MS;

  while (true) {
    if (!client.connected()){
      // Signal d'alarme/reconnexion
      digitalWrite(PIN_LED_RED, HIGH);
      vTaskDelay(delay500);
      digitalWrite(PIN_LED_RED, LOW);
      vTaskDelay(delay500);
    }
    else {
      // Connecté : Gère l'état logique OFF
      if (systemStatus == false){
        digitalWrite(PIN_LED_RED, HIGH); // LED Rouge allumée
      } else {
        digitalWrite(PIN_LED_RED, LOW);  // LED Rouge éteinte
      }
      vTaskDelay(delay100);
    }
  }
}

/**
 * @brief Tâche pour la LED VERTE (Indicateur Capteurs / État ON du système).
 * Fonctionnement:
 * - Clignote si les capteurs principaux (HR/SpO2) sont invalides.
 * - Allumée si capteurs OK ET systemStatus est TRUE.
 * - Éteinte si systemStatus est FALSE.
 */
void vTaskLedStatusSensors(void *pvParameters){
  Serial.printf("vTaskLedStatusSensors started on core %d (Controls Green LED)\n", xPortGetCoreID());
  
  const TickType_t delay500 = 500 / portTICK_PERIOD_MS;
  const TickType_t delay100 = 100 / portTICK_PERIOD_MS;

  while (true) {
    // Si le système est logiquement ON et les capteurs sont prêts
    if (systemStatus == true) {
      if (sensorsReady) {
        // Capteurs et Système OK
        digitalWrite(PIN_LED_GREEN, HIGH);
        vTaskDelay(delay100);
      } else {
        // Capteurs Invalides (Clignote)
        digitalWrite(PIN_LED_GREEN, HIGH);
        vTaskDelay(delay500);
        digitalWrite(PIN_LED_GREEN, LOW);
        vTaskDelay(delay500);
      }
    } else {
      // Si systemStatus est OFF (l'état est géré par la LED ROUGE)
      digitalWrite(PIN_LED_GREEN, LOW);
      vTaskDelay(delay100);
    }
  }
}