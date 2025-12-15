#include <WiFi.h>
#include <PubSubClient.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "Bautrel-project-1_inferencing.h"



const bool DEBUG = false;
// =========================================================================
// 1. CONFIGURATION RÉSEAU ET MQTT
// =========================================================================

// Paramètres Wi-Fi
 const char *WIFI_SSID = "iot";
const char *WIFI_PASSWORD = "iotisis;";
//  const char *WIFI_SSID = "IoT";
// const char *WIFI_PASSWORD = "jGo2umnNdBHnHSsIVWZYPmatS4c6QrKg"; 

// Paramètres MQTT
 const char *MQTT_SERVER = "172.18.32.43";
// const char *MQTT_SERVER = "192.168.1.21";
const int MQTT_PORT = 1883;
const char *MQTT_CLIENT_ID = "ESP32Client_AnalogSlave";

// Topics
const char *TOPIC_SYSTEM_STATE = "system/state";
const char *TOPIC_PUB_BIOMETRICS = "esp32/GSR-ECG-TINYML/capteurs/biometrie";
const char *TOPIC_PUB_CLASSIFICATION = "esp32/GSR-ECG-TINYML/ia/classification";
const long PUBLISH_INTERVAL_MS = 50;
const long INFERENCE_INTERVAL_MS = 2000; // Intervalle d'inférence IA


// =========================================================================
// 2. CONFIGURATION MATÉRIELLE (PINOUT)
// =========================================================================

// LEDs d'état
const int PIN_LED_RED = 26;   // Indique l'état MQTT
const int PIN_LED_GREEN = 27; // Indique l'état des Capteurs

// Capteurs Analogiques
const int PIN_KY039 = 36; // VP (ADC1_CHANNEL0)
const int PIN_GSR = 34;   // VN (ADC1_CHANNEL6)

// =========================================================================
// 3. VARIABLES GLOBALES D'ÉTAT
// =========================================================================

WiFiClient espClient;
PubSubClient client(espClient);
long lastMsg = 0;

// Variables lues (volatile pour la tâche LED Green)
volatile int ky039Value = 0;
volatile int gsrValue = 0;
volatile bool sensorsReadingOk = false; // Indique si les lectures sont dans une plage acceptable
volatile bool systemStatus = false; // État logique ON/OFF via MQTT
volatile char aiClassification[32] = "demarrage"; // Résultat de l'IA
volatile float features_input[EI_CLASSIFIER_DSP_INPUT_FRAME_SIZE];

// Handles des tâches FreeRTOS
TaskHandle_t xTaskLedRedHandle = NULL;
TaskHandle_t xTaskLedGreenHandle = NULL;
TaskHandle_t xTaskInferenceHandle = NULL;

// =========================================================================
// 4. PROTOTYPES DES FONCTIONS
// =========================================================================

void initializeWiFi();
void handleMqttMessage(char *topic, byte *message, unsigned int length);
void ensureMqttConnection();

// Tâches FreeRTOS (Core 1)
void vTaskLedRed(void *pvParameters);
void vTaskLedGreen(void *pvParameters);
void vTaskInference(void *pvParameters);

// =========================================================================
// 5. FONCTIONS ARDUINO PRINCIPALES
// =========================================================================

void setup() {
  Serial.begin(115200);
  Serial.printf("\n--- Démarrage ESP32 Analog Slave ---\n");

  // Initialisation des broches
  pinMode(PIN_KY039, INPUT);
  pinMode(PIN_GSR, INPUT);
  pinMode(PIN_LED_RED, OUTPUT);
  pinMode(PIN_LED_GREEN, OUTPUT);

  // Démarrer les tâches de gestion des LEDs sur le Core 1
  xTaskCreatePinnedToCore(
    vTaskLedRed,
    "TaskLedRed",
    2048,
    NULL,
    1,
    &xTaskLedRedHandle,
    1 
  );
  xTaskCreatePinnedToCore(
    vTaskLedGreen,
    "TaskLedGreen",
    2048,
    NULL,
    1,
    &xTaskLedGreenHandle,
    1 
  );
  xTaskCreatePinnedToCore(
    vTaskInference, // Nouvelle tâche d'inférence
    "TaskInference",
    4096, // Taille de stack augmentée pour l'IA
    NULL,
    2, // Priorité légèrement supérieure
    &xTaskInferenceHandle,
    1 
  );

  initializeWiFi();

  client.setServer(MQTT_SERVER, MQTT_PORT);
  client.setCallback(handleMqttMessage);

  Serial.println("ESP32 sensors ready!");


}

void loop() {
  // 1. Assurer la connexion MQTT
  ensureMqttConnection();
  client.loop(); // Traiter le trafic MQTT

  // 2. Tâche périodique (Lecture et Publication des données)
  if (millis() - lastMsg > PUBLISH_INTERVAL_MS) {
    lastMsg = millis();

    // Lire les capteurs
    ky039Value = analogRead(PIN_KY039);
    gsrValue = analogRead(PIN_GSR);

    // Mettre à jour l'état de lecture (simulé : si les deux sont > 50)
    // Adaptez cette condition à la plage normale de vos capteurs.
    if (ky039Value > 50 && gsrValue > 50) {
        sensorsReadingOk = true;
    } else {
        sensorsReadingOk = false;
    }


    // Construire le payload JSON
    String payload = "{";
    payload += "\"KY039\":" + String(ky039Value) + ",";
    payload += "\"GSR\":" + String(gsrValue);
    payload += "}";

    // Publier MQTT (Données et IA)
    client.publish(TOPIC_PUB_BIOMETRICS, payload.c_str());
    client.publish(TOPIC_PUB_CLASSIFICATION, (char*)aiClassification);

    // Afficher dans le moniteur série
    if(DEBUG){
    Serial.printf("KY039: %d | GSR: %d | Readings OK: %s | IA: %s\n", 
                ky039Value, gsrValue, sensorsReadingOk ? "YES" : "NO", (char*)aiClassification);
    }
  }  
}

// =========================================================================
// 6. FONCTIONS DE CONNEXION ET DE GESTION MQTT
// =========================================================================

void initializeWiFi() {
  Serial.print("Connecting to WiFi...");
  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  Serial.println("\nWiFi connected.");
  Serial.print("IP Address: ");
  Serial.println(WiFi.localIP());
}

void ensureMqttConnection() {
  while (!client.connected()) {
    Serial.print("Attempting MQTT connection...");
    String clientId = String(MQTT_CLIENT_ID) + "-" + String(random(0xffff), HEX);
    if (client.connect(clientId.c_str())) {
      Serial.println("OK");
      // S'abonner aux topics de contrôle LED à distance
      client.subscribe(TOPIC_SYSTEM_STATE);
    } else {
      Serial.print("Failed rc=");
      Serial.print(client.state());
      Serial.println(". Retrying in 5 seconds...");
      delay(5000);
    }
  }
}

/**
 * @brief Fonction de rappel (callback) pour la réception de messages MQTT.
 * Ce code permet le contrôle à distance des LEDs (en plus de leur gestion par RTOS).
 */
void handleMqttMessage(char *topic, byte *message, unsigned int length) {
  Serial.print("Message received on [");
  Serial.print(topic);
  Serial.print("]: ");
  
  String messageTemp = "";
  for (unsigned int i = 0; i < length; i++) {
    messageTemp += (char)message[i];
  }
  Serial.println(messageTemp);
  // Gérer le topic de l'état système

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

// =========================================================================
// 7. TÂCHES FREERTOS
// =========================================================================

/**
 * @brief Tâche pour la LED ROUGE (Indicateur MQTT).
 * Fonctionnement:
 * - Clignote si MQTT déconnecté.
 * - Allumée si MQTT connecté (pour laisser la main au contrôle à distance/manuel).
 */
void vTaskLedRed(void *pvParameters){
  Serial.printf("vTaskRedLed started on core %d (Controls Red LED)\n", xPortGetCoreID());

  const TickType_t delay500 = 500 / portTICK_PERIOD_MS;
  const TickType_t delay100 = 100 / portTICK_PERIOD_MS;

  while (true) {
    if(systemStatus == false) {
        if (!client.connected()){
        // Signal d'alarme/reconnexion
        digitalWrite(PIN_LED_RED, HIGH);
        vTaskDelay(delay500);
        digitalWrite(PIN_LED_RED, LOW);
        vTaskDelay(delay500);
      }
      else {
        digitalWrite(PIN_LED_GREEN, LOW);
        digitalWrite(PIN_LED_RED, HIGH);
        vTaskDelay(delay100);
      }
    }

  }
}

/**
 * @brief Tâche pour la LED VERTE (Indicateur Capteurs).
 * Fonctionnement:
 * - Clignote si sensorsReadingOk est FALSE (problème de lecture).
 * - Reste allumée si sensorsReadingOk est TRUE (lectures valides).
 */
void vTaskLedGreen(void *pvParameters){
  Serial.printf("vTaskLedGreen started on core %d (Controls Green LED)\n", xPortGetCoreID());
  
  const TickType_t delay500 = 500 / portTICK_PERIOD_MS;
  const TickType_t delay100 = 100 / portTICK_PERIOD_MS;

  while (true) {
    if(systemStatus == true) {
        if (sensorsReadingOk) {
        // Capteurs OK (LED allumée)
        digitalWrite(PIN_LED_GREEN, HIGH);
        digitalWrite(PIN_LED_RED, LOW);
        vTaskDelay(delay100);
      } else {
        // Capteurs Invalides (Clignote)
        digitalWrite(PIN_LED_GREEN, HIGH);
        vTaskDelay(delay500);
        digitalWrite(PIN_LED_GREEN, LOW);
        vTaskDelay(delay500);
      }
    }
  }
}

int ei_get_data_callback(size_t offset, size_t length, float *out_ptr) {
    if (offset + length > EI_CLASSIFIER_DSP_INPUT_FRAME_SIZE) {
        length = EI_CLASSIFIER_DSP_INPUT_FRAME_SIZE - offset;
    }
    // Copie des données depuis le tableau global/volatile
    memcpy(out_ptr, (float*)features_input + offset, length * sizeof(float));
    return (int)length;
}

void vTaskInference(void *pvParameters){
  Serial.printf("vTaskInference started on core %d (Controls Biometric AI)\n", xPortGetCoreID());

 // L'intervalle de la tâche est maintenant la durée totale de la fenêtre
 const TickType_t delayInference = INFERENCE_INTERVAL_MS / portTICK_PERIOD_MS;
  
  // Fréquence d'échantillonnage (delay entre chaque lecture)
  const TickType_t delaySample = (1000 / EI_CLASSIFIER_FREQUENCY) / portTICK_PERIOD_MS; 


  //Structure signal_t requise par run_classifier
  ei::signal_t ei_signal;
  ei_signal.total_length = EI_CLASSIFIER_DSP_INPUT_FRAME_SIZE;
  ei_signal.get_data = &ei_get_data_callback;

// Variable de contrôle pour la synchronisation
  static long lastInference = 0;


  while (true) {
  if (systemStatus == true) {

      // --- NOUVEAU: Collecte de la fenêtre de 125 échantillons ---
      
      // La boucle for gère le temps d'échantillonnage total (Window Size)
      Serial.printf("Collecting %d GSR samples @ %.0f Hz...\n", 
                    EI_CLASSIFIER_DSP_INPUT_FRAME_SIZE, EI_CLASSIFIER_FREQUENCY);

      for(int i = 0; i < EI_CLASSIFIER_DSP_INPUT_FRAME_SIZE; i++) {

        // La variable volatile gsrValue est mise à jour par la fonction loop() sur le Core 0.
        float rawValue = (float)gsrValue; 
        
        // 1. Préparer/Convertir les données d'entrée
        // ***************************************************************
        // CRITIQUE : APPLIQUER ICI VOTRE FORMULE DE CONVERSION ADC -> ÉCHELLE ENTRAÎNÉE (environ 420.x)
        // L'exemple ci-dessous est un PLACEHOLDER et ne doit pas être utilisé tel quel.
        // features_input[i] = rawValue * (VOTRE_COEFF); 
        // Si vous ne connaissez pas le coefficient, essayez le brut (non recommandé) :
        features_input[i] = rawValue; 
        // ***************************************************************

        // Attendre l'intervalle d'échantillonnage (Ex: 10 ms si 100 Hz)
        vTaskDelay(delaySample);
      }
      
   // 2. Exécuter l'Inférence (le signal est lu à partir de features_input via ei_get_data_callback)
   ei_impulse_result_t result = { 0 };

   EI_IMPULSE_ERROR res = run_classifier(&ei_signal, &result, false);

   if (res != EI_IMPULSE_OK) {
    Serial.printf("EI_IMPULSE_ERROR: %d\n", res);
    strncpy((char*)aiClassification, "Erreur AI", 32); 
   } else {
    
    // 3. Traiter le résultat: Trouver la classe avec le score le plus élevé
    float max_score = 0.0;
    int max_index = -1;
    
    for (size_t i = 0; i < EI_CLASSIFIER_LABEL_COUNT; i++) {
     if (result.classification[i].value > max_score) {
      max_score = result.classification[i].value;
      max_index = i;
     }
    }
    
    if (max_index != -1) {
     // Mise à jour de la classification (accessible dans loop via volatile)
     strncpy((char*)aiClassification, result.classification[max_index].label, 32);
     Serial.printf("AI Class: %s (Score: %.2f%%)\n", (char*)aiClassification, max_score * 100.0f);
    } else {
     strncpy((char*)aiClassification, "Non classifié", 32);
    }
   }
      
      // Si la durée de la boucle for (1.25s) + l'inférence est inférieure au délai total, on attend
      // Si "Window Increase" est inférieur à "Window Size", vous pourriez vouloir attendre moins longtemps ici.
   vTaskDelay(delayInference);

  } else {
   // Le système est OFF
   strncpy((char*)aiClassification, "Hors service", 32); 
   vTaskDelay(1000 / portTICK_PERIOD_MS);
  }
 }
}