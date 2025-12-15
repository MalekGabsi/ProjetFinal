#include <WiFi.h>
#include <PubSubClient.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

// =========================================================================
// 1. CONFIGURATION RÉSEAU ET MQTT
// =========================================================================

// Paramètres Wi-Fi 

const char *WIFI_SSID = "iot";
const char *WIFI_PASSWORD = "iotisis;";

//const char *WIFI_SSID = "IoT";
//const char *WIFI_PASSWORD = "jGo2umnNdBHnHSsIVWZYPmatS4c6QrKg";

// Paramètres MQTT
 const char *MQTT_SERVER = "172.18.32.43";
//const char *MQTT_SERVER = "192.168.1.21";
const int MQTT_PORT = 1883;
const char *MQTT_CLIENT_ID = "ESP32Client_Melody";

const char *TOPIC_SYSTEM_STATE = "system/state";
const char *TOPIC_SPEAKER_PLAY = "esp32/haut_parleur/play";
const char *TOPIC_SPEAKER_MUSIC = "esp32/haut_parleur/music";
const char *TOPIC_SPEAKER_LOOP = "esp32/haut_parleur/boucle";

// =========================================================================
// 2. CONFIGURATION MATÉRIELLE (PINOUT)
// =========================================================================

const int PIN_LED_RED = 27;     // LED d'état Rouge (Core 1 Task)
const int PIN_LED_GREEN = 25;   // LED d'état Verte (Contrôlée par MQTT/System state)
const int PIN_SPEAKER_TONE = 26; // Sortie Haut-Parleur/Buzzer

// =========================================================================
// 3. VARIABLES GLOBALES D'ÉTAT
// =========================================================================

WiFiClient espClient;
PubSubClient client(espClient);

// État du système/lecture (accès depuis différentes tâches et interruption MQTT)
volatile bool isPlaying = false; // Vrai si la mélodie doit être jouée (géré par TaskPlayTune)
volatile bool isLooping = false; // Vrai si la mélodie doit recommencer après la fin
volatile bool systemStatus = false; // État logique du système (ON/OFF via MQTT System/state)

// Handles des tâches FreeRTOS (pour contrôle et suppression)
TaskHandle_t xTaskPlayTuneHandle = NULL;
TaskHandle_t xTaskLedStatusHandle = NULL; // Renommé pour plus de clarté

// =========================================================================
// 4. DÉFINITIONS DES NOTES ET TEMPO
// =========================================================================

// Frequences des notes (Hertz)
// Octave Basse (D)
#define NOTE_D0 1    // Silence/Pause
#define NOTE_D1 262
#define NOTE_D2 293
#define NOTE_D3 329
#define NOTE_D4 349
#define NOTE_D5 392
#define NOTE_D6 440
#define NOTE_D7 494

// Octave Milieu (M)
#define NOTE_M1 523
#define NOTE_M2 586
#define NOTE_M3 658
#define NOTE_M4 697
#define NOTE_M5 783
#define NOTE_M6 879
#define NOTE_M7 987

// Octave Haute (H)
#define NOTE_H1 1045
#define NOTE_H2 1171
#define NOTE_H3 1316
#define NOTE_H4 1393
#define NOTE_H5 1563
#define NOTE_H6 1755
#define NOTE_H7 1971

// Durées relatives (Facteurs)
const float DURATION_WHOLE = 1.0;
const float DURATION_HALF = 0.5;
const float DURATION_QUARTER = 0.25;
const float DURATION_EIGHTH = 0.125;
const float DURATION_SIXTEENTH = 0.0625;

// Tempo de base pour le calcul de la durée des notes (en ms)
const int BASE_TEMPO_MS = 500;

// Mélodie 1 (Exemple : "Joyeux Anniversaire" ou autre)
const int tune1[] = {
  NOTE_M3, NOTE_M3, NOTE_M4, NOTE_M5,
  NOTE_M5, NOTE_M4, NOTE_M3, NOTE_M2,
  NOTE_M1, NOTE_M1, NOTE_M2, NOTE_M3,
  NOTE_M3, NOTE_M2, NOTE_M2,

  NOTE_M3, NOTE_M3, NOTE_M4, NOTE_M5,
  NOTE_M5, NOTE_M4, NOTE_M3, NOTE_M2,
  NOTE_M1, NOTE_M1, NOTE_M2, NOTE_M3,
  NOTE_M2, NOTE_M1, NOTE_M1,

  NOTE_M2, NOTE_M2, NOTE_M3, NOTE_M1,
  NOTE_M2, NOTE_M3, NOTE_M4, NOTE_M3, NOTE_M1,
  NOTE_M2, NOTE_M3, NOTE_M4, NOTE_M3, NOTE_M2,
  NOTE_M1, NOTE_M2, NOTE_D5, NOTE_D0,

  NOTE_M3, NOTE_M3, NOTE_M4, NOTE_M5,
  NOTE_M5, NOTE_M4, NOTE_M3, NOTE_M4, NOTE_M2,
  NOTE_M1, NOTE_M1, NOTE_M2, NOTE_M3,
  NOTE_M2, NOTE_M1, NOTE_M1
};

const float durt1[] = {
  1, 1, 1, 1,
  1, 1, 1, 1,
  1, 1, 1, 1,
  1.5, 0.5, 2,

  1, 1, 1, 1,
  1, 1, 1, 1,
  1, 1, 1, 1,
  1.5, 0.5, 2,

  1, 1, 1, 0.5, 0.5,
  1, 1, 1, 1,
  1, 1, 1, 1,
  1, 1, 1, 0.5, 0.5,

  1, 1, 1, 1,
  1.5, 0.5, 2
};
const int MELODY1_LENGTH = sizeof(tune1) / sizeof(tune1[0]);

// Pointeurs vers la mélodie actuellement sélectionnée
const int *currentMelody = tune1;
const float *currentDurations = durt1;
int currentLength = MELODY1_LENGTH;

// =========================================================================
// 5. PROTOTYPES DES FONCTIONS
// =========================================================================

void initializeWiFi();
void handleMqttMessage(char *topic, byte *message, unsigned int length);
void ensureMqttConnection();
void startPlayTuneTask();
void selectMelody(int melodyNumber);

// Tâches FreeRTOS
void vTaskLedStatus(void *pvParameters); 
void vTaskPlayTune(void *pvParameters);

// =========================================================================
// 6. FONCTIONS ARDUINO PRINCIPALES
// =========================================================================

void setup()
{
  Serial.begin(115200);
  Serial.printf("\n--- Démarrage ESP32 Melody Player ---\n");
  Serial.printf("SSID: %s\n", WIFI_SSID);

  // Initialisation des broches
  pinMode(PIN_LED_RED, OUTPUT);
  pinMode(PIN_LED_GREEN, OUTPUT);
  pinMode(PIN_SPEAKER_TONE, OUTPUT);
  
  // Démarrer la tâche de gestion des LEDs sur le Core 1
  xTaskCreatePinnedToCore(
    vTaskLedStatus,
    "TaskLedStatus",
    2048,
    NULL,
    1,
    &xTaskLedStatusHandle,
    1
  );
  
  initializeWiFi();
  
  // Configuration MQTT
  client.setServer(MQTT_SERVER, MQTT_PORT);
  client.setCallback(handleMqttMessage);
}

void loop()
{
  // 1. Assurer la connexion MQTT
  ensureMqttConnection();
  
  // 2. Traiter le trafic MQTT
  client.loop();

  // 3. Pas de traitement périodique supplémentaire requis dans le loop() principal,
  // car l'état des LEDs est géré par FreeRTOS et la musique par FreeRTOS.
}

// =========================================================================
// 7. FONCTIONS DE CONNEXION ET DE GESTION MQTT
// =========================================================================

void initializeWiFi()
{
  // Démarrer la connexion WiFi
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
    Serial.print("Attempting MQTT connection...");
    
    // Tentative de connexion avec l'ID client défini
    if (client.connect(MQTT_CLIENT_ID))
    {
      Serial.println("connected");
      
      // S'abonner aux topics de contrôle
      client.subscribe(TOPIC_SYSTEM_STATE);
      client.subscribe(TOPIC_SPEAKER_PLAY);
      client.subscribe(TOPIC_SPEAKER_MUSIC);
      client.subscribe(TOPIC_SPEAKER_LOOP);
    }
    else
    {
      Serial.print("failed, rc=");
      Serial.print(client.state());
      Serial.println(". Retrying in 5 seconds...");
      delay(5000); // Attendre 5 secondes avant de réessayer
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
  String messageTemp = "";
  for (unsigned int i = 0; i < length; i++)
  {
    messageTemp += (char)message[i];
  }
  Serial.println(messageTemp);

  // Traitement du topic 'System/state' (LED Verte)
  if (String(topic) == TOPIC_SYSTEM_STATE)
  {
    if (messageTemp == "on")
    {
      digitalWrite(PIN_LED_GREEN, HIGH);
      systemStatus = true;
      Serial.println("System state set to ON.");
    }
    else if (messageTemp == "off")
    {
      digitalWrite(PIN_LED_GREEN, LOW);
      systemStatus = false;
      Serial.println("System state set to OFF.");
    }
  }

  // Traitement du topic 'esp32_haut_parleur/play' (Démarrer/Arrêter la mélodie)
  if (String(topic) == TOPIC_SPEAKER_PLAY)
  {
    if (messageTemp == "on")
    {
      if (!isPlaying) {
          isPlaying = true; // Définir l'état (lu par la tâche)
          startPlayTuneTask(); // Créer et démarrer la tâche
      }
    }
    else if (messageTemp == "off") 
    {
      isPlaying = false; // Le drapeau indique à la tâche de s'arrêter
      noTone(PIN_SPEAKER_TONE);
      
      // Arrêt immédiat si la tâche existe toujours (sécurité)
      if (xTaskPlayTuneHandle != NULL) {
          TaskHandle_t tempHandle = xTaskPlayTuneHandle;
          xTaskPlayTuneHandle = NULL; // Réinitialiser le handle global
          vTaskDelete(tempHandle); 
          Serial.println("TaskPlayTune forcibly stopped and deleted.");
      }
      Serial.println("Melody playback stopped.");
    }
  }

  // Traitement du topic 'esp32_haut_parleur/music' (Sélection de la mélodie)
  if (String(topic) == TOPIC_SPEAKER_MUSIC)
  {
    if (messageTemp == "melody1") {
      selectMelody(1);
    } else if (messageTemp == "melody2") {
      selectMelody(2);
    } else if (messageTemp == "melody3") {
      selectMelody(3);
    }
  }

  // Traitement du topic 'esp32_haut_parleur/boucle' (Mode Boucle)
  if (String(topic) == TOPIC_SPEAKER_LOOP)
  {
    if (messageTemp == "on") {
      isLooping = true;
      Serial.println("Loop mode: ON");
    } else if (messageTemp == "off") {
      isLooping = false;
      Serial.println("Loop mode: OFF");
    }
  }
}

/**
 * @brief Sélectionne les pointeurs de mélodie basés sur le numéro.
 * @param melodyNumber Le numéro de la mélodie à choisir (1, 2, 3...)
 */
void selectMelody(int melodyNumber) {
  Serial.printf("Selecting melody: %d\n", melodyNumber);

  switch (melodyNumber) {
    case 1:
      currentMelody = tune1;
      currentDurations = durt1;
      currentLength = MELODY1_LENGTH;
      break;
    case 2:
      // PLACEHOLDER : Utilise melody1 pour l'instant
      currentMelody = tune1;
      currentDurations = durt1;
      currentLength = MELODY1_LENGTH;
      Serial.println("Melody 2 selected (Placeholder: using Melody 1).");
      break;
    case 3:
      // PLACEHOLDER : Utilise melody1 pour l'instant
      currentMelody = tune1;
      currentDurations = durt1;
      currentLength = MELODY1_LENGTH;
      Serial.println("Melody 3 selected (Placeholder: using Melody 1).");
      break;
    default:
      Serial.println("Invalid melody number. Keeping current melody.");
      break;
  }
}

// =========================================================================
// 8. TÂCHES FREERTOS
// =========================================================================

/**
 * @brief Tâche FreeRTOS pour jouer la mélodie sans bloquer le loop() principal.
 */
void vTaskPlayTune(void *pvParameters)
{
  Serial.printf("TaskPlayTune started on core %d\n", xPortGetCoreID());

  // Boucle de lecture principale (continue si 'playing' ET 'looping')
  do { 
    // Itérer sur toutes les notes de la mélodie
    for (int thisNote = 0; thisNote < currentLength; thisNote++) {
      
      // Vérification d'arrêt immédiat (prioritaire)
      if (!isPlaying) {
        goto end_task;
      }
      
      // 1. Calcul de la durée de la note en ms
      int noteDurationMs = (int)(BASE_TEMPO_MS * currentDurations[thisNote]);
      int noteFrequency = currentMelody[thisNote];

      if (noteFrequency > NOTE_D0) { 
          // Jouer la tonalité et la maintenir pour la durée calculée
          tone(PIN_SPEAKER_TONE, noteFrequency);
          delay(noteDurationMs); // Bloquant, mais seulement au sein de cette tâche
          noTone(PIN_SPEAKER_TONE);
          
          // Pause très courte entre les notes pour un effet staccato
          vTaskDelay((int)(noteDurationMs * 0.05) / portTICK_PERIOD_MS);
      } else {
          // Si c'est un silence (NOTE_D0), attendre la durée de la pause
          noTone(PIN_SPEAKER_TONE);
          vTaskDelay(noteDurationMs / portTICK_PERIOD_MS);
      }
    }
    
    // Si en mode boucle, faire une courte pause avant de recommencer
    if (isLooping && isPlaying) {
      Serial.println("Melody loop completed. Restarting...");
      vTaskDelay(500 / portTICK_PERIOD_MS);
    }

  } while (isPlaying && isLooping);
  
// Sortie propre de la tâche
end_task:
  isPlaying = false; // Réinitialisation de l'état
  noTone(PIN_SPEAKER_TONE);
  Serial.println("Melody finished or interrupted.");
  
  // CRUCIAL : Réinitialiser le handle global et supprimer la tâche elle-même
  xTaskPlayTuneHandle = NULL; 
  vTaskDelete(NULL); 
}

/**
 * @brief Crée et lance la tâche vTaskPlayTune.
 */
void startPlayTuneTask() {
  if (xTaskPlayTuneHandle != NULL) {
    Serial.println("TaskPlayTune is already running. Ignoring start command.");
    return;
  }
  
  // Créer la tâche sur le Core 1 (CPU secondaire)
  BaseType_t result = xTaskCreatePinnedToCore(
    vTaskPlayTune,
    "TaskPlayTune",
    4096,
    NULL,
    2,// Priorité
    &xTaskPlayTuneHandle, 
    1
  );

  if (result != pdPASS) {
      Serial.println("ERROR: Failed to create TaskPlayTune!");
  }
}

/**
 * @brief Tâche FreeRTOS pour la gestion des LEDs d'état (Rouge)
 * (S'exécute en permanence sur le Core 1)
 */
void vTaskLedStatus(void *pvParameters){
  Serial.printf("TaskLedStatus started on core %d\n", xPortGetCoreID());

  while (true) {
  
    // Gestion de l'état DECONNECTE
    if (!client.connected()){
      // LED Verte OFF (même si un message MQTT l'a mise à ON)
      digitalWrite(PIN_LED_GREEN, LOW); 
      
      // LED Rouge clignotante : Signal d'alarme / Reconnexion
      digitalWrite(PIN_LED_RED, HIGH);
      vTaskDelay(500 / portTICK_PERIOD_MS);
      digitalWrite(PIN_LED_RED, LOW);
      vTaskDelay(500 / portTICK_PERIOD_MS);
    }
    // Gestion de l'état CONNECTE
    else {
      // Si le système est logiquement "OFF" (via System/state MQTT)
      if (systemStatus == false){
        // LED Rouge ALLUMÉE : Indique que l'ESP32 est prêt/connecté mais désactivé
        digitalWrite(PIN_LED_RED, HIGH);
      }
      // Si le système est logiquement "ON"
      else {
        // LED Rouge ÉTEINTE : Système actif et opérationnel
        digitalWrite(PIN_LED_RED, LOW);
      }
      // Courte attente pour ne pas bloquer le core
      vTaskDelay(100 / portTICK_PERIOD_MS);
    }
  }
  // La tâche ne devrait jamais se terminer
}