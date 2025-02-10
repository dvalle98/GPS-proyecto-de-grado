#include "TinyGsmClientSIM7600.h"
typedef TinyGsmSim7600 TinyGsm;
typedef TinyGsmSim7600::GsmClientSim7600 TinyGsmClient;

#include <PubSubClient.h>
#include <freertos/FreeRTOS.h>
#include <freertos/semphr.h>
#include <freertos/queue.h>

#define TINY_GSM_USE_GPRS true

#define PIN_TX 27
#define PIN_RX 26
#define UART_BAUD 115200

#define PWR_PIN 4
#define LED_data_sent 0
#define PIN_VEL_status 12
#define POWER_PIN 25

const char apn[] = "internet.comcel.com.co";
const char mqtt_username[] = "test";
const char mqtt_password[] = "CloudTech*";
const char* broker = "18.212.130.131";
const char* GPSTopic = "gps/app/testliceo/puntos/868020034072685";
const char* AlertTopic = "gps/app/testliceo/alertas";
#define mqttKeepAlive 60
String imei;

TinyGsm modem(Serial1);
TinyGsmClient client(modem);
PubSubClient mqtt(client);

// Semáforo para acceso controlado al puerto serial
SemaphoreHandle_t xSemaphore;

// Cola para comunicar datos RFID entre tareas
QueueHandle_t xQueueRFID;

// Flag global para indicar si se tiene la ubicación GPS
volatile bool gpsReady = false;

// Declaración de tareas FreeRTOS
void taskSendGPSData(void* parameter);
void taskSendRFIDAlert(void* parameter);
void taskSimulateRFIDRead(void* parameter);

bool mqttConnect();

void setup() {
  Serial.begin(115200);
  Serial1.begin(115200, SERIAL_8N1, PIN_RX, PIN_TX);

  pinMode(LED_data_sent, OUTPUT);
  digitalWrite(LED_data_sent, 0);
  pinMode(PIN_VEL_status, OUTPUT);
  digitalWrite(PIN_VEL_status, 0);

  // Activar modem
  pinMode(POWER_PIN, OUTPUT);
  digitalWrite(POWER_PIN, 1);
  pinMode(PWR_PIN, OUTPUT);
  digitalWrite(PWR_PIN, 1);

  Serial.println(F("Starting modem...!"));
  modem.restart();

  if (modem.getSimStatus() == 0) {
    Serial.println("SIM ERROR..");
    ESP.restart();
  }

  Serial.print(F("Waiting for network..."));
  if (!modem.waitForNetwork(60000L, true)) {
    Serial.println(F(" fail"));
    delay(1000);
    ESP.restart();
  }
  Serial.println(F(" success"));

  Serial.print(F("Connecting to "));
  Serial.print(apn);
  if (!modem.gprsConnect(apn, "", "")) {
    Serial.println(F(" fail"));
    delay(1000);
    ESP.restart();
  }
  Serial.println(F(" success"));

  mqtt.setKeepAlive(mqttKeepAlive);
  mqtt.setServer(broker, 1883);

  modem.setGNSSMode(3, 0);
  modem.enableGPS();
  Serial.printf("GNSS mode: %u\n", modem.getGNSSMode());

  // Obtener la primera ubicación GPS
  Serial.println("Attempting to get initial GPS location...");
  for (int i = 0; i < 50; i++) {  // Intentar 10 veces
    float lat = 0, lon = 0, speed = 0, hdop = 0, cordN = 0;
    int hour = 0, minute = 0;

    modem.enableGPS();
    if (modem.getGPS(&lat, &lon, &speed, &cordN, &hdop, &hour, &minute)) {
      gpsReady = true;  // Se obtuvo la ubicación inicial
      Serial.println("Initial GPS location obtained.");
      break;
    }
    Serial.println("Waiting for GPS signal...");
    delay(5000);  // Esperar 5 segundos antes de intentar nuevamente
  }

  if (!gpsReady) {
    Serial.println("Failed to obtain initial GPS location. Restarting...");
    ESP.restart();
  }

  mqttConnect();

  // Crear semáforo binario
  xSemaphore = xSemaphoreCreateBinary();
  xSemaphoreGive(xSemaphore);  // Inicializar el semáforo disponible

  // Crear cola para comunicar el número RFID entre tareas
  xQueueRFID = xQueueCreate(10, sizeof(char[20]));  // 10 mensajes de hasta 20 caracteres cada uno

  // Crear tareas FreeRTOS
  xTaskCreate(taskSendGPSData, "SendGPSDataTask", 4096, NULL, 1, NULL);
  xTaskCreate(taskSendRFIDAlert, "SendRFIDAlertTask", 4096, NULL, 1, NULL);
  xTaskCreate(taskSimulateRFIDRead, "SimulateRFIDReadTask", 2048, NULL, 1, NULL);
}

void loop() {
  mqtt.loop();
}

// Función para conectar a MQTT
bool mqttConnect() {
  if (!mqtt.connected()) {

    imei = modem.getIMEI();
    char buffer[30];
    snprintf(buffer, sizeof(buffer), "GPS(%s)", imei.c_str());
    const char* IDclient = buffer;

    Serial.print(F("Connecting to MQTT broker..."));
    if (mqtt.connect(IDclient, mqtt_username, mqtt_password)) {
      Serial.println(F(" connected"));
      return true;
    } else {
      Serial.println(F(" failed"));
      return false;
    }
  }
  return true;
}

// Tarea para enviar datos de GPS periódicamente
void taskSendGPSData(void* parameter) {
  for (;;) {
    

    if (xSemaphoreTake(xSemaphore, 1000 / portTICK_PERIOD_MS) == pdTRUE) {

      if (!gpsReady) {  // Esperar hasta que se tenga ubicación GPS
      vTaskDelay(1000 / portTICK_PERIOD_MS);
      continue;
    }

      float lat = 0, lon = 0, speed = 0, hdop = 0, cordN = 0;
      int hour = 0, minute = 0;

      modem.enableGPS();
      if (modem.getGPS(&lat, &lon, &speed, &cordN, &hdop, &hour, &minute)) {
        char buffer_data[256];
        snprintf(buffer_data, sizeof(buffer_data), "{\"idplacabus\":\"%s\",\"latitude\":%.6f,\"longitude\":%.6f,\"speed\":%.1f,\"bearing\":%.2f,\"accuracy\":%.2f,\"hour\":%d,\"minute\":%d,\"volt\":%.1f,\"signal\":%d}",
                 imei.c_str(), lat, lon, speed, cordN, hdop, hour, minute, modem.getBattVoltage() / 1000.0F, modem.getSignalQuality());

        if (mqtt.publish(GPSTopic, buffer_data)) {
          Serial.println("GPS Data Sent");
        } else {
          Serial.println("Failed to send GPS data");
        }
      }
      xSemaphoreGive(xSemaphore);  // Liberar el semáforo
    }
    vTaskDelay(5000 / portTICK_PERIOD_MS);  // Enviar datos cada 5 segundos
  }
}

// Otras tareas permanecen iguales...


// Tarea para enviar alertas con el número de tarjeta RFID
void taskSendRFIDAlert(void* parameter) {
  char rfidTag[20];

  for (;;) {
    // Recibir el número RFID desde la cola
    if (xQueueReceive(xQueueRFID, &rfidTag, portMAX_DELAY) == pdTRUE) {
      if (xSemaphoreTake(xSemaphore, 1000 / portTICK_PERIOD_MS) == pdTRUE) {
        if (!mqttConnect()) {
          Serial.println("MQTT: Disconect");
        }

        char alertMessage[256];
        snprintf(alertMessage, sizeof(alertMessage), "{\"alert\":\"RFID detected\",\"rfid\":\"%s\"}", rfidTag);

        // Enviar alerta por MQTT
        if (mqtt.publish(AlertTopic, alertMessage)) {
          Serial.printf("MQTT: RFID alert sent -> %s\n", rfidTag);
        } else {
          Serial.println("MQTT: Failed to send RFID alert");
        }
        xSemaphoreGive(xSemaphore);  // Liberar el semáforo
      }
    }
  }
}

// Tarea para simular la lectura de tarjetas RFID
void taskSimulateRFIDRead(void* parameter) {
  for (;;) {
    char simulatedRFID[20];
    snprintf(simulatedRFID, sizeof(simulatedRFID), "RFID_TAG_%d", random(1000, 9999));
    Serial.printf("Simulated RFID Read: %s\n", simulatedRFID);

    // Enviar el número RFID a la cola
    if (xQueueSend(xQueueRFID, &simulatedRFID, portMAX_DELAY) == pdTRUE) {
      Serial.println("RFID sent to queue");
    }
    vTaskDelay(10000 / portTICK_PERIOD_MS);  // Simular una lectura cada 10 segundos
  }
}
