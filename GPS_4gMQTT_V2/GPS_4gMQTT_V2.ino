#include "recursos/TinyGsmClientSIM7600DV.h"
typedef TinyGsmSim7600 TinyGsm;
typedef TinyGsmSim7600::GsmClientSim7600 TinyGsmClient;
TinyGsm modem(Serial1);
TinyGsmClient client(modem);

#include <PubSubClient.h>
PubSubClient mqtt(client);

#include <freertos/FreeRTOS.h>
#include <freertos/semphr.h>

#include "recursos/datos.h"

// Semaphore for controlled access to the GpsData struct
SemaphoreHandle_t xSemaphore;

void setup() {
  Serial.begin(115200);
  Serial1.begin(115200, SERIAL_8N1, PIN_RX, PIN_TX);

  pinMode(LED_data_sent, OUTPUT);
  digitalWrite(LED_data_sent, 0);
  pinMode(PIN_VEL_status, OUTPUT);
  digitalWrite(PIN_VEL_status, 0);

  initializeModemGPS();
  mqttConnect();

  // Create binary semaphore
  xSemaphore = xSemaphoreCreateBinary();
  xSemaphoreGive(xSemaphore);  // Initialize semaphore as available

  // Create FreeRTOS tasks
  xTaskCreate(taskRFIDRead, "taskRFIDRead", 4096, NULL, 1, NULL);
}

void loop() {
  mqtt.loop();
  if (!mqtt.connected()) {
    checkNetworkConnection();
    mqttConnect();
  }

  GpsData currentData = createGpsData();

  Serial.println(F("Attempting to get location"));
  modem.enableGPS();
  digitalWrite(LED_data_sent, 1);
  
  // Get all data into the struct
  if (modem.getGPS(&currentData.lat, &currentData.lon,
                   &currentData.speed, &currentData.bearing,
                   &currentData.hdop, &currentData.hour,
                   &currentData.minute)) {

    currentData.voltage = modem.getBattVoltage() / 1000.0F;
    currentData.signal = modem.getSignalQuality();

    if (xSemaphoreTake(xSemaphore, portMAX_DELAY) == pdTRUE) {
      snprintf(currentData.RFID, sizeof(currentData.RFID), "%s", lastRFID);  // Safe copy
      xSemaphoreGive(xSemaphore);
    }

    // Critical data validation
    if (currentData.lat < -90.0 || currentData.lat > 90.0 || currentData.lon < -180.0 || currentData.lon > 180.0) {
      Serial.println(F("⚠️ Invalid GPS data"));
      return;
    }

    // Create dynamic payload
    JsonPayload payload = createJsonPayload(&currentData);

    if (payload.buffer) {
      if (mqtt.publish(config.GPSTopic, payload.buffer)) {
        digitalWrite(LED_data_sent, 0);
        Serial.println(F("✅ Data sent successfully"));        

        if (xSemaphoreTake(xSemaphore, portMAX_DELAY) == pdTRUE) {
          snprintf(lastRFID, sizeof(lastRFID), "%s", "0000000000");  // Reset RFID
          xSemaphoreGive(xSemaphore);
        }
      } else {
        Serial.println(F("⚠️ Failed to send data"));
      }
      freeJsonPayload(&payload);  // Free memory immediately
    }

    digitalWrite(PIN_VEL_status, (currentData.speed > 20));
  }
}

// Task for RFID card reading simulation
void taskRFIDRead(void* parameter) {
  for (;;) {
    // Simulate reading fixed RFID "1234567890"
    const char* newRFID = "1234567890";
    Serial.printf(" RFID Read: %s\n", newRFID);

    // Safe buffer access
    if (xSemaphoreTake(xSemaphore, portMAX_DELAY) == pdTRUE) {
      snprintf(lastRFID, sizeof(lastRFID), "%s", newRFID);
      xSemaphoreGive(xSemaphore);
    }
    vTaskDelay(60000 / portTICK_PERIOD_MS);  // Simulate read every 60 seconds
  }
}

// Function to connect MQTT
bool mqttConnect(void) {
  if (!mqtt.connected()) {
    Serial.print(F("Connecting to MQTT broker..."));
    if (mqtt.connect(config.IDclient, config.mqtt_username, config.mqtt_password)) {
      Serial.println(F(" ✅ Connected"));
      return true;
    } else {
      Serial.println(F(" ❌ Failed"));
      return false;
    }
  }
  return true;
}

// Function to initialize struct
GpsData createGpsData(void) {
  GpsData data;                                             // Reserve stack space
  memset(&data, 0, sizeof(data));                           // Initialization
  strncpy(data.RFID, "0000000000", sizeof(data.RFID) - 1);  // 10 zeros
  data.RFID[sizeof(data.RFID) - 1] = '\0';                  // Null termination
  return data;
}

// Function to create dynamic JSON payload
JsonPayload createJsonPayload(const GpsData* data) {
  JsonPayload payload = { NULL, 0 };

  // Calculate exact required size
  payload.size = snprintf(NULL, 0,
                          "{\"id\":\"%s\",\"lat\":%.6f,\"lon\":%.6f,"
                          "\"spd\":%.1f,\"brng\":%.2f,\"acc\":%.2f,"
                          "\"time\":\"%02d:%02d\",\"volt\":%.1f,\"sig\":%d,\"RFID\":\"%s\"}",
                          config.IDclient, data->lat, data->lon,
                          data->speed, data->bearing, data->hdop,
                          data->hour - 5, data->minute,
                          data->voltage, data->signal, data->RFID)
                 + 1;  // +1 for null terminator

  // Allocate exact memory
  payload.buffer = (char*)malloc(payload.size);
  if (payload.buffer) {
    snprintf(payload.buffer, payload.size,
             "{\"id\":\"%s\",\"lat\":%.6f,\"lon\":%.6f,"
             "\"spd\":%.1f,\"brng\":%.2f,\"acc\":%.2f,"
             "\"time\":\"%02d:%02d\",\"volt\":%.1f,\"sig\":%d,\"RFID\":\"%s\"}",
             config.IDclient, data->lat, data->lon,
             data->speed, data->bearing, data->hdop,
             data->hour - 5, data->minute,
             data->voltage, data->signal, data->RFID);
  }
  return payload;
}

// Function to free memory
void freeJsonPayload(JsonPayload* payload) {
  if (payload->buffer) {
    free(payload->buffer);
    payload->buffer = NULL;
    payload->size = 0;
  }
}

// Function to initialize Modem and GPS
void initializeModemGPS(void) {
  Serial.println(F("Starting modem...!"));

  // Enable modem
  pinMode(POWER_PIN, OUTPUT);
  digitalWrite(POWER_PIN, 1);
  pinMode(PWR_PIN, OUTPUT);
  digitalWrite(PWR_PIN, 1);

  modem.restart();
  checkNetworkConnection();

  mqtt.setKeepAlive(config.mqttKeepAlive);
  mqtt.setServer(config.broker, 1883);

  modem.setGNSSMode(3, 0);
  modem.enableGPS();
}

// Function to check the Network Connection
void checkNetworkConnection(void) {
  uint8_t attempts = 0;

  Serial.println(F("🔄 Checking network and GPRS..."));

  // 1️⃣ Check cellular network connection
  while (!modem.isNetworkConnected() && attempts < 2) {
    Serial.print(F("🌐 Network disconnected - Attempt "));
    Serial.println(attempts + 1);

    if (modem.waitForNetwork(15000L)) {  // 15s timeout
      Serial.println(F("✅ Network reconnected!"));
      break;
    } else {
      Serial.println(F("⚠️ Critical network failure, rebooting modem..."));
      modem.restart();
    }
    attempts++;
  }

  // 2️⃣ Check GPRS connection
  attempts = 0;  // Reset attempt counter
  while (!modem.isGprsConnected() && attempts < 2) {
    Serial.print(F("📡 GPRS disconnected - Attempt "));
    Serial.println(attempts + 1);

    if (modem.gprsConnect(config.apn, "", "")) {
      Serial.println(F("✅ GPRS reconnected!"));
      break;
    }
    attempts++;
    delay(500);
  }

  // If GPRS still disconnected after attempts
  if (!modem.isGprsConnected()) {
    Serial.println(F("❌ Critical error: Rebooting system..."));
    delay(1000);
    ESP.restart();
  }
}