#include "PN532DV.h"
// Define the pins for SPI communication with the PN532 module
//#define PN532_MISO (19)
//#define PN532_MOSI (23)
//#define PN532_SCK (18)
#define PN532_SS (5)  // 17


// Create an instance of the PN532 module
PN532DV nfc(PN532_SS, &SPI);  //Hardware SPI
//PN532DV nfc(PN532_SCK, PN532_MISO, PN532_MOSI, PN532_SS); //Software SPI


#include "TinyGsmClientSIM7600DV.h"
typedef TinyGsmSim7600 TinyGsm;
typedef TinyGsmSim7600::GsmClientSim7600 TinyGsmClient;
TinyGsm modem(Serial1);
TinyGsmClient client(modem);

#include <PubSubClient.h>
PubSubClient mqtt(client);

#include <freertos/FreeRTOS.h>
#include <freertos/semphr.h>

#include "datos.h"

// Semaphore for controlled access to the GpsData struct
SemaphoreHandle_t xSemaphoreRFIDhandle;
SemaphoreHandle_t xSemaphoreBuzzerHandle;

void setup() {
  Serial.begin(115200);
  Serial1.begin(115200, SERIAL_8N1, PIN_RX, PIN_TX);

  pinMode(LED_data_sent, OUTPUT);
  digitalWrite(LED_data_sent, 0);
  pinMode(PIN_VEL_status, OUTPUT);
  digitalWrite(PIN_VEL_status, 0);

  initializeModemGPS();
  mqttConnect();

  // Create binary semaphore to RFID handle
  xSemaphoreRFIDhandle = xSemaphoreCreateBinary();
  xSemaphoreGive(xSemaphoreRFIDhandle);  // Initialize semaphore as available

  // Create binary semaphore to buzzer handle
  xSemaphoreBuzzerHandle = xSemaphoreCreateBinary();
  xSemaphoreGive(xSemaphoreBuzzerHandle);  // Initialize semaphore as available

  if (initializePN532()) {
    // Create tasks for read RFD card
    xTaskCreate(taskRFIDRead, "taskRFIDRead", 4096, NULL, 1, NULL);
    Serial.println(F("=== Waiting for an ISO14443A card ==="));
  }
}

void loop() {
  mqtt.loop();
  if (!mqtt.connected()) {
    checkNetworkConnection();
    mqttConnect();
  }

  GpsData currentData = createGpsData();

  //Serial.println(F("Attempting to get location"));
  modem.enableGPS();
  digitalWrite(LED_data_sent, 1);

  // Get all data into the struct
  if (modem.getGPS(&currentData.lat, &currentData.lon,
                   &currentData.speed, &currentData.bearing,
                   &currentData.hdop, &currentData.hour,
                   &currentData.minute)) {

    currentData.voltage = modem.getBattVoltage() / 1000.0F;
    currentData.signal = modem.getSignalQuality();

    if (xSemaphoreTake(xSemaphoreRFIDhandle, portMAX_DELAY) == pdTRUE) {
      snprintf(currentData.RFID, sizeof(currentData.RFID), "%s", lastRFID);  // Safe copy
      xSemaphoreGive(xSemaphoreRFIDhandle);
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

        if (xSemaphoreTake(xSemaphoreRFIDhandle, portMAX_DELAY) == pdTRUE) {
          snprintf(lastRFID, sizeof(lastRFID), "%s", "0000000000");  // Reset RFID
          xSemaphoreGive(xSemaphoreRFIDhandle);
        }
      } else {
        Serial.println(F("⚠️ Failed to send data"));
      }
      freeJsonPayload(&payload);  // Free memory immediately
    }

    if (xSemaphoreTake(xSemaphoreRFIDhandle, portMAX_DELAY) == pdTRUE) {
      digitalWrite(PIN_VEL_status, (currentData.speed > 20));
      xSemaphoreGive(xSemaphoreRFIDhandle);
    }
  }
}

// Task for RFID card reading
void taskRFIDRead(void* parameter) {
  for (;;) {

    uint8_t uid[4] = { 0 };
    uint8_t uidLength;

    // Try to read a passive target ID (NFC card)
    if (nfc.readPassiveTargetID(PN532_MIFARE_ISO14443A, &uid[0], &uidLength, 500)) {
      if (!nfc.cardPresent) {
        nfc.cardPresent = true;
        Serial.println(F("Found a card!"));

        char formattedUID[11];
        // Process the UID to convert it to a decimal value
        uint64_t decimalUID = nfc.processUID(uid, uidLength);
        snprintf(formattedUID, 11, "%010llu", decimalUID);  // Format the UID to always have 10 digits

        // Safe buffer RFID access
        if (xSemaphoreTake(xSemaphoreRFIDhandle, portMAX_DELAY) == pdTRUE) {
          snprintf(lastRFID, sizeof(lastRFID), "%s", formattedUID);
          Serial.printf("RFID: %s \n", lastRFID);
          xSemaphoreGive(xSemaphoreRFIDhandle);
        }
        // Safe buzzer access
        if (xSemaphoreTake(xSemaphoreRFIDhandle, portMAX_DELAY) == pdTRUE) {
          digitalWrite(PIN_VEL_status, 1);
          delay(150);
          digitalWrite(PIN_VEL_status, 0);
          xSemaphoreGive(xSemaphoreRFIDhandle);
        }
      }
    } else {
      if (nfc.cardPresent) {
        Serial.println(F("Card removed!"));
        nfc.cardPresent = false;
      }
    }

    //vTaskDelay(60000 / portTICK_PERIOD_MS);  // Simulate read every 60 seconds
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

// Function to initialize the PN532 module
bool initializePN532(void) {
  nfc.begin();
  uint32_t versiondata = nfc.getFirmwareVersion();
  if (!versiondata) {
    Serial.println(F("❌ Didn't find PN53x board"));
    delay(1000);
    return false;
  }
  nfc.setPassiveActivationRetries(0xFF);
  return true;
}
