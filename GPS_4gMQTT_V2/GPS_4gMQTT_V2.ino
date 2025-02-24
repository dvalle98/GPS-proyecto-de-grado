#include "recursos/TinyGsmClientSIM7600DV.h"
typedef TinyGsmSim7600 TinyGsm;
typedef TinyGsmSim7600::GsmClientSim7600 TinyGsmClient;
TinyGsm modem(Serial1);
TinyGsmClient client(modem);

#include <PubSubClient.h>
PubSubClient mqtt(client);

#include <freertos/FreeRTOS.h>
#include <freertos/semphr.h>
//#include <freertos/queue.h>

#include "recursos/datos.h"

bool mqttConnect(void);
GpsData createGpsData(void);
JsonPayload createJsonPayload(const char* imei, const GpsData* data);
void freeJsonPayload(JsonPayload* payload);
void initializeModemGPS(void);

// Semáforo para acceso controlado al struct GpsData;
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

  // Crear semáforo binario
  xSemaphore = xSemaphoreCreateBinary();
  xSemaphoreGive(xSemaphore);  // Inicializar el semáforo disponible

  // Crear tareas FreeRTOS
  xTaskCreate(taskRFIDRead, "taskRFIDRead", 4096, NULL, 1, NULL);
}

void loop() {

  mqtt.loop();

  GpsData currentData = createGpsData();

  Serial.println(F("Intentando obtener ubicacion"));
  digitalWrite(LED_data_sent, 1);
  modem.enableGPS();
  //  Obtener todos los datos en el struct
  if (modem.getGPS(&currentData.lat, &currentData.lon,
                   &currentData.speed, &currentData.bearing,
                   &currentData.hdop, &currentData.hour,
                   &currentData.minute)) {
    currentData.voltage = modem.getBattVoltage() / 1000.0F;
    currentData.signal = modem.getSignalQuality();

    if (xSemaphoreTake(xSemaphore, portMAX_DELAY) == pdTRUE) {
      snprintf(currentData.RFID, sizeof(currentData.RFID), "%s", lastRFID);  // Seguro
      xSemaphoreGive(xSemaphore);
    }
    //  Validación de datos crítica
    if (currentData.lat < -90.0 || currentData.lat > 90.0 || currentData.lon < -180.0 || currentData.lon > 180.0) {
      Serial.println(F("Datos GPS inválidos"));
      return;
    }

    //  Crear payload dinámico
    JsonPayload payload = createJsonPayload(&currentData);

    if (payload.buffer) {
      if (mqtt.connected() && mqtt.publish(config.GPSTopic, payload.buffer)) {
        Serial.println(F("Datos enviados"));
        digitalWrite(LED_data_sent, 0);
      } else {
        Serial.println(F("Error enviando"));
      }
      freeJsonPayload(&payload);  // Liberar inmediatamente
    }
  }

  //delay(30000); // Espera controlada
}

// Tarea para la lectura de tarjetas RFID
void taskRFIDRead(void* parameter) {
  for (;;) {
    // Simular lectura de RFID fijo "1234567890"
    const char* newRFID = "1234567890";
    //snprintf(lastRFID, sizeof(lastRFID), "%s", randomID);  // Formato: 10 dígitos con ceros a la izquierda

    Serial.printf(" RFID Readed: %s\n", newRFID);

    // Acceso seguro al buffer compartido
    if (xSemaphoreTake(xSemaphore, portMAX_DELAY) == pdTRUE) {
      snprintf(lastRFID, sizeof(lastRFID), "%s", newRFID); // Copiar el nuevo 
      xSemaphoreGive(xSemaphore);
    }
    vTaskDelay(10000 / portTICK_PERIOD_MS);  // Simular una lectura cada 10 segundos
  }
}

bool mqttConnect(void) {
  if (!mqtt.connected()) {

    Serial.print(F("Connecting to MQTT broker..."));
    if (mqtt.connect(config.IDclient, config.mqtt_username, config.mqtt_password)) {
      Serial.println(F(" connected"));
      return true;
    } else {
      Serial.println(F(" failed"));
      return false;
    }
  }
  return true;
}

//  Función para inicializar struct
GpsData createGpsData(void) {
  GpsData data;                                             // Reserva espacio en el stack
  memset(&data, 0, sizeof(data));                           // Inicialización
  strncpy(data.RFID, "0000000000", sizeof(data.RFID) - 1);  // 10 ceros
  data.RFID[sizeof(data.RFID) - 1] = '\0';                  // Terminación nula
  return data;
}

//  Función para crear payload dinámico
JsonPayload createJsonPayload(const GpsData* data) {
  JsonPayload payload = { NULL, 0 };

  //  Calcular tamaño exacto necesario
  payload.size = snprintf(NULL, 0,
                          "{\"id\":\"%s\",\"lat\":%.6f,\"lon\":%.6f,"
                          "\"spd\":%.1f,\"brng\":%.2f,\"acc\":%.2f,"
                          "\"time\":\"%02d:%02d\",\"volt\":%.1f,\"sig\":%d,\"RFID\":%s}",
                          config.IDclient, data->lat, data->lon,
                          data->speed, data->bearing, data->hdop,
                          data->hour, data->minute,
                          data->voltage, data->signal, data->RFID)
                 + 1;  // +1 para el null terminator

  //  Asignar memoria exacta
  payload.buffer = (char*)malloc(payload.size);
  if (payload.buffer) {
    snprintf(payload.buffer, payload.size,
             "{\"id\":\"%s\",\"lat\":%.6f,\"lon\":%.6f,"
             "\"spd\":%.1f,\"brng\":%.2f,\"acc\":%.2f,"
             "\"time\":\"%02d:%02d\",\"volt\":%.1f,\"sig\":%d,\"RFID\":%s}",
             config.IDclient, data->lat, data->lon,
             data->speed, data->bearing, data->hdop,
             data->hour, data->minute,
             data->voltage, data->signal, data->RFID);
  }

  return payload;
}

//  Función para liberar memoria
void freeJsonPayload(JsonPayload* payload) {
  if (payload->buffer) {
    free(payload->buffer);
    payload->buffer = NULL;
    payload->size = 0;
  }
}


void initializeModemGPS(void) {
  Serial.println(F("Starting modem...!"));

  // Activar modem
  pinMode(POWER_PIN, OUTPUT);
  digitalWrite(POWER_PIN, 1);
  pinMode(PWR_PIN, OUTPUT);
  digitalWrite(PWR_PIN, 1);

  modem.restart();

  Serial.print(F("Waiting for network..."));
  if (!modem.waitForNetwork(60000L, true)) {
    Serial.println(F(" fail"));
    delay(1000);
    ESP.restart();
  }
  Serial.print(F(" success\nConnecting to: "));
  Serial.print(config.apn);

  if (!modem.gprsConnect(config.apn, "", "")) {
    Serial.println(F(" fail"));
    delay(1000);
    ESP.restart();
  }
  Serial.println(F(" success"));

  mqtt.setKeepAlive(config.mqttKeepAlive);
  mqtt.setServer(config.broker, 1883);

  modem.setGNSSMode(3, 0);
  modem.enableGPS();
}
