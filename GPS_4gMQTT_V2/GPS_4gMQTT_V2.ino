#include "TinyGsmClientSIM7600.h"
typedef TinyGsmSim7600 TinyGsm;
typedef TinyGsmSim7600::GsmClientSim7600 TinyGsmClient;

#include <PubSubClient.h>
#include <freertos/FreeRTOS.h>
#include <freertos/semphr.h>
#include <freertos/queue.h>

#include"datos.h"

bool mqttConnect(void);
GpsData createGpsData(void);
JsonPayload createJsonPayload(const char* imei, const GpsData* data);
void freeJsonPayload(JsonPayload* payload);

void initializeModemGPS(void);

void setup() {
    Serial.begin(115200);
    modem.init();
}

void loop() {
    static char imei[16] = {0}; // Memoria estática para IMEI
    if(!imei[0]) modem.getIMEI(imei, sizeof(imei));

    GpsData currentData = createGpsData();
    
    //  Obtener todos los datos en el struct
    if(modem.getGPS(&currentData.lat, &currentData.lon, 
                   &currentData.speed, &currentData.bearing,
                   &currentData.hdop, &currentData.hour,
                   &currentData.minute)) {
        currentData.voltage = modem.getBattVoltage() / 1000.0F;
        currentData.signal = modem.getSignalQuality();
        
        //  Validación de datos crítica
        if(currentData.lat < -90.0 || currentData.lat > 90.0 ||
           currentData.lon < -180.0 || currentData.lon > 180.0) {
            Serial.println(F("Datos GPS inválidos"));
            return;
        }
        
        //  Crear payload dinámico
        JsonPayload payload = createJsonPayload(imei, &currentData);
        
        if(payload.buffer) {
            if(mqtt.connected() && mqtt.publish(GPSTopic, payload.buffer)) {
                Serial.println(F("Datos enviados"));
            } else {
                Serial.println(F("Error enviando"));
            }
            freeJsonPayload(&payload); // Liberar inmediatamente
        }
    }
    
    //delay(30000); // Espera controlada
}


// Tarea para la lectura de tarjetas RFID
void taskRFIDRead(void* parameter) {
  for (;;) {
    char readedRFID[20];
    snprintf(readedRFID, sizeof(simulatedRFID), "RFID_TAG_%d", random(1000000000, 9999999999));
    Serial.printf(" RFID Readed: %s\n", readedRFID);

    // Modificar dato de RFID en el struct
    
    vTaskDelay(10000 / portTICK_PERIOD_MS);  // Simular una lectura cada 10 segundos
  }
}


bool mqttConnect(void) {
  if (!mqtt.connected()) {

    imei = modem.getIMEI();
    char buffer[30];
    snprintf(buffer, sizeof(buffer), "GPS(%s)", imei.c_str());
    const char* IDclient = buffer;

    Serial.print(F("Connecting to MQTT broker..."));
    if (mqtt.connect(IDclient, config.mqtt_username, config.mqtt_password)) {
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
    GpsData data; // Reserva espacio en el stack
    memset(&data, 0, sizeof(data)); // Inicialización
    return data;
}

//  Función para crear payload dinámico
JsonPayload createJsonPayload(const char* imei, const GpsData* data) {
    JsonPayload payload = {NULL, 0};
    
    //  Calcular tamaño exacto necesario
    payload.size = snprintf(NULL, 0, 
        "{\"id\":\"%s\",\"lat\":%.6f,\"lon\":%.6f,"
        "\"spd\":%.1f,\"brng\":%.2f,\"acc\":%.2f,"
        "\"time\":\"%02d:%02d\",\"volt\":%.1f,\"sig\":%d,\"RFID\":%d}",
        imei, data->lat, data->lon,
        data->speed, data->bearing, data->hdop,
        data->hour, data->minute,
        data->voltage, data->signal,data->RFID) + 1; // +1 para el null terminator
    
    //  Asignar memoria exacta
    payload.buffer = (char*)malloc(payload.size);
    if(payload.buffer) {
        snprintf(payload.buffer, payload.size,
            "{\"id\":\"%s\",\"lat\":%.6f,\"lon\":%.6f,"
            "\"spd\":%.1f,\"brng\":%.2f,\"acc\":%.2f,"
            "\"time\":\"%02d:%02d\",\"volt\":%.1f,\"sig\":%d,\"RFID\":%d}",
            imei, data->lat, data->lon,
            data->speed, data->bearing, data->hdop,
            data->hour, data->minute,
            data->voltage, data->signal,data->RFID);
    }
    
    return payload;
}

//  Función para liberar memoria
void freeJsonPayload(JsonPayload* payload) {
    if(payload->buffer) {
        free(payload->buffer);
        payload->buffer = NULL;
        payload->size = 0;
    }
}


void initializeModemGPS(void) {
  Serial.println(F("Starting modem...!"));

  Serial.print(F("Waiting for network..."));
  if (!modem.waitForNetwork(60000L, true)) {
    Serial.println(F(" fail"));
    delay(1000);
    ESP.restart();
  }
  Serial.println(F(" success"));

  Serial.print(F("Connecting to "));
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
}
