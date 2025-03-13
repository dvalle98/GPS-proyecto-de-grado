/* Codigo enfocado en la optimizacion del uso de los recursos para su funcionamiento continuo
   uso de variables globales reducido
   uso de buffer para evitar String
  */
#include "TinyGsmClientSIM7600.h"
typedef TinyGsmSim7600 TinyGsm;
typedef TinyGsmSim7600::GsmClientSim7600 TinyGsmClient;

#include <PubSubClient.h>

#define TINY_GSM_USE_GPRS true

#define PIN_TX 27
#define PIN_RX 26
#define UART_BAUD 115200

#define PWR_PIN 4
#define LED_data_sent 0
#define PIN_VEL_status 12
#define POWER_PIN 25

#define PMU_IRQ 35
#define I2C_SDA 21
#define I2C_SCL 22


const char* apn = "internet.comcel.com.co";
const char* mqtt_username = "test";
const char* mqtt_password = "CloudTech*";
const char* broker = "18.212.130.131";
const char* GPSTopic = "gps/app/testliceo/puntos/868020034072685";
//const char* GPSTopic = "Admin/test/Dispositivo_GPS/Datos/Coordenadas";
#define mqttKeepAlive 60
String imei;

TinyGsm modem(Serial1);
TinyGsmClient client(modem);
PubSubClient mqtt(client);

void setup() {
  Serial.begin(115200);
  Serial1.begin(115200, SERIAL_8N1, PIN_RX, PIN_TX);


  pinMode(LED_data_sent, OUTPUT);
  digitalWrite(LED_data_sent, 0);
  pinMode(PIN_VEL_status, OUTPUT);
  digitalWrite(PIN_VEL_status, 0);

  //activar modem
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

  /*Range – 0 to 15
  Bit0: GLONASS
  Bit1: BEIDOU
  Bit2: GALILEO
  Bit3: QZSS
  1: enable 0:disable
  GPS always support
  0011 = 3
  */
  modem.setGNSSMode(3, 0);
  modem.enableGPS();
  Serial.printf("GNSS mode: %u\n", modem.getGNSSMode());
}

void loop() {

  enviar_datosMQTT();

  mqtt.loop();
}

bool mqttConnect() {
  imei = modem.getIMEI();  // Obtener el IMEI como una String
  //const char* imeiCharArray = imei.c_str();  // Convertir la String a un const char*

  char buffer[30];  // Define un buffer lo suficientemente grande
  snprintf(buffer, 30, "GPS(%s)", imei.c_str());

  // Ahora puedes usar buffer como un const char*
  const char* IDclient = buffer;

  Serial.print(F("Connecting to "));
  Serial.print(broker);
  if (!mqtt.connect(IDclient, mqtt_username, mqtt_password)) {
    Serial.println(F(" fail"));
    return false;
  }
  Serial.println(F(" success"));


  return mqtt.connected();
}

void enviar_datosMQTT() {
  float lat = 0, lon = 0, speed = 0, hdop = 0, cordN = 0;
  int hour = 0, minute = 0;

  digitalWrite(LED_data_sent, HIGH);
  modem.enableGPS();
  Serial.println(F("waiting for location: "));

  if (modem.getGPS(&lat, &lon, &speed, &cordN, &hdop, &hour, &minute)) {

    digitalWrite(PIN_VEL_status, (speed > 60));

    char buffer_data[256];  // Definir un buffer lo suficientemente grande
    // Formatear el JSON en el buffer
    snprintf(buffer_data, sizeof(buffer_data), "{\"idplacabus\":\"%s\",\"latitude\":%.6f,\"longitude\":%.6f,\"speed\":%.1f,\"bearing\":%.2f,\"accuracy\":%.2f,\"hour\":%d,\"minute\":%d,\"volt\":%.1f,\"signal\":%d}",
             imei.c_str(), lat, lon, speed, cordN, hdop, hour, minute, modem.getBattVoltage() / 1000.0F, modem.getSignalQuality());

    // Imprimir el JSON
    //Serial.println(buffer_data);

    if (mqtt.publish(GPSTopic, buffer_data, sizeof(buffer_data))) {
      digitalWrite(LED_data_sent, LOW);
      Serial.println(F("Sent data"));
      delay(100);

    } else {

      Serial.println(F("Error sending data"));
      if (!mqtt.connected()) {
        Serial.println(F("=== MQTT IS NOT CONNECTED ==="));

        verificar_conexionRed();
        mqttConnect();
      }
    }
  }

}

void verificar_conexionRed() {
  Serial.print(F("Network checking:\t"));

  if (!modem.isNetworkConnected()) {
    Serial.print(F("Network disconnected\nConecting..."));

    if (!modem.waitForNetwork(60000L, true)) {
      Serial.println(F("\tfail"));
      delay(5000);

      Serial.println(F("Initializing modem..."));
      if (!modem.restart()) {
        Serial.println(F("Failed to restart modem, attempting to continue without restarting"));
      }
    }
    if (modem.isNetworkConnected()) {
      Serial.println(F("Network reconnected"));
    }
  } else if (modem.isNetworkConnected()) {
    Serial.println(F("Network is connected"));
  }
  
  Serial.print(F("GPRS checking:\t"));
  if (!modem.isGprsConnected()) {
    Serial.print(F("GPRS disconnected!\nConnecting to\t:"));
    Serial.println(apn);
    if (!modem.gprsConnect(apn, "", "")) {
      Serial.println(F("\tfail"));
      delay(1000);
      ESP.restart();
    }
    if (modem.isGprsConnected()) {
      Serial.println(F("GPRS reconnected"));
    }
  }
}
