#define TINY_GSM_MODEM_SIM7600
#include <TinyGsmClient.h>
#include <PubSubClient.h>
#include <ArduinoJson.h>

#define TINY_GSM_MODEM_SIM7600
#define TINY_GSM_USE_GPRS true

#define SerialMon Serial
#define SerialAT Serial1
#define TINY_GSM_DEBUG SerialMon

const char apn[] = "internet.comcel.com.co";
const char mqtt_username[] = "test";
const char mqtt_password[] = "CloudTech*";
const char *broker = "18.212.130.131";
const char *GPSTopic = "Admin/test/Dispositivo_GPS2/Datos/Coordenadas";
const int mqttKeepAlive = 60;

TinyGsm modem(SerialAT);
TinyGsmClient client(modem);
PubSubClient mqtt(client);

int ledStatus = LOW;
uint32_t lastReconnectAttempt = 0;


float lat = 0;
float lon = 0;
float speed = 0;
float alt = 0;
int vsat = 0;
int usat = 0;
float accuracy = 0;
int year = 0;
int month = 0;
int day = 0;
int hour = 0;
int minute = 0;
uint16_t milliVolts = 0;

char sendbuffer[120];
const char *Bat_value;
RTC_DATA_ATTR int bootCount = 0;

#define PIN_TX 27
#define PIN_RX 26
#define UART_BAUD 115200
#define PWR_PIN 4
#define LED_PIN 12
#define POWER_PIN 25
#define IND_PIN 36
#define PMU_IRQ 35
#define I2C_SDA 21
#define I2C_SCL 22
#define BAT_ADC 35

bool mqttConnect() {
  String imei = modem.getIMEI();             // Obtener el IMEI como una String
  const char* imeiCharArray = imei.c_str();  // Convertir la String a un const char*

  //concatenar el IMEI dentro de tu const char
  char buffer[30];  // buffer lo suficientemente grande
  snprintf(buffer, sizeof(buffer), "GPS(%s)", imeiCharArray);

  // usar buffer como un const char*
  const char* result = buffer;


  SerialMon.print("Connecting to ");
  SerialMon.print(broker);
  if (!mqtt.connect("GPS2", mqtt_username, mqtt_password)) {
    SerialMon.println(" fail");
    return false;
  }
  SerialMon.println(" success");
  return mqtt.connected();
}

void setup() {
  SerialMon.begin(115200);
  delay(10);

  pinMode(LED_PIN, OUTPUT);
  digitalWrite(LED_PIN, LOW);
  pinMode(POWER_PIN, OUTPUT);
  digitalWrite(POWER_PIN, HIGH);
  pinMode(PWR_PIN, OUTPUT);
  digitalWrite(PWR_PIN, HIGH);
  delay(500);
  digitalWrite(PWR_PIN, LOW);
  pinMode(IND_PIN, INPUT);

  SerialAT.begin(115200, SERIAL_8N1, PIN_RX, PIN_TX);

  SerialMon.println("Iniciando modulo");
  modem.restart();

  SerialMon.print("Waiting for network...");
  if (!modem.waitForNetwork()) {
    SerialMon.println(" fail");
    delay(10000);
    return;
  }
  SerialMon.println(" success");

#if TINY_GSM_USE_GPRS
  SerialMon.print(F("Connecting to "));
  SerialMon.print(apn);
  if (!modem.gprsConnect(apn, "", "")) {
    SerialMon.println(" fail");
    delay(10000);
    return;
  }
  SerialMon.println(" success");
#endif

  mqtt.setKeepAlive(mqttKeepAlive);
  mqtt.setServer(broker, 1883);
}

void loop() {
  verificar_conexionRed();

  if (!mqtt.connected()) {
    SerialMon.println("=== MQTT IS NOT CONNECTED ===");
    uint32_t t = millis();
    if (t - lastReconnectAttempt > 10000L) {
      lastReconnectAttempt = t;
      if (mqttConnect()) {
        lastReconnectAttempt = 0;
      }
    }
    delay(100);
    return;
  } else {
    enviar_datosMQTT();
  }

  mqtt.loop();
}

void enviar_datosMQTT() {
  if (lat <= 0 || lon <= 0) {

    modem.sendAT("+SGPIO=0,4,1,1");
    modem.waitResponse(10000L);
    SerialMon.println("+SGPIO=0,4,1,1");

    modem.enableGPS();
    //lat, lon, speed, alt, vsat, usat, accuracy, year, month, day, hour, minute, second
    if (modem.getGPS(&lat, &lon, &speed, &alt, &vsat, &usat, &accuracy, &year, &month, &day, &hour, &minute)) {

      digitalWrite(LED_PIN, ledStatus);

      DynamicJsonDocument doc(320);
      doc["latitud"] = String(lat, 8);
      doc["longitud"] = String(lon, 8);
      doc["velocidad"] = String(speed);

      doc["accuracy"] = String(accuracy);

      doc["hour"] = String(hour - 5);
      doc["minute"] = String(minute);
      doc["volt"] = String(modem.getBattVoltage() / 1000.0F);

      String json;
      serializeJson(doc, json);

      if (mqtt.publish(GPSTopic, json.c_str())) {
        SerialMon.print("Datos publicados:\t");
        Serial.println(json);
      } else {
        SerialMon.println("Error publicando");
      }

      digitalWrite(LED_PIN, !ledStatus);
    }
  }
}

void verificar_conexionRed() {
  if (!modem.isNetworkConnected()) {
    SerialMon.println("Network disconnected");
    if (!modem.waitForNetwork(60000L, true)) {
      SerialMon.println(" fail");
      delay(5000);

      Serial.println("Initializing modem...");
      if (!modem.restart()) {
        Serial.println("Failed to restart modem, attempting to continue without restarting");
      }
    }
    if (modem.isNetworkConnected()) {
      SerialMon.println("Network re-connected");
    }

#if TINY_GSM_USE_GPRS
    if (!modem.isGprsConnected()) {
      SerialMon.println("GPRS disconnected!");
      SerialMon.print(F("Connecting to "));
      SerialMon.print(apn);
      if (!modem.gprsConnect(apn, "", "")) {
        SerialMon.println(" fail");
        delay(10000);
        return;
      }
      if (modem.isGprsConnected()) {
        SerialMon.println("GPRS reconnected");
      }
    }
#endif
  }
}
