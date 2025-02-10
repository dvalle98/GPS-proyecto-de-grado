
#define TINY_GSM_MODEM_SIM7600

// Set serial for debug console (to the Serial Monitor, default speed 115200)
#define SerialMon Serial

// Set serial for AT commands (to the module)
// Use Hardware Serial on Mega, Leonardo, Micro
#define SerialAT Serial1

// Define the serial console for debug prints, if needed
#define TINY_GSM_DEBUG SerialMon

// Define how you're planning to connect to the internet
// These defines are only for this example; they are not needed in other code.
#define TINY_GSM_USE_GPRS true


// Your GPRS credentials, if any
const char apn[] = "internet.comcel.com.co";
const char gprsUser[] = "";
const char gprsPass[] = "";


// MQTT details
const char *broker = "18.212.130.131";

const char *GPSTopic = "Admin/test/Dispositivo_GPS/Datos/Coordenadas";
const char *mqtt_username = "test";
const char *mqtt_password = "CloudTech*";


#include <TinyGsmClient.h>
#include <PubSubClient.h>
const int mqttKeepAlive = 60;  // keep-alive time in seconds

#include <ArduinoJson.h>

#ifdef DUMP_AT_COMMANDS
#include <StreamDebugger.h>
StreamDebugger debugger(SerialAT, SerialMon);
TinyGsm modem(debugger);
#else
TinyGsm modem(SerialAT);
#endif

TinyGsmClient client(modem);
PubSubClient mqtt(client);


#define uS_TO_S_FACTOR 1000000ULL /* Conversion factor for micro seconds to seconds */
#define TIME_TO_SLEEP 60          /* Time ESP32 will go to sleep (in seconds) */

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

float speed_kph = 0;
float heading = 0;
float speed_mph = 0;
float altitude = 0;
float lat = 0;
float lon = 0;
float speed = 0;
char Lat[20];
char Lon[20];
char sendbuffer[120];
const char *Bat_value;
RTC_DATA_ATTR int bootCount = 0;


int ledStatus = LOW;

uint32_t lastReconnectAttempt = 0;

void enviar_datosMQTT() {

  while (lat <= 0 || lon <= 0) {
    modem.sendAT("+SGPIO=0,4,1,1");
    if (modem.waitResponse(10000L) != 1) {
      Serial.println(" SGPIO=0,4,1,1 false ");
    }
    modem.enableGPS();
    Serial.println("Requesting current GPS/GNSS/GLONASS location");


    if (modem.getGPS(&lat, &lon, &speed)) {
      //getGPSImpl(lat, lon, speed, alt, vsat, usat, accuracy,year, month, day, hour, minute, second)
      Serial.println("Latitude: " + String(lat, 8) + "\tLongitude: " + String(lon, 8) + "\tSpeed: " + String(speed));

      digitalWrite(LED_PIN, ledStatus);
      JsonDocument doc;

      // Agregar las variables al objeto JSON
      doc["latitud"] = String(lat, 8);
      doc["longitud"] = String(lon, 8);
      doc["velocidad"] = String(speed);

      String json = doc.as<String>();

      uint8_t buffer[120];
      size_t n = json.length() + 1;
      json.toCharArray((char *)buffer, n);

      for (int i = 0; i < n; i++) {
        sendbuffer[i] = (char)buffer[i];
      }

      sendbuffer[n - 1] = '\0';

      Serial.print("Sending: ");
      Serial.println(sendbuffer);  // (Latitude,Longitude)
      mqtt.publish(GPSTopic, sendbuffer);

      digitalWrite(LED_PIN, !ledStatus);
    }
  }
}


boolean mqttConnect() {
  SerialMon.print("Connecting to ");
  SerialMon.print(broker);

  boolean status = mqtt.connect("GPS", mqtt_username, mqtt_password);

  if (status == false) {
    SerialMon.println(" fail");
    return false;
  }
  SerialMon.println(" success");
  digitalWrite(LED_PIN, !ledStatus);
  return mqtt.connected();
}


void setup() {
  SerialMon.begin(115200);
  delay(10);


  // Onboard LED light, it can be used freely
  pinMode(LED_PIN, OUTPUT);
  digitalWrite(LED_PIN, ledStatus);

  // POWER_PIN : This pin controls the power supply of the SIM7600
  pinMode(POWER_PIN, OUTPUT);
  digitalWrite(POWER_PIN, HIGH);

  // PWR_PIN ： This Pin is the PWR-KEY of the SIM7600
  // The time of active low level impulse of PWRKEY pin to power on module , type 500 ms
  pinMode(PWR_PIN, OUTPUT);
  digitalWrite(PWR_PIN, HIGH);
  delay(500);
  digitalWrite(PWR_PIN, LOW);

  // IND_PIN: It is connected to the SIM7600 status Pin,
  // through which you can know whether the module starts normally.
  pinMode(IND_PIN, INPUT);


  DBG("Wait...");

  delay(2000);

  SerialAT.begin(UART_BAUD, SERIAL_8N1, PIN_RX, PIN_TX);
  if (bootCount == 0) {
    // Restart takes quite some time
    // To skip it, call init() instead of restart()
    Serial.println("Initializing modem...");
    if (!modem.restart()) {
      Serial.println("Failed to restart modem, attempting to continue without restarting");
    }
    bootCount++;
  }
  String name = modem.getModemName();
  DBG("Modem Name:", name);

  String modemInfo = modem.getModemInfo();
  DBG("Modem Info:", modemInfo);


#if TINY_GSM_USE_GPRS && defined TINY_GSM_MODEM_XBEE
  // The XBee must run the gprsConnect function BEFORE waiting for network!
  modem.gprsConnect(apn, gprsUser, gprsPass);
#endif

  SerialMon.print("Waiting for network...");
  if (!modem.waitForNetwork()) {
    SerialMon.println(" fail");
    delay(10000);
    return;
  }
  SerialMon.println(" success");

  if (modem.isNetworkConnected()) {
    SerialMon.println("Network connected");
  }

#if TINY_GSM_USE_GPRS
  // GPRS connection parameters are usually set after network registration
  SerialMon.print(F("Connecting to "));
  SerialMon.print(apn);
  if (!modem.gprsConnect(apn, gprsUser, gprsPass)) {
    SerialMon.println(" fail");
    delay(10000);
    return;
  }
  SerialMon.println(" success");

  if (modem.isGprsConnected()) {
    SerialMon.println("GPRS connected");
  }
#endif

  // MQTT Broker setup
  mqtt.setKeepAlive(600);
  mqtt.setServer(broker, 1883);
  //mqtt.setCallback(mqttCallback);
}

//#############################################################    VOID LOOP    ################################################################
void loop() {
  verificar_conexionRed();

  if (!mqtt.connected()) {
    SerialMon.println("=== MQTT IS NOT CONNECTED ===");
    // Reconnect every 10 seconds
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

void verificar_conexionRed() {
  // Make sure we're still registered on the network
  if (!modem.isNetworkConnected()) {
    SerialMon.println("Network disconnected");
    if (!modem.waitForNetwork(180000L, true)) {
      SerialMon.println(" fail");
      delay(10000);
      return;
    }
    if (modem.isNetworkConnected()) {
      SerialMon.println("Network re-connected");
    }

#if TINY_GSM_USE_GPRS
    // and make sure GPRS/EPS is still connected
    if (!modem.isGprsConnected()) {
      SerialMon.println("GPRS disconnected!");
      SerialMon.print(F("Connecting to "));
      SerialMon.print(apn);
      if (!modem.gprsConnect(apn, gprsUser, gprsPass)) {
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