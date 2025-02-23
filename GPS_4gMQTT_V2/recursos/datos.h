#define TINY_GSM_USE_GPRS true

#define PIN_TX 27
#define PIN_RX 26
#define UART_BAUD 115200

#define PWR_PIN 4
#define LED_data_sent 0
#define PIN_VEL_status 12
#define POWER_PIN 25

char lastRFID[11] = "0000000000"; // Inicializado con 10 ceros + '\0'

struct Configuracion {
  const char* apn;
  const char* mqtt_username;
  const char* mqtt_password;
  const char* broker;
  const char* GPSTopic;
  //const char* AlertTopic;
  int mqttKeepAlive;
  const char* IDclient;
};


// Definimos la configuración como una constante en Flash (evita uso de RAM)
const Configuracion config PROGMEM = {
  "internet.comcel.com.co",                    // APN
  "test",                                      // MQTT Username
  "CloudTech*",                                // MQTT Password
  "18.212.130.131",                            // MQTT Broker
  "gps/app/testliceo/puntos/868020034072685",  // GPS Topic
  //"gps/app/testliceo/alertas",                 // Alert Topic
  60,                                           // Keep Alive
  modem.getIMEI().c_str()
};

//  Struct para encapsular todos los datos relacionados
typedef struct {
    float lat;
    float lon;
    float speed;
    float hdop;
    float bearing;
    int hour;
    int minute;
    float voltage;
    int signal;
    char RFID[11]; 
} GpsData;

//  Struct para manejo de memoria dinámica del mensaje
typedef struct {
    char* buffer;
    size_t size;
} JsonPayload;
