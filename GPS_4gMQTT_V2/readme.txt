# GPS 4G MQTT Transmisor de Datos

Proyecto IoT para transmitir datos GPS, estado de la batería y lecturas RFID mediante un módulo SIM7600 y el protocolo MQTT. Diseñado para dispositivos embebidos con soporte FreeRTOS.

## 🌟 Características Principales
- **📍 Datos GPS en tiempo real**: Latitud, longitud, velocidad, precisión (HDOP), y hora UTC.
- **📶 Conectividad 4G/LTE**: Uso del módulo SIM7600 para comunicación GPRS.
- **🔋 Monitoreo de Batería**: Voltaje reportado en cada ciclo.
- **🏷️ Lectura RFID Simulada**: Actualización periódica de un RFID predeterminado (`0000000000` → `1234567890`).
- **🔄 Multitarea**: Tareas concurrentes para GPS, RFID y MQTT con FreeRTOS.
- **📦 Payload JSON Dinámico**: Generación eficiente de mensajes MQTT.

---

## 🛠️ Hardware Requerido
- Placa microcontroladora (Ej: ESP32, Arduino MKR).
- Módulo SIM7600 (4G/LTE + GPS integrado).
- Antena GPS activa y antena 4G.
- Tarjeta SIM con plan de datos.
- Fuente de alimentación (Batería LiPo recomendada).

---

## 📦 Dependencias
- [TinyGSM](https://github.com/vshymanskyy/TinyGSM): Biblioteca para módems GSM.
- [PubSubClient](https://github.com/knolleary/pubsubclient): Cliente MQTT.
- FreeRTOS: Para manejo de tareas concurrentes (incluido en el IDE de Arduino para ESP32).

---

## 🔧 Configuración
1. **Clonar el repositorio**:
   ```bash
   git clone https://github.com/tu-usuario/gps-4g-mqtt.git


## 📌 Cambios Recientes (Últimas Modificaciones)

### **1. Manejo Mejorado de RFID**
- **Nuevo Sistema de Lectura Simulada**:
  - Se implementó una tarea FreeRTOS (`taskRFIDRead`) que actualiza el RFID predeterminado (`0000000000`) a `1234567890` cada 10 segundos.
  - Uso de semáforos (`xSemaphore`) para acceso seguro a la variable compartida `lastRFID`.
  ```cpp
  // Tarea RFID (FreeRTOS)
  void taskRFIDRead(void* parameter) {
    if (xSemaphoreTake(xSemaphore, portMAX_DELAY) == pdTRUE) {
      snprintf(lastRFID, sizeof(lastRFID), "%s", "1234567890"); // Actualización segura
      xSemaphoreGive(xSemaphore);
    }
  }