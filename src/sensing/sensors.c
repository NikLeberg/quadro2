/*
 * File: sensors.c
 * ----------------------------
 * Author: Niklaus Leuenberger
 * Date:   2019-11-28
 * ----------------------------
 * Implementiert die einzelnen Sensoren und erweitert ggf. deren Funktionen
 *    BNO080:     - Beschleunigung
 *                - Rotation
 *                - Magnetisch Nord
 *                > lineare Beschleunigung
 *                > Orientierung
 *                > uvm.
 *    HC-SR04:    - Abstand zum Boden per Ultraschall
 *    BN-220:     - GPS, GLONASS, BeiDou, SBAS, Galileo
 * 
 * Alle Sensoren sind in ihrem eigenen Task implementiert.
 * Neue Sensordaten werden dem Sensor-Task mitgeteilt welcher den absoluten Status des Systems
 * aktualisiert, Sensor-Fusion betreibt und an registrierte Handler weiterleitet.
 *
 * Höhe z:
 *  - lineare Beschleunigung (Worldframe) doppelt integriert über Zeit
 *  - Ultraschall
 *  - GPS Höhe über Meer tariert auf Startposition
 *  - Drucksensor tariert auf Startposition
 */


/** Externe Abhängigkeiten **/

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "esp_log.h"


/** Interne Abhängigkeiten **/

#include "resources.h"
#include "i2c.h"
#include "bno.h"
#include "ultrasonic.h"
#include "gps.h"
#include "sensor_types.h"


/** Variablendeklaration **/

struct sensors_t {
    struct {
        uint8_t dummy;
    };
};
static struct sensors_t sensors;


/** Private Functions **/

/*
 * Function: sensors_task
 * ----------------------------
 * Haupttask. Verwaltet alle Sensorikdaten.
 *
 * void* arg: Dummy für FreeRTOS
 */
void sensors_task(void* arg);


/** Implementierung **/

bool sensors_init(gpio_num_t scl, gpio_num_t sda,                                   // I2C
                  uint8_t bnoAddr, gpio_num_t bnoInterrupt, gpio_num_t bnoReset,    // BNO080
                  gpio_num_t ultTrigger, gpio_num_t ultEcho,                        // Ultraschall
                  gpio_num_t gpsRxPin, gpio_num_t gpsTxPin) {                       // GPS
    // Input-Queue erstellen
    xSensors_input = xQueueCreate(16, sizeof(struct sensors_input_t));
    // I2C initialisieren
    bool ret = false;
    ESP_LOGD("sensors", "I2C init");
    ret = i2c_init(scl, sda);
    ESP_LOGD("sensors", "I2C %s", ret ? "error" : "ok");
    // BNO initialisieren + Reports für Beschleunigung, Orientierung und Druck aktivieren
    ESP_LOGD("sensors", "BNO init");
    ret |= bno_init(bnoAddr, bnoInterrupt, bnoReset);
    ESP_LOGD("sensors", "BNO %s", ret ? "error" : "ok");
    // Ultraschall initialisieren
    ESP_LOGD("sensors", "ULT init");
    ret |= ult_init(ultTrigger, ultEcho);
    ESP_LOGD("sensors", "ULT %s", ret ? "error" : "ok");
    // GPS initialisieren
    ESP_LOGD("sensors", "GPS init");
    ret |= gps_init(gpsRxPin, gpsTxPin);
    ESP_LOGD("sensors", "GPS %s", ret ? "error" : "ok");
    // installiere task
    if (xTaskCreate(&sensors_task, "sensors", 3 * 1024, NULL, xSensors_PRIORITY, &xSensors_handle) != pdTRUE) return true;
    return ret;
}

void sensors_setHome() {
    // bno hat kein Home, vielleicht aber Kalibrieren?
    ult_setHome();
    gps_setHome();
    // Fusion zurücksetzen
    return;
}

/* Haupttask */

void sensors_task(void* arg) {
    // Variablen
    struct sensors_input_t input;
    // Loop
    while (true) {
        if (xQueueReceive(xSensors_input, &input, 5000 / portTICK_PERIOD_MS) == pdTRUE) {
            switch (input.type) {
                case (SENSORS_ACCELERATION): {
                    ESP_LOGI("sensors", "%llu,A,%f,%f,%f", input.timestamp, input.vector.x, input.vector.y, input.vector.z);
                    // sensors_fuseX(input.type, input.vector.x, input.timestamp);
                    // sensors_fuseY(input.type, input.vector.y, input.timestamp);
                    // sensors_fuseZ(input.type, input.vector.z, input.timestamp);
                    break;
                }
                case (SENSORS_ULTRASONIC): {
                    ESP_LOGI("sensors", "%llu,U,%f", input.timestamp, input.distance);
                    // sensors_fuseZ(input.type, input.distance, input.timestamp);
                    break;
                }
                case (SENSORS_ALTIMETER): {
                    ESP_LOGI("sensors", "%llu,B,%f", input.timestamp, input.distance);
                    // sensors_fuseZ(input.type, input.distance, input.timestamp);
                    break;
                }
                case (SENSORS_ORIENTATION): {
                    ESP_LOGI("sensors", "%llu,O,%f,%f,%f,%f", input.timestamp, input.orientation.i, input.orientation.j, input.orientation.k, input.orientation.real);
                    break;
                }
                case (SENSORS_POSITION): {
                    ESP_LOGI("sensors", "%llu,P,%f,%f,%f", input.timestamp, input.vector.x, input.vector.y, input.vector.z);
                    // sensors_fuseX(input.type, input.vector.x, input.timestamp);
                    // sensors_fuseY(input.type, input.vector.y, input.timestamp);
                    // sensors_fuseZ(input.type, input.vector.z, input.timestamp);
                    break;
                }
                case (SENSORS_GROUNDSPEED): {
                    ESP_LOGI("sensors", "%llu,S,%f,%f", input.timestamp, input.vector.x, input.vector.y);
                    // sensors_fuseX(input.type, input.vector.x, input.timestamp);
                    // sensors_fuseY(input.type, input.vector.y, input.timestamp);
                    break;
                }
            }
            // lösche wenn Platz gering wird
            if (uxQueueSpacesAvailable(xSensors_input) <= 1) {
                xQueueReset(xSensors_input);
                ESP_LOGE("sensors", "queue reset!");
            }
        } else {
            ESP_LOGD("sensors", "%llu,online", esp_timer_get_time());
        }
    }
}
