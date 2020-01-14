/*
 * File: sensors.h
 * ----------------------------
 * Author: Niklaus Leuenberger
 * Date:   2019-11-28
 * ----------------------------
 *   Implementiert die einzelnen Sensoren und erweitert ggf. deren Funktionen
 *      BNO080:     - Beschleunigung
 *                  - Rotation
 *                  - Magnetisch Nord
 *                  > lineare Beschleunigung
 *                  > Orientierung
 *                  > uvm.
 *      HC-SR04:    - Abstand zum Boden per Ultraschall
 *      BN-220:     - GPS, GLONASS, BeiDou, SBAS, Galileo
 * 
 *   Alle Sensoren sind in ihrem eigenen Task implementiert.
 *   Neue Sensordaten werden dem Sensor-Task mitgeteilt welcher den absoluten Status des Systems
 *   aktualisiert, ev. Sensor-Fusion betreibt und an registrierte Handler weiterleitet.
 *   BNO Interrupt -> bno-task -> sensor-task -> fly-task
 * 
 *   Höhe:
 *      - lineare Beschleunigung (Worldframe) doppelt integriert über Zeit
 *      - Ultraschall
 *      - GPS Höhe über Meer tariert auf Startposition
 *      - Drucksensor tariert auf Startposition
 */


#pragma once

/** Externe Abhängigkeiten **/
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include <stdio.h>
#include <stdbool.h>
#include "esp_log.h"
//#include "../../lib/yasML/matrices.h"

/** Interne Abhängigkeiten **/
#include "resources.h"
#include "bno.h"
#include "ultrasonic.h"


/** Variablendeklaration **/

// enum sensors_input_type_t {
//     SENSORS_ACCELERATION = SH2_LINEAR_ACCELERATION, // m/s^2
//     SENSORS_ORIENTATION = SH2_ROTATION_VECTOR, // Quaternion
//     SENSORS_PRESSURE = SH2_PRESSURE, // ?
//     SENSORS_ALTIMETER, // umgerechneter Druck Pa -> m
//     SENSORS_ULTRASONIC // m
// };

// struct sensors_input_t {
//     enum sensors_input_type_t type;
//     int64_t timestamp;
//     union {
//         struct { // generischer Vektor
//             float x, y, z;
//         } vector;
//         float distance; // generischer Distanz
//     };
// };

struct sensors_t {
    struct {
        int64_t lastMeasurementTimestamp;
        //Matrix *x, *xP, *F, *P, *R, *z, *K, *Q;
    } altitude;
};
struct sensors_t sensors;


/** Public Functions **/

/*
 * Function: sensors_init
 * ----------------------------
 *   Initialisiert interne Konfiguration und startet Haupttask.
 *
 *   gpio_num_t scl: I2C-Bus Clock-Pin
 *   gpio_num_t sda: I2C-Bus Daten-Pin
 *   uint8_t bnoAddr: BNO I2C Adresse
 *   gpio_num_t bnoInterrupt: BNO Interrupt Pin (Data ready)
 *   gpio_num_t bnoReset: BNO Reset Pin
 *
 *   returns: false bei Erfolg, sonst true 
 */
bool sensors_init(gpio_num_t scl, gpio_num_t sda,
                  uint8_t bnoAddr, gpio_num_t bnoInterrupt, gpio_num_t bnoReset,
                  gpio_num_t ultTrigger, gpio_num_t ultEcho);


/** Private Functions **/

/*
 * Function: sensors_task
 * ----------------------------
 *   Haupttask. Verwaltet alle Sensorik und aktiviert / deaktiviert
 *   Sensoren nach bedarf.
 *
 *   void* arg: Dummy für FreeRTOS
 */
void sensors_task(void* arg);

// ToDo


/** Implementierung **/

bool sensors_init(gpio_num_t scl, gpio_num_t sda,                                   // I2C
                  uint8_t bnoAddr, gpio_num_t bnoInterrupt, gpio_num_t bnoReset,    // BNO080
                  gpio_num_t ultTrigger, gpio_num_t ultEcho) {                      // Ultraschall
    // Input-Queue erstellen
    xSensors_input = xQueueCreate(16, sizeof(struct sensors_input_t));
    // I2C initialisieren
    i2c_init(scl, sda);
    // BNO initialisieren + Reports für Beschleunigung, Orientierung und Druck aktivieren
    if (bno_init(bnoAddr, bnoInterrupt, bnoReset)) return true;
    // Ultraschall initialisieren
    if (ult_init(ultTrigger, ultEcho)) return true;
    // installiere task
    if (xTaskCreate(&sensors_task, "sensors", 2 * 1024, NULL, xSensors_PRIORITY, &xSensors_handle) != pdTRUE) return true;
    return false;
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
                    break;
                }
                case (SENSORS_ULTRASONIC): {
                    ESP_LOGI("sensors", "%llu,U,%f", input.timestamp, input.distance);
                    break;
                }
                case (SENSORS_ALTIMETER): {
                    ESP_LOGI("sensors", "%llu,B,%f", input.timestamp, input.distance);
                    break;
                }
                case (SENSORS_ORIENTATION):
                case (SENSORS_PRESSURE):
                default:
                    break;
            }
        } else {
            ESP_LOGD("sensors", "%llu,online", esp_timer_get_time());
        }
    }
}


// void sensor_fuseAltitude(struct sensors_input_t measurement) {
//     // Variablen
//     Matrix *x = sensors.altitude.x; // Zustand
//     Matrix *F = sensors.altitude.F; // Pysikalisches Modell
//     Matrix *P = sensors.altitude.P; // Wahrscheinlichkeitsverteilung (Magisch!)
//     Matrix *Q = sensors.altitude.Q; // Unsicherheit der Voraussage
//     Matrix *R = sensors.altitude.R; // Sensor-Unsicherheit
//     // Physikmodell um deltaT anpassen
//     float deltaT = (measurement.timestamp - sensors.altitude.lastMeasurementTimestamp) / (1000 * 1000); // ms -> s
//     F->numbers[1][0] = deltaT; // m = m0 + v * deltaT + 0.5 * a * deltaT^2
//     F->numbers[2][0] = 0.5f * deltaT * deltaT;
//     F->numbers[2][1] = deltaT; // v = v0 + a * deltaT
//     // Zustand voraussagen (xHat = F * x)
//     Matrix *xP = multiply(F, x);
//     // Wahrscheinlichkeitsverteilung voraussagen (PHat = F*P*FT+Q)
//     Matrix *multi = multiply(F, P);
//     Matrix *trans = transpose(F);
//     Matrix *PHat = multiply(multi, trans);
//     add(PHat, Q); // + Unsicherheit der Voraussage
//     destroy_matrix(multi);
//     destroy_matrix(trans);
//     // Sensor-Verlässlichkeit verringern um prognostizierte zurückgelegte Distanz (alle Messungen sind älter geworden)
//     float deltaM = xP->numbers[0][0] - x->numbers[0][0];
//     R->numbers[0][0] += deltaM / (0.5f * deltaT * deltaT); // Accelerometer
//     R->numbers[1][1] += deltaM; // Ultraschall
//     R->numbers[2][2] += deltaM; // Barometer
//     // Messung (z) aktualisieren und Verlässlichkeit (R) des Sensors zurücksetzen resp. neu setzen
//     switch (measurement.type) {
//         case (SENSORS_ACCELEROMETER): {
//             sensors.altitude.z->numbers[0][0] = measurement.bno.un.linearAcceleration.z;
//             if (measurement.bno.status < 2) {
//                 R->numbers[0][0] = 0.35f; // 0.35 m/s2
//             }
//             break;
//         }
//         case (SENSORS_ULTRASONIC): {
//             sensors.altitude.z->numbers[0][1] = (measurement.data - sensors.altitude.offsetUltrasonic);
//             R->numbers[1][1] = 0.005f; // 5mm
//             break;
//         }
//         case (SENSORS_PRESSURE): {
//             float h = (228.15f / 0.0065f) * (1.0f - powf(measurement.bno.un.pressure.value / 1013.25f, (1.0f / 5.255f)));
//             sensors.altitude.z->numbers[0][2] = (h - sensors.altitude.offsetBarometer);
//             R->numbers[2][2] = 1.0f; // 1 m
//             break;
//         }
//         //case (SENSORS_GPS): { // ToDo
//         //    break;
//         //}
//     }
//     // Kalman-Gain rechnen K=PHat*HT*(H*PHat*HT+R)^-1
//     // Messung in Status fusionieren
//     // Wahrscheinlichkeitsverteilung korrigieren P = (I-K*H)*PHat
    
//     // verwendete einmalige Matrizen freigeben
//     destroy_matrix(xP);
//     destroy_matrix(PHat);
// }
