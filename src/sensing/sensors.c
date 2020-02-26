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
#include <math.h>
#include <string.h>
#include "eekf.h"


/** Interne Abhängigkeiten **/

#include "resources.h"
#include "i2c.h"
#include "bno.h"
#include "ultrasonic.h"
#include "gps.h"
#include "sensor_types.h"
#include "sensors.h"


/** Variablendeklaration **/

struct sensors_t {
    int64_t timeouts[SENSORS_MAX];

    struct { // Fusion der Z Achse (Altitude)
        eekf_context ekf;
        eekf_mat x, P, z;
        int64_t lastTimestamp;
    } Z;
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

// ToDo
static void sensors_fuseZ(enum sensors_input_type_t type, float z, int64_t timestamp);
eekf_return sensors_fuseZ_transition(eekf_mat* xp, eekf_mat* Jf, eekf_mat const *x, eekf_mat const *u, void* userData);
eekf_return sensors_fuseZ_measurement(eekf_mat* zp, eekf_mat* Jh, eekf_mat const *x, void* userData);
static void sensors_fuseZ_reset();


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
    // Kalman Filter initialisieren
    EEKF_CALLOC_MATRIX(sensors.Z.x, 2, 1);
    EEKF_CALLOC_MATRIX(sensors.Z.P, 2, 2);
    EEKF_CALLOC_MATRIX(sensors.Z.z, 3, 1);
    sensors_fuseZ_reset();
    eekf_init(&sensors.Z.ekf, &sensors.Z.x, &sensors.Z.P, sensors_fuseZ_transition, sensors_fuseZ_measurement, NULL);
    // installiere task
    uint32_t otherCore = (xPortGetCoreID() == 0) ? 1 : 0;
    if (xTaskCreatePinnedToCore(&sensors_task, "sensors", 3 * 1024, NULL, xSensors_PRIORITY, &xSensors_handle, otherCore) != pdTRUE) return true;
    return ret;
}

void sensors_setHome() {
    bno_setHome(); // nur Barometer
    ult_setHome();
    gps_setHome();
    // Fusion zurücksetzen
    sensors_fuseZ_reset();
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
                    // ESP_LOGI("sensors", "%llu,A,%f,%f,%f,%f", input.timestamp, input.vector.x, input.vector.y, input.vector.z, input.accuracy);
                    // sensors_fuseX(input.type, input.vector.x, input.timestamp);
                    // sensors_fuseY(input.type, input.vector.y, input.timestamp);
                    sensors_fuseZ(input.type, input.vector.z, input.timestamp);
                    break;
                }
                case (SENSORS_ORIENTATION): {
                    // ESP_LOGI("sensors", "%llu,O,%f,%f,%f,%f,%f", input.timestamp, input.orientation.i, input.orientation.j, input.orientation.k, input.orientation.real, input.accuracy);
                    break;
                }
                case (SENSORS_ALTIMETER): {
                    // ESP_LOGI("sensors", "%llu,B,%f,%f", input.timestamp, input.distance, input.accuracy);
                    sensors_fuseZ(input.type, input.distance, input.timestamp);
                    break;
                }
                case (SENSORS_ULTRASONIC): {
                    // ESP_LOGI("sensors", "%llu,U,%f", input.timestamp, input.distance);
                    sensors_fuseZ(input.type, input.distance, input.timestamp);
                    break;
                }
                case (SENSORS_POSITION): {
                    ESP_LOGI("sensors", "%llu,P,%f,%f,%f,%f", input.timestamp, input.vector.x, input.vector.y, input.vector.z, input.accuracy);
                    // sensors_fuseX(input.type, input.vector.x, input.timestamp);
                    // sensors_fuseY(input.type, input.vector.y, input.timestamp);
                    sensors_fuseZ(input.type, input.vector.z, input.timestamp);
                    break;
                }
                case (SENSORS_GROUNDSPEED): {
                    // ESP_LOGI("sensors", "%llu,S,%f,%f,%f", input.timestamp, input.vector.x, input.vector.y, input.accuracy);
                    // sensors_fuseX(input.type, input.vector.x, input.timestamp);
                    // sensors_fuseY(input.type, input.vector.y, input.timestamp);
                    break;
                }
                default:
                    continue;
            }
            // lösche wenn Platz gering wird
            if (uxQueueSpacesAvailable(xSensors_input) <= 1) {
                xQueueReset(xSensors_input);
                ESP_LOGE("sensors", "queue reset!");
            }
            // Timeouterkennung
            sensors.timeouts[input.type] = input.timestamp;
            input.timestamp = input.timestamp - (SENSORS_TIMEOUT_MS * 1000);
            for (uint8_t i = 0; i < SENSORS_POSITION; ++i) { // ignoriere vorerst GPS
                if (sensors.timeouts[i] < input.timestamp) {
                    ESP_LOGE("sensors", "timeout of sensor %u", i);
                }
            }
        } else {
            ESP_LOGD("sensors", "%llu,online", esp_timer_get_time());
        }
    }
}

static void sensors_fuseZ_reset() {
    // Zustand
    *EEKF_MAT_EL(sensors.Z.x, 0, 0) = 0.0f;
    *EEKF_MAT_EL(sensors.Z.x, 1, 0) = 0.0f;
    // Unsicherheit
    *EEKF_MAT_EL(sensors.Z.P, 0, 0) = 0.0f;
    *EEKF_MAT_EL(sensors.Z.P, 0, 1) = 0.0f;
    *EEKF_MAT_EL(sensors.Z.P, 1, 0) = 0.0f;
    *EEKF_MAT_EL(sensors.Z.P, 1, 1) = 1.0f;
    // Messung
    *EEKF_MAT_EL(sensors.Z.z, 0, 0) = 0.0f;
    *EEKF_MAT_EL(sensors.Z.z, 1, 0) = 0.0f;
    *EEKF_MAT_EL(sensors.Z.z, 2, 0) = 0.0f;
    // DEBUG
    ESP_LOGV("sensors", "fuseZ reset");
}

static void sensors_fuseZ(enum sensors_input_type_t type, float z, int64_t timestamp) {
    // Voraussagen oder Korrigieren
    if (type == SENSORS_ACCELERATION) { // Voraussagen
        // vergangene Zeit
        if (timestamp < sensors.Z.lastTimestamp) return; // verspätete Messung
        float dt = (timestamp - sensors.Z.lastTimestamp) / 1000.0f / 1000.0f;
        sensors.Z.lastTimestamp = timestamp;
        sensors.Z.ekf.userData = (void*) &dt;
        // Input
        EEKF_DECL_MAT_INIT(u, 1, 1, z);
        // Unsicherheit der Voraussage
        z = fabsf(z) + SENSORS_FUSE_Z_ERROR_ACCELERATION;
        EEKF_DECL_MAT(Q, 2, 2);
        *EEKF_MAT_EL(Q, 0, 0) = 0.25f * z * powf(dt, 4.0f);
        *EEKF_MAT_EL(Q, 0, 1) = 0.5f * z * powf(dt, 3.0f);
        *EEKF_MAT_EL(Q, 1, 0) = 0.5f * z * powf(dt, 3.0f);
        *EEKF_MAT_EL(Q, 1, 1) = z * dt * dt;
        // Ausführen
        eekf_return ret;
        ret = eekf_predict(&sensors.Z.ekf, &u, &Q);
        if (ret != eEekfReturnOk) { // Rechenfehler
            ESP_LOGE("sensors", "fuseZ predict error: %u", ret);
        }
    } else { // Korrigieren
        sensors.Z.ekf.userData = (void*) &type;
        // Mssunsicherheit
        EEKF_DECL_MAT_INIT(R, 3, 3, 0);
        switch (type) {
            case (SENSORS_ULTRASONIC):
                *EEKF_MAT_EL(sensors.Z.z, 0, 0) = z;
                *EEKF_MAT_EL(R, 0, 0) = SENSORS_FUSE_Z_ERROR_ULTRASONIC;
                break;
            case (SENSORS_ALTIMETER):
                *EEKF_MAT_EL(sensors.Z.z, 1, 0) = z;
                *EEKF_MAT_EL(R, 1, 1) = SENSORS_FUSE_Z_ERROR_BAROMETER;
                break;
            case (SENSORS_POSITION):
                *EEKF_MAT_EL(sensors.Z.z, 2, 0) = z;
                *EEKF_MAT_EL(R, 2, 2) = SENSORS_FUSE_Z_ERROR_GPS;
            default:
                break;
        }
        // Ausführen
        eekf_return ret;
        ret = eekf_correct(&sensors.Z.ekf, &sensors.Z.z, &R);
        if (ret != eEekfReturnOk) { // Rechenfehler
            ESP_LOGE("sensors", "fuseZ correct error: %u", ret);
        }
    }
    // DEBUG
    ESP_LOGD("sensors", "0,F,%f", *EEKF_MAT_EL(sensors.Z.x, 0, 0));
}

eekf_return sensors_fuseZ_transition(eekf_mat* xp, eekf_mat* Jf, eekf_mat const *x,
                                     eekf_mat const *u, void* userData) {
    float dt = *((float*) userData);
    // Physikmodell an dt anpassen
    *EEKF_MAT_EL(*Jf, 0, 0) = 1.0f;
    *EEKF_MAT_EL(*Jf, 0, 1) = dt;
    *EEKF_MAT_EL(*Jf, 1, 0) = 0.0f;
    *EEKF_MAT_EL(*Jf, 1, 1) = 1.0f;
    // gemäss Modell vorausrechnen
    // xp = F * x
    if (NULL == eekf_mat_mul(xp, Jf, x)) return eEekfReturnComputationFailed;
    // Beschleunigung als Input dazurechnen
    EEKF_DECL_MAT_INIT(G, 2, 1, 0.5f * dt * dt, dt);
    EEKF_DECL_MAT_INIT(gu, 2, 1, 0);
    // xp += G * u
    if (NULL == eekf_mat_add(xp, xp, eekf_mat_mul(&gu, &G, u))) return eEekfReturnComputationFailed;
    // Limits einhalten
    float *predictedVelocity = EEKF_MAT_EL(*xp, 1, 0);
    if (*predictedVelocity > SENSORS_FUSE_Z_X_VEL_LIMIT) *predictedVelocity = SENSORS_FUSE_Z_X_VEL_LIMIT;
    else if (*predictedVelocity < -SENSORS_FUSE_Z_X_VEL_LIMIT) *predictedVelocity = -SENSORS_FUSE_Z_X_VEL_LIMIT;
    return eEekfReturnOk;
}

eekf_return sensors_fuseZ_measurement(eekf_mat* zp, eekf_mat* Jh, eekf_mat const *x, void* userData) {
    enum sensors_input_type_t type = *((enum sensors_input_type_t*) userData);
    // Messmodell an Messung anpassen
    memset(Jh->elements, 0.0f, sizeof(eekf_value) * Jh->rows * Jh->cols);
    switch (type) {
        case (SENSORS_ULTRASONIC):
            *EEKF_MAT_EL(*Jh, 0, 0) = 1.0f;
            break;
        case (SENSORS_ALTIMETER):
            *EEKF_MAT_EL(*Jh, 1, 0) = 1.0f;
            break;
        case (SENSORS_POSITION):
            *EEKF_MAT_EL(*Jh, 2, 0) = 1.0f;
            break;
        default:
            return eEekfReturnParameterError;
    }
    // zp = H * x
    if (NULL == eekf_mat_mul(zp, Jh, x)) return eEekfReturnComputationFailed;
	return eEekfReturnOk;
}
