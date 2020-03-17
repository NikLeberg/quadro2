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
 *    BN-880Q:    - GPS, GLONASS, BeiDou, SBAS, Galileo
 * 
 * Alle Sensoren sind in ihrem eigenen Task implementiert.
 * Neue Sensordaten werden dem Sensor-Task mitgeteilt welcher den absoluten Status des Systems
 * aktualisiert, Sensor-Fusion betreibt und an registrierte Handler weiterleitet.
 * 
 * Systemstatus: (alle Werte im ENU Koordinatensystem)
 *  - Orientierung
 *  - geschätzte Position
 *  - geschätzte Geschwindigkeit
 *
 * Höhe z:
 *  - lineare z Beschleunigung (Worldframe) doppelt integriert über Zeit
 *  - Ultraschall
 *  - GPS Höhe über Meer tariert auf Startposition
 *  - Drucksensor tariert auf Startposition
 * 
 * Position y:
 *  - lineare y Beschleunigung (Worldframe) doppelt integriert über Zeit
 *  - GPS y-Positionskomponente
 * 
 * Position x:
 *  - lineare x Beschleunigung (Worldframe) doppelt integriert über Zeit
 *  - GPS x-Positionskomponente
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

#include "intercom.h"
#include "resources.h"
#include "i2c.h"
#include "bno.h"
#include "ultrasonic.h"
#include "gps.h"
#include "sensor_types.h"
#include "sensors.h"


/** Variablendeklaration **/

struct sensors_t {
    struct { // Fusion der Z Achse (Altitude)
        eekf_context ekf;
        eekf_mat x, P, z;
        int64_t lastTimestamp;
        eekf_value errorAcceleration, errorUltrasonic, errorBarometer, errorGPS;
        eekf_value limitVelocity;
    } Z;

    struct { // Fusion der Y Achse (Latitude)
        eekf_context ekf;
        eekf_mat x, P, z;
        int64_t lastTimestamp;
        eekf_value errorAcceleration, errorGPS, errorVelocity;
        eekf_value limitVelocity;
    } Y;

    struct { // Fusion der X Achse (Longitude)
        eekf_context ekf;
        eekf_mat x, P, z;
        int64_t lastTimestamp;
        eekf_value errorAcceleration, errorGPS, errorVelocity;
        eekf_value limitVelocity;
    } X;

    sensors_event_t *states[SENSORS_MAX];

    struct {
        float altitude;
        float distance;
        vector_t position;
    } homes;
};
static struct sensors_t sensors;

static command_t sensors_commands[SENSORS_COMMAND_MAX] = {
    COMMAND("setHome"),
    COMMAND("setAltimeterToGPS"),
    COMMAND("resetFusion")
};
static COMMAND_LIST("sensors", sensors_commands, SENSORS_COMMAND_MAX);

static setting_t sensors_settings[SENSORS_SETTING_MAX] = {
    SETTING("zErrAccel",        &sensors.Z.errorAcceleration,   VALUE_TYPE_FLOAT),
    SETTING("zErrUltrasonic",   &sensors.Z.errorUltrasonic,     VALUE_TYPE_FLOAT),
    SETTING("zErrBarometer",    &sensors.Z.errorBarometer,      VALUE_TYPE_FLOAT),
    SETTING("zErrGPS",          &sensors.Z.errorGPS,            VALUE_TYPE_FLOAT),
    SETTING("zLimitVelocity",   &sensors.Z.limitVelocity,       VALUE_TYPE_FLOAT),

    SETTING("yErrAccel",        &sensors.Y.errorAcceleration,   VALUE_TYPE_FLOAT),
    SETTING("yErrGPS",          &sensors.Y.errorGPS,            VALUE_TYPE_FLOAT),
    SETTING("yErrVelocity",     &sensors.Y.errorVelocity,       VALUE_TYPE_FLOAT),
    SETTING("yLimitVelocity",   &sensors.Y.limitVelocity,       VALUE_TYPE_FLOAT),
    
    SETTING("xErrAccel",        &sensors.X.errorAcceleration,   VALUE_TYPE_FLOAT),
    SETTING("xErrGPS",          &sensors.X.errorGPS,            VALUE_TYPE_FLOAT),
    SETTING("xErrVelocity",     &sensors.X.errorVelocity,       VALUE_TYPE_FLOAT),
    SETTING("xLimitVelocity",   &sensors.X.limitVelocity,       VALUE_TYPE_FLOAT)
};
static SETTING_LIST("sensors", sensors_settings, SENSORS_SETTING_MAX);

static pv_t sensors_pvs[SENSORS_PV_MAX] = {
    PV("x", VALUE_TYPE_FLOAT),
    PV("y", VALUE_TYPE_FLOAT),
    PV("z", VALUE_TYPE_FLOAT),
};
static PV_LIST("sensors", sensors_pvs, SENSORS_PV_MAX);


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
static void sensors_processCommand(sensors_command_t command);
static void sensors_processData(sensors_event_t *event);
static void sensors_fuseZ_reset();
static void sensors_fuseZ(sensors_event_type_t type, float z, int64_t timestamp);
eekf_return sensors_fuseZ_transition(eekf_mat* xp, eekf_mat* Jf, eekf_mat const *x, eekf_mat const *u, void* userData);
eekf_return sensors_fuseZ_measurement(eekf_mat* zp, eekf_mat* Jh, eekf_mat const *x, void* userData);
static void sensors_fuseY_reset();
static void sensors_fuseY(sensors_event_type_t type, float y, int64_t timestamp);
eekf_return sensors_fuseY_transition(eekf_mat* xp, eekf_mat* Jf, eekf_mat const *x, eekf_mat const *u, void* userData);
eekf_return sensors_fuseY_measurement(eekf_mat* zp, eekf_mat* Jh, eekf_mat const *x, void* userData);
static void sensors_fuseX_reset();
static void sensors_fuseX(sensors_event_type_t type, float x, int64_t timestamp);
eekf_return sensors_fuseX_transition(eekf_mat* xp, eekf_mat* Jf, eekf_mat const *x, eekf_mat const *u, void* userData);
eekf_return sensors_fuseX_measurement(eekf_mat* zp, eekf_mat* Jh, eekf_mat const *x, void* userData);


/** Implementierung **/

bool sensors_init(gpio_num_t scl, gpio_num_t sda,                                   // I2C
                  uint8_t bnoAddr, gpio_num_t bnoInterrupt, gpio_num_t bnoReset,    // BNO080
                  gpio_num_t ultTrigger, gpio_num_t ultEcho,                        // Ultraschall
                  gpio_num_t gpsRxPin, gpio_num_t gpsTxPin) {                       // GPS
    // Intercom-Queue erstellen
    xSensors = xQueueCreate(16, sizeof(event_t));
    // an Intercom anbinden
    commandRegister(xSensors, sensors_commands);
    settingRegister(xSensors, sensors_settings);
    pvRegister(xSensors, sensors_pvs);
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
    // Kalman Filter Z initialisieren
    EEKF_CALLOC_MATRIX(sensors.Z.x, 2, 1); // 2 States: Position, Geschwindigkeit
    EEKF_CALLOC_MATRIX(sensors.Z.P, 2, 2);
    EEKF_CALLOC_MATRIX(sensors.Z.z, 3, 1); // 3 Messungen: Ultraschall, Barometer, GPS
    sensors_fuseZ_reset();
    eekf_init(&sensors.Z.ekf, &sensors.Z.x, &sensors.Z.P, sensors_fuseZ_transition, sensors_fuseZ_measurement, NULL);
    // Kalman Filter Y initialisieren
    EEKF_CALLOC_MATRIX(sensors.Y.x, 2, 1); // 2 States: Position, Geschwindigkeit
    EEKF_CALLOC_MATRIX(sensors.Y.P, 2, 2);
    EEKF_CALLOC_MATRIX(sensors.Y.z, 2, 1); // 1 Messung: GPS, GPS-Geschwindigkeit
    sensors_fuseY_reset();
    eekf_init(&sensors.Y.ekf, &sensors.Y.x, &sensors.Y.P, sensors_fuseY_transition, sensors_fuseY_measurement, NULL);
    // Kalman Filter X initialisieren
    EEKF_CALLOC_MATRIX(sensors.X.x, 2, 1); // 2 States: Position, Geschwindigkeit
    EEKF_CALLOC_MATRIX(sensors.X.P, 2, 2);
    EEKF_CALLOC_MATRIX(sensors.X.z, 1, 1); // 1 Messung: GPS
    sensors_fuseX_reset();
    eekf_init(&sensors.X.ekf, &sensors.X.x, &sensors.X.P, sensors_fuseX_transition, sensors_fuseX_measurement, NULL);
    // installiere task
    if (xTaskCreate(&sensors_task, "sensors", 3 * 1024, NULL, xSensors_PRIORITY, NULL) != pdTRUE) return true;
    return ret;
}


/* Haupttask */

void sensors_task(void* arg) {
    // Variablen
    event_t event;
    // Loop
    while (true) {
        xQueueReceive(xSensors, &event, portMAX_DELAY);
        switch (event.type) {
            case (EVENT_COMMAND): // Befehl erhalten
                sensors_processCommand((sensors_command_t)event.data);
                break;
            case (EVENT_INTERNAL): // Sensorupdate erhalten
                sensors_processData((sensors_event_t*)event.data);
                break;
            case (EVENT_PV):
            default:
                // ungültig
                break;
        }
        // lösche wenn Platz gering wird
        if (uxQueueSpacesAvailable(xSensors) <= 1) {
            xQueueReset(xSensors);
            ESP_LOGE("sensors", "queue reset!");
        }
    }
}

static void sensors_processCommand(sensors_command_t command) {
    switch (command) {
        case (SENSORS_COMMAND_SET_HOME):
            // Altitude
            if (sensors.states[SENSORS_ALTIMETER]) {
                sensors.homes.altitude = sensors.states[SENSORS_ALTIMETER]->vector.z;
            }
            // Ultraschall
            if (sensors.states[SENSORS_ULTRASONIC]) {
                sensors.homes.distance = sensors.states[SENSORS_ULTRASONIC]->vector.z;
            }
            // Position
            if (sensors.states[SENSORS_POSITION]) {
                sensors.homes.position = sensors.states[SENSORS_POSITION]->vector;
            }
            // break; nach setHome immer auch Fusion zurücksetzen
        case (SENSORS_COMMAND_RESET_FUSION):
            sensors_fuseX_reset();
            sensors_fuseY_reset();
            sensors_fuseZ_reset();
            break;
        case (SENSORS_COMMAND_SET_ALTIMETER_TO_GPS):
            if (sensors.states[SENSORS_POSITION] && sensors.states[SENSORS_ALTIMETER]) {
                sensors.homes.altitude += sensors.states[SENSORS_POSITION]->vector.z - sensors.states[SENSORS_ALTIMETER]->vector.z;
            }
            break;
        default:
            break;
    }
    return;
}

static void sensors_processData(sensors_event_t *event) {
    // Sensorzustand speichern
    sensors.states[event->type] = event;
    // Verarbeiten
    switch (event->type) {
        case (SENSORS_ACCELERATION):
            ESP_LOGI("sensors", "%llu,A,%f,%f,%f,%f", event->timestamp, event->vector.x, event->vector.y, event->vector.z, event->accuracy);
            // sensors_fuseX(event->type, event->vector.x, event->timestamp);
            // sensors_fuseY(event->type, event->vector.y, event->timestamp);
            // sensors_fuseZ(event->type, event->vector.z, event->timestamp);
            break;
        case (SENSORS_ORIENTATION):
            ESP_LOGI("sensors", "%llu,O,%f,%f,%f,%f,%f", event->timestamp, event->orientation.i, event->orientation.j, event->orientation.k, event->orientation.real, event->accuracy);
            break;
        case (SENSORS_ALTIMETER):
            event->vector.z -= sensors.homes.altitude;
            ESP_LOGI("sensors", "%llu,B,%f,%f", event->timestamp, event->vector.z, event->accuracy);
            // sensors_fuseZ(event->type, event->distance, event->timestamp);
            break;
        case (SENSORS_ULTRASONIC):
            event->vector.z -= sensors.homes.distance;
            ESP_LOGI("sensors", "%llu,U,%f", event->timestamp, event->vector.z);
            // sensors_fuseZ(event->type, event->distance, event->timestamp);
            break;
        case (SENSORS_POSITION):
            event->vector.x -= sensors.homes.position.x;
            event->vector.y -= sensors.homes.position.y;
            event->vector.z -= sensors.homes.position.z;
            ESP_LOGI("sensors", "%llu,P,%f,%f,%f,%f", event->timestamp, event->vector.x, event->vector.y, event->vector.z, event->accuracy);
            // sensors_fuseX(event->type, event->vector.x, event->timestamp);
            // sensors_fuseY(event->type, event->vector.y, event->timestamp);
            // sensors_fuseZ(event->type, event->vector.z, event->timestamp);
            break;
        case (SENSORS_GROUNDSPEED):
            ESP_LOGI("sensors", "%llu,S,%f,%f,%f", event->timestamp, event->vector.x, event->vector.y, event->accuracy);
            // sensors_fuseX(event->type, event->vector.x, event->timestamp);
            // sensors_fuseY(event->type, event->vector.y, event->timestamp);
            // sensors_fuseZ(event->type, event->vector.z, event->timestamp);
            break;
        default:
            break;
    }
}


/* Z Altitute Fusion */

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

static void sensors_fuseZ(sensors_event_type_t type, float z, int64_t timestamp) {
    // Voraussagen oder Korrigieren
    if (type == SENSORS_ACCELERATION) { // Voraussagen
        // vergangene Zeit
        if (timestamp <= sensors.Z.lastTimestamp) return; // verspätete Messung
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
                return; // unbekannter Sensortyp
        }
        // Ausführen
        eekf_return ret;
        ret = eekf_lazy_correct(&sensors.Z.ekf, &sensors.Z.z, &R);
        if (ret != eEekfReturnOk) { // Rechenfehler
            ESP_LOGE("sensors", "fuseZ correct error: %u", ret);
        }
    }
    // DEBUG
    ESP_LOGD("sensors", "Fz,%f,%f,Z,%f,%f,%f", *EEKF_MAT_EL(sensors.Z.x, 0, 0), *EEKF_MAT_EL(sensors.Z.x, 1, 0), *EEKF_MAT_EL(sensors.Z.z, 0, 0), *EEKF_MAT_EL(sensors.Z.z, 1, 0), *EEKF_MAT_EL(sensors.Z.z, 2, 0));
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
    if (*predictedVelocity > SENSORS_FUSE_Z_LIMIT_VEL) *predictedVelocity = SENSORS_FUSE_Z_LIMIT_VEL;
    else if (*predictedVelocity < -SENSORS_FUSE_Z_LIMIT_VEL) *predictedVelocity = -SENSORS_FUSE_Z_LIMIT_VEL;
    return eEekfReturnOk;
}

eekf_return sensors_fuseZ_measurement(eekf_mat* zp, eekf_mat* Jh, eekf_mat const *x, void* userData) {
    sensors_event_type_t type = *((sensors_event_type_t*) userData);
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


/* Y Longitude Fusion */

static void sensors_fuseY_reset() {
    // Zustand
    *EEKF_MAT_EL(sensors.Y.x, 0, 0) = 0.0f;
    *EEKF_MAT_EL(sensors.Y.x, 1, 0) = 0.0f;
    // Unsicherheit
    *EEKF_MAT_EL(sensors.Y.P, 0, 0) = 0.0f;
    *EEKF_MAT_EL(sensors.Y.P, 0, 1) = 0.0f;
    *EEKF_MAT_EL(sensors.Y.P, 1, 0) = 0.0f;
    *EEKF_MAT_EL(sensors.Y.P, 1, 1) = 1.0f;
    // Messung
    *EEKF_MAT_EL(sensors.Y.z, 0, 0) = 0.0f;
    // DEBUG
    ESP_LOGV("sensors", "fuseY reset");
}

static void sensors_fuseY(sensors_event_type_t type, float y, int64_t timestamp) {
    // Voraussagen oder Korrigieren
    if (type == SENSORS_ACCELERATION) { // Voraussagen
        // vergangene Zeit
        if (timestamp <= sensors.Y.lastTimestamp) return; // verspätete Messung
        float dt = (timestamp - sensors.Y.lastTimestamp) / 1000.0f / 1000.0f;
        sensors.Y.lastTimestamp = timestamp;
        sensors.Y.ekf.userData = (void*) &dt;
        // Input
        EEKF_DECL_MAT_INIT(u, 1, 1, y);
        // Unsicherheit der Voraussage
        y = fabsf(y) + SENSORS_FUSE_Y_ERROR_ACCELERATION;
        EEKF_DECL_MAT(Q, 2, 2);
        *EEKF_MAT_EL(Q, 0, 0) = 0.25f * y * powf(dt, 4.0f);
        *EEKF_MAT_EL(Q, 0, 1) = 0.5f * y * powf(dt, 3.0f);
        *EEKF_MAT_EL(Q, 1, 0) = 0.5f * y * powf(dt, 3.0f);
        *EEKF_MAT_EL(Q, 1, 1) = y * dt * dt;
        // Ausführen
        eekf_return ret;
        ret = eekf_predict(&sensors.Y.ekf, &u, &Q);
        if (ret != eEekfReturnOk) { // Rechenfehler
            ESP_LOGE("sensors", "fuseY predict error: %u", ret);
        }
    } else { // Korrigieren
        sensors.Y.ekf.userData = (void*) &type;
        // Messunsicherheit
        EEKF_DECL_MAT_INIT(R, 2, 2, 0);
        switch (type) {
            case (SENSORS_POSITION):
                *EEKF_MAT_EL(sensors.Y.z, 0, 0) = y;
                *EEKF_MAT_EL(R, 0, 0) = SENSORS_FUSE_Y_ERROR_GPS;
            case (SENSORS_GROUNDSPEED):
                *EEKF_MAT_EL(sensors.Y.z, 1, 0) = y;
                *EEKF_MAT_EL(R, 1, 1) = SENSORS_FUSE_Y_ERROR_VELOCITY;
            default:
                return; // unbekannter Sensortyp
        }
        // Ausführen
        eekf_return ret;
        ret = eekf_lazy_correct(&sensors.Y.ekf, &sensors.Y.z, &R);
        if (ret != eEekfReturnOk) { // Rechenfehler
            ESP_LOGE("sensors", "fuseY correct error: %u", ret);
        }
    }
    // DEBUG
    ESP_LOGD("sensors", "Fy,%f,%f,Z,%f,%f", *EEKF_MAT_EL(sensors.Y.x, 0, 0), *EEKF_MAT_EL(sensors.Y.x, 1, 0), *EEKF_MAT_EL(sensors.Y.z, 0, 0), *EEKF_MAT_EL(sensors.Y.z, 1, 0));
}

eekf_return sensors_fuseY_transition(eekf_mat* xp, eekf_mat* Jf, eekf_mat const *x,
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
    if (*predictedVelocity > SENSORS_FUSE_Y_LIMIT_VEL) *predictedVelocity = SENSORS_FUSE_Y_LIMIT_VEL;
    else if (*predictedVelocity < -SENSORS_FUSE_Y_LIMIT_VEL) *predictedVelocity = -SENSORS_FUSE_Y_LIMIT_VEL;
    return eEekfReturnOk;
}

eekf_return sensors_fuseY_measurement(eekf_mat* zp, eekf_mat* Jh, eekf_mat const *x, void* userData) {
    sensors_event_type_t type = *((sensors_event_type_t*) userData);
    // Messmodell an Messung anpassen
    memset(Jh->elements, 0.0f, sizeof(eekf_value) * Jh->rows * Jh->cols);
    switch (type) {
        case (SENSORS_POSITION):
            *EEKF_MAT_EL(*Jh, 0, 0) = 1.0f;
            break;
        case (SENSORS_GROUNDSPEED):
            *EEKF_MAT_EL(*Jh, 1, 1) = 1.0f;
        default:
            return eEekfReturnParameterError;
    }
    // zp = H * x
    if (NULL == eekf_mat_mul(zp, Jh, x)) return eEekfReturnComputationFailed;
	return eEekfReturnOk;
}


/* X Longitude Fusion */

static void sensors_fuseX_reset() {
    // Zustand
    *EEKF_MAT_EL(sensors.X.x, 0, 0) = 0.0f;
    *EEKF_MAT_EL(sensors.X.x, 1, 0) = 0.0f;
    // Unsicherheit
    *EEKF_MAT_EL(sensors.X.P, 0, 0) = 0.0f;
    *EEKF_MAT_EL(sensors.X.P, 0, 1) = 0.0f;
    *EEKF_MAT_EL(sensors.X.P, 1, 0) = 0.0f;
    *EEKF_MAT_EL(sensors.X.P, 1, 1) = 1.0f;
    // Messung
    *EEKF_MAT_EL(sensors.X.z, 0, 0) = 0.0f;
    // DEBUG
    ESP_LOGV("sensors", "fuseX reset");
}

static void sensors_fuseX(sensors_event_type_t type, float x, int64_t timestamp) {
    // Voraussagen oder Korrigieren
    if (type == SENSORS_ACCELERATION) { // Voraussagen
        // vergangene Zeit
        if (timestamp <= sensors.X.lastTimestamp) return; // verspätete Messung
        float dt = (timestamp - sensors.X.lastTimestamp) / 1000.0f / 1000.0f;
        sensors.X.lastTimestamp = timestamp;
        sensors.X.ekf.userData = (void*) &dt;
        // Input
        EEKF_DECL_MAT_INIT(u, 1, 1, x);
        // Unsicherheit der Voraussage
        x = fabsf(x) + SENSORS_FUSE_X_ERROR_ACCELERATION;
        EEKF_DECL_MAT(Q, 2, 2);
        *EEKF_MAT_EL(Q, 0, 0) = 0.25f * x * powf(dt, 4.0f);
        *EEKF_MAT_EL(Q, 0, 1) = 0.5f * x * powf(dt, 3.0f);
        *EEKF_MAT_EL(Q, 1, 0) = 0.5f * x * powf(dt, 3.0f);
        *EEKF_MAT_EL(Q, 1, 1) = x * dt * dt;
        // Ausführen
        eekf_return ret;
        ret = eekf_predict(&sensors.X.ekf, &u, &Q);
        if (ret != eEekfReturnOk) { // Rechenfehler
            ESP_LOGE("sensors", "fuseX predict error: %u", ret);
        }
    } else { // Korrigieren
        sensors.X.ekf.userData = (void*) &type;
        // Mssunsicherheit
        EEKF_DECL_MAT_INIT(R, 1, 1, 0);
        switch (type) {
            case (SENSORS_POSITION):
                *EEKF_MAT_EL(sensors.X.z, 0, 0) = x;
                *EEKF_MAT_EL(R, 0, 0) = SENSORS_FUSE_X_ERROR_GPS;
            case (SENSORS_GROUNDSPEED): // ToDo?
            default:
                return; // unbekannter Sensortyp
        }
        // Ausführen
        eekf_return ret;
        ret = eekf_lazy_correct(&sensors.X.ekf, &sensors.X.z, &R);
        if (ret != eEekfReturnOk) { // Rechenfehler
            ESP_LOGE("sensors", "fuseX correct error: %u", ret);
        }
    }
    // DEBUG
    ESP_LOGD("sensors", "Fx,%f,%f,Z,%f", *EEKF_MAT_EL(sensors.X.x, 0, 0), *EEKF_MAT_EL(sensors.X.x, 1, 0), *EEKF_MAT_EL(sensors.X.z, 0, 0));
}

eekf_return sensors_fuseX_transition(eekf_mat* xp, eekf_mat* Jf, eekf_mat const *x,
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
    if (*predictedVelocity > SENSORS_FUSE_X_LIMIT_VEL) *predictedVelocity = SENSORS_FUSE_X_LIMIT_VEL;
    else if (*predictedVelocity < -SENSORS_FUSE_X_LIMIT_VEL) *predictedVelocity = -SENSORS_FUSE_X_LIMIT_VEL;
    return eEekfReturnOk;
}

eekf_return sensors_fuseX_measurement(eekf_mat* zp, eekf_mat* Jh, eekf_mat const *x, void* userData) {
    sensors_event_type_t type = *((sensors_event_type_t*) userData);
    // Messmodell an Messung anpassen
    memset(Jh->elements, 0.0f, sizeof(eekf_value) * Jh->rows * Jh->cols);
    switch (type) {
        case (SENSORS_POSITION):
            *EEKF_MAT_EL(*Jh, 0, 0) = 1.0f;
            break;
        // case (SENSORS_GROUNDSPEED):
        //     *EEKF_MAT_EL(*Jh, 0, 1) = 1.0f;
        default:
            return eEekfReturnParameterError;
    }
    // zp = H * x
    if (NULL == eekf_mat_mul(zp, Jh, x)) return eEekfReturnComputationFailed;
	return eEekfReturnOk;
}
