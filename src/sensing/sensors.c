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
 * Die Sensortask verarbeiten ihre Datan so weit wie möglich, wenn Daten anderer Sensoren gebraucht
 * würden, wird der "Rohwert" übergeben.
 * Neue Sensordaten werden dem Sensor-Task mitgeteilt welcher:
 *  - Homepunkte anwendet
 *  - ggf. Rohwerte mit anderen Sensoren verarbeitet
 *  - Sensorfusion betreibt
 *  - den globalen Status aktualisiert
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


/** Interne Abhängigkeiten **/

#include "intercom.h"
#include "resources.h"
#include "i2c.h"
#include "bno.h"
#include "flow.h"
#include "gps.h"
#include "ina.h"
#include "eekf.h"
#include "sensor_types.h"
#include "sensors.h"


/** Variablendeklaration **/

struct sensors_t {
    uint32_t timeout; // in us
    uint32_t timedOut; // Bitfeld der inaktiven Sensoren, Bits gem. sensors_event_type_t

    struct {
        uint32_t fast;   // schnelle Sensorik: Orientierung
        uint32_t medium; // mittlere Sensorik: Beschleunigung, Ultraschall
        uint32_t slow;   // langsame Sensorik: GPS, Barometer
    } rate;

    struct { // Fusion der Z Achse (Altitude)
        eekf_context ekf;
        eekf_mat x, P, z;
        int64_t lastTimestamp;
        eekf_value errorAcceleration, errorLidar, errorBarometer, errorGPS;
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

    sensors_event_t *rawData[SENSORS_MAX];

    struct { // verarbeitete Sensordaten:  Struktur     Elemente        Einheit     Quelle
        sensors_event_t orientation;    // orientation  i j k real      Quaternion  bno
        sensors_event_t euler;          // vector       pitch roll yaw  rad         bno
        sensors_event_t rotation;       // vector       x y z           rad/s       bno
        sensors_event_t acceleration;   // vector       x y z           m/s^2       bno
        sensors_event_t altitude;       // value                        m           bme
        sensors_event_t distance;       // value                        m           lidar
        sensors_event_t flow;           // vector       x y             m/s         flow
        sensors_event_t volt;           // value                        V           ina
        sensors_event_t coordinates;    // vector       x y z           m           gps
        sensors_event_t speed;          // vector       x y z           m/s         gps
        // Sensorfusion
        sensors_event_t velocity;       // vector       x y z           m/s         fusion
        sensors_event_t position;       // vector       x y z           m           fusion
        // Semaphor
        SemaphoreHandle_t lock;
    } data;

    struct {
        float altitude;
        float distance;
        vector_t position;
    } homes;

    struct {
        float warning;
        float low;
    } voltage;

    struct {
        float scaleX;
        float scaleY;
    } flow;
};
static struct sensors_t sensors;

static command_t sensors_commands[SENSORS_COMMAND_MAX] = {
    COMMAND("setHome"),
    COMMAND("setAltimeterToGPS"),
    COMMAND("resetFusion"),
    COMMAND("resetQueue"),
    COMMAND("updateRate")
};
static COMMAND_LIST("sensors", sensors_commands, SENSORS_COMMAND_MAX);

static setting_t sensors_settings[SENSORS_SETTING_MAX] = {
    SETTING("timeout",          &sensors.timeout,               VALUE_TYPE_UINT),
    SETTING("rateFast",         &sensors.rate.fast,             VALUE_TYPE_UINT),
    SETTING("rateMedium",       &sensors.rate.medium,           VALUE_TYPE_UINT),
    SETTING("rateSlow",         &sensors.rate.slow,             VALUE_TYPE_UINT),

    SETTING("zErrAccel",        &sensors.Z.errorAcceleration,   VALUE_TYPE_FLOAT),
    SETTING("zErrLidar",        &sensors.Z.errorLidar,          VALUE_TYPE_FLOAT),
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
    SETTING("xLimitVelocity",   &sensors.X.limitVelocity,       VALUE_TYPE_FLOAT),

    SETTING("voltWarning",      &sensors.voltage.warning,       VALUE_TYPE_FLOAT),
    SETTING("voltLow",          &sensors.voltage.low,           VALUE_TYPE_FLOAT),

    SETTING("flowXScale",       &sensors.flow.scaleX,           VALUE_TYPE_FLOAT),
    SETTING("flowYScale",       &sensors.flow.scaleY,           VALUE_TYPE_FLOAT)
};
static SETTING_LIST("sensors", sensors_settings, SENSORS_SETTING_MAX);

static pv_t sensors_pvs[SENSORS_PV_MAX] = {
    PV("timeout", VALUE_TYPE_UINT),
    PV("orientation", VALUE_TYPE_NONE),
    PV("x", VALUE_TYPE_FLOAT),
    PV("vx", VALUE_TYPE_FLOAT),
    PV("y", VALUE_TYPE_FLOAT),
    PV("vy", VALUE_TYPE_FLOAT),
    PV("z", VALUE_TYPE_FLOAT),
    PV("vz", VALUE_TYPE_FLOAT),
    PV("volt", VALUE_TYPE_FLOAT),
    PV("voltWarning", VALUE_TYPE_NONE),
    PV("voltLow", VALUE_TYPE_FLOAT),
    PV("flowX", VALUE_TYPE_FLOAT),
    PV("flowY", VALUE_TYPE_FLOAT),
    PV("gyroX", VALUE_TYPE_FLOAT),
    PV("gyroY", VALUE_TYPE_FLOAT),
    PV("gyroZ", VALUE_TYPE_FLOAT)
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
static void sensors_detectTimeout(int64_t timestamp);
static inline void sensors_resetTimeout(sensors_event_type_t sensor);
static inline void sensors_setTimeout(sensors_event_type_t sensor);
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
                  uint8_t bnoAddress, gpio_num_t bnoInterrupt, gpio_num_t bnoReset, // BNO080
                  gpio_num_t flowRxPin,                                             // Optischer Fluss & Lidar
                  gpio_num_t gpsRxPin, gpio_num_t gpsTxPin,                         // GPS
                  uint8_t inaAddress) {                                             // INA219
    // Intercom-Queue erstellen
    xSensors = xQueueCreate(16, sizeof(event_t));
    // an Intercom anbinden
    commandRegister(xSensors, sensors_commands);
    settingRegister(xSensors, sensors_settings);
    pvRegister(xSensors, sensors_pvs);
    // Datenzugriffsschutz
    sensors.data.lock = xSemaphoreCreateMutex();
    xSemaphoreGive(sensors.data.lock);
    // I2C initialisieren
    bool ret = false;
    ESP_LOGD("sensors", "I2C init");
    ret = i2c_init(scl, sda);
    ESP_LOGD("sensors", "I2C %s", ret ? "error" : "ok");
    // BNO initialisieren + Reports für Beschleunigung, Orientierung, Druck und Rotation aktivieren
    ESP_LOGD("sensors", "BNO init");
    ret = bno_init(bnoAddress, bnoInterrupt, bnoReset, sensors.rate.fast, sensors.rate.medium, sensors.rate.slow, sensors.rate.medium);
    ESP_LOGD("sensors", "BNO %s", ret ? "error" : "ok");
    // Optischer Fluss initialisieren
    ESP_LOGD("sensors", "Flow init");
    ret = flow_init(flowRxPin);
    ESP_LOGD("sensors", "Flow %s", ret ? "error" : "ok");
    // GPS initialisieren
    ESP_LOGD("sensors", "GPS init");
    ret = gps_init(gpsRxPin, gpsTxPin, sensors.rate.slow);
    ESP_LOGD("sensors", "GPS %s", ret ? "error" : "ok");
    // Spannungssensor INA initialisieren
    ESP_LOGD("sensors", "INA init");
    ret = ina_init(inaAddress, &sensors.rate.slow);
    ESP_LOGD("sensors", "INA %s", ret ? "error" : "ok");
    // Kalman Filter Z initialisieren
    EEKF_CALLOC_MATRIX(sensors.Z.x, 2, 1); // 2 States: Position, Geschwindigkeit
    EEKF_CALLOC_MATRIX(sensors.Z.P, 2, 2);
    EEKF_CALLOC_MATRIX(sensors.Z.z, 3, 1); // 3 Messungen: Ultraschall, Lidar, GPS
    sensors_fuseZ_reset();
    eekf_init(&sensors.Z.ekf, &sensors.Z.x, &sensors.Z.P, sensors_fuseZ_transition, sensors_fuseZ_measurement, NULL);
    // Kalman Filter Y initialisieren
    EEKF_CALLOC_MATRIX(sensors.Y.x, 2, 1); // 2 States: Position, Geschwindigkeit
    EEKF_CALLOC_MATRIX(sensors.Y.P, 2, 2);
    EEKF_CALLOC_MATRIX(sensors.Y.z, 2, 1); // 2 Messungen: GPS, GPS-Geschwindigkeit
    sensors_fuseY_reset();
    eekf_init(&sensors.Y.ekf, &sensors.Y.x, &sensors.Y.P, sensors_fuseY_transition, sensors_fuseY_measurement, NULL);
    // Kalman Filter X initialisieren
    EEKF_CALLOC_MATRIX(sensors.X.x, 2, 1); // 2 States: Position, Geschwindigkeit
    EEKF_CALLOC_MATRIX(sensors.X.P, 2, 2);
    EEKF_CALLOC_MATRIX(sensors.X.z, 2, 1); // 2 Messungen: GPS, GPS-Geschwindigkeit
    sensors_fuseX_reset();
    eekf_init(&sensors.X.ekf, &sensors.X.x, &sensors.X.P, sensors_fuseX_transition, sensors_fuseX_measurement, NULL);
    // installiere task
    if (xTaskCreate(&sensors_task, "sensors", 8 * 1024, NULL, xSensors_PRIORITY, NULL) != pdTRUE) return true;
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
            ESP_LOGE("sensors", "queue reset! tLag %llu", esp_timer_get_time() - ((sensors_event_t*)event.data)->timestamp);
        }
    }
}

static void sensors_processCommand(sensors_command_t command) {
    switch (command) {
        case (SENSORS_COMMAND_SET_HOME):
            // Altitude
            sensors.homes.altitude = sensors.data.altitude.value;
            sensors.data.altitude.value = 0.0f;
            // Lidar
            sensors.homes.distance = sensors.data.distance.value;
            sensors.data.distance.value = 0.0f;
            // Position
            sensors.homes.position = sensors.data.coordinates.vector;
            sensors.data.coordinates.vector.x = 0.0f;
            sensors.data.coordinates.vector.y = 0.0f;
            sensors.data.coordinates.vector.z = 0.0f;
            // break; nach setHome immer auch Fusion zurücksetzen
        case (SENSORS_COMMAND_RESET_FUSION):
            sensors_fuseX_reset();
            sensors_fuseY_reset();
            sensors_fuseZ_reset();
            break;
        case (SENSORS_COMMAND_SET_ALTIMETER_TO_GPS):
            sensors.homes.altitude += sensors.data.coordinates.vector.z - sensors.data.altitude.value;
            sensors.data.altitude.value = sensors.data.coordinates.vector.z;
            break;
        case (SENSORS_COMMAND_RESET_QUEUE):
            xQueueReset(xSensors);
            break;
        case (SENSORS_COMMAND_UPDATE_RATE):
            bno_updateRate(sensors.rate.fast, sensors.rate.medium, sensors.rate.slow, sensors.rate.medium);
            gps_updateRate(sensors.rate.slow);
            break;
        default:
            break;
    }
    return;
}

static void sensors_processData(sensors_event_t *event) {
    int64_t timestamp = event->timestamp;
    sensors_event_type_t type = event->type;
    // Sensorzustand speichern
    sensors.rawData[type] = event;
    // Datenzugriff sperren
    if (xSemaphoreTake(sensors.data.lock, SENSORS_DATA_LOCK / portTICK_PERIOD_MS) == pdFALSE) configASSERT(false);
    // Verarbeiten
    switch (type) {
        case (SENSORS_ACCELERATION):
            sensors.data.acceleration = *event;
            sensors_fuseX(SENSORS_ACCELERATION, sensors.data.acceleration.vector.x, timestamp);
            sensors_fuseY(SENSORS_ACCELERATION, sensors.data.acceleration.vector.y, timestamp);
            sensors_fuseZ(SENSORS_ACCELERATION, sensors.data.acceleration.vector.z, timestamp);
            break;
        case (SENSORS_ORIENTATION):
            sensors.data.orientation = *event;
            bno_toEuler(&sensors.data.euler.vector, &sensors.data.orientation.orientation);
            sensors.data.euler.timestamp = timestamp;
            pvPublish(xSensors, SENSORS_PV_ORIENTATION);
            break;
        case (SENSORS_ALTIMETER):
            sensors.data.altitude.value = event->vector.z - sensors.homes.altitude;
            sensors.data.altitude.timestamp = timestamp;
            sensors_fuseZ(SENSORS_ALTIMETER, sensors.data.altitude.value, timestamp);
            break;
        case (SENSORS_ROTATION):
            sensors.data.rotation = *event;
            pvPublishFloat(xSensors, SENSORS_PV_GYRO_X, event->vector.x);
            pvPublishFloat(xSensors, SENSORS_PV_GYRO_Y, event->vector.y);
            pvPublishFloat(xSensors, SENSORS_PV_GYRO_Z, event->vector.z);
            break;
        case (SENSORS_POSITION):
            sensors.data.coordinates.vector.x = event->vector.x - sensors.homes.position.x;
            sensors.data.coordinates.vector.y = event->vector.y - sensors.homes.position.y;
            sensors.data.coordinates.vector.z = event->vector.z - sensors.homes.position.z;
            sensors.data.coordinates.accuracy = event->accuracy;
            sensors.data.coordinates.timestamp = timestamp;
            sensors_fuseX(SENSORS_POSITION, sensors.data.coordinates.vector.x, timestamp);
            sensors_fuseY(SENSORS_POSITION, sensors.data.coordinates.vector.y, timestamp);
            sensors_fuseZ(SENSORS_POSITION, sensors.data.coordinates.vector.z, timestamp);
            break;
        case (SENSORS_GROUNDSPEED):
            sensors.data.speed = *event;
            sensors_fuseX(SENSORS_GROUNDSPEED, sensors.data.speed.vector.x, timestamp);
            sensors_fuseY(SENSORS_GROUNDSPEED, sensors.data.speed.vector.y, timestamp);
            // sensors_fuseZ(SENSORS_GROUNDSPEED, sensors.data.speed.vector.z, timestamp);
            break;
        case (SENSORS_VOLTAGE):
            sensors.data.volt = *event;
            if (sensors.data.volt.value < sensors.voltage.warning) {
                pvPublish(xSensors, SENSORS_PV_VOLTAGE_WARN);
            } else if (sensors.data.volt.value < sensors.voltage.low) {
                pvPublish(xSensors, SENSORS_PV_VOLTAGE_LOW);
            }
            pvPublishFloat(xSensors, SENSORS_PV_VOLTAGE, sensors.data.volt.value);
            break;
        case (SENSORS_OPTICAL_FLOW): {
            // skalieren von Pixel/s -> rad/s & ggf. Achsen kehren
            float x = event->vector.x * sensors.flow.scaleX;
            float y = event->vector.y * sensors.flow.scaleY;
            // mit Gyro kompensieren
            x -= sensors.data.rotation.vector.x;
            y -= sensors.data.rotation.vector.y;
            ESP_LOGD("sensors", "flow 1\t%f\t%f\t%f", event->vector.x, event->vector.y, event->accuracy);
            // mit Höhe zu Geschwindigkeit umrechnen
            vector_t flow = {0};
            flow.x = tanf(x / 2.0f) * 2.0f * sensors.data.position.vector.z;
            flow.y = tanf(y / 2.0f) * 2.0f * sensors.data.position.vector.z;
            // nur um Yaw Drehung korrigieren
            orientation_t rotateZ = sensors.data.orientation.orientation;
            float thetaZ = atan2f(rotateZ.k, rotateZ.real);
            rotateZ.i = 0.0f;
            rotateZ.j = 0.0f;
            rotateZ.k = sinf(thetaZ);
            rotateZ.real = cosf(thetaZ);
            //bno_toWorldFrame(&flow, &rotateZ); // DEBUG ToDo: nur um Yaw drehen
            ESP_LOGD("sensors", "flow 2\t\t\t\t\t\t\t%f\t%f\t%u", flow.x, flow.y, uxTaskGetStackHighWaterMark(NULL));
            // kopiere Rest
            sensors.data.flow.vector = flow;
            sensors.data.flow.timestamp = timestamp;
            sensors.data.flow.accuracy = event->accuracy;
            pvPublishFloat(xSensors, SENSORS_PV_FLOW_X, sensors.data.flow.vector.x);
            pvPublishFloat(xSensors, SENSORS_PV_FLOW_Y, sensors.data.flow.vector.y);
            // ToDo: der Fusion übergeben
            // ToDo: Fusion auf Flow anpassen, ggf. Fehler Flow != Fehler SpeedGPS
            // sensors_fuseX(SENSORS_GROUNDSPEED, sensors.data.flow.vector.x, timestamp);
            // sensors_fuseY(SENSORS_GROUNDSPEED, sensors.data.flow.vector.y, timestamp);
            break;
        }
        case (SENSORS_LIDAR): {
            vector_t distance = {.x = 0.0f, .y = 0.0f, .z = -event->value};
            bno_toWorldFrame(&distance, &sensors.data.orientation.orientation);
            sensors.data.distance.value = (-distance.z) - sensors.homes.distance;
            sensors.data.distance.timestamp = timestamp;
            sensors_fuseZ(SENSORS_LIDAR, sensors.data.distance.value, timestamp);
            break;
        }
        default:
            break;
    }
    // Datenzugriff freigeben
    xSemaphoreGive(sensors.data.lock);
    // Timeoutkontrolle
    uint32_t timedOut = sensors.timedOut;
    sensors_resetTimeout(event->type); // Timeout des aktuellen Sensors zurücksetzen
    sensors_detectTimeout(event->timestamp); // Timeout der anderen Sensoren erkennen
    // bei Änderung ob neuerdings offline oder online, melden
    if (timedOut != sensors.timedOut) pvPublishUint(xSensors, SENSORS_PV_TIMEOUT, sensors.timedOut);
}

static void sensors_detectTimeout(int64_t timestamp) {
    for (sensors_event_type_t i = 0; i < SENSORS_MAX; ++i) {
        if (!sensors.rawData[i]) {
            sensors_setTimeout(i);
        } else if ((timestamp - sensors.rawData[i]->timestamp) > sensors.timeout) {
            sensors_setTimeout(i);
            sensors.rawData[i]->timestamp = timestamp;
        }
    }
}

static inline void sensors_resetTimeout(sensors_event_type_t sensor) {
    sensors.timedOut ^= (0x1 << sensor); 
}

static inline void sensors_setTimeout(sensors_event_type_t sensor) {
    sensors.timedOut |= (0x1 << sensor); 
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
        z = fabsf(z) + sensors.Z.errorAcceleration;
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
            case (SENSORS_LIDAR):
                *EEKF_MAT_EL(sensors.Z.z, 0, 0) = z;
                *EEKF_MAT_EL(R, 0, 0) = sensors.Z.errorLidar;
                break;
            case (SENSORS_ALTIMETER):
                *EEKF_MAT_EL(sensors.Z.z, 1, 0) = z;
                *EEKF_MAT_EL(R, 1, 1) = sensors.Z.errorBarometer;
                break;
            case (SENSORS_POSITION):
                *EEKF_MAT_EL(sensors.Z.z, 2, 0) = z;
                *EEKF_MAT_EL(R, 2, 2) = sensors.Z.errorGPS;
                break;
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
    //ESP_LOGD("sensors", "Fz,%f,%f,Z,%f,%f,%f", *EEKF_MAT_EL(sensors.Z.x, 0, 0), *EEKF_MAT_EL(sensors.Z.x, 1, 0), *EEKF_MAT_EL(sensors.Z.z, 0, 0), *EEKF_MAT_EL(sensors.Z.z, 1, 0), *EEKF_MAT_EL(sensors.Z.z, 2, 0));
    // Publish
    sensors.data.position.vector.z = *EEKF_MAT_EL(sensors.Z.x, 0, 0);
    pvPublishFloat(xSensors, SENSORS_PV_Z, *EEKF_MAT_EL(sensors.Z.x, 0, 0));
    sensors.data.velocity.vector.z = *EEKF_MAT_EL(sensors.Z.x, 1, 0);
    pvPublishFloat(xSensors, SENSORS_PV_VZ, *EEKF_MAT_EL(sensors.Z.x, 1, 0));
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
    if (*predictedVelocity > sensors.Z.limitVelocity) *predictedVelocity = sensors.Z.limitVelocity;
    else if (*predictedVelocity < -sensors.Z.limitVelocity) *predictedVelocity = -sensors.Z.limitVelocity;
    return eEekfReturnOk;
}

eekf_return sensors_fuseZ_measurement(eekf_mat* zp, eekf_mat* Jh, eekf_mat const *x, void* userData) {
    sensors_event_type_t type = *((sensors_event_type_t*) userData);
    // Messmodell an Messung anpassen
    memset(Jh->elements, 0.0f, sizeof(eekf_value) * Jh->rows * Jh->cols);
    switch (type) {
        case (SENSORS_LIDAR):
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
        y = fabsf(y) + sensors.Y.errorAcceleration;
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
                *EEKF_MAT_EL(R, 0, 0) = sensors.Y.errorGPS;
                break;
            case (SENSORS_GROUNDSPEED):
                *EEKF_MAT_EL(sensors.Y.z, 1, 0) = y;
                *EEKF_MAT_EL(R, 1, 1) = sensors.Y.errorVelocity;
                break;
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
    //ESP_LOGD("sensors", "Fy,%f,%f,Z,%f,%f", *EEKF_MAT_EL(sensors.Y.x, 0, 0), *EEKF_MAT_EL(sensors.Y.x, 1, 0), *EEKF_MAT_EL(sensors.Y.z, 0, 0), *EEKF_MAT_EL(sensors.Y.z, 1, 0));
    // Publish
    sensors.data.position.vector.y = *EEKF_MAT_EL(sensors.Y.x, 0, 0);
    pvPublishFloat(xSensors, SENSORS_PV_Y, *EEKF_MAT_EL(sensors.Y.x, 0, 0));
    sensors.data.velocity.vector.y = *EEKF_MAT_EL(sensors.Y.x, 1, 0);
    pvPublishFloat(xSensors, SENSORS_PV_VY, *EEKF_MAT_EL(sensors.Y.x, 1, 0));
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
    if (*predictedVelocity > sensors.Y.limitVelocity) *predictedVelocity = sensors.Y.limitVelocity;
    else if (*predictedVelocity < -sensors.Y.limitVelocity) *predictedVelocity = -sensors.Y.limitVelocity;
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
            break;
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
        x = fabsf(x) + sensors.X.errorAcceleration;
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
        EEKF_DECL_MAT_INIT(R, 2, 2, 0);
        switch (type) {
            case (SENSORS_POSITION):
                *EEKF_MAT_EL(sensors.X.z, 0, 0) = x;
                *EEKF_MAT_EL(R, 0, 0) = sensors.X.errorGPS;
                break;
            case (SENSORS_GROUNDSPEED):
                // *EEKF_MAT_EL(sensors.X.z, 1, 0) = x;
                // *EEKF_MAT_EL(R, 1, 1) = sensors.X.errorVelocity;
                // break;
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
    //ESP_LOGD("sensors", "Fx,%f,%f,Z,%f,%f", *EEKF_MAT_EL(sensors.X.x, 0, 0), *EEKF_MAT_EL(sensors.X.x, 1, 0), *EEKF_MAT_EL(sensors.X.z, 0, 0), *EEKF_MAT_EL(sensors.X.z, 1, 0));
    // Publish
    sensors.data.position.vector.x = *EEKF_MAT_EL(sensors.X.x, 0, 0);
    pvPublishFloat(xSensors, SENSORS_PV_X, *EEKF_MAT_EL(sensors.X.x, 0, 0));
    sensors.data.velocity.vector.x = *EEKF_MAT_EL(sensors.X.x, 1, 0);
    pvPublishFloat(xSensors, SENSORS_PV_VX, *EEKF_MAT_EL(sensors.X.x, 1, 0));
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
    if (*predictedVelocity > sensors.X.limitVelocity) *predictedVelocity = sensors.X.limitVelocity;
    else if (*predictedVelocity < -sensors.X.limitVelocity) *predictedVelocity = -sensors.X.limitVelocity;
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
        //     *EEKF_MAT_EL(*Jh, 1, 1) = 1.0f;
        //     break;
        default:
            return eEekfReturnParameterError;
    }
    // zp = H * x
    if (NULL == eekf_mat_mul(zp, Jh, x)) return eEekfReturnComputationFailed;
	return eEekfReturnOk;
}
