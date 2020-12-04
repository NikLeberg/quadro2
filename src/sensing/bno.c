/*
 * File: bno.c
 * ----------------------------
 * Author: Niklaus Leuenberger
 * Date:   2020-02-09
 * ----------------------------
 * Implementiert den BNO080 Sensor mittels SensorHub-2 Bibliothek von Hillcrest
 * 
 * Pinbelegung:
 * - SA0, PS0 & PS1 an GND
 * - BOOT & CL0 an 3.3V
 * - EDA an SDI
 * - ECL an SCK
 */


/** Externe Abhängigkeiten **/

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include <string.h>
#include <math.h>
#include "sh2.h"
#include "sh2_hal.h"
#include "sh2_SensorValue.h"
#include "sh2_err.h"
#include "esp_log.h"
#include "driver/gpio.h"


/** Interne Abhängigkeiten **/

#include "intercom.h"
#include "resources.h"
#include "i2c.h"
#include "sensor_types.h"
#include "sensors.h"
#include "bno.h"


/** Variablendeklaration **/

#ifndef M_PI_2
    #define M_PI_2 1.57079632679489661923f
#endif

typedef struct {
    int64_t timestamp;
} bno_event_t;

static struct {
    uint8_t address;
    gpio_num_t resetPin, interruptPin;
    sh2_rxCallback_t *onRx;
    uint8_t rxBuffer[SH2_HAL_MAX_TRANSFER];
    SemaphoreHandle_t sh2Lock;
    uint32_t timeout;

    sensors_event_t acceleration;
    sensors_event_t orientation;
    sensors_event_t altitude;
    sensors_event_t rotation;
    event_t forward;
} bno;

static QueueHandle_t xBno;


/** Private Functions **/

/*
 * Function: bno_task
 * ----------------------------
 * Haupttask. Verwaltet BNO-Events
 *
 * void* arg: Dummy für FreeRTOS
 */
void bno_task(void* arg);

/*
 * Function: bno_sensorEvent
 * ----------------------------
 * Event-Handler der von der sh2-Lib bei Ankunft von Sensordaten ausgeführt wird.
 * Sendet verarbeitete Daten an Sensors-Queue.
 *
 * void * cookie: NULL
 * sh2_SensorEvent_t *event: Event der sh2-Lib
 */
static void bno_sensorEvent(void * cookie, sh2_SensorEvent_t *event);

/*
 * Function: bno_interrupt
 * ----------------------------
 * Interrupt-Handler wird bei CHANGE des interuptPins vom BNO aufgeführt.
 * Sendet bei negativer Flanke einen Event mit aktueller Zeit an Haupttask.
 *
 * void* arg: Dummy
 */
static void IRAM_ATTR bno_interrupt(void* arg);

/*
 * Function: bno_initDone
 * ----------------------------
 * Gibt Semaphor während der Initialisierung an die bno_init Funktion zurück welche im
 * blockiert-Zustand darauf wartet.
 *
 * void *cookie: pointer des Semaphors
 * sh2_AsyncEvent_t *event: Event der sh2-Lib
 */
static void bno_initDone(void *cookie, sh2_AsyncEvent_t *event);

/*
 * Function: bno_sensorEnable
 * ----------------------------
 * Aktiviert den angegebenen Sensor, sh2 entscheidet der gegebene oder ein ähnlicher
 * Intervall genutzt wird.
 *
 * sh2_SensorId_t sensorId: Id des Sensorreports
 * uint32_t interval_us: Intervall in Microsekunden.
 */
static bool bno_sensorEnable(sh2_SensorId_t sensorId, uint32_t interval_us);


/** Implementierung **/

bool bno_init(uint8_t address, gpio_num_t interruptPin, gpio_num_t resetPin,
              uint32_t rateOrientation, uint32_t rateAcceleration, uint32_t ratePressure, uint32_t rateGyro) {
    // Parameter speichern
    bno.address = address;
    bno.interruptPin = interruptPin;
    bno.resetPin = resetPin;
    // Input Queue
    xBno = xQueueCreate(4, sizeof(bno_event_t));
    bno.acceleration.type = SENSORS_ACCELERATION;
    bno.orientation.type = SENSORS_ORIENTATION;
    bno.altitude.type = SENSORS_ALTIMETER;
    bno.rotation.type = SENSORS_ROTATION;
    bno.forward.type = EVENT_INTERNAL;
    // konfiguriere Pins und aktiviere Interrupts
    gpio_config_t gpioConfig;
    gpioConfig.pin_bit_mask = ((1ULL) << resetPin);
    gpioConfig.mode = GPIO_MODE_OUTPUT;
    gpioConfig.pull_up_en = GPIO_PULLUP_DISABLE;
    gpioConfig.pull_down_en = GPIO_PULLDOWN_DISABLE;
    gpioConfig.intr_type = GPIO_INTR_DISABLE;
    gpio_config(&gpioConfig); // Resetpin
    gpio_set_level(resetPin, 1);
    gpioConfig.pin_bit_mask = ((1ULL) << interruptPin);
    gpioConfig.mode = GPIO_MODE_INPUT;
    gpioConfig.pull_up_en = GPIO_PULLUP_DISABLE;
    gpioConfig.pull_down_en = GPIO_PULLDOWN_DISABLE;
    gpioConfig.intr_type = GPIO_INTR_NEGEDGE;
    gpio_config(&gpioConfig); // Interruptpin
    gpio_install_isr_service(ESP_INTR_FLAG_IRAM);
    if (gpio_isr_handler_add(interruptPin, &bno_interrupt, NULL)) return true;
    // Task starten
    bno.timeout = (rateOrientation / portTICK_PERIOD_MS) + 1;
    if (xTaskCreate(&bno_task, "bno", 3 * 1024, NULL, xSensors_PRIORITY - 1, NULL) != pdTRUE) return true;
    // SensorHub-2 Bibliothek starten
    bno.sh2Lock = xSemaphoreCreateBinary();
    SemaphoreHandle_t sInitDone = xSemaphoreCreateBinary();
    if (sh2_initialize(&bno_initDone, sInitDone)) return true;
    if (xSemaphoreTake(sInitDone, BNO_STARTUP_WAIT_MS / portTICK_PERIOD_MS) == pdFALSE) return true; // warte auf Initialisierung
    vSemaphoreDelete(sInitDone);
    // Standartsensoren aktivieren
    if (sh2_setSensorCallback(&bno_sensorEvent, NULL)) return true;
    if (bno_sensorEnable(SH2_ROTATION_VECTOR, rateOrientation * 1000)) return true;
    if (bno_sensorEnable(SH2_LINEAR_ACCELERATION, rateAcceleration * 1000)) return true;
    if (bno_sensorEnable(SH2_PRESSURE, ratePressure * 1000)) return true;
    if (bno_sensorEnable(SH2_GYROSCOPE_CALIBRATED, rateGyro * 1000)) return true;
    return false;
}

void bno_task(void* arg) {
    // Variablen
    bno_event_t event;
    uint16_t readLength, rxRemaining = 0;
    uint8_t timeoutCount = 0;
    while (!bno.onRx) vTaskDelay(10); // auf sh2-Lib Registrierung (nach reset) warten
    // Loop
    while (true) {
        if (xQueueReceive(xBno, &event, bno.timeout) == pdTRUE) {
            // Datenlänge berechnen, mindestens Header, maximal MAX_TRANSFER
            readLength = rxRemaining;
            if (readLength < SHTP_HEADER_LEN) readLength = SHTP_HEADER_LEN;
            if (readLength > SH2_HAL_MAX_TRANSFER) readLength = SH2_HAL_MAX_TRANSFER;
            // Lesen
            if (i2c_read(bno.address, bno.rxBuffer, readLength)) continue;
            // Ermittle Datenlänge des SHTP-Packets
            uint16_t cargoLength;
            cargoLength = ((bno.rxBuffer[1] << 8) + (bno.rxBuffer[0])) & 0x7fff;
            if (!cargoLength) continue;
            // verbleibende Daten berechnen
            if (cargoLength > readLength) {
                rxRemaining = (cargoLength - readLength) + SHTP_HEADER_LEN;
            } else rxRemaining = 0;
            // an sh2-Lib übergeben
            bno.onRx(NULL, bno.rxBuffer, readLength, event.timestamp);
            timeoutCount = 0;
        } else { // vermutlich ein Interrupt verpasst, prüfe
            if (gpio_get_level(bno.interruptPin) == 0 || ++timeoutCount > 3) {
                if (timeoutCount > 3) pvPublishUint(xSensors, SENSORS_PV_TIMEOUT, (0x1 << SENSORS_ORIENTATION)); // DEBUG, FixMe
                event.timestamp = esp_timer_get_time();
                xQueueSendToBack(xBno, &event, 0);
            }
        }
    }
}

void bno_toWorldFrame(vector_t *vector, orientation_t *quaternion) {
    // (q.r * q.r - dot(q, q)) * v + 2.0f * dot(q, v) * q + 2.0f * q.r * cross(q, v);
    vector_t v = *vector;
    orientation_t q;
    if (quaternion) q = *quaternion;
    else q = bno.orientation.orientation;
    float factor;

    factor  = q.real * q.real;
    factor -= q.i * q.i;
    factor -= q.j * q.j;
    factor -= q.k * q.k;
    v.x *= factor;
    v.y *= factor;
    v.z *= factor;

    factor  = vector->x * q.i;
    factor += vector->y * q.j;
    factor += vector->z * q.k;
    factor *= 2.0f;
    v.x += q.i * factor;
    v.y += q.j * factor;
    v.z += q.k * factor;

    factor = q.real * 2.0f;
    v.x += factor * (q.j * vector->z - q.k * vector->y);
    v.y += factor * (q.k * vector->x - q.i * vector->z);
    v.z += factor * (q.i * vector->y - q.j * vector->x);

    *vector = v;
    return;
}

void bno_toLocalFrame(vector_t *vector, orientation_t *quaternion) {
    orientation_t q;
    if (quaternion) q = *quaternion;
    else q = bno.orientation.orientation;
    // Invertiere Quaternion mittels Conjugate. Normalisierung nicht nötig da Einheitsquaternion.
    // Conjugate des Quaternions
    q.i = -q.i;
    q.j = -q.j;
    q.k = -q.k;
    bno_toWorldFrame(vector, &q);
}

// https://math.stackexchange.com/questions/2975109/how-to-convert-euler-angles-to-quaternions-and-get-the-same-euler-angles-back-fr
void bno_toEuler(vector_t *euler, orientation_t *quaternion) {
    orientation_t q;
    if (quaternion) q = *quaternion;
    else q = bno.orientation.orientation;
    float i2 = q.i * q.i;
    float j2 = q.j * q.j;
    float k2 = q.k * q.k;
    float t0 = 2.0f * (q.real * q.i + q.j * q.k);
    float t1 = 1.0f - 2.0f * (i2 + j2);
    euler->x = atan2f(t0, t1); // roll
    float t2 = 2.0f * (q.real * q.j - q.k * q.i);
    if (t2 > 1.0f) t2 = 1.0f;
    if (t2 < -1.0f) t2 = -1.0f;
    euler->y = asinf(t2); // pitch
    float t3 = 2.0f * (q.real * q.k + q.i * q.j);
    float t4 = 1.0f - 2.0f * (j2 + k2);
    euler->z = atan2f(t3, t4); // yaw
    return;
}

static bool bno_sensorEnable(sh2_SensorId_t sensorId, uint32_t interval_us) {
    sh2_SensorConfig_t config = {
        changeSensitivityEnabled : false,
        wakeupEnabled : false,
        changeSensitivityRelative : false,
        alwaysOnEnabled : true,
        changeSensitivity : 0,
        reportInterval_us : interval_us,
        batchInterval_us: 0
    };
    if (sh2_setSensorConfig(sensorId, &config)) return true;
    return false;
}

static void bno_sensorEvent(void * cookie, sh2_SensorEvent_t *event) {
    sh2_SensorValue_t value;
    if (sh2_decodeSensorEvent(&value, event)) return;
    // Daten verarbeitet an Sensortask weitergeben
    switch (value.sensorId) {
        case (SH2_LINEAR_ACCELERATION): { // Beschleunigung in globalem Koordinatensystem
            vector_t v;
            v.x = value.un.linearAcceleration.x;
            v.y = value.un.linearAcceleration.y;
            v.z = value.un.linearAcceleration.z;
            bno_toWorldFrame(&v, NULL);
            bno.acceleration.vector = v;
            bno.acceleration.accuracy = value.status & 0b00000011;
            bno.acceleration.timestamp = value.timestamp;
            bno.forward.data = &bno.acceleration;
            break;
        }
        case (SH2_ROTATION_VECTOR):
            bno.orientation.orientation.i = value.un.rotationVector.i;
            bno.orientation.orientation.j = value.un.rotationVector.j;
            bno.orientation.orientation.k = value.un.rotationVector.k;
            bno.orientation.orientation.real = value.un.rotationVector.real;
            bno.orientation.accuracy = value.un.rotationVector.accuracy;
            bno.orientation.timestamp = value.timestamp;
            bno.forward.data = &bno.orientation;
            break;
        case (SH2_PRESSURE): // Druck in Meter über Meer umrechnen
            bno.altitude.vector.z = (228.15f / 0.0065f) * (1.0f - powf(value.un.pressure.value / 1013.25f, (1.0f / 5.255f)));
            bno.altitude.accuracy = value.status & 0b00000011;
            bno.altitude.timestamp = value.timestamp;
            bno.forward.data = &bno.altitude;
            break;
        case (SH2_GYROSCOPE_CALIBRATED):
            bno.rotation.vector.x = value.un.gyroscope.x;
            bno.rotation.vector.y = value.un.gyroscope.y;
            bno.rotation.vector.z = value.un.gyroscope.z;
            bno.rotation.timestamp = value.timestamp;
            bno.forward.data = &bno.rotation;
            break;
        default:
            return;
    }
    xQueueSendToBack(xSensors, &bno.forward, 0);
    return;
}

static void IRAM_ATTR bno_interrupt(void* arg) {
    BaseType_t woken;
    bno_event_t event;
    bool level;
    if (bno.interruptPin < 32) { // gpio_get_level ist nicht im IRAM
        level = (GPIO.in >> bno.interruptPin) & 0x1;
    } else {
        level = (GPIO.in1.data >> (bno.interruptPin - 32)) & 0x1;
    }
    if (!level) {
        event.timestamp = esp_timer_get_time();
        xQueueSendToBackFromISR(xBno, &event, &woken);
        if (woken == pdTRUE) portYIELD_FROM_ISR();
    }
}

static void bno_initDone(void *cookie, sh2_AsyncEvent_t *event) {
    if (event->eventId == SH2_RESET && cookie != NULL) {
        xSemaphoreGive((SemaphoreHandle_t) cookie);
    }
}

void bno_updateRate(uint32_t rateOrientation, uint32_t rateAcceleration, uint32_t ratePressure, uint32_t rateGyro) {
    int32_t result;
    result = sh2_reinitialize();
    ESP_LOGD("bno", "reinit: %i", result);
    bno.timeout = (rateOrientation / portTICK_PERIOD_MS) + 1;
    result = bno_sensorEnable(SH2_ROTATION_VECTOR, rateOrientation * 1000);
    ESP_LOGD("bno", "enable quat: %i", result);
    result = bno_sensorEnable(SH2_LINEAR_ACCELERATION, rateAcceleration * 1000);
    ESP_LOGD("bno", "enable accel: %i", result);
    result = bno_sensorEnable(SH2_PRESSURE, ratePressure * 1000);
    ESP_LOGD("bno", "enable press: %i", result);
    result = bno_sensorEnable(SH2_GYROSCOPE_CALIBRATED, rateGyro * 1000);
    ESP_LOGD("bno", "enable gyro: %i", result);
}

/** sh2-hal Implementierung (sh2_hal.h) **/
int sh2_hal_reset(bool dfuMode, sh2_rxCallback_t *onRx, void *cookie) {
    // DFU-Modus nicht unterstützt
    configASSERT(!dfuMode);
    // Sensor-Reset
    gpio_set_level(bno.resetPin, 0);
    vTaskDelay(100 / portTICK_PERIOD_MS);
    gpio_set_level(bno.resetPin, 1);
    vTaskDelay(200 / portTICK_PERIOD_MS);
    // Callback speichern
    bno.onRx = onRx;
    return SH2_OK;
}

int sh2_hal_tx(uint8_t *pData, uint32_t len) {
    return i2c_write(bno.address, pData, len);
}

int sh2_hal_rx(uint8_t *pData, uint32_t len) {
    configASSERT(false);
    return SH2_ERR; // DFU Modus nicht implementiert
}

int sh2_hal_block(void) {
    if (xSemaphoreTake(bno.sh2Lock, BNO_STARTUP_WAIT_MS / portTICK_PERIOD_MS) == pdFALSE) {
        configASSERT(false); // sh2 blockiert
    }
    return SH2_OK;
}

int sh2_hal_unblock(void) {
    xSemaphoreGive(bno.sh2Lock);
    return SH2_OK;
}