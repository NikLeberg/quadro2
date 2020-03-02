/*
 * File: bno.c
 * ----------------------------
 * Author: Niklaus Leuenberger
 * Date:   2020-02-09
 * ----------------------------
 * Implementiert den BNO080 Sensor mittels SensorHub-2 Bibliothek von Hillcrest
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

#include "i2c.h"
#include "bno.h"
#include "resources.h"
#include "sensor_types.h"


/** Variablendeklaration **/

enum bno_input_type_t {
    BNO_INPUT_INTERRUPT
};

typedef void (bno_dataCallback_t)(sh2_SensorValue_t value);

struct bno_input_t {
    enum bno_input_type_t type;
    union {
        struct { // BNO_INPUT_INTERRUPT
            int64_t timestamp;
        };
    };
};

struct bno_t {
    uint8_t address;
    gpio_num_t resetPin, interruptPin;
    sh2_rxCallback_t *onRx;
    uint8_t rxBuffer[SH2_HAL_MAX_TRANSFER];

    portMUX_TYPE spinLock;
    sh2_RotationVectorWAcc_t lastOrientation;

    bool setHome;
    float homeAltitude;
};
static struct bno_t bno;


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

bool bno_init(uint8_t address, gpio_num_t interruptPin, gpio_num_t resetPin) {
    // Parameter speichern
    bno.address = address;
    bno.interruptPin = interruptPin;
    bno.resetPin = resetPin;
    // Input Queue & Spinlock erstellen
    xBno_input = xQueueCreate(4, sizeof(struct bno_input_t));
    bno.spinLock.owner = portMUX_FREE_VAL;
    bno.spinLock.count = 0;
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
    if (xTaskCreate(&bno_task, "bno", 3 * 1024, NULL, xSensors_PRIORITY - 1, &xBno_handle) != pdTRUE) return true;
    // SensorHub-2 Bibliothek starten
    SemaphoreHandle_t sInitDone = xSemaphoreCreateBinary();
    if (sh2_initialize(&bno_initDone, sInitDone)) return true;
    if (xSemaphoreTake(sInitDone, BNO_STARTUP_WAIT_MS / portTICK_PERIOD_MS) == pdFALSE) return true; // warte auf Initialisierung
    vSemaphoreDelete(sInitDone);
    // Standartsensoren aktivieren (Intervall so schnell wie möglich)
    if (sh2_setSensorCallback(&bno_sensorEvent, NULL)) return true;
    if (bno_sensorEnable(SH2_ROTATION_VECTOR, BNO_DATA_RATE_IMU_US)) return true;
    if (bno_sensorEnable(SH2_LINEAR_ACCELERATION, BNO_DATA_RATE_IMU_US)) return true;
    if (bno_sensorEnable(SH2_PRESSURE, BNO_DATA_RATE_PRESSURE_US)) return true;
    return false;
}

void bno_task(void* arg) {
    // Variablen
    struct bno_input_t input;
    uint16_t readLength, rxRemaining = 0;
    uint8_t timeoutCount = 0;
    // Loop
    while (true) {
        if (xQueueReceive(xBno_input, &input, (BNO_DATA_RATE_IMU_US / 1000 / portTICK_PERIOD_MS) + 1) == pdTRUE) {
            switch (input.type) {
                case (BNO_INPUT_INTERRUPT): {
                    // ist sh2-Lib registriert?
                    if (!bno.onRx) break;
                    // Datenlänge berechnen, mindestens Header, maximal MAX_TRANSFER
                    readLength = rxRemaining;
                    if (readLength < SHTP_HEADER_LEN) readLength = SHTP_HEADER_LEN;
                    if (readLength > SH2_HAL_MAX_TRANSFER) readLength = SH2_HAL_MAX_TRANSFER;
                    // Lesen
                    if (i2c_read(bno.address, bno.rxBuffer, readLength)) break;
                    // Ermittle Datenlänge des SHTP-Packets
                    uint16_t cargoLength;
                    cargoLength = ((bno.rxBuffer[1] << 8) + (bno.rxBuffer[0])) & 0x7fff;
                    if (!cargoLength) break;
                    // verbleibende Daten berechnen
                    if (cargoLength > readLength) {
                        rxRemaining = (cargoLength - readLength) + SHTP_HEADER_LEN;
                    } else rxRemaining = 0;
                    // an sh2-Lib übergeben
                    bno.onRx(NULL, bno.rxBuffer, readLength, input.timestamp);
                    break;
                }
            }
            timeoutCount = 0;
        } else { // vermutlich ein Interrupt verpasst, prüfe
            if (gpio_get_level(bno.interruptPin) == 0 || ++timeoutCount > 10) {
                struct bno_input_t input;
                input.type = BNO_INPUT_INTERRUPT;
                input.timestamp = esp_timer_get_time();
                xQueueSendToBack(xBno_input, &input, 0);
            }
        }
    }
}

void bno_toWorldFrame(struct vector_t *vector) {
    // (q.r * q.r - dot(q, q)) * v + 2.0f * dot(q, v) * q + 2.0f * q.r * cross(q, v);
    struct vector_t v = *vector;
    portENTER_CRITICAL(&bno.spinLock);
    sh2_RotationVectorWAcc_t q = bno.lastOrientation;
    portEXIT_CRITICAL(&bno.spinLock);
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

void bno_setHome() {
    bno.setHome = true;
    return;
}

static bool bno_sensorEnable(sh2_SensorId_t sensorId, uint32_t interval_us) {
    sh2_SensorConfig_t config;
    config.changeSensitivityEnabled = false;
    config.wakeupEnabled = false;
    config.changeSensitivityRelative = false;
    config.alwaysOnEnabled = false;
    config.changeSensitivity = 0;
    config.reportInterval_us = interval_us;
    config.batchInterval_us = 0;
    if (sh2_setSensorConfig(sensorId, &config)) return true;
    return false;
}

static void bno_sensorEvent(void * cookie, sh2_SensorEvent_t *event) {
    sh2_SensorValue_t value;
    if (sh2_decodeSensorEvent(&value, event)) return;
    // Daten verarbeitet an Sensortask weitergeben
    struct sensors_input_t forward;
    switch (value.sensorId) {
        case (SH2_LINEAR_ACCELERATION): // Beschleunigung in globalem Koordinatensystem
            forward.type = SENSORS_ACCELERATION;
            forward.vector.x = value.un.linearAcceleration.x;
            forward.vector.y = value.un.linearAcceleration.y;
            forward.vector.z = value.un.linearAcceleration.z;
            bno_toWorldFrame(&forward.vector);
            forward.accuracy = value.status & 0b00000011;
            break;
        case (SH2_ROTATION_VECTOR):
            portENTER_CRITICAL(&bno.spinLock);
            bno.lastOrientation = value.un.rotationVector; // interne Kopie
            portEXIT_CRITICAL(&bno.spinLock);
            forward.type = SENSORS_ORIENTATION;
            forward.orientation.i = value.un.rotationVector.i;
            forward.orientation.j = value.un.rotationVector.j;
            forward.orientation.k = value.un.rotationVector.k;
            forward.orientation.real = value.un.rotationVector.real;
            forward.accuracy = value.un.rotationVector.accuracy;
            break;
        case (SH2_PRESSURE): // Druck in Meter über Meer umrechnen
            forward.type = SENSORS_ALTIMETER;
            forward.distance = (228.15f / 0.0065f) * (1.0f - powf(value.un.pressure.value / 1013.25f, (1.0f / 5.255f)));
            forward.accuracy = value.status & 0b00000011;
            // Homepunkt anwenden
            if (bno.setHome) {
                bno.homeAltitude = forward.distance;
                forward.distance = 0.0f;
                bno.setHome = false;
            } else {
                forward.distance -= bno.homeAltitude;
            }
            break;
        default:
            break;
    }
    forward.timestamp = value.timestamp;
    xQueueSendToBack(xSensors_input, &forward, 0);
    return;
}

static void IRAM_ATTR bno_interrupt(void* arg) {
    BaseType_t woken;
    struct bno_input_t input;
    bool level;
    if (bno.interruptPin < 32) { // gpio_get_level ist nicht im IRAM
        level = (GPIO.in >> bno.interruptPin) & 0x1;
    } else {
        level = (GPIO.in1.data >> (bno.interruptPin - 32)) & 0x1;
    }
    if (!level) {
        input.type = BNO_INPUT_INTERRUPT;
        input.timestamp = esp_timer_get_time();
        xQueueSendToBackFromISR(xBno_input, &input, &woken);
        if (woken == pdTRUE) portYIELD_FROM_ISR();
    }
}

static void bno_initDone(void *cookie, sh2_AsyncEvent_t *event) {
    if (event->eventId == SH2_RESET && cookie != NULL) {
        xSemaphoreGive((SemaphoreHandle_t) cookie);
    }
}

/** sh2-hal Implementierung (sh2_hal.h) **/
int sh2_hal_reset(bool dfuMode, sh2_rxCallback_t *onRx, void *cookie) {
    // DFU-Modus nicht unterstützt
    configASSERT(!dfuMode);
    // Callback registrieren
    bno.onRx = onRx;
    // Sensor-Reset
    gpio_set_level(bno.resetPin, 0);
    vTaskDelay(100 / portTICK_RATE_MS);
    gpio_set_level(bno.resetPin, 1);
    vTaskDelay(200 / portTICK_RATE_MS);
    return false;
}

int sh2_hal_tx(uint8_t *pData, uint32_t len) {
    return i2c_write(bno.address, pData, len);
}

int sh2_hal_rx(uint8_t *pData, uint32_t len) {
    configASSERT(false);
    return true; // DFU Modus nicht implementiert
}

int sh2_hal_block(void) {
    return false; // rx-tx blockiert automatisch
}

int sh2_hal_unblock(void) {
    return false; // rx-tx blockiert automatisch
}