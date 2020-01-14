/*
 * File: bno.h
 * ----------------------------
 * Author: Niklaus Leuenberger
 * Date:   2019-12-04
 * ----------------------------
 *   Implementiert den BNO080 Sensor mittels SensorHub-2 Bibliothek
 * ----------------------------
 * ToDo:   Report-List korrekt freigeben, free() triggert assert
 */


#pragma once

/** Externe Abhängigkeiten **/
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include <stdio.h>
#include <stdbool.h>
#include <string.h>
#include <math.h>
#include "driver/gpio.h"
#include "sh2.h"
#include "sh2_hal.h"
#include "sh2_SensorValue.h"
#include "sh2_err.h"

/** Interne Abhängigkeiten **/
#include "resources.h"
#include "i2c.h"
#include "sensor_types.h"


/** Variablendeklaration **/

#define BNO_DATA_RATE_ORIENTATION_US 20000 // 20 ms
#define BNO_DATA_RATE_LINEAR_ACCELERATION_US 20000 // 20 ms
#define BNO_DATA_RATE_PRESSURE_US 100000 // 100 ms

enum bno_input_type_t {
    BNO_INPUT_INTERRUPT = 0,
    BNO_INPUT_REPORT_ENABLE,
    BNO_INPUT_REPORT_DISABLE
};

typedef void (bno_dataCallback_t)(sh2_SensorValue_t value);

struct bno_report_t {
    sh2_SensorId_t sensorId;
    uint32_t interval_us;
    bno_dataCallback_t* onData;
    uint64_t timestamp;
};

struct bno_report_list_t {
    struct bno_report_t *report;
    struct bno_report_list_t *next;
};

struct bno_input_t {
    enum bno_input_type_t type;
    union {
        struct {    // BNO_INPUT_INTERRUPT
            int64_t timestamp;
        };
        struct bno_report_t report; // BNO_REPORT_ENABLE
    };
};

struct bno_t {
    uint8_t address;
    gpio_num_t resetPin, interruptPin;
    sh2_rxCallback_t *onRx;
    uint8_t rxBuffer[SH2_HAL_MAX_TRANSFER];
    int64_t timeoutLinAccel, timeoutOrientation, timeoutPressure;

    struct bno_report_list_t activeReports;
};
static struct bno_t bno;


/** Public Functions **/

/*
 * Function: bno_init
 * ----------------------------
 *   Initialisiert Sensor und blockiert bis Reset fertig.
 *
 *   uint8_t bnoAddr: BNO I2C Adresse
 *   gpio_num_t bnoInterrupt: BNO Interrupt Pin (Data ready)
 *   gpio_num_t bnoReset: BNO Reset Pin
 *
 *   returns: false bei Erfolg, sonst true 
 */
static bool bno_init(uint8_t address, gpio_num_t interruptPin, gpio_num_t resetPin);

/*
 * Function: bno_sensorRegister
 * ----------------------------
 *   Aktiviert Sensorreport im sh2, empfangene Daten werden an Callback weitergeleitet.
 *   Callback wird im Kontext des bno_task's aufgerufen.
 *
 *   sh2_SensorId_t sensorId: sh2 Sensor-ID des zu aktivierenden Reports
 *   uint32_t interval_us: Zyklus in der die Daten erhalten werden sollen
 *   bno_dataCallback_t* onData: Callback der mit Daten gefüttert wird
 *
 *   returns: false bei Erfolg, sonst true wenn Queue voll ist
 */
bool bno_sensorRegister(sh2_SensorId_t sensorId, uint32_t interval_us, bno_dataCallback_t* onData);

/*
 * Function: bno_sensorUnregister
 * ----------------------------
 *   Deaktiviert Sensorreport.
 *
 *   sh2_SensorId_t sensorId: sh2 Sensor-ID des zu deaktivierenden Reports
 *   bno_dataCallback_t* onData: Callback
 *
 *   returns: false bei Erfolg, sonst true wenn Queue voll ist
 */
bool bno_sensorUnregister(sh2_SensorId_t sensorId, bno_dataCallback_t* onData);


/** Private Functions **/

/*
 * Function: bno_task
 * ----------------------------
 *   Haupttask. Verwaltet BNO-Events
 *
 *   void* arg: Dummy für FreeRTOS
 */
void bno_task(void* arg);

/*
 * Function: bno_reportEnable
 * ----------------------------
 *   Aktiviert, wenn nötig, Reports über sh2 und hängt diese an interne activeReports-Liste.
 *
 *   bno_report_t *report: report
 */
static void bno_reportEnable(struct bno_report_t *report);

/*
 * Function: bno_sensorEvent
 * ----------------------------
 *   Event-Handler der von der sh2-Lib bei Ankunft von Sensordaten aufgeführt wird.
 *   Triggert Callbacks von registrierten Reports.
 *
 *   void * cookie: NULL
 *   sh2_SensorEvent_t *event: Event der sh2-Lib
 */
static void bno_sensorEvent(void * cookie, sh2_SensorEvent_t *event);

/*
 * Function: bno_reportDisable
 * ----------------------------
 *   Löscht Callbacks aus interner activeReports-Liste. Wenn kein Callback auf Sensor mehr registriert ist,
 *   dann zusätzlich Report per sh2 deaktivieren.
 *
 *   bno_report_t *report: report
 */
static void bno_reportDisable(struct bno_report_t *report);

/*
 * Function: bno_interrupt
 * ----------------------------
 *   Interrupt-Handler wird bei CHANGE des interuptPins vom BNO aufgeführt.
 *   Sendet bei negativer Flanke einen Event mit aktueller Zeit an Haupttask.
 *
 *   void* arg: Dummy
 */
static void IRAM_ATTR bno_interrupt(void* arg);

/*
 * Function: bno_initDone
 * ----------------------------
 *   Gibt Semaphor während der Initialisierung an die bno_init Funktion zurück welche im
 *   blockiert-Zustand darauf wartet.
 *
 *   void *cookie: pointer des Semaphors
 *   sh2_AsyncEvent_t *event: Event der sh2-Lib
 */
static void bno_initDone(void *cookie, sh2_AsyncEvent_t *event);

bool bno_sensorEnable(sh2_SensorId_t sensorId, uint32_t interval_us);
static void bno_checkForTimeout(int64_t timestamp);
static bool bno_enableDefault();


/** Implementierung **/

static bool bno_init(uint8_t address, gpio_num_t interruptPin, gpio_num_t resetPin) {
    // Parameter speichern
    bno.address = address;
    bno.interruptPin = interruptPin;
    bno.resetPin = resetPin;
    // Input Queue erstellen
    xBno_input = xQueueCreate(4, sizeof(struct bno_input_t));
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
    gpio_install_isr_service(ESP_INTR_FLAG_EDGE | ESP_INTR_FLAG_IRAM | ESP_INTR_FLAG_LOWMED);
    if (gpio_isr_handler_add(interruptPin, &bno_interrupt, NULL)) return true;
    // Task starten, pinned da Interrupt an CPU gebunden sind.
    if (xTaskCreatePinnedToCore(&bno_task, "bno", 3 * 1024, NULL, xSensors_PRIORITY + 1, &xBno_handle, xPortGetCoreID()) != pdTRUE) return true;
    // SensorHub-2 Bibliothek starten
    SemaphoreHandle_t sInitDone = xSemaphoreCreateBinary();
    if (sh2_initialize(&bno_initDone, sInitDone)) return true;
    xSemaphoreTake(sInitDone, portMAX_DELAY); // warte auf Initialisierung
    vSemaphoreDelete(sInitDone);
    if (sh2_setSensorCallback(&bno_sensorEvent, NULL)) return true;
    // Standartsensoren aktivieren (Intervall so schnell wie möglich)
    if (bno_enableDefault()) return true;
    return false;
}

// bool bno_sensorRegister(sh2_SensorId_t sensorId, uint32_t interval_us, bno_dataCallback_t* onData) {
//     struct bno_input_t input;
//     input.type = BNO_INPUT_REPORT_ENABLE;
//     input.report.sensorId = sensorId;
//     input.report.interval_us = interval_us;
//     input.report.onData = onData;
//     if (xQueueSendToBack(xBno_input, &input, 0) != pdTRUE) return true;
//     return false;
// }

bool bno_sensorEnable(sh2_SensorId_t sensorId, uint32_t interval_us) {
    struct bno_input_t input;
    input.type = BNO_INPUT_REPORT_ENABLE;
    input.report.sensorId = sensorId;
    input.report.interval_us = interval_us;
    if (xQueueSendToBack(xBno_input, &input, 0) != pdTRUE) return true;
    return false;
}

// bool bno_sensorUnregister(sh2_SensorId_t sensorId, bno_dataCallback_t* onData) {
//     struct bno_input_t input;
//     input.type = BNO_INPUT_REPORT_DISABLE;
//     input.report.sensorId = sensorId;
//     input.report.onData = onData;
//     if (xQueueSendToBack(xBno_input, &input, 0) != pdTRUE) return true;
//     return false;
// }

void bno_task(void* arg) {
    // Variablen
    struct bno_input_t input;
    uint16_t readLength, rxRemaining = 0;
    // Loop
    while (true) {
        if (xQueueReceive(xBno_input, &input, 1) == pdTRUE) {
            switch (input.type) {
                case (BNO_INPUT_INTERRUPT): {
                    do { // Daten empfangen solange es etwas zu empfangen gibt, eliminiert ein Warten um ein FreeRTOS-Tick
                        // Interrupt deaktivieren damit wir ohne Unterbrechung erneut lesen können
                        //gpio_set_intr_type(bno.interruptPin, GPIO_INTR_NEGEDGE);
                        // ist sh2-Lib registriert?
                        if (!bno.onRx) break;
                        // Datenlänge berechnen, mindestens Header, maximal MAX
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
                        // Interrupt reaktivieren
                        //gpio_set_intr_type(bno.interruptPin, GPIO_INTR_LOW_LEVEL);
                        // an sh2-Lib übergeben
                        bno.onRx(NULL, bno.rxBuffer, readLength, input.timestamp);
                    } while (rxRemaining);
                    break;
                }
                case (BNO_INPUT_REPORT_ENABLE): {
                    bno_reportEnable(&input.report);
                    break;
                }
                case (BNO_INPUT_REPORT_DISABLE): {
                    // bno_reportDisable(&input.report);
                    break;
                }
            }
        } else { // vermutlich ein Interrupt verpasst, prüfe
            int64_t currentTimestamp = esp_timer_get_time();
            if (gpio_get_level(bno.interruptPin) == 0) {
                struct bno_input_t input;
                input.type = BNO_INPUT_INTERRUPT;
                input.timestamp = currentTimestamp;
                xQueueSendToBack(xBno_input, &input, 0);
            }
            // Timeouterkennung
            bno_checkForTimeout(currentTimestamp);
        }
    }
}

static void bno_reportEnable(struct bno_report_t *report) {
    // // schon aktiv?
    // bool skipConfigure = false;
    // struct bno_report_list_t *current = &bno.activeReports;
    // while (current->next) {
    //     if (current->report->sensorId == report->sensorId) {
    //         if (current->report->interval_us <= report->interval_us) {
    //             // Report mit gleichem oder schnellerem Intervall schon aktiv,
    //             // nicht Neukonfigurieren sondern nur Callback registrieren
    //             skipConfigure = true;
    //             break;
    //         }
    //     }
    //     current = current->next;
    // }
    // // Report aktivieren
    // if (!skipConfigure) {
        sh2_SensorConfig_t config;
        config.changeSensitivityEnabled = false;
        config.wakeupEnabled = false;
        config.changeSensitivityRelative = false;
        config.alwaysOnEnabled = false;
        config.changeSensitivity = 0;
        config.reportInterval_us = report->interval_us;
        config.batchInterval_us = 0;
        if (sh2_setSensorConfig(report->sensorId, &config)) return;
    // }
    // // suche Ende der Reports Liste
    // current = &bno.activeReports;
    // while (current->next) current = current->next;
    // // leeren Platz für zukünftigen Report anhängen (letztes Element immer leer)
    // current->next = (struct bno_report_list_t*) calloc(1, sizeof(struct bno_report_list_t));
    // current->next->report = (struct bno_report_t*) calloc(1, sizeof(struct bno_report_t));
    // memcpy(current->report, report, sizeof(struct bno_report_t)); // speichern
    // current->report->timestamp = 0;
}

static void bno_sensorEvent(void * cookie, sh2_SensorEvent_t *event) {
    sh2_SensorValue_t value;
    if (sh2_decodeSensorEvent(&value, event)) return;
    // Daten an Sensortask weitergeben
    struct sensors_input_t input;
    // Timeoutprävention, prüfe ob default Sensoren inaktiv wurden
    switch (value.sensorId) {
        case (SH2_LINEAR_ACCELERATION):
            bno.timeoutLinAccel = value.timestamp;
            break;
        case (SH2_ROTATION_VECTOR):
            bno.timeoutOrientation = value.timestamp;
            break;
        case (SH2_PRESSURE):
            bno.timeoutPressure = value.timestamp;
            break;
        default:
            break;
    }
    // Zusätzliche Aktionen
    input.type = value.sensorId;
    switch (value.sensorId) {
        case (SH2_LINEAR_ACCELERATION): // Daten kopieren
            // toDo: Vektor anhand Orientierung in World-Frame rechnen
            input.vector.x = value.un.linearAcceleration.x;
            input.vector.y = value.un.linearAcceleration.y;
            input.vector.z = value.un.linearAcceleration.z;
            break;
        case (SH2_ROTATION_VECTOR):
            // ToDo
            break;
        case (SH2_PRESSURE): // Druck in Meter über Meer umrechnen
            input.distance = (228.15 / 0.0065) * (1 - powf(value.un.pressure.value / 1013.25, (1 / 5.255)));
            input.type = SENSORS_ALTIMETER;
            break;
        default:
            break;
    }
    if (value.sensorId == SH2_PRESSURE) { // Druck in Meter über Meer umrechnen
        input.distance = (228.15 / 0.0065) * (1 - powf(value.un.pressure.value / 1013.25, (1 / 5.255)));
        input.type = SENSORS_ALTIMETER;
    } else {
        input.type = value.sensorId;
    }
    input.timestamp = value.timestamp;
    xQueueSendToBack(xSensors_input, &input, 0);
    // Timeouterkennung
    bno_checkForTimeout(value.timestamp);
    return;
    // Ende

    // struct bno_report_list_t *current = &bno.activeReports;
    // while (current->next) {
    //     if (current->report->sensorId == value.sensorId) {
    //         // ein Sensor kann auf mehreren Reports mit unterschiedlichen Frequenzen registriert sein,
    //         // nur an Callback übergeben wenn dies gemäss Intervall erwünscht ist
    //         if (value.timestamp - current->report->timestamp > current->report->interval_us) {
    //             current->report->timestamp = value.timestamp - 5000; // 5 ms Jitter zulassen
    //             current->report->onData(value);
    //         }
    //     }
    //     current = current->next;
    // }
}

static void bno_checkForTimeout(int64_t timestamp) {
    return;

    bool shouldReenable = false;
    if (bno.timeoutLinAccel != 0 && timestamp - bno.timeoutLinAccel > (BNO_DATA_RATE_LINEAR_ACCELERATION_US * 2)) {
        shouldReenable = true;
        bno.timeoutLinAccel = timestamp;
        ESP_LOGE("bno", "linear acceleration timeout");
    } else if (bno.timeoutOrientation != 0 && timestamp - bno.timeoutOrientation > (BNO_DATA_RATE_ORIENTATION_US * 2)) {
        shouldReenable = true;
        bno.timeoutOrientation = timestamp;
        ESP_LOGE("bno", "rotation vector timeout");
    } else if (bno.timeoutPressure != 0 && timestamp - bno.timeoutPressure > (BNO_DATA_RATE_PRESSURE_US * 2)) {
        shouldReenable = true;
        bno.timeoutPressure = timestamp;
        ESP_LOGE("bno", "pressure timeout");
    }
    // DEBUG
    // uint16_t numErrors = 10;
    // sh2_ErrorRecord_t *pErrors = (sh2_ErrorRecord_t*) calloc(10, sizeof(sh2_ErrorRecord_t));
    // if (sh2_getErrors(0, pErrors, &numErrors) == SH2_OK) {
    //     for (uint8_t i = 0; i < numErrors; ++i) {
    //         ESP_LOGD("bno", "error:%u source:%u error:%u module:%u code:%u", i, pErrors[i].source, pErrors[i].error, pErrors[i].module, pErrors[i].code);
    //     }
    // }
    //if (shouldReenable) bno_enableDefault();
}


static bool bno_enableDefault() {
    if (bno_sensorEnable(SH2_LINEAR_ACCELERATION, BNO_DATA_RATE_LINEAR_ACCELERATION_US)) return true;
    //if (bno_sensorEnable(SH2_ROTATION_VECTOR, BNO_DATA_RATE_ORIENTATION_US)) return true;
    if (bno_sensorEnable(SH2_PRESSURE, BNO_DATA_RATE_PRESSURE_US)) return true;
    return false;
}

static void bno_reportDisable(struct bno_report_t *report) {
    return;
    // Ende

    // // suche Ende der Reports Liste
    // struct bno_report_list_t *current = &bno.activeReports;
    // struct bno_report_list_t *toDelete = NULL;
    // uint8_t sumRegisterdToSensor = 0;
    // while (current->next) {
    //     if (current->report->sensorId == report->sensorId) {
    //         ++sumRegisterdToSensor; // Anzahl registrierter Reports auf diesem Sensor
    //         if (current->report->onData == report->onData) {
    //             toDelete = current;
    //         }
    //     }
    //     current = current->next;
    // }
    // // umhängen
    // current = &bno.activeReports;
    // while (current->next) {
    //     if (current->next == toDelete) {
    //         current->next = current->next->next;
    //         break;
    //     } else current = current->next;
    // }
    // // löschen
    // // ToDo, triggert: assertion "heap != NULL && "free() target pointer is outside heap areas""
    // //if (toDelete && toDelete->report) {
    // //   free(toDelete->report);
    // //   free(toDelete);
    // //}
    // // wenn dies der letzte Registrierte Report für diesen Sensor war
    // if (sumRegisterdToSensor <= 1) {
    //     sh2_SensorConfig_t config;
    //     config.changeSensitivityEnabled = false;
    //     config.wakeupEnabled = false;
    //     config.changeSensitivityRelative = false;
    //     config.alwaysOnEnabled = false;
    //     config.changeSensitivity = 0;
    //     config.reportInterval_us = 0;
    //     config.batchInterval_us = 0;
    //     sh2_setSensorConfig(report->sensorId, &config);
    // }
}

static void IRAM_ATTR bno_interrupt(void* arg) {
    BaseType_t woken;
    struct bno_input_t input;
    if (gpio_get_level(bno.interruptPin) == 0) {
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

// sh2-hal Implementierung (sh2_hal.h)
int sh2_hal_reset(bool dfuMode, sh2_rxCallback_t *onRx, void *cookie) {
    // DFU-Modus nicht unterstützt
    configASSERT(!dfuMode);
    // Callback registrieren
    bno.onRx = onRx;
    // Sensor-Reset
    gpio_set_level(bno.resetPin, 0);
    vTaskDelay(10 / portTICK_RATE_MS);
    gpio_set_level(bno.resetPin, 1);
    vTaskDelay(100 / portTICK_RATE_MS);
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