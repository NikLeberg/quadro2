/*
 * File: ina.h
 * ----------------------------
 * Author: Niklaus Leuenberger
 * Date:   2020-05-11
 * ----------------------------
 * Spannungs- und Strom Sensor INA219 von Texas Instruments.
 * Nur Spannungsmessung als Batterieüberwachung implementiert.
 */


/** Externe Abhängigkeiten **/

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"


/** Interne Abhängigkeiten **/

#include "intercom.h"
#include "resources.h"
#include "i2c.h"
#include "sensor_types.h"
#include "sensors.h"
#include "ina.h"


/** Variablendeklaration **/

static struct {
    uint8_t address;
    uint32_t *rate;

    sensors_event_t voltage;
    event_t forward;
} ina;


/** Private Functions **/

/*
 * Function: ina_task
 * ----------------------------
 * Haupttask. Verwaltet INA-Loop
 *
 * void* arg: Dummy für FreeRTOS
 */
void ina_task(void* arg);


/** Implementierung **/

bool ina_init(uint8_t address, uint32_t *rate) {
    // Parameter speichern
    ina.address = address;
    ina.rate = rate;
    // Weiterleitung
    ina.voltage.type = SENSORS_VOLTAGE;
    ina.forward.type = EVENT_INTERNAL;
    ina.forward.data = &ina.voltage;
    // konfiguriere Sensor
    uint8_t config[] = {0x00, 0b00111001, 0b10011111}; // +-320 mV - 532 us - kontinuierlich
    if (i2c_write(address, config, sizeof(config))) return true;
    uint8_t reg = 0x02;
    if (i2c_write(address, &reg, 1)) return true; // setze Registerpointer auf "Bus voltage"
    // Task starten
    if (xTaskCreate(&ina_task, "ina", 1 * 1024, NULL, xSensors_PRIORITY - 1, NULL) != pdTRUE) return true;
    return false;
}

void ina_task(void* arg) {
    // Variablen
    TickType_t lastWakeTime = xTaskGetTickCount();
    uint8_t raw[2];
    // Loop
    while (true) {
        // warte auf nächste Messung
        vTaskDelayUntil(&lastWakeTime, *ina.rate / portTICK_PERIOD_MS);
        // Messung aus Register lesen und umrechnen
        if (i2c_read(ina.address, raw, 2)) continue;
        ina.voltage.value = ((uint16_t)(raw[0] << 8 | raw[1]) >> 3) * 0.004f; // LSB: 4 mV -> 0.004 V
        ina.voltage.timestamp = esp_timer_get_time();
        // Spannung weiterleiten an Sensortask
        xQueueSendToBack(xSensors, &ina.forward, 0);
    }
}
