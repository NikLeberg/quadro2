/*
 * File: ultrasonic.h
 * ----------------------------
 * Author: Niklaus Leuenberger
 * Date:   2020-01-04
 * ----------------------------
 *   Interruptgesteuerten Ultaschallsensor
 */


#pragma once

/** Externe Abhängigkeiten **/
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include <stdio.h>
#include <stdbool.h>
#include <string.h>
#include "driver/gpio.h"

/** Interne Abhängigkeiten **/
#include "resources.h"
#include "sensor_types.h"


/** Variablendeklaration **/

#define ULT_DATA_RATE_MS 100 // 100 ms
#define ULT_MAX_CM 5.0f // 5 m
#define ULT_NO_GROUND_COUNT 256
#define ULT_NO_GROUND_WAIT 1000 // 1 s

enum ult_input_type_t {
    ULT_INPUT_ECHO = 0
};

enum ult_task_step_t {
    ULT_STEP_ERROR = 0,
    ULT_STEP_START,
    ULT_STEP_AWAIT_HIGH,
    ULT_STEP_AWAIT_LOW
};

struct ult_input_t {
    struct {
        int64_t timestamp;
        bool level;
    };
};

struct ult_data_t {
    uint32_t distance; // Distanz in cm
};

struct ult_t {
    gpio_num_t triggerPin, echoPin;
    TickType_t updateRate;
};
static struct ult_t ult;


/** Public Functions **/

/*
 * Function: ult_init
 * ----------------------------
 *   Initialisiert Sensor und startet zyklisches Update.
 *
 *   TickType_t updateFrequency: Update-Frequenz
 *   gpio_num_t triggerPin: Trigger Pin, Output
 *   gpio_num_t echoPin: Echo Pin, Input
 *
 *   returns: false bei Erfolg, sonst true wenn Sensor nicht vorhanden
 */
static bool ult_init(gpio_num_t triggerPin, gpio_num_t echoPin);


/** Private Functions **/

/*
 * Function: ult_task
 * ----------------------------
 *   Haupttask. Verwaltet Events
 *
 *   void* arg: Dummy für FreeRTOS
 */
void ult_task(void* arg);

/*
 * Function: ult_interrupt
 * ----------------------------
 *   Interrupt-Handler wird bei CHANGE des echoPins aufgeführt.
 *   Sendet bei neg oder pos Flanke einen Event mit aktueller Zeit an Haupttask.
 *
 *   void* arg: Dummy
 */
static void IRAM_ATTR ult_interrupt(void* arg);

/** Implementierung **/

static bool ult_init(gpio_num_t triggerPin, gpio_num_t echoPin) {
    // Parameter speichern
    ult.triggerPin = triggerPin;
    ult.echoPin = echoPin;
    // Input Queue erstellen
    xUlt_input = xQueueCreate(2, sizeof(struct ult_input_t));
    // konfiguriere Pins und aktiviere Interrupts
    gpio_config_t gpioConfig;
    gpioConfig.pin_bit_mask = ((1ULL) << triggerPin);
    gpioConfig.mode = GPIO_MODE_OUTPUT;
    gpioConfig.pull_up_en = GPIO_PULLUP_DISABLE;
    gpioConfig.pull_down_en = GPIO_PULLDOWN_DISABLE;
    gpioConfig.intr_type = GPIO_INTR_DISABLE;
    gpio_config(&gpioConfig); // Trigger Pin
    gpio_set_level(triggerPin, 1);
    gpioConfig.pin_bit_mask = ((1ULL) << echoPin);
    gpioConfig.mode = GPIO_MODE_INPUT;
    gpioConfig.pull_up_en = GPIO_PULLUP_DISABLE;
    gpioConfig.pull_down_en = GPIO_PULLDOWN_DISABLE;
    gpioConfig.intr_type = GPIO_INTR_ANYEDGE;
    gpio_config(&gpioConfig); // Echo Pin
    gpio_install_isr_service(ESP_INTR_FLAG_EDGE | ESP_INTR_FLAG_IRAM | ESP_INTR_FLAG_LOWMED);
    if (gpio_isr_handler_add(echoPin, &ult_interrupt, NULL)) return true;
    // Sende Trigger und warte auf Antwort
    gpio_set_level(ult.triggerPin, 1);
    vTaskDelay(1);
    gpio_set_level(ult.triggerPin, 0);
    struct ult_input_t dummy;
    if (xQueueReceive(xUlt_input, &dummy, 40 / portTICK_PERIOD_MS) != pdTRUE) return true;
    gpio_set_level(ult.triggerPin, 1);
    // Task starten
    if (xTaskCreatePinnedToCore(&ult_task, "ult", 1 * 1024, NULL, xSensors_PRIORITY + 1, &xUlt_handle, xPortGetCoreID()) != pdTRUE) return true;
    return false;
}

void ult_task(void* arg) {
    // Variablen
    struct ult_input_t input;
    int64_t startTimestamp = 0;
    TickType_t lastWakeTime = 0;
    uint16_t noGroundSince = 0; // hochzählen wie lange schon über maximalem Abstand
    // Loop
    while (true) {
        // Setze Trigger HIGH und warte auf nächsten Messintervall
        gpio_set_level(ult.triggerPin, 1);
        if (noGroundSince > ULT_NO_GROUND_COUNT) { // verringere Frequenz wenn kein Boden in messbarer Nähe
            vTaskDelayUntil(&lastWakeTime, ULT_NO_GROUND_WAIT / portTICK_PERIOD_MS);
        } else {
            vTaskDelayUntil(&lastWakeTime, ULT_DATA_RATE_MS / portTICK_PERIOD_MS);
        }
        // Starte Messung, Trigger HIGH -> LOW
        gpio_set_level(ult.triggerPin, 0);
        // Warte auf Echo HIGH
        if (xQueueReceive(xUlt_input, &input, 1) != pdTRUE || input.level != 1) continue;
        startTimestamp = input.timestamp;
        // Warte auf Echo LOW (bis zu 40 ms wenn nichts erkannt)
        if (xQueueReceive(xUlt_input, &input, 40 / portTICK_PERIOD_MS) != pdTRUE || input.level != 0) continue;
        // Abstand berechnen
        int64_t deltaT = input.timestamp - startTimestamp;
        float distance = (deltaT * 343.46f) / (2.0f * 1000.0f * 1000.0f);
        if (distance > ULT_MAX_CM) {
            ++noGroundSince;
            continue;
        } else noGroundSince = 0;
        // Abstand weiterleiten an Sensortask
        struct sensors_input_t forward;
        forward.type = SENSORS_ULTRASONIC;
        forward.timestamp = input.timestamp;
        forward.distance = distance;
        xQueueSendToBack(xSensors_input, &forward, 0);
    }
}

static void IRAM_ATTR ult_interrupt(void* arg) {
    BaseType_t woken;
    struct ult_input_t input;
    input.level = gpio_get_level(ult.echoPin);
    input.timestamp = esp_timer_get_time();
    xQueueSendToBackFromISR(xUlt_input, &input, &woken);
    if (woken == pdTRUE) portYIELD_FROM_ISR();
}
