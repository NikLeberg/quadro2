/*
 * File: ultrasonic.c
 * ----------------------------
 * Author: Niklaus Leuenberger
 * Date:   2020-01-04
 * ----------------------------
 * Interruptgesteuerter Ultaschallsensor
 */


/** Externe Abhängigkeiten **/

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include <string.h>


/** Interne Abhängigkeiten **/

#include "intercom.h"
#include "resources.h"
#include "sensor_types.h"
#include "bno.h"
#include "ultrasonic.h"


/** Variablendeklaration **/

typedef struct __attribute__((packed)) {
    bool level : 1;
    int64_t timestamp : 63;
} ult_event_t;

static struct {
    gpio_num_t triggerPin, echoPin;

    sensors_event_t distance;
    event_t forward;
} ult;

static QueueHandle_t xUlt;


/** Private Functions **/

/*
 * Function: ult_task
 * ----------------------------
 * Haupttask. Verwaltet Events
 *
 * void* arg: Dummy für FreeRTOS
 */
void ult_task(void* arg);

/*
 * Function: ult_interrupt
 * ----------------------------
 * Interrupt-Handler wird bei CHANGE des echoPins aufgeführt.
 * Sendet bei neg oder pos Flanke einen Event mit aktueller Zeit an Haupttask.
 *
 * void* arg: Dummy
 */
static void IRAM_ATTR ult_interrupt(void* arg);


/** Implementierung **/

bool ult_init(gpio_num_t triggerPin, gpio_num_t echoPin) {
    // Parameter speichern
    ult.triggerPin = triggerPin;
    ult.echoPin = echoPin;
    // Input Queue erstellen
    xUlt = xQueueCreate(3, sizeof(ult_event_t));
    ult.distance.type = SENSORS_ULTRASONIC;
    ult.forward.type = EVENT_INTERNAL;
    ult.forward.data = &ult.distance;
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
    gpio_install_isr_service(ESP_INTR_FLAG_IRAM);
    if (gpio_isr_handler_add(echoPin, &ult_interrupt, NULL)) return true;
    // sende Trigger und prüfe auf Antwort
    ult_event_t event = {0};
    gpio_set_level(ult.triggerPin, 1);
    vTaskDelay(1);
    gpio_set_level(ult.triggerPin, 0);
    if (xQueueReceive(xUlt, &event, 40 / portTICK_PERIOD_MS) != pdTRUE) return true;
    gpio_set_level(ult.triggerPin, 1);
    // Task starten
    if (xTaskCreate(&ult_task, "ult", 1 * 1024, NULL, xSensors_PRIORITY - 1, NULL) != pdTRUE) return true;
    return false;
}

void ult_task(void* arg) {
    // Variablen
    ult_event_t event;
    int64_t startTimestamp;
    TickType_t lastWakeTime = xTaskGetTickCount();
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
        if (xQueueReceive(xUlt, &event, 1) != pdTRUE || event.level != 1) continue;
        startTimestamp = event.timestamp;
        // Warte auf Echo LOW (bis zu 40 ms wenn nichts erkannt)
        if (xQueueReceive(xUlt, &event, 40 / portTICK_PERIOD_MS) != pdTRUE || event.level != 0) continue;
        // Abstand berechnen
        int64_t deltaT = (event.timestamp - startTimestamp) / 2;
        float distance = (deltaT * 343.46f) / (1000.0f * 1000.0f);
        if (distance > ULT_MAX_CM) {
            ++noGroundSince;
            continue;
        } else noGroundSince = 0;
        // Korrigieren gemäss aktueller Orientierung
        vector_t vector = {.x = 0.0f, .y = 0.0f, .z = -distance};
        bno_toWorldFrame(&vector, NULL);
        distance = -vector.z;
        if (distance < 0.0f) continue; // durch Rechnung ungültig geworden
        ult.distance.vector.z = distance;
        ult.distance.timestamp = event.timestamp - deltaT; // Messzeitpunkt war in der Hälfte der benötigten Zeit
        // Abstand weiterleiten an Sensortask
        xQueueSendToBack(xSensors, &ult.forward, 0);
    }
}

static void IRAM_ATTR ult_interrupt(void* arg) {
    BaseType_t woken;
    ult_event_t event;
    if (ult.echoPin < 32) { // gpio_get_level ist nicht im IRAM
        event.level = (GPIO.in >> ult.echoPin) & 0x1;
    } else {
        event.level = (GPIO.in1.data >> (ult.echoPin - 32)) & 0x1;
    }
    event.timestamp = esp_timer_get_time();
    xQueueSendToBackFromISR(xUlt, &event, &woken);
    if (woken == pdTRUE) portYIELD_FROM_ISR();
}
