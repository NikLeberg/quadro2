/*
 * File: flow.c
 * ----------------------------
 * Author: Niklaus Leuenberger
 * Date:   2020-07-20
 * ----------------------------
 * Mateksys Optical Flow Sensor basierend auf PMW3901.
 * Kommuniziert über Multiwii Serial Protokoll Version 2 (MSPv2).
 */


/** Externe Abhängigkeiten **/

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"


/** Interne Abhängigkeiten **/

#include "intercom.h"
#include "resources.h"
#include "uart.h"
#include "sensor_types.h"
#include "sensors.h"
#include "flow.h"


/** Variablendeklaration **/

#ifndef M_PI_2
    #define M_PI_2 1.57079632679489661923f
#endif

typedef struct __attribute__((packed)) {
    uint8_t start;      // $
    uint8_t version;    // X
    uint8_t direction;  // <
    uint8_t flags;      // immer 0
    uint16_t command;   // 0x1f01 = range, 0x1f02 = flow
    uint16_t size;      // Payload länge
} flow_mspv2_header_t;

typedef struct __attribute__((packed)) {
    uint8_t quality;    // 0 - 255, Optische Qualität
    int32_t motionX;    // anzahl Pixelbewegungen in X-Achse
    int32_t motionY;    // anzahl Pixelbewegungen in Y-Achse
} flow_motion_t;

typedef struct __attribute__((packed)) {
    uint8_t quality;    // 255 -> Gültig, 0 -> Ungültig
    int32_t distance;   // Distanz in mm
} flow_distance_t;

static struct {
    sensors_event_t velocity;
    sensors_event_t distance;
    event_t forward;
} flow;

static SemaphoreHandle_t xFlow;


/** Private Functions **/

/*
 * Function: flow_task
 * ----------------------------
 * Haupttask. Empfange MSP Nachrichten und verarbeite diese.
 *
 * void* arg: Dummy für FreeRTOS
 */
static void flow_task(void* arg);


/** Implementierung **/

bool flow_init(gpio_num_t uartRxPin) {
    // Sensor Weiterleitung
    flow.velocity.type = SENSORS_OPTICAL_FLOW;
    flow.distance.type = SENSORS_RANGE;
    flow.forward.type = EVENT_INTERNAL;
    // Uart einrichten
    if (uart_init(FLOW_UART, UART_PIN_NO_CHANGE, uartRxPin, 115200, xFlow)) return true;
    // Task starten
    if (xTaskCreate(&flow_task, "flow", 1 * 1024, NULL, xSensors_PRIORITY - 1, NULL) != pdTRUE) return true;
    return false;
}

static void flow_task(void* arg) {
    // Variablen
    uint8_t buffer[sizeof(flow_motion_t)];
    // Loop
    while (true) {
        uart_rxInterrupt(FLOW_UART, true);
        xSemaphoreTake(xFlow, portMAX_DELAY);
        if (uart_read(FLOW_UART, buffer, sizeof(flow_mspv2_header_t))) continue;
        flow_mspv2_header_t *header = (flow_mspv2_header_t*)buffer;
        if (header->command == 0x1f01 && header->size == sizeof(flow_distance_t)) { // Entfernung
            if (uart_read(FLOW_UART, buffer, sizeof(flow_distance_t))) continue;
            flow_distance_t *distance = (flow_distance_t*)buffer;
            if (distance->quality) {
                flow.distance.vector.x = distance->distance / 1000.0; // mm -> m
            }
            flow.distance.timestamp = esp_timer_get_time();
            flow.forward.data = &flow.distance;
        } else if (header->command == 0x1f02 && header->size == sizeof(flow_motion_t)) { // Optischer Fluss
            if (uart_read(FLOW_UART, buffer, sizeof(flow_motion_t))) continue;
            flow_motion_t *motion = (flow_motion_t*)buffer;
            flow.velocity.vector.x = motion->motionX; // ToDo: Skalieren und Integrieren
            flow.velocity.vector.y = motion->motionY; // pixel/s -> rad/s, sensor_task soll dann mit Gyro dies korrigieren
            flow.velocity.accuracy = motion->quality;
            flow.velocity.timestamp = esp_timer_get_time();
            flow.forward.data = &flow.velocity;
        }
        uart_readByte(FLOW_UART, buffer); // Prüfsumme einlesen und ignorieren
        // Daten weiterleiten
        xQueueSendToBack(xSensors, &flow.forward, 0);
    }
}
