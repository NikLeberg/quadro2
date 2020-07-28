/*
 * File: flow.c
 * ----------------------------
 * Author: Niklaus Leuenberger
 * Date:   2020-07-20
 * ----------------------------
 * Mateksys Optical Flow Sensor basierend auf PMW3901.
 * Kommuniziert über Multiwii Serial Protokoll Version 2 (MSPv2).
 * - Sendet mit ~10Hz Flow Rate in Pixel
 * - ebenfalls sendet es Lidar Entfernung (max. 2 m)
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
    uint16_t id;        // 0x1f01 = range, 0x1f02 = flow
    uint16_t size;      // Payload länge
} flow_mspv2_header_t;

#define FLOW_MOTION_ID  0x1f02

typedef struct __attribute__((packed)) {
    uint8_t quality;    // 0 - 255, Optische Qualität
    int32_t motionX;    // anzahl Pixelbewegungen in X-Achse
    int32_t motionY;    // anzahl Pixelbewegungen in Y-Achse
} flow_motion_t;

#define FLOW_RANGE_ID  0x1f01

typedef struct __attribute__((packed)) {
    uint8_t quality;    // 255 -> Gültig, 0 -> Ungültig
    int32_t distance;   // Distanz in mm
} flow_distance_t;

static struct {
    sensors_event_t velocity;
    sensors_event_t distance;
    event_t forward;
} flow;

static QueueHandle_t xFlow;


/** Private Functions **/

/*
 * Function: flow_task
 * ----------------------------
 * Haupttask. Empfange MSP Nachrichten und verarbeite diese.
 *
 * void* arg: Dummy für FreeRTOS
 */
static void flow_task(void* arg);

// ToDo
static void flow_processRange(flow_distance_t *data, int64_t timestamp);
static void flow_processMotion(flow_motion_t *data, int64_t timestamp);


/** Implementierung **/

bool flow_init(gpio_num_t uartRxPin) {
    // Sensor Weiterleitung
    flow.velocity.type = SENSORS_OPTICAL_FLOW;
    flow.distance.type = SENSORS_LIDAR;
    flow.forward.type = EVENT_INTERNAL;
    // Uart einrichten
    xFlow = xQueueCreate(2, sizeof(int64_t));
    if (uart_init(FLOW_UART, UART_PIN_NO_CHANGE, uartRxPin, 115200, xFlow)) return true;
    // Task starten
    if (xTaskCreate(&flow_task, "flow", 3 * 1024, NULL, xSensors_PRIORITY - 1, NULL) != pdTRUE) return true;
    return false;
}

static void flow_task(void* arg) {
    // Variablen
    uint8_t buffer[sizeof(flow_motion_t)];
    int64_t timestamp;
    uint8_t c, step = 0, crc = 0;
    uint16_t id, length, pos = 0;
    // Loop
    while (true) {
        uart_rxInterrupt(FLOW_UART, true);
        xQueueReceive(xFlow, &timestamp, portMAX_DELAY);
        while (uart_rxAvailable(FLOW_UART)) {
            c = uart_read(FLOW_UART);
            switch (step) {
                case 0: // Start
                    if (c == '$') {
                        crc = 0;
                        pos = 0;
                        step++; // ToDo: mitzählen wie viel übersprungen wurde und timestamp neu rechnen
                    }
                    break;
                case 1: // Version
                    if (c == 'X') step++;
                    else step = 0;
                    break;
                case 2: // Direction
                    if (c == '<') step++;
                    else step = 0;
                    break;
                case 3: // Flags
                    if (c == 0) step++;
                    else step = 0;
                    break;
                case 4: // Id LSB
                    id = c;
                    step++;
                    break;
                case 5: // Id MSB
                    id |= (c << 8);
                    if (id == FLOW_RANGE_ID || id == FLOW_MOTION_ID) step++;
                    else step = 0;
                    break;
                case 6: // Size LSB
                    length = c;
                    step++;
                    break;
                case 7: // Size MSB
                    length |= (c << 8);
                    if (length <= sizeof(flow_motion_t)) step++;
                    else step = 0;
                    break;
                case 8: // Payload
                    buffer[pos++] = c;
                    if (pos == length) step++;
                    break;
                case 9: // Checksum
                    if (c == crc) {
                        if (id == FLOW_RANGE_ID) {
                            flow_processRange((flow_distance_t*)&buffer[0], timestamp);
                        } else if (id == FLOW_MOTION_ID) {
                            flow_processMotion((flow_motion_t*)&buffer[0], timestamp);
                        }
                    }
                    step = 0;
                    break;
            }
            if (step > 3) { // Prüfsumme rechnen
                crc ^= c;
                for (uint8_t i = 0; i < 8; i++) {
                    if (crc & 0x80) {
                        crc = (crc << 1) ^ 0xD5;
                    } else {
                        crc = crc << 1;
                    }
                }
            }
        }
    }
}

static void flow_processRange(flow_distance_t *data, int64_t timestamp) {
    if (data->quality != 255) return;
    flow.distance.value = data->distance / 1000.0; // mm -> m
    flow.distance.timestamp = timestamp;
    flow.forward.data = &flow.distance;
    xQueueSendToBack(xSensors, &flow.forward, 0);
}

static void flow_processMotion(flow_motion_t *data, int64_t timestamp) {
    flow.velocity.vector.x = data->motionX; // ToDo: Skalieren, mittels Gyro Daten zu kalibrieren
    flow.velocity.vector.y = data->motionY; // pixel/s -> rad/s, sensor_task soll dann mit Gyro dies korrigieren
    flow.velocity.accuracy = data->quality;
    flow.velocity.timestamp = timestamp;
    flow.forward.data = &flow.velocity;
    xQueueSendToBack(xSensors, &flow.forward, 0);
}
