/*
 * File: gps.h
 * ----------------------------
 * Author: Niklaus Leuenberger
 * Date:   2020-01-15
 * ----------------------------
 *   GPS u-blox Parser
 */


#pragma once

/** Externe Abhängigkeiten **/
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include <stdio.h>
#include <stdbool.h>
#include <string.h>
#include "driver/uart.h"

/** Interne Abhängigkeiten **/
#include "resources.h"
#include "sensor_types.h"


/** Variablendeklaration **/

#define GPS_UART UART_NUM_1
#define GPS_RX_BUFFER_SIZE 1024

enum gps_input_type_t {
    gps_INPUT_ECHO = 0
};

struct gps_input_t {
    struct {
        ;
    };
};

struct gps_t {
    gpio_num_t rxPin, txPin;
};
static struct gps_t gps;


/** Public Functions **/

/*
 * Function: gps_init
 * ----------------------------
 *   Initialisiert Sensor und startet zyklisches Update.
 * 
 *   gpio_num_t rxPin: UART Data-In
 *   gpio_num_t txPin: UART Data-Out
 *
 *   returns: false bei Erfolg, sonst true bei Fehler
 */
static bool gps_init(gpio_num_t rxPin, gpio_num_t txPin);


/** Private Functions **/

/*
 * Function: gps_task
 * ----------------------------
 *   Haupttask. Verwaltet Events
 *
 *   void* arg: Dummy für FreeRTOS
 */
void gps_task(void* arg);

/** Implementierung **/

static bool gps_init(gpio_num_t rxPin, gpio_num_t txPin) {
    // Parameter speichern
    gps.rxPin = rxPin;
    gps.txPin = txPin;
    // Input Queue erstellen
    //xgps_input = xQueueCreate(2, sizeof(struct gps_input_t));
    // konfiguriere UART1
    uart_config_t uart_config = {
        .baud_rate = 115200,
        .data_bits = UART_DATA_8_BITS,
        .parity = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE
    };
    uart_driver_install(GPS_UART, GPS_RX_BUFFER_SIZE * 2, 0, 0, NULL, 0);
    uart_param_config(UART_NUM_1, &uart_config);
    uart_set_pin(UART_NUM_1, gps.txPin, gps.txPin, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);
    // konfigurieren
    // NMEA deaktivieren

    // Task starten
    if (xTaskCreate(&gps_task, "gps", 1 * 1024, NULL, xSensors_PRIORITY + 1, &xGps_handle) != pdTRUE) return true;
    return false;
}

void gps_task(void* arg) {
    // Variablen
    uint8_t data[GPS_RX_BUFFER_SIZE + 1];
    uint16_t bytesRead = 0, bytePosition = 0;
    // Loop
    while (true) {
        //
        bytesRead = uart_read_bytes(UART_NUM_1, data + bytePosition, GPS_RX_BUFFER_SIZE - bytePosition, 50 / portTICK_RATE_MS);
        if (bytesRead > 0) {
            bytePosition += bytesRead;
        }
    }
}


uint8_t gps_receiveMessage(uint8_t *buffer, TickType_t timeout) {
    // Variablen
    uint16_t bytesToRead = 6, bytesRead = 0, rxPosition = 0, rxLength;
    uint8_t ckA = 0, ckB = 0;
    // Lesen
    while (true) {
        bytesRead = uart_read_bytes(UART_NUM_1, buffer + rxPosition, bytesToRead, timeout);
        if (bytesRead < 0) return 0; // Empfangsfehler
        else if (bytesRead < bytesToRead) return 0; // innerhalb Timeout zu wenig empfangen
        // UBX-Protokoll Header prüfen
        if (buffer[0] != 0xb5 || buffer[1] != 0x62) continue;
        bytesToRead = ((uint16_t) buffer[5] << 8) + buffer[4] + 2; // Payload + 2 Byte Checksum
        if (rxPosition == 0) { // erst Header gelesen, lese Payload
            rxPosition = bytesRead;
            continue;
        }
        // Checksumme berechnen und prüfen
        rxLength = rxPosition + bytesRead;
        for (uint8_t i = 2; i < (rxLength - 2); ++i) {
            ckA = ckA + buffer[i];
            ckB = ckB + ckA;
        }
        if (ckA != buffer[rxLength - 1] || ckB != buffer[rxLength]) return 0; // Prüfsummenfehler
        return rxLength;
    }
}
