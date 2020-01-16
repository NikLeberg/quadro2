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
#include "esp_log.h"

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
        .baud_rate = 256000,
        .data_bits = UART_DATA_8_BITS,
        .parity = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE
    };
    uart_driver_install(GPS_UART, GPS_RX_BUFFER_SIZE * 2, 0, 0, NULL, 0);
    uart_param_config(UART_NUM_1, &uart_config);
    uart_set_pin(UART_NUM_1, gps.txPin, gps.txPin, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);
    // konfigurieren
    // UBX-CFG-PRT: NMEA deaktivieren, UART Baudrate auf 256000 setzen
    const char msgPort[] = {0xB5, 0x62, 0x06, 0x00, 0x14, 0x00, 0x01,
                            0x00, 0x00, 0x00, 0xC0, 0x08, 0x00, 0x00,
                            0x00, 0xE8, 0x03, 0x00, 0x01, 0x00, 0x01,
                            0x00, 0x00, 0x00, 0x00, 0x00, 0xd0, 0xf8};
    gps_sendUBX(msgPort, sizeof(msgPort), false, 1000 / portTICK_PERIOD_MS);
    uart_wait_tx_done(GPS_UART, 1000 / portTICK_PERIOD_MS);
    // eigener UART auf neue Baudrate setzen und Buffer löschen
    uart_set_baudrate(GPS_UART, 256000);
    uart_flush(GPS_UART);
    // UBX-CFG-PMS: Power auf Balanced setzen
    const char msgPower[] = {0xB5, 0x62, 0x06, 0x86, 0x08, 0x00, 0x00, 0x01,
                            0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x95, 0x61};
    if (gps_sendUBX(msgPort, sizeof(msgPort), false, 1000 / portTICK_PERIOD_MS) ) return true; // NAK Empfangen
    // AK / NAK empfangen
    // UBX-CFG-MSG: Zu empfangende Nachrichten setzen
    const char msgMessages[] = {};
    // AK / NAK empfangen
    // ?: Rate auf 10Hz
    uint8_t msgRate[] = {};
    // AK / NAK empfangen

    // Task starten
    if (xTaskCreate(&gps_task, "gps", 1 * 1024, NULL, xSensors_PRIORITY + 1, &xGps_handle) != pdTRUE) return true;
    return false;
}

void gps_task(void* arg) {
    // Variablen
    uint8_t data[GPS_RX_BUFFER_SIZE + 1];
    data[GPS_RX_BUFFER_SIZE] = '\0';
    uint16_t bytesRead = 0, bytePosition = 0;
    // Loop
    while (true) {
        //
        gps_receiveUBX(data, 10000);
    }
}

/*
 * Sync1 (0xb5) | Sync 2 (0x62) | Class | ID | Size LSB | Size MSB | Payload ... | CK_A | CK_B
 * 0            | 1             | 2     | 3  | 4        | 5        | 6       ... |
 */
uint32_t gps_receiveUBX(uint8_t *buffer, TickType_t timeout) {
    // Variablen
    uint32_t bytesToRead = 1, bytesRead = 0, rxPosition = 0, rxLength;
    uint8_t ckA = 0, ckB = 0;
    TickType_t startTick = xTaskGetTickCount();
    // Lesen
    while (true) {
        // 1. Suche nach UBX-Sync-Header
        // 2. Lese Payload länge
        // 3. Empfange Payload
        // 4. Prüfsumme überprüfen
        timeout -= xTaskGetTickCount() - startTick;
        bytesRead = uart_read_bytes(GPS_UART, buffer + rxPosition, bytesToRead, timeout);
        ESP_LOGD("gps", "Bytes %u von %u gelesen nach Position %u: %s", bytesRead, bytesToRead, rxPosition, (char*) buffer);
        if (bytesRead < 0) return 0; // Empfangsfehler
        else if (bytesRead < bytesToRead) return 0; // innerhalb Timeout zu wenig empfangen
        // UBX-Protokoll Header prüfen
        if (buffer[0] != 0xb5) continue;
        rxPosition += bytesRead;
        if (rxPosition == 1) { // Sync gefunden, Rest des Headers empfangen
            bytesToRead = 5;
            continue;
        } else if (rxPosition == 2) { // Header komplett, Payload-Länge ermitteln
            if (buffer[1] != 0x62) return 0; // Fehler
            bytesToRead = ((uint16_t) buffer[5] << 8) + buffer[4] + 2; // Payload + 2 Byte Checksum
            continue;
        }
        // Checksumme berechnen und prüfen
        rxLength = rxPosition + bytesRead;
        for (uint8_t i = 2; i < (rxLength - 2); ++i) {
            ckA = ckA + buffer[i];
            ckB = ckB + ckA;
        }
        if (ckA != buffer[rxLength - 2] || ckB != buffer[rxLength - 1]) return 0; // Prüfsummenfehler
        return rxLength;
    }
}


bool gps_sendUBX(const char *buffer, uint32_t length, bool aknowledge, TickType_t timeout) {
    TickType_t startTick = xTaskGetTickCount();
    if (uart_write_bytes(GPS_UART, buffer, length) < 0) return true; // Sendefehler
    if (!aknowledge) return false; // keine Antwort erforderlich
    timeout -= xTaskGetTickCount() - startTick;
    uart_wait_tx_done(GPS_UART, timeout); // warte bis alles gesendet wurde
    uint8_t msg[10];
    timeout -= xTaskGetTickCount() - startTick;
    uint32_t rxLength = gps_receiveUBX(msg, timeout);
    if (rxLength < 10) return true; // fehlerhafte Antwort
    if (msg[3] && msg[6] == buffer[2] && msg[7] == buffer[3]) return false; // AK erhalten und Class & ID übereinstimmend
    return true;
}
