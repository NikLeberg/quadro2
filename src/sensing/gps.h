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
#include "freertos/semphr.h"
#include <stdio.h>
#include <stdbool.h>
#include <string.h>
#include "driver/uart.h" // uart_reg.h uart_struct.h
#include "esp_log.h"

/** Interne Abhängigkeiten **/
#include "resources.h"
#include "sensor_types.h"


/** Variablendeklaration **/

#define GPS_UART UART_NUM_1

enum gps_input_type_t {
    GPS_INPUT_DUMMY = 0
};

struct gps_input_t {
    uint8_t *frame; // muss mit free() freigegeben werden
    uint8_t length;
    int64_t timestamp;
};

struct gps_t {
    gpio_num_t rxPin, txPin;
    SemaphoreHandle_t txSemphr;
};
static struct gps_t gps;

extern uart_dev_t UART0;
extern uart_dev_t UART1;
extern uart_dev_t UART2;
DRAM_ATTR uart_dev_t* const UART[UART_NUM_MAX] = {&UART0, &UART1, &UART2};


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

static uint32_t gps_receiveUBX(uint8_t *buffer, TickType_t timeout);
static bool gps_sendUBX(uint8_t *buffer, uint8_t length, bool aknowledge, TickType_t timeout);
static void IRAM_ATTR gps_interrupt(void* arg);
static bool gps_uart_baudrate(uint32_t baud_rate);
static bool gps_uart_enable(gpio_num_t txPin, gpio_num_t rxPin);

/** Implementierung **/

static bool gps_init(gpio_num_t rxPin, gpio_num_t txPin) {
    // Parameter speichern
    gps.rxPin = rxPin;
    gps.txPin = txPin;
    // Input Queue erstellen
    xGps_input = xQueueCreate(2, sizeof(struct gps_input_t));
    gps.txSemphr = xSemaphoreCreateMutex();
    xSemaphoreGive(gps.txSemphr);
    //
    gps_uart_enable(gps.txPin, gps.txPin);

    // konfiguriere UART1 mit ESP-IDF default Treiber aber lösche diesen danach?
    uart_config_t uart_config = {
        .baud_rate = 9600,
        .data_bits = UART_DATA_8_BITS,
        .parity = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE
    };
    uart_driver_install(GPS_UART, 2 * 128, 0, 0, NULL, 0);
    uart_param_config(GPS_UART, &uart_config);
    uart_set_pin(GPS_UART, gps.txPin, gps.txPin, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);
    // konfigurieren
    // UBX-CFG-PRT: NMEA deaktivieren, UART Baudrate auf 256000 setzen
    const char msgPort[] = {0xB5, 0x62, 0x06, 0x00, 0x14, 0x00, 0x01,
                            0x00, 0x00, 0x00, 0xC0, 0x08, 0x00, 0x00,
                            0x00, 0xE8, 0x03, 0x00, 0x01, 0x00, 0x01,
                            0x00, 0x00, 0x00, 0x00, 0x00, 0xd0, 0xf8};
    gps_sendUBX(msgPort, sizeof(msgPort), true, 1000 / portTICK_PERIOD_MS); // AK aktivieren aber nicht auswerten, so wird auf tx done gewartet
    // eigener UART auf neue Baudrate setzen und Buffer löschen
    uart_set_baudrate(GPS_UART, 256000);
    // UBX-CFG-PMS: Power auf Balanced setzen
    const char msgPower[] = {0xB5, 0x62, 0x06, 0x86, 0x08, 0x00, 0x00, 0x01,
                             0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x95, 0x61};
    if (gps_sendUBX(msgPower, sizeof(msgPower), true, portMAX_DELAY)) return true; // NAK Empfangen
    // UBX-CFG-MSG: Zu empfangende Nachrichten setzen, UBX-NAV-PVT (0x01 0x07)
    const char msgMessages[] = {0xB5, 0x62, 0x06, 0x01, 0x03, 0x00, 0x01, 0x07, 0x01, 0x1C, 0x5A};
    if (gps_sendUBX(msgMessages, sizeof(msgMessages), true, 1000 / portTICK_PERIOD_MS)) return true; // NAK Empfangen
    // UBX-CFG-RATE: Daten-Rate auf 10Hz setzen
    const char msgRate[] = {0xB5, 0x62, 0x06, 0x08, 0x06, 0x00, 0x64, 0x00, 0x01, 0x00, 0x00, 0x00, 0x79, 0x10};
    if (gps_sendUBX(msgRate, sizeof(msgRate), true, 1000 / portTICK_PERIOD_MS)) return true; // NAK Empfangen

    // Task starten
    if (xTaskCreate(&gps_task, "gps", 1 * 1024, NULL, xSensors_PRIORITY + 1, &xGps_handle) != pdTRUE) return true;
    return false;
}

void gps_task(void* arg) {
    // Variablen
    // Loop
    while (true) {
        //
        //gps_receiveUBX(data, 10000 / portTICK_PERIOD_MS);
    }
}

/*
 * Sync1 (0xb5) | Sync 2 (0x62) | Class | ID | Size LSB | Size MSB | Payload ... | CK_A | CK_B
 * 0            | 1             | 2     | 3  | 4        | 5        | 6       ... |
 */
static uint32_t gps_receiveUBX(uint8_t *buffer, TickType_t timeout) {
    // Variablen
    uint32_t bytesToRead = 1, rxPosition = 0, rxLength;
    int32_t bytesRead = 0;
    uint8_t ckA = 0, ckB = 0;
    TickType_t startTick = xTaskGetTickCount();
    // Lesen
    while (true) {
        // 1. Suche nach UBX-Sync-Header
        // 2. Lese Payload länge
        // 3. Empfange Payload
        // 4. Prüfsumme überprüfen
        if (timeout != portMAX_DELAY) timeout -= xTaskGetTickCount() - startTick;
        bytesRead = uart_read_bytes(UART_NUM_0, buffer + rxPosition, bytesToRead, timeout);
        ESP_LOGD("gps", "Bytes %u von %u gelesen nach Position %u:", bytesRead, bytesToRead, rxPosition);
        ESP_LOG_BUFFER_HEXDUMP("gps", buffer, bytesRead, ESP_LOG_DEBUG);
        if (bytesRead < 0) return 0; // Empfangsfehler
        if (bytesRead < bytesToRead) return 0; // innerhalb Timeout zu wenig empfangen
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

static bool gps_sendUBX(uint8_t *buffer, uint8_t length, bool aknowledge, TickType_t timeout) {
    TickType_t startTick = xTaskGetTickCount();
    // Zustand prüfen
    if (length > UART_FIFO_LEN) return true; // zu gross für FIFO
    if (xSemaphoreTake(gps.txSemphr, timeout) == pdFALSE) return true; // tx busy, Timeout
    if (UART[GPS_UART]->status.txfifo_cnt) return true; // FIFO nicht leer
    // Prüfsumme rechnen
    if (!(buffer[length - 2] || buffer[length - 1])) {
        for (uint8_t i = 2; i < (length - 2); ++i) {
            buffer[length - 2] = buffer[length - 2] + buffer[i];
            buffer[length - 1] = buffer[length - 1] + buffer[length - 2];
        }
    }
    // in FIFO schreiben
    for (uint8_t i = 0; i < length; ++i) {
        WRITE_PERI_REG(UART_FIFO_AHB_REG(GPS_UART), buffer[i]);
    }
    // AK / NAK
    if (!aknowledge) return false; // kein AK erforderlich
    uint8_t *response = NULL;
    timeout -= xTaskGetTickCount() - startTick;
    if (gps_receiveUBX(response, timeout) < 10); // Timeout oder ungültige Antwort
    if (response[3] && response[6] == buffer[2] && response[7] == buffer[3]) return false; // AK ok
    return true;
}

static void IRAM_ATTR gps_interrupt(void* arg) {
    uart_dev_t *uart = UART[GPS_UART];
    uint32_t status = uart->int_st.val;
    uart->int_clr.val = UART_INTR_MASK; // alle Interrupts zurücksetzen
    if (status & UART_TX_DONE_INT_ST_M) { // tx beendet, entsperren
        BaseType_t woken = pdFALSE;
        xSemaphoreGiveFromISR(gps.txSemphr, &woken);
        if (woken == pdTRUE) portYIELD_FROM_ISR();
    } else if (status & UART_RXFIFO_TOUT_INT_ST_M ||
               status & UART_RXFIFO_OVF_INT_ST_M) { // rx-Frame erhalten oder rx-FIFO Überlauf
        // FIFO einlesen als UBX-Frame
        uint16_t frameSize = 0; // erwartete Länge
        uint8_t framePos = 0, class = 0x00, id = 0x00, *frame = NULL, ckA = 0, ckB = 0;
        while (uart->status.rxfifo_cnt) {
            uint8_t c = uart->fifo.rw_byte;
            switch (framePos) {
                case (0): { // Sync 1
                    if (c == 0xb5) framePos = 1;
                    continue;
                }
                case (1): { // Sync 2
                    if (c == 0x62) ++framePos;
                    else framePos = 0;
                    continue;
                }
                case (2): { // Class
                    class = c;
                    ++framePos;
                    continue;
                }
                case (3): { // Id
                    id = c;
                    ++framePos;
                    continue;
                }
                case (4): { // Size LSB
                    frameSize = c;
                    ++framePos;
                    continue;
                }
                case (5): { // Size MSB
                    frameSize |= ((uint16_t) c << 8);
                    frameSize += 6 + 2; // Header + Prüfsumme hinzufügen
                    if (frameSize > UART_FIFO_LEN) { // zu gross für FIFO, verwerfen
                        framePos = 0;
                        continue;
                    }
                    // Speicher für Frame allozieren und Header rekonstruieren;
                    frame = (uint8_t*) calloc(frameSize, sizeof(uint8_t));
                    frame[0] = 0xb5;
                    frame[1] = 0x62;
                    frame[2] = class;
                    frame[3] = id;
                    frame[4] = 0x0f & frameSize;
                    frame[5] = frameSize >> 8;
                    ++framePos;
                    continue;
                }
                default: { // Framepayload füllen
                    frame[framePos] = c;
                    ++framePos;
                    continue;
                }
            }
        }
        // empfangen == erwartet?
        if (framePos != frameSize) return; // Frame verwerfen
        // Prüfsumme Ok?
        for (uint8_t i = 2; i < (frameSize - 2); ++i) {
            ckA = ckA + frame[i];
            ckB = ckB + ckA;
        }
        if (ckA != frame[frameSize - 2] || ckB != frame[frameSize - 1]) return; // Frame verwerfen
        // in Queue einreihen
        struct gps_input_t input;
        input.frame = frame;
        input.length = frameSize;
        input.timestamp = esp_timer_get_time();
        // ToDo
    } else if (status & UART_FRM_ERR_INT_ST_M) { // x-Frame Error
        // FIFO zurücksetzen
        while (uart->status.rxfifo_cnt) {
            READ_PERI_REG(uart->fifo.rw_byte);
        }
        uart->conf0.rxfifo_rst = 1;
    }
}

static bool gps_uart_baudrate(uint32_t baud_rate) {
    int uart_clk_freq;
    if (UART[GPS_UART]->conf0.tick_ref_always_on == 0) {
        uart_clk_freq = REF_CLK_FREQ;
    } else {
        uart_clk_freq = esp_clk_apb_freq();
    }
    uint32_t clk_div = (((uart_clk_freq) << 4) / baud_rate);
    if (clk_div < 16) return true;
    else {
        UART[GPS_UART]->clk_div.div_int = clk_div >> 4;
        UART[GPS_UART]->clk_div.div_frag = clk_div & 0xf;
    }
    return false;
}

static bool gps_uart_enable(gpio_num_t txPin, gpio_num_t rxPin) {
    uart_dev_t *uart = UART[GPS_UART];
    // Aktivieren
    periph_module_enable(GPS_UART + 1);
    // Data-Bits
    uart->conf0.bit_num = UART_DATA_8_BITS;
    // Parity
    uart->conf0.parity_en = 0;
    // Stop-Bits
    uart->conf0.stop_bit_num = UART_STOP_BITS_1;
    // Flow-Control
    uart->conf0.tx_flow_en = 0;
    uart->conf1.rx_flow_en = 0;
    // Baud
    if (gps_uart_baudrate(9600)) return true;
    // Pins
    if (uart_set_pin(GPS_UART, txPin, rxPin, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE)) return true;
    // Interrupt Handler registrieren
    if (esp_intr_alloc(ETS_UART0_INTR_SOURCE + GPS_UART, ESP_INTR_FLAG_IRAM, gps_interrupt, NULL, NULL)) return true;
    // Interrupts aktivieren
    uart->int_clr.val = UART_INTR_MASK;
    uart->conf1.rx_tout_thrhd = 100;
    uart->conf1.rx_tout_en = 1;
    uart->int_ena.val = UART_TX_DONE_INT_ST_M | UART_RXFIFO_TOUT_INT_ENA_M | UART_RXFIFO_OVF_INT_ENA_M | UART_FRM_ERR_INT_ENA_M;

    return false;
}