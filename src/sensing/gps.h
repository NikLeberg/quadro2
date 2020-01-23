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
#include "driver/gpio.h"
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

static uint8_t gps_receiveUBX(uint8_t **buffer, TickType_t timeout);
static bool gps_sendUBX(uint8_t *buffer, uint8_t length, bool aknowledge, TickType_t timeout);
static void IRAM_ATTR gps_interrupt(void* arg);
static bool gps_uart_baudrate(uint32_t baud_rate);
static bool gps_uart_enable(gpio_num_t txPin, gpio_num_t rxPin);
static uint32_t gps_uart_auto_baud();
static void IRAM_ATTR gps_uart_rx_fifo_reset();

/** Implementierung **/

static bool gps_init(gpio_num_t rxPin, gpio_num_t txPin) {
    // Parameter speichern
    gps.rxPin = rxPin;
    gps.txPin = txPin;
    // Input Queue erstellen
    xGps_input = xQueueCreate(2, sizeof(struct gps_input_t));
    gps.txSemphr = xSemaphoreCreateMutex();
    xSemaphoreGive(gps.txSemphr);
    // eigener UART Treiber installieren
    gps_uart_enable(gps.txPin, gps.rxPin);
    // konfigurieren
    // UBX-CFG-RST: u-Blox Chip zurücksetzen - Hardware Reset
    uint8_t msgReset[] = {0xB5, 0x62, 0x06, 0x04, 0x04, 0x00, 0xFF, 0xFF, 0x00, 0x00, 0x0C, 0x5D};
    gps_sendUBX(msgReset, sizeof(msgReset), false, 1000 / portTICK_PERIOD_MS);
    vTaskDelay(3000 / portTICK_PERIOD_MS);
    // UBX-CFG-PRT: NMEA deaktivieren, UART Baudrate auf 256000 setzen
    uint8_t msgPort[] = {0xB5, 0x62, 0x06, 0x00, 0x14, 0x00, 0x01,
                         0x00, 0x00, 0x00, 0xC0, 0x08, 0x00, 0x00,
                         0x00, 0xE8, 0x03, 0x00, 0x01, 0x00, 0x01,
                         0x00, 0x00, 0x00, 0x00, 0x00, 0xd0, 0xf8};
    gps_sendUBX(msgPort, sizeof(msgPort), true, 1000 / portTICK_PERIOD_MS); // AK aktivieren aber nicht auswerten, so wird auf tx done gewartet
    // eigener UART auf neue Baudrate setzen und Buffer löschen
    gps_uart_baudrate(256000);
    // UBX-CFG-PMS: Power auf Balanced setzen
    uint8_t msgPower[] = {0xB5, 0x62, 0x06, 0x86, 0x08, 0x00, 0x00, 0x01,
                          0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x95, 0x61};
    if (gps_sendUBX(msgPower, sizeof(msgPower), true, portMAX_DELAY)) return true; // NAK Empfangen
    // UBX-CFG-MSG: Zu empfangende Nachrichten setzen, UBX-NAV-PVT (0x01 0x07)
    uint8_t msgMessages[] = {0xB5, 0x62, 0x06, 0x01, 0x03, 0x00, 0x01, 0x07, 0x01, 0x1C, 0x5A};
    if (gps_sendUBX(msgMessages, sizeof(msgMessages), true, 1000 / portTICK_PERIOD_MS)) return true; // NAK Empfangen
    // UBX-CFG-RATE: Daten-Rate auf 10Hz setzen
    uint8_t msgRate[] = {0xB5, 0x62, 0x06, 0x08, 0x06, 0x00, 0x64, 0x00, 0x01, 0x00, 0x00, 0x00, 0x79, 0x10};
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
static uint8_t gps_receiveUBX(uint8_t **buffer, TickType_t timeout) {
    // Variablen
    struct gps_input_t input;
    // Test
    if (!buffer) return 0;
    // Empfangen aus Queue
    if (xQueueReceive(xGps_input, &input, timeout) == pdFALSE) {
        *buffer = NULL;
        return 0;
    } else {
        *buffer = input.frame;
        return input.length;
    }
}

static bool gps_sendUBX(uint8_t *buffer, uint8_t length, bool aknowledge, TickType_t timeout) {
    TickType_t startTick = xTaskGetTickCount();
    // Zustand prüfen
    //ESP_LOG_BUFFER_HEXDUMP("gps", buffer, length, ESP_LOG_DEBUG);
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
        UART[GPS_UART]->fifo.rw_byte = buffer[i];
        //WRITE_PERI_REG(UART_FIFO_AHB_REG(GPS_UART), buffer[i]);
    }
    // tx done Interrupt aktivieren
    UART[GPS_UART]->int_ena.tx_done = 1;
    // AK / NAK
    if (!aknowledge) return false; // kein AK erforderlich
    uint8_t *response;
    bool akReceived = false;
    timeout -= xTaskGetTickCount() - startTick;
    if (gps_receiveUBX(&response, timeout) == 10) { // gültige Länge, AK, Class & Id übereinstimmend
        if (response[3] && response[6] == buffer[2] && response[7] == buffer[3]) {
            akReceived = true;
        }
    }
    if (response) free(response);
    if (akReceived) return false;
    else return true; // Ungültige Antwort oder Timeout
}

static void IRAM_ATTR gps_interrupt(void* arg) {
    uart_dev_t *uart = UART[GPS_UART];
    BaseType_t woken = pdFALSE;
    uint32_t status = uart->int_st.val;
    uart->int_clr.val = status; // aktive Interrupts zurücksetzen
    if (status & UART_TX_DONE_INT_ST_M) { // tx beendet, entsperren
        uart->int_ena.tx_done = 0; // interrupt deaktivieren
        xSemaphoreGiveFromISR(gps.txSemphr, &woken);
        if (woken == pdTRUE) portYIELD_FROM_ISR();
    } else if (status & UART_RXFIFO_TOUT_INT_ST_M || // rx-Frame erhalten
               status & UART_RXFIFO_FULL_INT_ST_M || // rx-FIFO bald voll
               status & UART_RXFIFO_OVF_INT_ST_M) {  // rx-FIFO Überlauf
        // FIFO einlesen als UBX-Frame
        int16_t frameSize = 0; // erwartete Länge
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
        // etwas empfangen?
        if (!framePos && !frameSize) return; // nichts empfangen
        // empfangen == erwartet?
        if (framePos != frameSize) {
            free(frame);
            return; // Frame verwerfen
        }
        // Prüfsumme Ok?
        for (uint8_t i = 2; i < (frameSize - 2); ++i) {
            ckA = ckA + frame[i];
            ckB = ckB + ckA;
        }
        if (ckA != frame[frameSize - 2] || ckB != frame[frameSize - 1]) {
            free(frame);
            return; // Frame verwerfen
        }
        // in Queue einreihen
        struct gps_input_t input;
        input.frame = frame;
        input.length = frameSize;
        input.timestamp = esp_timer_get_time();
        xQueueSendToBackFromISR(xGps_input, &input, &woken);
        if (woken == pdTRUE) portYIELD_FROM_ISR();
    } else if (status & UART_FRM_ERR_INT_ST_M) { // rx-Frame Error
        // FIFO zurücksetzen
        gps_uart_rx_fifo_reset();
    }
}

static bool gps_uart_baudrate(uint32_t baud_rate) {
    uint32_t clk_div = ((REF_CLK_FREQ << 4) / baud_rate);
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
    // fehlerhafte Frames nicht im FIFO speichern
    uart->conf0.err_wr_mask = 1;
    // Clock
    uart->conf0.tick_ref_always_on = 1;
    // Pins
    if (uart_set_pin(GPS_UART, txPin, rxPin, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE)) return true;
    // Baud
    //gps_uart_baudrate(9600);
    uint32_t baud = gps_uart_auto_baud();
    ESP_LOGD("gps", "auto-baud -> %u", baud);
    // Interrupt Handler registrieren
    if (esp_intr_alloc(ETS_UART0_INTR_SOURCE + GPS_UART, ESP_INTR_FLAG_IRAM, gps_interrupt, NULL, NULL)) return true;
    // Interrupts aktivieren
    uart->int_clr.val = UART_INTR_MASK;
    uart->conf1.rx_tout_thrhd = 10; // Interrupt nach Ende eines Frames
    uart->conf1.rx_tout_en = 1;
    uart->conf1.rxfifo_full_thrhd = 120; // Interrupt kurz vor rx-FIFO Überlauf damit dieser noch ohne Verlust geleert werden kann
    uart->int_ena.val = UART_RXFIFO_TOUT_INT_ENA_M | UART_RXFIFO_OVF_INT_ENA_M | UART_FRM_ERR_INT_ENA_M | UART_RXFIFO_FULL_INT_ENA_M;

    return false;
}

static uint32_t IRAM_ATTR gps_uart_auto_baud() {
    uart_dev_t *uart = UART[GPS_UART];
    uint32_t low_period = 0, high_period = 0;
    uint32_t intena_reg = uart->int_ena.val;
    // Interrupts deaktivieren
    uart->int_ena.val = 0;
    uart->int_clr.val = UART_INTR_MASK;
    // Voreinstellungen
    uart->auto_baud.glitch_filt = 8;
    uart->auto_baud.en = 0;
    // aktivieren und 100 Impulse abwarten
    uart->auto_baud.en = 1;
    while (uart->rxd_cnt.edge_cnt < 100) {
        ets_delay_us(10);
    }
    low_period = uart->lowpulse.min_cnt;
    high_period = uart->highpulse.min_cnt;
    // Auto-Baud deaktivieren
    uart->auto_baud.en = 0;
    // ermittelte Baudrate einstellen
    uart->clk_div.div_int = (low_period > high_period) ? high_period : low_period;
    uart->clk_div.div_frag = 0;
    // FIFO zurücksetzen
    gps_uart_rx_fifo_reset();
    // Interrupts wieder aktivieren
    uart->int_ena.val = intena_reg;
    // return Baudrate, baud = APB / divider
    return APB_CLK_FREQ / ((low_period > high_period) ? high_period : low_period);
}

static void IRAM_ATTR gps_uart_rx_fifo_reset() {
    uart_dev_t *uart = UART[GPS_UART];
    while (uart->status.rxfifo_cnt) {
        (volatile void) uart->fifo.rw_byte;
    }
    //uart->conf0.rxfifo_rst = 1;
    return;
}
