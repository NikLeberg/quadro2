/*
 * File: gps.c
 * ----------------------------
 * Author: Niklaus Leuenberger
 * Date:   2020-01-15
 * ----------------------------
 * GPS u-blox Parser für Beitian BN-880Q Sensor
 * 
 * UBX Frame Layout:
 * Sync 1 (0xb5) | Sync 2 (0x62) | Class | ID | Size LSB | Size MSB | Payload ... | CK_A | CK_B
 * 0             | 1             | 2     | 3  | 4        | 5        | 6       ... |
 */


/** Externe Abhängigkeiten **/

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "freertos/semphr.h"
#include <string.h>
#include <math.h>
#include "driver/uart.h" // uart_reg.h uart_struct.h
#include "esp_log.h"


/** Interne Abhängigkeiten **/

#include "gps.h"
#include "resources.h"
#include "sensor_types.h"


/** Variablendeklaration **/

#ifndef M_PI
    #define M_PI 3.14159265358979323846
#endif

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

    bool setHome;
    struct vector_t home;
};
static struct gps_t gps;

extern uart_dev_t UART0;
extern uart_dev_t UART1;
extern uart_dev_t UART2;
DRAM_ATTR uart_dev_t* const UART[UART_NUM_MAX] = {&UART0, &UART1, &UART2};


/** Private Functions **/

/*
 * Function: gps_task
 * ----------------------------
 * Haupttask. Wartet auf Frames die per ISR empfangen wurden, verarbeitet die Rohdaten
 * und leitet diese an sensors-Queue weiter.
 *
 * void* arg: Dummy für FreeRTOS
 */
void gps_task(void* arg);

/*
 * Function: gps_sendUBX
 * ----------------------------
 * Haupttask. Wartet auf Frames die per ISR empfangen wurden, verarbeitet die Rohdaten
 * und leitet diese an sensors-Queue weiter.
 *
 * uint8_t *buffer: Pointer zur Nachricht, ckA und ckB können NULL sein -> werden berechnet
 * uint8_t length: gesamtlänge der Nachricht (Header & Payload)
 * bool aknowledge: false -> nur senden, true -> warte auf positive Bestätigung
 * TickType_t timeout: maximale Blockzeit
 *
 * returns: false -> Erfolg, true -> Error
 */
static bool gps_sendUBX(uint8_t *buffer, uint8_t length, bool aknowledge, TickType_t timeout);

/*
 * Function: gps_interrupt
 * ----------------------------
 * ISR. Ausgeführt bei UART-Events. Setzt UART zurück bei Fehler oder parst UBX-Frames.
 *
 * void* arg: Dummy
 */
static void gps_interrupt(void* arg);

/*
 * Function: gps_uartBaudrate
 * ----------------------------
 * Setze UART Baudrate.
 *
 * uint32_t baud_rate: Baud e.g. 9600
 */
static bool gps_uartBaudrate(uint32_t baud_rate);

/*
 * Function: gps_uartEnable
 * ----------------------------
 * Aktiviere eigener UART Treiber auf den gegebenen Pins und setze Baud auf 9600.
 *
 * gpio_num_t txPin: Host Data-Out, GPS Data-In
 * gpio_num_t rxPin: Host Data-In, GPS Data-Out
 */
static bool gps_uartEnable(gpio_num_t txPin, gpio_num_t rxPin);

/*
 * Function: gps_uartAutoBaud
 * ----------------------------
 * Erkenne UART Baudrate.
 * 
 * ACHTUNG: Busy-Waits für eine unbestimmte Zeit bis 100 Pulse am rxPin erkannt wurden.
 *
 * returns uint32_t: erkannte Baudrate
 */
static uint32_t gps_uartAutoBaud();

/*
 * Function: gps_uartRxFifoReset
 * ----------------------------
 * Leere UART FIFO.
 */
static void gps_uartRxFifoReset();


/** Implementierung **/

bool gps_init(gpio_num_t rxPin, gpio_num_t txPin) {
    // Parameter speichern
    gps.rxPin = rxPin;
    gps.txPin = txPin;
    // Input Queue erstellen
    xGps_input = xQueueCreate(2, sizeof(struct gps_input_t));
    gps.txSemphr = xSemaphoreCreateMutex();
    xSemaphoreGive(gps.txSemphr);
    // eigener UART Treiber installieren
    gps_uartEnable(gps.txPin, gps.rxPin);
    // u-Blox Chip konfigurieren
    // UBX-CFG-PRT: NMEA deaktivieren, UART Baudrate auf 256000 setzen
    uint8_t msgPort[] = {0xB5, 0x62, 0x06, 0x00, 0x14, 0x00, 0x01,
                         0x00, 0x00, 0x00, 0xC0, 0x08, 0x00, 0x00,
                         0x00, 0xE8, 0x03, 0x00, 0x01, 0x00, 0x01,
                         0x00, 0x00, 0x00, 0x00, 0x00, 0xd0, 0xf8};
    gps_sendUBX(msgPort, sizeof(msgPort), true, 1000 / portTICK_PERIOD_MS); // AK aktivieren aber nicht auswerten, so wird auf tx done gewartet
    // eigener UART auf neue Baudrate setzen und Buffer löschen
    gps_uartBaudrate(256000);
    gps_uartRxFifoReset();
    // UBX-CFG-PMS: Power auf Balanced setzen
    uint8_t msgPower[] = {0xB5, 0x62, 0x06, 0x86, 0x08, 0x00, 0x00, 0x01,
                          0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x95, 0x61};
    if (gps_sendUBX(msgPower, sizeof(msgPower), true, 1000 / portTICK_PERIOD_MS)) return true; // NAK Empfangen
    // UBX-CFG-MSG: Zu empfangende Nachrichten setzen, UBX-NAV-PVT (0x01 0x07)
    uint8_t msgMessages[] = {0xB5, 0x62, 0x06, 0x01, 0x03, 0x00, 0x01, 0x07, 0x01, 0x13, 0x51};
    if (gps_sendUBX(msgMessages, sizeof(msgMessages), true, 1000 / portTICK_PERIOD_MS)) return true; // NAK Empfangen
    // UBX-CFG-RATE: Daten-Rate setzen
    uint8_t msgRate[] = {0xB5, 0x62, 0x06, 0x08, 0x06, 0x00, (0xff & GPS_DATA_RATE_MS), (GPS_DATA_RATE_MS >> 8), 0x01, 0x00, 0x00, 0x00, NULL, NULL};
    if (gps_sendUBX(msgRate, sizeof(msgRate), true, 1000 / portTICK_PERIOD_MS)) return true; // NAK Empfangen
    // Task starten, pinned da UART Interrupt an CPU gebunden ist.
    if (xTaskCreatePinnedToCore(&gps_task, "gps", 3 * 1024, NULL, xSensors_PRIORITY + 1, &xGps_handle, xPortGetCoreID()) != pdTRUE) return true;
    return false;
}

void gps_setHome() {
    gps.setHome = true;
    return;
}

void gps_task(void* arg) {
    // Variablen
    struct gps_input_t input;
    struct sensors_input_t forward;
    // UBX-NAV-PVT Frames parsen
    while (true) {
        if (input.frame) free(input.frame); // altes Frame freigeben
        xQueueReceive(xGps_input, &input, portMAX_DELAY);
        if (input.length < 92 + 8) continue; // Mindestlänge (Payload + Header)
        uint8_t *f = input.frame;
        if (f[2] != 0x01 && f[3] != 0x07) continue; // kein UBX-NAV-PVT Frame
        forward.timestamp = input.timestamp;
        // // Zeit
        // uint16_t jear = (f[11] << 8) | (f[10]);
        // uint8_t month = f[12];
        // uint8_t day = f[13];
        // uint8_t hour = f[14];
        // uint8_t minute = f[15];
        // uint8_t second = f[16];
        // Fix-Typ & Satelitenanzahl
        uint8_t fixType = f[26];
        // uint8_t satNum = f[29];
        if (fixType == 0 || fixType == 5) continue; // noch kein Fix oder nur Zeit-Fix
        // Position als y = Longitude / x = Latitude / z = Altitude
        // Laitüde - Quer / Logitude - oben nach unten
        forward.type = SENSORS_POSITION;
        forward.vector.y = (int32_t) ((f[33] << 24) | (f[32] << 16) | (f[31] << 8) | (f[30])) * 1e-7; // °
        forward.vector.x = (int32_t) ((f[37] << 24) | (f[36] << 16) | (f[35] << 8) | (f[34])) * 1e-7; // °
        forward.vector.z = (int32_t) ((f[45] << 24) | (f[44] << 16) | (f[43] << 8) | (f[42])) / 1e+3; // m
        float hAccuracy = (uint32_t) ((f[49] << 24) | (f[48] << 16) | (f[47] << 8) | (f[46])) / 1e+3;
        float vAccuracy = (uint32_t) ((f[53] << 24) | (f[52] << 16) | (f[51] << 8) | (f[50])) / 1e+3;
        if (hAccuracy > vAccuracy) forward.accuracy = hAccuracy;
        else forward.accuracy = vAccuracy;
        // Longitude & Latitude in Meter umrechnen
        // -> https://gis.stackexchange.com/questions/2951
        forward.vector.y *= 111111.0f * cosf(forward.vector.x * M_PI / 180.0f);
        forward.vector.x *= 111111.0f;
        // Homepunkt anwenden (aber nur bei 3D-Fix & DOP < 2 m)
        if (gps.setHome && fixType >= 3 && forward.accuracy <= 2.0f) {
            gps.home = forward.vector;
            forward.vector.x = 0.0f;
            forward.vector.y = 0.0f;
            forward.vector.z = 0.0f;
            gps.setHome = false;
        } else {
            forward.vector.x -= gps.home.x;
            forward.vector.y -= gps.home.y;
            forward.vector.z -= gps.home.z;
        }
        xQueueSendToBack(xSensors_input, &forward, 0);
        // Geschwindigkeit
        forward.type = SENSORS_GROUNDSPEED; 
        // Koordinatensystem wechseln: GPS ist im NED, quadro ist im ENU
        forward.vector.y = (int32_t) ((f[57] << 24) | (f[56] << 16) | (f[55] << 8) | (f[54])) / 1e+3;
        forward.vector.x = (int32_t) ((f[61] << 24) | (f[60] << 16) | (f[59] << 8) | (f[58])) / 1e+3;
        forward.vector.z = -(int32_t) ((f[65] << 24) | (f[64] << 16) | (f[63] << 8) | (f[62])) / 1e+3;
        forward.accuracy = (uint32_t) ((f[77] << 24) | (f[76] << 16) | (f[75] << 8) | (f[74])) / 1e+3;
        xQueueSendToBack(xSensors_input, &forward, 0);
    }
}

static bool gps_sendUBX(uint8_t *buffer, uint8_t length, bool aknowledge, TickType_t timeout) {
    TickType_t startTick = xTaskGetTickCount();
    // Zustand prüfen
    if (length > UART_FIFO_LEN) return true; // zu gross für FIFO
    if (UART[GPS_UART]->status.txfifo_cnt) return true; // FIFO nicht leer
    if (xSemaphoreTake(gps.txSemphr, timeout) == pdFALSE) return true; // tx busy, Timeout
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
    }
    // tx done Interrupt aktivieren
    UART[GPS_UART]->int_ena.tx_done = 1;
    // AK / NAK
    if (!aknowledge) return false; // kein AK erforderlich
    bool akReceived = false;
    timeout -= xTaskGetTickCount() - startTick;
    // empfange aus Queue
    struct gps_input_t input;
    if (xQueueReceive(xGps_input, &input, timeout) == pdFALSE) return true; // Timeout
    if (input.length == 10) { // gültige Länge, AK, Class & Id übereinstimmend
        if (input.frame[3] && input.frame[6] == buffer[2] && input.frame[7] == buffer[3]) {
            akReceived = true;
        }
    }
    free(input.frame);
    if (akReceived) return false;
    else return true; // Ungültige Antwort
}

static void gps_interrupt(void* arg) {
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
        uint16_t frameSize = 0, payloadSize = 0; // erwartete Länge
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
                    payloadSize = c;
                    ++framePos;
                    continue;
                }
                case (5): { // Size MSB
                    payloadSize |= ((uint16_t) c << 8);
                    frameSize = payloadSize + 6 + 2; // Header + Prüfsumme hinzufügen
                    if (frameSize > UART_FIFO_LEN) { // zu gross für FIFO, verwerfen
                        framePos = 0;
                        continue;
                    }
                    // Speicher für Frame allozieren und Header rekonstruieren;
                    frame = (uint8_t*) calloc(frameSize, sizeof(uint8_t));
                    if (!frame) continue;
                    frame[0] = 0xb5;
                    frame[1] = 0x62;
                    frame[2] = class;
                    frame[3] = id;
                    frame[4] = payloadSize;
                    frame[5] = c; // payloadSize >> 8
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
        if (!framePos && !frameSize)  return; // nichts empfangen
        // empfangen == erwartet?
        if (framePos != frameSize) {
            if (frame) free(frame);
            return; // Frame verwerfen
        }
        // Prüfsumme Ok?
        for (uint8_t i = 2; i < (frameSize - 2); ++i) {
            ckA = ckA + frame[i];
            ckB = ckB + ckA;
        }
        if (ckA != frame[frameSize - 2] || ckB != frame[frameSize - 1]) {
            if (frame) free(frame);
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
        gps_uartRxFifoReset();
    }
}

static bool gps_uartBaudrate(uint32_t baud_rate) {
    uint32_t clk_div = ((APB_CLK_FREQ << 4) / baud_rate);
    if (clk_div < 16) return true;
    else {
        UART[GPS_UART]->clk_div.div_int = clk_div >> 4;
        UART[GPS_UART]->clk_div.div_frag = clk_div & 0xf;
    }
    return false;
}

static bool gps_uartEnable(gpio_num_t txPin, gpio_num_t rxPin) {
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
    gps_uartBaudrate(9600);
    // Interrupt Handler registrieren
    if (esp_intr_alloc(ETS_UART0_INTR_SOURCE + GPS_UART, 0, gps_interrupt, NULL, NULL)) return true;
    // Interrupts aktivieren
    uart->int_clr.val = UART_INTR_MASK;
    uart->conf1.rx_tout_thrhd = 10; // Interrupt nach Ende eines Frames
    uart->conf1.rx_tout_en = 1;
    uart->conf1.rxfifo_full_thrhd = 120; // Interrupt kurz vor rx-FIFO Überlauf damit dieser noch ohne Verlust geleert werden kann
    uart->int_ena.val = UART_RXFIFO_TOUT_INT_ENA_M | UART_RXFIFO_OVF_INT_ENA_M | UART_FRM_ERR_INT_ENA_M | UART_RXFIFO_FULL_INT_ENA_M;

    return false;
}

static uint32_t gps_uartAutoBaud() {
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
    gps_uartRxFifoReset();
    // Interrupts wieder aktivieren
    uart->int_ena.val = intena_reg;
    // return Baudrate, baud = APB / divider
    return APB_CLK_FREQ / ((low_period > high_period) ? high_period : low_period);
}

static void gps_uartRxFifoReset() {
    uart_dev_t *uart = UART[GPS_UART];
    while (uart->status.rxfifo_cnt) {
        (volatile void) uart->fifo.rw_byte;
    }
    //uart->conf0.rxfifo_rst = 1;
    return;
}
