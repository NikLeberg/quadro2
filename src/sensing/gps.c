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
#include "freertos/semphr.h"
#include "ringbuf.h" // mit eigener Erweiterung
#include <string.h>
#include <math.h>
#include "driver/uart.h" // uart_reg.h uart_struct.h
#include "esp_log.h"


/** Interne Abhängigkeiten **/

#include "intercom.h"
#include "resources.h"
#include "sensor_types.h"
#include "gps.h"


/** Variablendeklaration **/

#ifndef M_PI
    #define M_PI 3.14159265358979323846
#endif

static struct {
    gpio_num_t rxPin, txPin;
    SemaphoreHandle_t txSemphr;

    sensors_event_t position;
    sensors_event_t speed;
    event_t forward;
} gps;

static RingbufHandle_t xGps;

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
 * Sende ein vorbereitetes UBX-Frame an das GPS
 *
 * uint8_t *buffer: Pointer zur Nachricht, ckA und ckB können NULL sein -> werden dann berechnet
 * uint8_t length: gesamtlänge der Nachricht (Header & Payload)
 * bool aknowledge: false -> nur senden, true -> warte auf positive Bestätigung
 * TickType_t timeout: maximale Blockzeit
 *
 * returns: false -> Erfolg, true -> Error
 */
static bool gps_sendUBX(uint8_t *buffer, uint8_t length, bool aknowledge, TickType_t timeout);

/*
 * Function: gps_receiveUBX
 * ----------------------------
 * Empfange ein UBX-Frame
 *
 * uint8_t *payload: Buffer für Payload des Frames (mindestens so gross wie length)
 * uint8_t class: Class der zu empfangenden Nachricht
 * uint8_t id: Id der zu empfangenden Nachricht
 * uint8_t length: Länge des Payloads
 * TickType_t timeout: maximale Blockzeit
 *
 * returns: false -> Erfolg, true -> Error
 */
static bool gps_receiveUBX(uint8_t *payload, uint8_t class, uint8_t id, uint16_t length, TickType_t timeout);

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
// static uint32_t gps_uartAutoBaud();

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
    xGps = xRingbufferCreate(256, RINGBUF_TYPE_BYTEBUF_REGISTER_READ);
    gps.position.type = SENSORS_POSITION;
    gps.speed.type = SENSORS_GROUNDSPEED;
    gps.forward.type = EVENT_INTERNAL;
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
    // UBX-CFG-PMS: Auf Full Power setzen
    uint8_t msgPower[] = {0xB5, 0x62, 0x06, 0x86, 0x08, 0x00, 0x00, 0x00,
                          0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x94, 0x5A};
    if (gps_sendUBX(msgPower, sizeof(msgPower), true, 1000 / portTICK_PERIOD_MS)) return true; // NAK Empfangen
    // UBX-CFG-NAV5: Navigationsmodus auf Airborne <2g setzen
    uint8_t msgNav[] = {0xB5, 0x62, 0x06, 0x24, 0x24, 0x00, 0x01, 0x00,
                        0x07, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
                        0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
                        0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
                        0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
                        0x00, 0x00, 0x56, 0xD6};
    if (gps_sendUBX(msgNav, sizeof(msgNav), true, 1000 / portTICK_PERIOD_MS)) return true; // NAK Empfangen
    // UBX-CFG-GNSS: Aktiviere Galileo, deaktiviere QZSS
    uint8_t msgGNSS[] = {0xB5, 0x62, 0x06, 0x3e, 0x3c, 0x00, 0x00, 0x20, 0xFF, 0x07, // 32 aktivierte Kanäle
                         0x00, 0x08, 0x10, 0x00, 0x01, 0x00, 0x01, 0x01, // GPS mit min 8 max 16 Kanäle Aktiviert
                         0x01, 0x01, 0x03, 0x00, 0x01, 0x00, 0x01, 0x01, // SBAS mit min 0 max 3 Kanäle Aktiviert
                         0x02, 0x08, 0x0a, 0x00, 0x01, 0x00, 0x01, 0x01, // Galileo mit min 8 max 10 Kanäle Aktiviert
                         0x03, 0x08, 0x10, 0x00, 0x00, 0x00, 0x01, 0x01, // BeiDou Deaktiviert
                         0x04, 0x00, 0x08, 0x00, 0x00, 0x00, 0x01, 0x03, // IMAS Deaktiviert
                         0x05, 0x00, 0x03, 0x00, 0x00, 0x00, 0x01, 0x05, // QZSS Deaktiviert
                         0x06, 0x08, 0x0e, 0x00, 0x01, 0x00, 0x01, 0x01, // GOLONASS mit min 8 max 14 Kanäle Aktiviert
                         NULL, NULL};
    if (gps_sendUBX(msgGNSS, sizeof(msgGNSS), true, 1000 / portTICK_PERIOD_MS)) return true; // NAK Empfangen
    // UBX-CFG-MSG: Zu empfangende Nachrichten setzen, UBX-NAV-PVT (0x01 0x07)
    uint8_t msgMessages[] = {0xB5, 0x62, 0x06, 0x01, 0x03, 0x00, 0x01, 0x07, 0x01, 0x13, 0x51};
    if (gps_sendUBX(msgMessages, sizeof(msgMessages), true, 1000 / portTICK_PERIOD_MS)) return true; // NAK Empfangen
    // UBX-CFG-RATE: Daten-Rate setzen
    uint8_t msgRate[] = {0xB5, 0x62, 0x06, 0x08, 0x06, 0x00, (0xff & GPS_DATA_RATE_MS),
                        (GPS_DATA_RATE_MS >> 8), 0x01, 0x00, 0x00, 0x00, NULL, NULL};
    if (gps_sendUBX(msgRate, sizeof(msgRate), true, 1000 / portTICK_PERIOD_MS)) return true; // NAK Empfangen
    // Task starten
    if (xTaskCreate(&gps_task, "gps", 3 * 1024, NULL, xSensors_PRIORITY - 1, NULL) != pdTRUE) return true;
    return false;
}

void gps_task(void* arg) {
    // Variablen
    uint8_t nav[92];
    vector_t v;
    // UBX-NAV-PVT Frames parsen
    while (true) {
        gps_receiveUBX(nav, 0x01, 0x07, 92, portMAX_DELAY);
        gps.position.timestamp = esp_timer_get_time();
        gps.speed.timestamp = gps.position.timestamp;
        // Zeit
        // uint16_t jear = (nav[5] << 8) | (nav[4]);
        // uint8_t month = nav[6];
        // uint8_t day = nav[7];
        // uint8_t hour = nav[8];
        // uint8_t minute = nav[9];
        // uint8_t second = nav[10];
        // Fix-Typ & Satelitenanzahl
        uint8_t fixType = nav[20];
        // uint8_t satNum = nav[23];
        // if (fixType == 0 || fixType == 5) continue; // noch kein Fix oder nur Zeit-Fix
        // Position als y = Longitude / x = Latitude / z = Altitude
        // Laitude - Quer / Logitude - oben nach unten
        v.y = (int32_t) ((nav[27] << 24) | (nav[26] << 16) | (nav[25] << 8) | (nav[24])) * 1e-7; // °
        v.x = (int32_t) ((nav[31] << 24) | (nav[30] << 16) | (nav[29] << 8) | (nav[28])) * 1e-7; // °
        v.z = (int32_t) ((nav[39] << 24) | (nav[38] << 16) | (nav[37] << 8) | (nav[36])) / 1e+3; // m
        gps.position.accuracy = (uint32_t) ((nav[43] << 24) | (nav[42] << 16) | (nav[41] << 8) | (nav[40])) / 1e+3; // HDOP
        // float vAccuracy = (uint32_t) ((nav[47] << 24) | (nav[46] << 16) | (nav[45] << 8) | (nav[44])) / 1e+3; // VDOP
        // Longitude & Latitude in Meter umrechnen
        // -> https://gis.stackexchange.com/questions/2951
        gps.position.vector.y = v.y * 111111.0f * cosf(v.x * M_PI / 180.0f);
        gps.position.vector.x = v.x * 111111.0f;
        gps.position.vector.z = v.z;
        gps.forward.data = &gps.position;
        xQueueSendToBack(xSensors, &gps.forward, 0);
        // Geschwindigkeit
        // Koordinatensystem wechseln: GPS ist im NED, quadro ist im ENU
        gps.speed.vector.y = (int32_t) ((nav[51] << 24) | (nav[50] << 16) | (nav[49] << 8) | (nav[48])) / 1e+3;
        gps.speed.vector.x = (int32_t) ((nav[55] << 24) | (nav[54] << 16) | (nav[53] << 8) | (nav[52])) / 1e+3;
        gps.speed.vector.z = -(int32_t) ((nav[59] << 24) | (nav[58] << 16) | (nav[57] << 8) | (nav[56])) / 1e+3;
        gps.speed.accuracy = (uint32_t) ((nav[71] << 24) | (nav[70] << 16) | (nav[69] << 8) | (nav[68])) / 1e+3;
        gps.forward.data = &gps.speed;
        xQueueSendToBack(xSensors, &gps.forward, 0);
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
    uint8_t akPayload[2];
    if (timeout != portMAX_DELAY) timeout -= xTaskGetTickCount() - startTick;
    if (gps_receiveUBX(akPayload, 0x05, 0x01, 2, timeout)) return true;
    if (akPayload[0] == buffer[2] && akPayload[1] == buffer[3]) return false;
    else return true;
}

static bool gps_receiveUBX(uint8_t *payload, uint8_t class, uint8_t id, uint16_t length, TickType_t timeout) {
    TickType_t startTick = xTaskGetTickCount();
    uint8_t *sync = NULL;
    do { // suche nach Sync "u"
        sync = xRingbufferReceiveUpTo(xGps, NULL, timeout, 1);
        if (!sync) return true; // timeout
        vRingbufferReturnItem(xGps, sync);
        if (timeout != portMAX_DELAY) timeout -= xTaskGetTickCount() - startTick;
    } while (*sync != 0xb5);
    size_t rxLength = 0;
    uint8_t *rx = xRingbufferReceiveUpTo(xGps, &rxLength, timeout, length + 7);
    bool ret = true;
    if (rxLength >= 7) {
        if (rxLength == length + 7
        && rx[0] == 0x62 // Sync 2 "B"
        && rx[1] == class && rx[2] == id // Class & Id
        && rx[3] == (0x00ff & length) && rx[4] == (length >> 8)) { // Länge
            uint8_t ckA = 0, ckB = 0; // Prüfsumme Ok?
            for (uint16_t i = 1; i < length + 5; i++) {
                ckA += rx[i];
                ckB += ckA;
            }
            if (ckA == rx[length + 5] && ckB == rx[length + 6]) {
                memcpy(payload, rx + 5, length);
                ret = false;
            }
        }
    }
    vRingbufferReturnItem(xGps, rx);
    return ret;
}

static void gps_interrupt(void* arg) {
    uart_dev_t *uart = UART[GPS_UART];
    BaseType_t woken = pdFALSE;
    uint32_t status = uart->int_st.val;
    uart->int_clr.val = status; // aktive Interrupts zurücksetzen
    if (status & UART_TX_DONE_INT_ST_M) { // tx beendet, entsperren
        uart->int_ena.tx_done = 0; // interrupt deaktivieren
        xSemaphoreGiveFromISR(gps.txSemphr, &woken);
    } else if (status & UART_RXFIFO_TOUT_INT_ST_M || // rx-Frame erhalten
               status & UART_RXFIFO_FULL_INT_ST_M || // rx-FIFO bald voll
               status & UART_RXFIFO_OVF_INT_ST_M) {  // rx-FIFO Überlauf
        // FIFO in Ringbuffer einlesen
        xRingbufferSendFromISR(xGps, (const void *)&uart->fifo.rw_byte, (size_t)uart->status.rxfifo_cnt, &woken);
    } else if (status & UART_FRM_ERR_INT_ST_M) { // rx-Frame Error
        // FIFO zurücksetzen
        gps_uartRxFifoReset();
    }
    if (woken == pdTRUE) portYIELD_FROM_ISR();
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

// static uint32_t gps_uartAutoBaud() {
//     uart_dev_t *uart = UART[GPS_UART];
//     uint32_t low_period = 0, high_period = 0;
//     uint32_t intena_reg = uart->int_ena.val;
//     // Interrupts deaktivieren
//     uart->int_ena.val = 0;
//     uart->int_clr.val = UART_INTR_MASK;
//     // Voreinstellungen
//     uart->auto_baud.glitch_filt = 8;
//     uart->auto_baud.en = 0;
//     // aktivieren und 100 Impulse abwarten
//     uart->auto_baud.en = 1;
//     while (uart->rxd_cnt.edge_cnt < 100) {
//         ets_delay_us(10);
//     }
//     low_period = uart->lowpulse.min_cnt;
//     high_period = uart->highpulse.min_cnt;
//     // Auto-Baud deaktivieren
//     uart->auto_baud.en = 0;
//     // ermittelte Baudrate einstellen
//     uart->clk_div.div_int = (low_period > high_period) ? high_period : low_period;
//     uart->clk_div.div_frag = 0;
//     // FIFO zurücksetzen
//     gps_uartRxFifoReset();
//     // Interrupts wieder aktivieren
//     uart->int_ena.val = intena_reg;
//     // return Baudrate, baud = APB / divider
//     return APB_CLK_FREQ / ((low_period > high_period) ? high_period : low_period);
// }

static void gps_uartRxFifoReset() {
    uart_dev_t *uart = UART[GPS_UART];
    while (uart->status.rxfifo_cnt) {
        (volatile void) uart->fifo.rw_byte;
    }
    //uart->conf0.rxfifo_rst = 1;
    return;
}
