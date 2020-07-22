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
#include <string.h>
#include <math.h>
#include "esp_log.h"


/** Interne Abhängigkeiten **/

#include "intercom.h"
#include "resources.h"
#include "sensor_types.h"
#include "uart.h"
#include "gps.h"


/** Variablendeklaration **/

#ifndef M_PI
    #define M_PI 3.14159265358979323846f
#endif

typedef struct __attribute__((packed)) {
    uint8_t sync1;  // 0xb5
    uint8_t sync2;  // 0b62
    uint8_t class;
    uint8_t id;
    uint16_t size;
    uint8_t ckA;
    uint8_t ckB;
} gps_ubx_nav_pvt_t;

static struct {
    SemaphoreHandle_t rxSemphr;

    sensors_event_t position;
    sensors_event_t speed;
    event_t forward;
} gps;


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


/** Implementierung **/

bool gps_init(gpio_num_t rxPin, gpio_num_t txPin, uint32_t rate) {
    // Input Queue erstellen
    gps.position.type = SENSORS_POSITION;
    gps.speed.type = SENSORS_GROUNDSPEED;
    gps.forward.type = EVENT_INTERNAL;
    gps.rxSemphr = xSemaphoreCreateBinary();
    // eigener UART Treiber installieren
    uart_init(GPS_UART, txPin, rxPin, 9600, gps.rxSemphr);
    // u-Blox Chip konfigurieren
    // UBX-CFG-PRT: NMEA deaktivieren, UART Baudrate auf 256000 setzen
    uint8_t msgPort[] = {0xB5, 0x62, 0x06, 0x00, 0x14, 0x00, 0x01,
                         0x00, 0x00, 0x00, 0xC0, 0x08, 0x00, 0x00,
                         0x00, 0xE8, 0x03, 0x00, 0x01, 0x00, 0x01,
                         0x00, 0x00, 0x00, 0x00, 0x00, 0xd0, 0xf8};
    gps_sendUBX(msgPort, sizeof(msgPort), true, 1000 / portTICK_PERIOD_MS); // AK aktivieren aber nicht auswerten, so wird auf tx done gewartet
    // eigener UART auf neue Baudrate setzen und Buffer löschen
    uart_baud(GPS_UART, 256000);
    uart_rxFifoReset(GPS_UART);
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
    uint8_t msgRate[] = {0xB5, 0x62, 0x06, 0x08, 0x06, 0x00, (0xff & rate),
                        (rate >> 8), 0x01, 0x00, 0x00, 0x00, NULL, NULL};
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
        if (fixType == 0 || fixType == 5) continue; // noch kein Fix oder nur Zeit-Fix
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

void gps_updateRate(uint32_t rate) {
    // UBX-CFG-RATE: Daten-Rate setzen
    uint8_t msgRate[] = {0xB5, 0x62, 0x06, 0x08, 0x06, 0x00, (0xff & rate),
                        (rate >> 8), 0x01, 0x00, 0x00, 0x00, NULL, NULL};
    bool result = gps_sendUBX(msgRate, sizeof(msgRate), false, 0); // kein NAK Empfang, sonst kriegt GPS-Task Probleme aus irgend einem Grund (semphr?)
    ESP_LOGD("gps", "update rate: %u", result);
}

static bool gps_sendUBX(uint8_t *buffer, uint8_t length, bool aknowledge, TickType_t timeout) {
    TickType_t startTick = xTaskGetTickCount();
    // Prüfsumme rechnen
    if (!(buffer[length - 2] || buffer[length - 1])) {
        for (uint8_t i = 2; i < (length - 2); ++i) {
            buffer[length - 2] = buffer[length - 2] + buffer[i];
            buffer[length - 1] = buffer[length - 1] + buffer[length - 2];
        }
    }
    // Schreiben
    if (uart_txAvailable(GPS_UART) < length) return true;
    for (uint8_t i = 0; i < length; i++) {
        uart_write(GPS_UART, buffer[i]);
    }
    // AK / NAK
    if (!aknowledge) return false; // kein AK erforderlich
    uint8_t akPayload[2];
    if (timeout != portMAX_DELAY) {
        TickType_t dTick = xTaskGetTickCount() - startTick;
        if (dTick >= timeout) timeout = 0;
        else timeout -= dTick;
    }
    if (gps_receiveUBX(akPayload, 0x05, 0x01, 2, timeout)) return true;
    if (akPayload[0] == buffer[2] && akPayload[1] == buffer[3]) return false;
    else return true;
}

static bool gps_receiveUBX(uint8_t *payload, uint8_t class, uint8_t id, uint16_t length, TickType_t timeout) {
    TickType_t startTick = xTaskGetTickCount();
    while (timeout) {
        if (timeout != portMAX_DELAY) {
            TickType_t dTick = xTaskGetTickCount() - startTick;
            if (dTick >= timeout) timeout = 0;
            else timeout -= dTick;
        }
        uart_rxInterrupt(GPS_UART, true);
        if (xSemaphoreTake(gps.rxSemphr, timeout) == pdFALSE) break;
        uint8_t c, position = 0;
        while (uart_rxAvailable(GPS_UART)) {
            c = uart_read(GPS_UART);
            ++position;
            switch (position) {
                case (1): // Sync 1
                    if (c != 0xb5) position = 0;
                    break;
                case (2): // Sync 2
                    if (c != 0x62) position = 0;
                    break;
                case (3): // Class
                    if (c != class) position = 0;
                    break;
                case (4): // ID
                    if (c != id) position = 0;
                    break;
                case (5): // Size LSB
                    if (c != (length & 0x00ff)) position = 0;
                    break;
                case (6): // Size MSB
                    if (c != (length >> 8)) position = 0;
                    break;
                default: // Payload
                    if (position - 7 < length) payload[position - 7] = c;
                    else return false; // Frame empfangen (ohne Prüfsummencheck)
                    break;
            }
        }
    }
    return true; // timeout
}
