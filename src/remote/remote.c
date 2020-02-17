/*
 * File: remote.c
 * ----------------------------
 * Author: Niklaus Leuenberger
 * Date:   2019-12-13
 * ----------------------------
 * Implementiert eine Remote Steuerung per Webbrowser & Verwaltung für persistente Einstellungen.
 * Ordner www/ enthält index.html und ? für die Webkonnektivität, diese werden in das Binary-File eingebettet:
 *    -> board_build.embed_txtfiles https://docs.platformio.org/en/latest/platforms/espressif32.html#embedding-binary-data
 * Webbrowser <-> quadro2 kommuniziert über eine persistente Websocket-Verbindung.
 */


/** Externe Abhängigkeiten **/

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "freertos/event_groups.h"
#include "netif/wlanif.h"
#include "esp_wifi.h"
#include "esp_event.h"
#include "esp_event_loop.h"
#include "esp_log.h"
#include "lwip/err.h"
#include "lwip/sys.h"
// #define CONFIG_ESPHTTPD_BACKLOG_SUPPORT 1
#include "esp.h" //libesphttpd
#include "httpd.h"
#include "httpd-freertos.h"
#include "httpd-freertos.c"
#include "cgiwebsocket.h"
#include "route.h"
#include "nvs.h"
#include "nvs_flash.h"


/** Interne Abhängigkeiten **/

#include "resources.h"
#include "remote.h"


/** Variablendeklaration **/

enum remote_input_type_t {
    REMOTE_INPUT_CONNECTED = 0,
    REMOTE_INPUT_DISCONNECTED,
    REMOTE_INPUT_MESSAGE_RECEIVE,
    REMOTE_INPUT_MESSAGE_SEND
};

struct remote_input_message_t {
    char* data;
    uint8_t length;
    Websock *ws;
    int64_t timestamp;
};

struct remote_input_t {
    enum remote_input_type_t type;
    union {
        struct remote_input_message_t message;
    };
};

struct remote_setting_linked_list_t {
    char *taskTag;
    char *settingTag;
    remote_settingUpdate_t *updateCallback;
    void *updateCallbackCookie;
    struct setting_t *setting;
    struct remote_setting_linked_list_t *next;
};

struct remote_t {
    HttpdFreertosInstance httpd;
    char httpdConn[sizeof(RtosConnType) * REMOTE_CONNECTION_COUNT];

    uint8_t connected;
    Websock *ws;

    vprintf_like_t defaultLog;
    char disabledLogs[5];

    struct remote_setting_linked_list_t *settingsHead;
};
static struct remote_t remote;


/** Private Functions **/

/*
 * Function: remote_task
 * ----------------------------
 * Haupttask. Verwaltet Events
 *
 * void* arg: Dummy für FreeRTOS
 */
void remote_task(void* arg);

/*
 * Function: remote_initWlan
 * ----------------------------
 * Startet TCP/IP-Stack und verbindet mit WLAN.
 * 
 * char* ssid: Name des zu verbindenden WLAN
 * char* pw: PW des zu verbindenden WLAN
 *
 * returns: false bei Erfolg, sonst true
 */
static bool remote_initWlan(char* ssid, char* pw);

/*
 * WebSocket Nachrichtenlayout
 * ----------------------------
 * s - Status, r - Report, c - Control, l - Log
 * s: s? - Anfrage, s0 - Antwort nOk / Fatal, s1 - Antwort Ok, Ping/Pong-Mechanismus
 * r: ro - (Report-Orientation) roZahl,Zahl,Zahl, ra - (Report-Acceleration)
 * c: werden an control-Task weitergeleitet
 * l: umgeleitete ESP_LOG Strings
 * 
 * ToDo?: in Binary senden
 */

/*
 * Function: remote_processMessage
 * ----------------------------
 * Per Websocket empfangene Nachricht verarbeiten, weiterleiten.
 * 
 * struct remote_input_message_t *message: empfangene Message
 */
static void remote_processMessage(struct remote_input_message_t *message);

/*
 * Function: remote_sendMessage
 * ----------------------------
 * Nachricht per WebSocket senden. Wenn message->ws == NULL, dann mittels Broadcast
 * 
 * struct remote_input_message_t *message: zu sendende Message
 */
static void remote_sendMessage(struct remote_input_message_t *message);


/** Callbacks **/

/*
 * Function: remote_connectionEventHandler
 * ----------------------------
 * Systemevent-Handler für WiFi Statusänderungen.
 * 
 * void *ctx: Usercontext (Unbenutzt)
 * system_event_t *event: Systemevent
 *
 * returns: immer esp_err_t ESP_OK
 */
static esp_err_t remote_connectionEventHandler(void *ctx, system_event_t *event);

/*
 * Function: remote_wsReceive
 * ----------------------------
 * Callback für httpd-Task, Nachricht empfangen.
 * Daten kopieren und eigenem Task weiterleiten.
 * 
 * Websock *ws: WebSocket mit Event
 * char *data: Nachricht
 * int len: Nachrichtenlänge
 * int flags: Nachrichtenflags, (Binary/Text)
 */
static void remote_wsReceive(Websock *ws, char *data, int len, int flags);

/*
 * Function: remote_wsConnect
 * ----------------------------
 * Callback für httpd-Task, eine neue WebSocket Verbindung wurde geöffnet.
 * Leitet Event eigenem Task weiter.
 * 
 * Websock *ws: WebSocket mit Event
 */
static void remote_wsConnect(Websock *ws);

/*
 * Function: remote_wsDisconnect
 * ----------------------------
 * Callback für httpd-Task, ein WebSocket hat die Verbindung geschlossen.
 * Leitet Event eigenem Task weiter.
 * 
 * Websock *ws: WebSocket mit Event
 */
static void remote_wsDisconnect(Websock *ws);

/*
 * Function: remote_sensorData
 * ----------------------------
 * Callback für BNO-Task, wird ausgeführt wenn Sensordaten in gewünschter Frequenz
 * empfangen wurden. Leitet Daten in formatierter Form dem eigenen Task weiter.
 * 
 * sh2_SensorValue_t value: Sensor Event & Daten
 */
// static void remote_sensorData(sh2_SensorValue_t value);

/*
 * Function: remote_sendEmbedded
 * ----------------------------
 * Callback für httpd-Server. Sendet in der Firmware enthaltene Binärfiles.
 * Verknüpfung wird in builtInUrls[] erstellt. Automatische Mime und gzip Erkennung.
 * 
 * HttpdConnData *connData: aktive Verbindung mit weiteren cgi-Parametern
 * - connData->cgiArg: Startadresse der Binary
 * - connData->cgiArg2: Endadresse der Binary
 */
static CgiStatus remote_sendEmbedded(HttpdConnData *connData);

/*
 * Function: remote_printLog
 * ----------------------------
 * vprintf-like ESP_LOG-Funktion. Leitet Logs per Broadcast an WebSockets weiter.
 * Originale Log-Funktion über UART0 wird immernoch aufgerufen.
 * 
 * const char *format: printf-Formatstring
 * va_list arguments: Argumentliste
 *
 * returns: anzahl geschriebener Bytes oder negativ bei Fehler
 */
int remote_printLog(const char *format, va_list arguments);

/*
 * Function: remote_settingSet
 * ----------------------------
 * Verändert eine Einstellung und aktualisiert NVS.
 * Callback kann Änderung mit true ablehnen.
 * 
 * struct remote_setting_linked_list_t *currentSetting: Einstellung aus linked List
 * char *value: neuer Wert in Textform
 *
 * returns: false -> Erfolg, true -> Error / Abgelehnt
 */
static bool remote_settingSet(struct remote_setting_linked_list_t *currentSetting, char *value);

//
static void remote_processSetting(struct remote_input_message_t *message);
struct remote_setting_linked_list_t* remote_settingSelect(char *taskTag, char *settingTag);


/** Files **/

extern char _binary_src_remote_www_index_html_start;
extern char _binary_src_remote_www_index_html_end;
extern char _binary_src_remote_www_manifest_json_start;
extern char _binary_src_remote_www_manifest_json_end;
extern char _binary_src_remote_www_favicon_ico_gz_start;
extern char _binary_src_remote_www_favicon_ico_gz_end;
extern char _binary_src_remote_www_script_js_start;
extern char _binary_src_remote_www_script_js_end;

HttpdBuiltInUrl builtInUrls[] = {
    // WebSocket
    ROUTE_WS("/ws", remote_wsConnect),
    // Website
    ROUTE_CGI_ARG2("/", remote_sendEmbedded, &_binary_src_remote_www_index_html_start, &_binary_src_remote_www_index_html_end),
    ROUTE_CGI_ARG2("/index.html", remote_sendEmbedded, &_binary_src_remote_www_index_html_start, &_binary_src_remote_www_index_html_end),
    ROUTE_CGI_ARG2("/manifest.json", remote_sendEmbedded, &_binary_src_remote_www_manifest_json_start, &_binary_src_remote_www_manifest_json_end),
    ROUTE_CGI_ARG2("/favicon.ico", remote_sendEmbedded, &_binary_src_remote_www_favicon_ico_gz_start, &_binary_src_remote_www_favicon_ico_gz_end),
    ROUTE_CGI_ARG2("/script.js", remote_sendEmbedded, &_binary_src_remote_www_script_js_start, &_binary_src_remote_www_script_js_end),
    ROUTE_END()
};


/** Implementierung **/

bool remote_init(char* ssid, char* pw) {
    // NVS starten
    #if (REMOTE_SETTINGS_USE_NVS == 1)
        if (nvs_flash_init()) {
            if (nvs_flash_erase() || nvs_flash_init()) return true;
        }
    #endif
    remote.settingsHead = calloc(1, sizeof(struct remote_setting_linked_list_t));
    // Wlan starten
    if (remote_initWlan(ssid, pw)) return true;
    // Input Queue erstellen
    xRemote_input = xQueueCreate(32, sizeof(struct remote_input_t));
    // Task starten
    if (xTaskCreate(&remote_task, "remote", 3 * 1024, NULL, xRemote_PRIORITY, &xRemote_handle) != pdTRUE) return true;
    // libesphttpd Bibliothek starten
	if (httpdFreertosInit(&remote.httpd, builtInUrls, 80U, remote.httpdConn, REMOTE_CONNECTION_COUNT, HTTPD_FLAG_NONE)) return true;
	if (httpdFreertosStart(&remote.httpd)) return true;
    esp_log_level_set("cgiwebsocket", ESP_LOG_INFO);
    esp_log_level_set("httpd-freertos", ESP_LOG_INFO);
    // Sensorcallbacks registrieren (2 Hz)
    // bno_sensorRegister(SH2_LINEAR_ACCELERATION, 500000, &remote_sensorData);
    // bno_sensorRegister(SH2_ROTATION_VECTOR, 500000, &remote_sensorData);
    // Log umleiten
    remote.defaultLog = esp_log_set_vprintf(&remote_printLog);
    return false;
}

void remote_task(void* arg) {
    // Variablen
    struct remote_input_t input;
    bool timeoutPending = false;
    int64_t lastContact = 0, now;
    // Loop
    while (true) {
        if (xQueueReceive(xRemote_input, &input, 500 / portTICK_RATE_MS) == pdTRUE) {
            switch (input.type) {
                case REMOTE_INPUT_CONNECTED: {
                    ++remote.connected;
                    if (remote.connected == 1){}; // erste Verbindung, event propagieren ToDo
                    break;
                }
                case REMOTE_INPUT_DISCONNECTED: {
                    --remote.connected;
                    if (!remote.connected){}; // war letzte Verbindung, event propagieren ToDo
                    break;
                }
                case REMOTE_INPUT_MESSAGE_RECEIVE: {
                    remote_processMessage(&input.message);
                    free(input.message.data);
                    lastContact = input.message.timestamp;
                    timeoutPending = false;
                    break;
                }
                case REMOTE_INPUT_MESSAGE_SEND: {
                    remote_sendMessage(&input.message);
                    free(input.message.data);
                    break;
                }
                default:
                    break;
            }
        }
        // Timeout (0.5 s) erkennen
        now = esp_timer_get_time();
        if (remote.connected && (now - lastContact > 500000)) {
            if (timeoutPending) { // Timeout fällig -> echter Timeout! -> Notstopp
                // ToDo Event propagieren
                ESP_LOGE("remote", "timeout!");
                timeoutPending = false;
            } else if (remote.ws) {
                cgiWebsocketSend(&remote.httpd.httpdInstance, remote.ws,"s?", 2, WEBSOCK_FLAG_NONE);
                timeoutPending = true;
                lastContact = now;
            }
        }
    }
}

bool remote_settingRegister(const char *taskTag, const char *settingTag, remote_settingUpdate_t *updateCallback, void *updateCallbackCookie, struct setting_t *setting) {
    // Liste nach Ende suchen
    struct remote_setting_linked_list_t *currentSetting = remote.settingsHead;
    while (currentSetting->next) { // Ende suchen, prüfe ob bereits registriert
        if (!strcmp(taskTag, currentSetting->taskTag) && !strcmp(settingTag, currentSetting->settingTag)) {
            return true;
        }
        currentSetting = currentSetting->next;
    }
    // Einstellung anhängen & verlinken
    currentSetting->setting = calloc(1, sizeof(struct setting_t)); // neue Einstellung
    *currentSetting->setting = *setting;
    currentSetting->taskTag = malloc(strlen(taskTag) + 1);
    strcpy(currentSetting->taskTag, taskTag);
    currentSetting->settingTag = malloc(strlen(settingTag) + 1);
    strcpy(currentSetting->settingTag, settingTag);
    currentSetting->updateCallback = updateCallback;
    currentSetting->updateCallbackCookie = updateCallbackCookie;
    currentSetting->next = calloc(1, sizeof(struct remote_setting_linked_list_t)); // neues Listenelement bereithalten
    // NVS durchsuchen
    #if (REMOTE_SETTINGS_USE_NVS == 1)
        nvs_handle nvsHandle;
        if (nvs_open(taskTag, NVS_READWRITE, &nvsHandle)) return true;
        esp_err_t ret = nvs_get_u32(nvsHandle, settingTag, &setting->value.ui);
        if (ret == ESP_ERR_NVS_NOT_FOUND) { // Key noch nicht vorhanden -> erstellen
            if (nvs_set_u32(nvsHandle, settingTag, setting->value.ui)) return true; // Fehler beim speichern
            // Keine Veränderung vorgenommen, Beende Erfolgreich
            if (nvs_commit(nvsHandle)) return true;
            nvs_close(nvsHandle);
            return false;
        } else if (ret) return true;
        // Einstellung registriert und mit NVS Wert aktualisiert -> mitteilen
        currentSetting->updateCallback(updateCallbackCookie, currentSetting->setting);
        if (nvs_commit(nvsHandle)) return true;
        nvs_close(nvsHandle);
    #endif
    return false; // Beende Erfolgreich
}

static esp_err_t remote_connectionEventHandler(void *ctx, system_event_t *event) {
    ESP_LOGD("remote", "WiFi Event: %d", event->event_id);
    switch(event->event_id) {
        case SYSTEM_EVENT_STA_START:
            esp_wifi_connect();
            break;
        case SYSTEM_EVENT_STA_GOT_IP: {
            uint32_t ip = event->event_info.got_ip.ip_info.ip.addr;
            ESP_LOGD("remote", "IP - %u.%u.%u.%u", ip & 0xff, (ip >> 8) & 0xff, (ip >> 16) & 0xff, (ip >> 24));
            break;
        }
        case SYSTEM_EVENT_STA_DISCONNECTED:
            // Verbindung wiederherstellen, wenn nicht absichtlich
            if (event->event_info.disconnected.reason != WIFI_REASON_ASSOC_LEAVE
            && event->event_info.disconnected.reason != WIFI_REASON_AUTH_FAIL) {
                esp_wifi_stop();
            }
            break;
        case SYSTEM_EVENT_STA_STOP:
            esp_wifi_start();
            break;
        default:
            break;
    }
    return ESP_OK;
}

static bool remote_initWlan(char* ssid, char* pw) {
    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    wifi_config_t wifi_config = {0};
    strcpy((char*)wifi_config.sta.ssid, ssid);
    strcpy((char*)wifi_config.sta.password, pw);
    tcpip_adapter_init();
    if (esp_event_loop_init(remote_connectionEventHandler, NULL)
    || esp_wifi_init(&cfg)
    || esp_wifi_set_storage(WIFI_STORAGE_RAM)
    || esp_wifi_set_mode(WIFI_MODE_STA)
    || esp_wifi_set_ps(WIFI_PS_NONE)
    || esp_wifi_set_config(ESP_IF_WIFI_STA, &wifi_config)
    || esp_wifi_start()) return true;
    return false;
}

static void remote_processMessage(struct remote_input_message_t *message) {
    if (message->length < 1) return;
    switch (*message->data) {
        case 's': // Status
            if (*(message->data+1) != '1') return; // ToDo: Notstopp, Fehler
            break;
        case 'c': // Control
            // ToDo -> Weiterleiten an control
            break;
        case 'l': // Log
            memset(remote.disabledLogs, 0, 5);
            memcpy(remote.disabledLogs, (message->data + 1), message->length - 1);
            break;
        case 'e': // Einstellungen
            remote_processSetting(message);
            break;
        case 'r': // Report, wird vom Handy nicht verschickt
        default:
            break;
    }
}

static void remote_sendMessage(struct remote_input_message_t *message) {
    if (remote.connected) {
        if (message->ws) { // spezifischer WS
            cgiWebsocketSend(&remote.httpd.httpdInstance, message->ws, message->data, message->length, WEBSOCK_FLAG_NONE);
        } else if (remote.ws) { // letzter WS
            cgiWebsocketSend(&remote.httpd.httpdInstance, remote.ws, message->data, message->length, WEBSOCK_FLAG_NONE);
        // } else { // alle WS (super unsicher)
        //     cgiWebsockBroadcast(&remote.httpd.httpdInstance, "/ws", message->data, message->length, WEBSOCK_FLAG_NONE);
        }
    }
}

static void remote_wsReceive(Websock *ws, char *data, int len, int flags) {
    struct remote_input_t input;
    input.type = REMOTE_INPUT_MESSAGE_RECEIVE;
    input.message.ws = ws;
    input.message.length = len;
    input.message.data = (char*) malloc(len * sizeof(char));
    memcpy(input.message.data, data, len);
    input.message.timestamp = esp_timer_get_time();
    xQueueSend(xRemote_input, &input, 0);
}

static void remote_wsConnect(Websock *ws) {
    // als aktiver ws speichern
    remote.ws = ws;
    // Event melden
	struct remote_input_t input;
    input.type = REMOTE_INPUT_CONNECTED;
    xQueueSend(xRemote_input, &input, 0);
    // Callbacks registrieren
	ws->recvCb = remote_wsReceive;
    ws->closeCb = remote_wsDisconnect;
    // Hallo senden
	cgiWebsocketSend(&remote.httpd.httpdInstance, ws, "quadro2", 7, WEBSOCK_FLAG_NONE);
}

static void remote_wsDisconnect(Websock *ws) {
    // inaktiv setzen
    if (remote.ws == ws) remote.ws = NULL;
    // Event melden
	struct remote_input_t input;
    input.type = REMOTE_INPUT_DISCONNECTED;
    xQueueSend(xRemote_input, &input, 0);
}

// static void remote_sensorData(sh2_SensorValue_t value) {
//     char *string = (char*) malloc(128 * sizeof(char));
//     int length = 0;
//     switch (value.sensorId) {
//         case SH2_LINEAR_ACCELERATION: {
//             sh2_Accelerometer_t *acceleration = &value.un.linearAcceleration;
//             length = snprintf(string, 128, "ra%f,%f,%f", acceleration->x, acceleration->y, acceleration->z);
//             break;
//         }
//         case SH2_ROTATION_VECTOR: {
//             sh2_RotationVectorWAcc_t *rotation = &value.un.rotationVector;
//             length = snprintf(string, 128, "ro%f,%f,%f,%f,%f", rotation->accuracy, rotation->i, rotation->j, rotation->k, rotation->real);
//             break;
//         }
//     }
//     if (length <= 0 || length > 128) {
//         free(string);
//         return;
//     }
//     struct remote_input_t input;
//     input.type = REMOTE_INPUT_MESSAGE_SEND;
//     input.message.ws = NULL; // Broadcast
//     input.message.length = length;
//     input.message.data = string;
//     input.message.timestamp = 0;
//     xQueueSend(xRemote_input, &input, 0);
// }

inline bool remote_sendCommand(char *command, Websock *ws) {
    char *string = (char*) malloc(128 * sizeof(char));
    int length = strlen(command) + 1;
    if (length > 128) {
        free(string);
        return true;
    }
    string[0] = 'c';
    strcpy(string + 1, command);
    struct remote_input_t input;
    input.type = REMOTE_INPUT_MESSAGE_SEND;
    input.message.ws = ws;
    input.message.length = length;
    input.message.data = string;
    input.message.timestamp = 0;
    return (xQueueSend(xRemote_input, &input, 0) != pdTRUE);
}

CgiStatus remote_sendEmbedded(HttpdConnData *connData) {
    const char *fileStart = connData->cgiArg;
    const char *fileEnd = (const char*) connData->cgiArg2 - 1; // Null-Terminator wird nicht gebraucht
    uint32_t *sentLength = (uint32_t*) &connData->cgiData;
    uint32_t fileLength = fileEnd - fileStart;
    uint32_t remainingLength;
    // Prüfe
    if (!fileStart || !fileEnd || fileEnd <= fileStart) return HTTPD_CGI_NOTFOUND;
    if (*sentLength == 0) {
        httpdStartResponse(connData, 200);
        httpdHeader(connData, "Content-Type", httpdGetMimetype(connData->url));
        if (*fileStart == 0x1f && *(fileStart + 1) == 0x8b && *(fileStart + 2) == 0x08) { // Magic-Nummer von gzip-DEFLATE
            httpdHeader(connData, "Content-Encoding", "gzip");
        }
        httpdEndHeaders(connData);
    }
    // sende in Chunks von 1024 Bytes
    remainingLength = fileLength - *sentLength;
    if (remainingLength <= 1024) {
        httpdSend(connData, fileStart + *sentLength, remainingLength);
        *sentLength += remainingLength;
        return HTTPD_CGI_DONE;
    } else {
        httpdSend(connData, fileStart+*sentLength, 1024);
        *sentLength += 1024;
        return HTTPD_CGI_MORE;
    }
}

int remote_printLog(const char * format, va_list arguments) {
    for (uint8_t i = 0; i < 5; ++i) {
        if (format[0] == remote.disabledLogs[i]) {
            return remote.defaultLog(format, arguments);
        }
    }
    char *string = (char*) malloc(128 * sizeof(char));
    int length = 0;
    string[0] = 'l';
    length = vsnprintf(string + 1, 127, format, arguments) + 1;
    if (length <= 0 || length > 128) {
        free(string);
        return remote.defaultLog(format, arguments);
    }
    struct remote_input_t input;
    input.type = REMOTE_INPUT_MESSAGE_SEND;
    input.message.ws = NULL; // Broadcast
    input.message.length = length;
    input.message.data = string;
    input.message.timestamp = 0;
    xQueueSend(xRemote_input, &input, 0);
    return remote.defaultLog(format, arguments);
}

static bool remote_settingSet(struct remote_setting_linked_list_t *currentSetting, char *value) {
    if (!currentSetting || !currentSetting->setting) return true;
    // Wert auslesen
    uint32_t valueNew;
    switch (currentSetting->setting->type) {
        case (SETTING_TYPE_UINT):
            valueNew = (uint32_t) strtoul(value, NULL, 10);
            break;
        case (SETTING_TYPE_INT): {
            int64_t i = strtol(value, NULL, 10);
            if (i > INT32_MAX) i = INT32_MAX;
            else if (i < INT32_MIN) i = INT32_MIN;
            valueNew = (uint32_t) i;
            break;
        }
        case (SETTING_TYPE_FLOAT):
            valueNew = (uint32_t) strtof(value, NULL);
            break;
        default:
            return true;
    }
    uint32_t valueOld = currentSetting->setting->value.ui;
    currentSetting->setting->value.ui = valueNew;
    ESP_LOGD("remote", "key %u -> %u", valueOld, valueNew);
    // weiterleiten
    if (currentSetting->updateCallback(currentSetting->updateCallbackCookie, currentSetting->setting)) {
        currentSetting->setting->value.ui = valueOld; // wiederherstellen
        ESP_LOGD("remote", "key wiederhergestellt");
        return true;
    }
    // NVS aktualisieren
    #if (REMOTE_SETTINGS_USE_NVS == 1)
        nvs_handle nvsHandle;
        if (nvs_open(currentSetting->taskTag, NVS_READWRITE, &nvsHandle)) return true;
        if (nvs_set_u32(nvsHandle, currentSetting->settingTag, currentSetting->setting->value.ui)) return true; // Fehler beim speichern
        if (nvs_commit(nvsHandle)) return true;
        nvs_close(nvsHandle);
    #endif
    return false;
}

static void remote_processSetting(struct remote_input_message_t *message) {
    // Tags aufschlüsseln
    switch (*(message->data+1)) {
        case ('?'): // Auflistung
            break;
        case ('v'): // aktueller Wert
            break;
        case ('s'): { // Wert aktualisieren -> esquadro2:abcd123:321
            char *taskTag = strtok(message->data + 2, ":");
            char *settingTag = strtok(NULL, ":");
            char *value = strtok(NULL, ":");
            struct remote_setting_linked_list_t *currentSetting = remote_settingSelect(taskTag, settingTag);
            bool ret = remote_settingSet(currentSetting, value);
            ESP_LOGD("remote", "settingSet -> %u", ret);
            break;
        }
    }
}

struct remote_setting_linked_list_t* remote_settingSelect(char *taskTag, char *settingTag) {
    // Liste durchsuchen
    struct remote_setting_linked_list_t *currentSetting = remote.settingsHead;
    while (currentSetting->next) {
        if (!strcmp(taskTag, currentSetting->taskTag) && !strcmp(settingTag, currentSetting->settingTag)) {
            return currentSetting;
        }
        currentSetting = currentSetting->next;
    }
    return NULL;
}
