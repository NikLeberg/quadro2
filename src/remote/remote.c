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
#include "esp.h" //libesphttpd
#include "httpd.h"
#include "httpd-freertos.h"
#include "httpd-freertos.c"
#include "cgiwebsocket.h"
#include "route.h"
#include "ringbuf.h"
#include "tiny-json.h"


/** Interne Abhängigkeiten **/

#include "intercom.h"
#include "resources.h"
#include "remote.h"


/** Variablendeklaration **/

typedef enum {
    REMOTE_EVENT_WS_RECEIVE = 0,
    REMOTE_EVENT_WS_SEND
} remote_event_t;

struct remote_t {
    HttpdFreertosInstance httpd;
    char httpdConn[sizeof(RtosConnType) * REMOTE_CONNECTION_COUNT];
    RingbufHandle_t wsRxBuffer, wsTxBuffer;
    json_t jsonBuffer[8];

    uint8_t connected;
    Websock *ws;

    vprintf_like_t defaultLog;
} remote;

static pv_t remote_pvs[REMOTE_PV_MAX] = {
    PV("connections", VALUE_TYPE_UINT),
    PV("timeout", VALUE_TYPE_NONE)
};
static PV_LIST("remote", remote_pvs, REMOTE_PV_MAX);


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
 * Gesendete Nachrichten sind eine art relaxed JSON die per eval() in ein Objekt gewandelt werden:
 *  var j = eval('([0,[...]])');
 * Empfangene Nachrichten sind valides JSON die per cJSON geparst werden.
 * 
 * JSON Layout:
 *  [ Nachrichtentyp, Daten der Nachricht (Text, Zahl oder JSON) ]
 */
typedef enum {
    REMOTE_MESSAGE_STATE = 1,   // Zahl, 0: Error, 1: Alles i.O.
    REMOTE_MESSAGE_LOG,         // String mit Logtext
    REMOTE_MESSAGE_COMMAND,     // JSON: [ Owner, Command ]
    REMOTE_MESSAGE_SETTING,     // JSON: [ Owner, Setting, Wert ] -> mit Wert: Schreiben, ohne: Lesen
    REMOTE_MESSAGE_PARAMETER,   // JSON: [ Owner, Parameter, Wert ] -> mit Wert: Schreiben, ohne: Lesen
    REMOTE_MESSAGE_PV,          // JSON: [ Publisher, PV, Wert ] -> mit Wert: Publication, ohne: Subscribe
    REMOTE_MESSAGE_COMMANDS,    // JSON: [ [ "owner", [ "command1", "command2", ... ] ], ... ]
    REMOTE_MESSAGE_SETTINGS,    // JSON: [ [ "owner", [ "setting1", "setting2", ... ] ], ... ]
    REMOTE_MESSAGE_PARAMETERS,  // JSON: [ [ "owner", [ "parameter1", "parameter2", ... ] ], ... ]
    REMOTE_MESSAGE_PVS,         // JSON: [ [ "owner", [ "pv1", "pv2", ... ] ], ... ]
} remote_message_type_t;

/*
 * Function: remote_messageProcess
 * ----------------------------
 * Per Websocket empfangene Nachricht verarbeiten, weiterleiten.
 * 
 * char *message: Nachricht
 * size_t length: Länge der Nachricht
 */
static void remote_messageProcess(char *message, size_t length);

/*
 * Function: remote_messageSend
 * ----------------------------
 * Nachricht per WebSocket senden. (nur an den letzten aktiven Websocket)
 * 
 * char *message: Nachricht
 * size_t length: Länge der Nachricht
 */
static void remote_messageSend(char *message, size_t length);


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
    // Wlan starten
    esp_log_level_set("efuse", ESP_LOG_INFO);
    if (remote_initWlan(ssid, pw)) return true;
    // Intercom-Queue erstellen
    xRemote = xQueueCreate(32, sizeof(event_t));
    // an Intercom anbinden
    pvRegister(xRemote, remote_pvs);
    // Rx/Tx Ringbuffer erstellen
    remote.wsRxBuffer = xRingbufferCreate(1 * 1024, RINGBUF_TYPE_NOSPLIT);
    remote.wsTxBuffer = xRingbufferCreate(1 * 1024, RINGBUF_TYPE_NOSPLIT);
    // Task starten
    if (xTaskCreate(&remote_task, "remote", 3 * 1024, NULL, xRemote_PRIORITY, NULL) != pdTRUE) return true;
    // libesphttpd Bibliothek starten
	if (httpdFreertosInit(&remote.httpd, builtInUrls, 80U, remote.httpdConn, REMOTE_CONNECTION_COUNT, HTTPD_FLAG_NONE)) return true;
	if (httpdFreertosStart(&remote.httpd)) return true;
    esp_log_level_set("cgiwebsocket", ESP_LOG_INFO);
    esp_log_level_set("httpd-freertos", ESP_LOG_INFO);
    // Log umleiten
    remote.defaultLog = esp_log_set_vprintf(&remote_printLog);
    return false;
}

void remote_task(void* arg) {
    // Variablen
    event_t event;
    bool timeoutPending = false;
    TickType_t lastContact = xTaskGetTickCount();
    // Loop
    while (true) {
        if (xQueueReceive(xRemote, &event, 500 / portTICK_RATE_MS) == pdTRUE) {
            switch (event.type) {
                case (EVENT_COMMAND): // keine Befehle
                    break;
                case (EVENT_PV): // keine Subscriptions
                    break;
                case (EVENT_INTERNAL): { // Daten in Rx / Tx Ringbuffer
                    remote_event_t type = (remote_event_t)event.data;
                    size_t length;
                    char *message = NULL;
                    if (type == REMOTE_EVENT_WS_RECEIVE) {
                        message = xRingbufferReceive(remote.wsRxBuffer, &length, 0);
                        if (message) {
                            remote_messageProcess(message, length);
                            lastContact = xTaskGetTickCount();
                            timeoutPending = false;
                            vRingbufferReturnItem(remote.wsRxBuffer, message);
                        }
                    } else if (type == REMOTE_EVENT_WS_SEND) {
                        message = xRingbufferReceive(remote.wsTxBuffer, &length, 0);
                        if (message) {
                            remote_messageSend(message, length);
                            vRingbufferReturnItem(remote.wsTxBuffer, message);
                        }
                    }
                    break;
                }
                default:
                    break;
            }
        }
        // Timeout erkennen
        if (remote.connected && (xTaskGetTickCount() - lastContact > (REMOTE_TIMEOUT_MS / portTICK_PERIOD_MS))) {
            if (timeoutPending) { // Timeout fällig -> echter Timeout! -> Notstopp
                timeoutPending = false;
                pvPublish(xRemote, REMOTE_PV_TIMEOUT);
            } else {
                timeoutPending = true;
                lastContact = xTaskGetTickCount();
                char message[] = "[ ]";
                message[1] = REMOTE_MESSAGE_STATE + 48; // zu ASCII
                remote_messageSend(message, sizeof(message) - 1);
            }
        }
    }
}

static esp_err_t remote_connectionEventHandler(void *ctx, system_event_t *event) {
    ESP_LOGD("remote", "WiFi Event: %d", event->event_id);
    switch(event->event_id) {
        case SYSTEM_EVENT_STA_START:
            esp_wifi_connect();
            break;
        case SYSTEM_EVENT_STA_GOT_IP: {
            uint32_t ip = event->event_info.got_ip.ip_info.ip.addr;
            ESP_LOGI("remote", "IP - %u.%u.%u.%u", ip & 0xff, (ip >> 8) & 0xff, (ip >> 16) & 0xff, (ip >> 24));
            break;
        }
        case SYSTEM_EVENT_STA_DISCONNECTED:
            // Verbindung wiederherstellen, wenn nicht absichtlich
            switch(event->event_info.disconnected.reason) {
                case (WIFI_REASON_AUTH_FAIL):
                    #if (REMOTE_RESET_AFTER_STOP == 0)
                        break;
                    #endif
                case (WIFI_REASON_ASSOC_LEAVE):
                    esp_wifi_stop();
            }
            break;
        case SYSTEM_EVENT_STA_STOP:
            #if (REMOTE_START_AFTER_STOP == 1)
                esp_wifi_start();
            #elif (REMOTE_RESET_AFTER_STOP == 1)
                esp_restart();
            #endif
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

static void remote_messageProcess(char *message, size_t length) {
    message[length] = '\0';
    const json_t *j = json_create(message, remote.jsonBuffer, sizeof(remote.jsonBuffer) / sizeof(json_t));
    if (j && json_getType(j) == JSON_ARRAY) {
        const json_t *jType = json_getChild(j);
        if (jType && json_getType(jType) == JSON_INTEGER) {
            const json_t *jData = json_getSibling(jType);
            remote_message_type_t type = json_getInteger(jType);
            switch (type) {
                case (REMOTE_MESSAGE_STATE):
                    break;
                case (REMOTE_MESSAGE_COMMAND): { // [owner, command]
                    if (jData) {
                        const json_t *jOwner = json_getChild(jData);
                        const json_t *jCommand = (jOwner) ? json_getSibling(jOwner) : NULL;
                        if (jCommand && json_getType(jOwner) == JSON_INTEGER && json_getType(jCommand) == JSON_INTEGER) {
                            intercom_commandSend2(json_getInteger(jOwner), json_getInteger(jCommand));
                        }
                        break;
                    }
                }
                case (REMOTE_MESSAGE_SETTING): // [owner, setting, ?value]
                    break;
                case (REMOTE_MESSAGE_PARAMETER): // [owner, parameter, ?value]
                    break;
                case (REMOTE_MESSAGE_PV):
                    break;
                case (REMOTE_MESSAGE_COMMANDS): {
                    if (!jData) {
                        char buffer[1024];
                        size_t length = 0;
                        FILE *f = fmemopen(buffer, sizeof(buffer), "w");
                        length += fprintf(f, "[%d,[", REMOTE_MESSAGE_COMMANDS);
                        const char *owner, *command;
                        uint32_t i = 0, j = 0;
                        while (true) {
                            owner = intercom_commandOwnerName(i);
                            if (!owner) break;
                            length += fprintf(f, (i == 0) ? "[\"%s\",[" : ",[\"%s\",[", owner);
                            j = 0;
                            while (true) {
                                command = intercom_commandCommandName(i, j);
                                if (!command) break;
                                length += fprintf(f, (j == 0) ? "\"%s\"" : ",\"%s\"", command);
                                j++;
                            }
                            fputs("]]", f);
                            length += 2;
                            i++;
                        }
                        fputs("]]", f);
                        length += 2;
                        fclose(f);
                        buffer[length] = '\0';
                        remote_messageSend(buffer, length);
                    }
                    break;
                }
                case (REMOTE_MESSAGE_SETTINGS):
                    break;
                case (REMOTE_MESSAGE_PARAMETERS):
                    break;
                case (REMOTE_MESSAGE_PVS):
                    break;
                case (REMOTE_MESSAGE_LOG):
                default:
                    break;
            }
        }
    }
    return;
}

static void remote_messageSend(char *message, size_t length) {
    if (remote.connected && remote.ws) {
        if (cgiWebsocketSend(&remote.httpd.httpdInstance, remote.ws, message, length, WEBSOCK_FLAG_NONE) <= 0) {
            remote.ws = NULL; // Sendefehler, Websocket inaktiv setzen
        }
    }
}

static void remote_wsReceive(Websock *ws, char *data, int len, int flags) {
    if (xRingbufferSend(remote.wsRxBuffer, data, len, 0) == pdTRUE) {
        event_t event = {EVENT_INTERNAL, REMOTE_EVENT_WS_RECEIVE};
        xQueueSendToBack(xRemote, &event, 0);
    }
}

static void remote_wsConnect(Websock *ws) {
    // als aktiver ws speichern
    remote.ws = ws;
    ++remote.connected;
    // Event publizieren
    pvPublishUint(xRemote, REMOTE_PV_CONNECTIONS, remote.connected);
    // Callbacks registrieren
	ws->recvCb = remote_wsReceive;
    ws->closeCb = remote_wsDisconnect;
    // Hallo senden
	cgiWebsocketSend(&remote.httpd.httpdInstance, ws, "quadro2", 7, WEBSOCK_FLAG_NONE);
}

static void remote_wsDisconnect(Websock *ws) {
    // inaktiv setzen
    if (remote.ws == ws) remote.ws = NULL;
    --remote.connected;
    // Event publizieren
    pvPublishUint(xRemote, REMOTE_PV_CONNECTIONS, remote.connected);
}

inline bool remote_sendCommand(char *command, Websock *ws) {
    return false;
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
    char buffer[128];
    size_t length = 0;
    FILE *f = fmemopen(buffer, sizeof(buffer), "w");
    length += fprintf(f, "[%d,\"", REMOTE_MESSAGE_LOG);
    length += vfprintf(f, format, arguments);
    length += fprintf(f, "\"]");
    fclose(f);
    // RET entfernen
    for (size_t i = 0; i <= length; ++i) {
        if (buffer[i] == '\n') buffer[i] = ' ';
    }
    if (xRingbufferSend(remote.wsTxBuffer, buffer, length, 0) == pdTRUE) {
        event_t event = {EVENT_INTERNAL, (void*)REMOTE_EVENT_WS_SEND};
        xQueueSendToBack(xRemote, &event, 0);
    }
    return remote.defaultLog(format, arguments);
}
