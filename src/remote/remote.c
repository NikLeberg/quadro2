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
#include "tiny-json.h"


/** Interne Abhängigkeiten **/

#include "intercom.h"
#include "resources.h"
#include "remote.h"


/** Variablendeklaration **/

struct remote_t {
    HttpdFreertosInstance httpd;
    char httpdConn[sizeof(RtosConnType) * REMOTE_CONNECTION_COUNT];
    json_t jsonBuffer[8];

    uint8_t connected;
    Websock *ws;
    TickType_t lastContact;
    bool timeoutPending;

    esp_log_level_t logLevel;
    vprintf_like_t defaultLog;
} remote;

static command_t remote_commands[REMOTE_COMMAND_MAX] = {
    COMMAND("resetQueue")
};
static COMMAND_LIST("remote", remote_commands, REMOTE_COMMAND_MAX);

static setting_t remote_settings[REMOTE_SETTING_MAX] = {
    SETTING("logLevel", &remote.logLevel, VALUE_TYPE_UINT)
};
static SETTING_LIST("remote", remote_settings, REMOTE_SETTING_MAX);

static pv_t remote_pvs[REMOTE_PV_MAX] = {
    PV("connections", VALUE_TYPE_UINT),
    PV("timeout", VALUE_TYPE_NONE),
    PV("stateError", VALUE_TYPE_NONE)
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
 * Function: remote_pvForward
 * ----------------------------
 * Leitet per Intercom empfangene Prozessvariabel-Events per WebSocket weiter.
 * 
 * pv_t *pv: empfangener pv
 */
static void remote_pvForward(pv_t *pv);

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
 * Function: remote_intercomGetSet
 * ----------------------------
 * Interpretiere JSON Anfragen für Einstellungen und Parameter.
 * Schreiben & Lesen oder nur Lesen.
 * 
 * remote_message_type_t type: Typ der Liste (siehe intercom.h)
 * json_t *jData: empfangene JSON Antwort
 */
static void remote_intercomGetSet(remote_message_type_t type, const json_t *jData);

/*
 * Function: remote_intercomListSend
 * ----------------------------
 * Baue und sende JSON Array von intercom spezifischen Listen je nach typ.
 * 
 * remote_message_type_t type: Typ der Liste (siehe intercom.h)
 */
static void remote_intercomListSend(remote_message_type_t type);

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

extern char _binary_src_remote_www_index_min_html_start;
extern char _binary_src_remote_www_index_min_html_end;
extern char _binary_src_remote_www_manifest_json_start;
extern char _binary_src_remote_www_manifest_json_end;
extern char _binary_src_remote_www_favicon_svg_gz_start;
extern char _binary_src_remote_www_favicon_svg_gz_end;
extern char _binary_src_remote_www_script_min_js_start;
extern char _binary_src_remote_www_script_min_js_end;

HttpdBuiltInUrl builtInUrls[] = {
    // WebSocket
    ROUTE_WS("/ws", remote_wsConnect),
    // Website
    ROUTE_CGI_ARG2("/", remote_sendEmbedded, &_binary_src_remote_www_index_min_html_start, &_binary_src_remote_www_index_min_html_end),
    ROUTE_CGI_ARG2("/index.html", remote_sendEmbedded, &_binary_src_remote_www_index_min_html_start, &_binary_src_remote_www_index_min_html_end),
    ROUTE_CGI_ARG2("/manifest.json", remote_sendEmbedded, &_binary_src_remote_www_manifest_json_start, &_binary_src_remote_www_manifest_json_end),
    ROUTE_CGI_ARG2("/favicon.svg", remote_sendEmbedded, &_binary_src_remote_www_favicon_svg_gz_start, &_binary_src_remote_www_favicon_svg_gz_end),
    ROUTE_CGI_ARG2("/script.js", remote_sendEmbedded, &_binary_src_remote_www_script_min_js_start, &_binary_src_remote_www_script_min_js_end),
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
    commandRegister(xRemote, remote_commands);
    settingRegister(xRemote, remote_settings);
    pvRegister(xRemote, remote_pvs);
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
    // Loop
    while (true) {
        if (xQueueReceive(xRemote, &event, 500 / portTICK_RATE_MS) == pdTRUE) {
            switch (event.type) {
                case (EVENT_COMMAND): {
                    xQueueReset(xRemote);
                    break;
                }
                case (EVENT_PV): // weiterleiten
                    remote_pvForward((pv_t*)event.data);
                    break;
                case (EVENT_INTERNAL): {
                    break;
                }
                default:
                    break;
            }
        }
        // Timeout erkennen
        if (remote.connected && (xTaskGetTickCount() - remote.lastContact > (REMOTE_TIMEOUT_MS / portTICK_PERIOD_MS))) {
            if (remote.timeoutPending) { // Timeout fällig -> echter Timeout! -> Notstopp
                remote.timeoutPending = false;
                pvPublish(xRemote, REMOTE_PV_TIMEOUT);
            } else {
                remote.timeoutPending = true;
                remote.lastContact = xTaskGetTickCount();
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
                case (WIFI_REASON_AUTH_EXPIRE):
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

static void remote_pvForward(pv_t *pv) {
    uint32_t subscriberNum, pvNum;
    if (intercom_pvIndex(pv, &subscriberNum, &pvNum)) return;
    char buffer[32];
    size_t length;
    switch (pv->type) {
        case (VALUE_TYPE_NONE):
            length = snprintf(buffer, sizeof(buffer), "[%d,[%u,%u,%d,true]]", REMOTE_MESSAGE_PV, subscriberNum, pvNum, VALUE_TYPE_NONE);
            break;
        case (VALUE_TYPE_UINT):
            length = snprintf(buffer, sizeof(buffer), "[%d,[%u,%u,%d,%u]]", REMOTE_MESSAGE_PV, subscriberNum, pvNum, VALUE_TYPE_UINT, pv->value.ui);
            break;
        case (VALUE_TYPE_INT):
            length = snprintf(buffer, sizeof(buffer), "[%d,[%u,%u,%d,%d]]", REMOTE_MESSAGE_PV, subscriberNum, pvNum, VALUE_TYPE_INT, pv->value.i);
            break;
        case (VALUE_TYPE_FLOAT):
            length = snprintf(buffer, sizeof(buffer), "[%d,[%u,%u,%d,%.9g]]", REMOTE_MESSAGE_PV, subscriberNum, pvNum, VALUE_TYPE_FLOAT, pv->value.f);
            break;
        default:
            return;
    }
    if (length > 0 && length < sizeof(buffer)) {
        remote_messageSend(buffer, length);
    }
}

static void remote_messageProcess(char *message, size_t length) {
    message[length] = '\0'; // Fixme!
    const json_t *j = json_create(message, remote.jsonBuffer, sizeof(remote.jsonBuffer) / sizeof(json_t));
    if (j && json_getType(j) == JSON_ARRAY) {
        const json_t *jType = json_getChild(j);
        if (jType && json_getType(jType) == JSON_INTEGER) {
            const json_t *jData = json_getSibling(jType);
            remote_message_type_t type = json_getInteger(jType);
            switch (type) {
                case (REMOTE_MESSAGE_STATE): // [1,1] - ok, [1,0] - error
                    if (message[3] != '1') pvPublish(xRemote, REMOTE_PV_STATE_ERROR);
                    break;
                case (REMOTE_MESSAGE_LOG):
                    break;
                case (REMOTE_MESSAGE_COMMAND): { // [owner, command]
                    if (jData) {
                        const json_t *jOwner = json_getChild(jData);
                        const json_t *jCommand = (jOwner) ? json_getSibling(jOwner) : NULL;
                        if (jCommand && json_getType(jOwner) == JSON_INTEGER && json_getType(jCommand) == JSON_INTEGER) { // valid
                            intercom_commandSend2(json_getInteger(jOwner), json_getInteger(jCommand));
                        }
                    }
                    break;
                }
                case (REMOTE_MESSAGE_SETTING): // [owner, setting, ?value] - get / set
                case (REMOTE_MESSAGE_PARAMETER): // [owner, parameter, ?value] - get / set
                    if (jData) remote_intercomGetSet(type, jData);
                    break;
                case (REMOTE_MESSAGE_PV): { // [publisher, pv] - subscribe
                    if (jData) {
                        const json_t *jPublisher = json_getChild(jData);
                        const json_t *jPv = (jPublisher) ? json_getSibling(jPublisher) : NULL;
                        if (jPv && json_getType(jPublisher) == JSON_INTEGER && json_getType(jPv) == JSON_INTEGER) { // valid
                            pv_t *pv;
                            pv = intercom_pvSubscribe2(xRemote, json_getInteger(jPublisher), json_getInteger(jPv));
                            if (pv && pv->type != VALUE_TYPE_NONE) remote_pvForward(pv); // zuletzt bekannter Zustand schicken
                        }
                    }
                    break;
                }
                case (REMOTE_MESSAGE_COMMANDS): // [type] - lade Liste
                case (REMOTE_MESSAGE_SETTINGS):
                case (REMOTE_MESSAGE_PARAMETERS):
                case (REMOTE_MESSAGE_PVS):
                    if (!jData) remote_intercomListSend(type);
                    break;
                default:
                    break;
            }
        }
    }
    return;
}

typedef value_type_t (*typeFunc_t)(uint32_t, uint32_t);
typedef bool (*getFunc_t)(uint32_t, uint32_t, value_t*);
typedef bool (*setFunc_t)(uint32_t, uint32_t, value_t*);

static void remote_intercomGetSet(remote_message_type_t type, const json_t *jData) {
    // Funktionen auswählen
    typeFunc_t typeFunc;
    getFunc_t getFunc;
    setFunc_t setFunc;
    switch (type) {
        case (REMOTE_MESSAGE_SETTING):
            typeFunc = &intercom_settingType2;
            getFunc = &intercom_settingGet2;
            setFunc = &intercom_settingSet2;
            break;
        case (REMOTE_MESSAGE_PARAMETER):
            typeFunc = &intercom_parameterType2;
            getFunc = &intercom_parameterGet2;
            setFunc = &intercom_parameterSet2;
            break;
        default:
            return;
    }
    const json_t *jNode = json_getChild(jData);
    const json_t *jElement = (jNode) ? json_getSibling(jNode) : NULL;
    const json_t *jValue = (jElement) ? json_getSibling(jElement) : NULL;
    if (jElement && json_getType(jNode) == JSON_INTEGER && json_getType(jElement) == JSON_INTEGER) { // valid
        uint32_t node = json_getInteger(jNode);
        uint32_t element = json_getInteger(jElement);
        value_type_t valueType = typeFunc(node, element);
        value_t value;
        if (jValue) { // set
            if (valueType == VALUE_TYPE_UINT) value.ui = json_getInteger(jValue);
            else if (valueType == VALUE_TYPE_INT) value.i = json_getInteger(jValue);
            else if (valueType == VALUE_TYPE_FLOAT) value.f = json_getReal(jValue);
            else return;
            setFunc(node, element, &value);
        } // get (auch immer nach set)
        getFunc(node, element, &value);
        char buffer[32];
        size_t length;
        if (valueType == VALUE_TYPE_UINT) {
            length = snprintf(buffer, sizeof(buffer), "[%d,[%u,%u,%d,%u]]", type, node, element, valueType, value.ui);
        } else if (valueType == VALUE_TYPE_INT) {
            length = snprintf(buffer, sizeof(buffer), "[%d,[%u,%u,%d,%d]]", type, node, element, valueType, value.i);
        } else if (valueType == VALUE_TYPE_FLOAT) {
            length = snprintf(buffer, sizeof(buffer), "[%d,[%u,%u,%d,%.9g]]", type, node, element, valueType, value.f);
        } else return;
        if (length > 0 && length <= sizeof(buffer)) {
            remote_messageSend(buffer, length);
        }
    }
}

typedef const char* (*nameNodeFunc_t)(uint32_t);
typedef const char* (*nameElementFunc_t)(uint32_t, uint32_t);

static void remote_intercomListSend(remote_message_type_t type) {
    // Funktionen auswählen
    nameNodeFunc_t node;
    nameElementFunc_t element;
    switch (type) {
        case (REMOTE_MESSAGE_COMMANDS):
            node = &intercom_commandNameOwner;
            element = &intercom_commandNameCommand;
            break;
        case (REMOTE_MESSAGE_SETTINGS):
            node = &intercom_settingNameOwner;
            element = &intercom_settingNameSetting;
            break;
        case (REMOTE_MESSAGE_PARAMETERS):
            node = &intercom_parameterNameOwner;
            element = &intercom_parameterNameParameter;
            break;
        case (REMOTE_MESSAGE_PVS):
            node = &intercom_pvNamePublisher;
            element = &intercom_pvNamePv;
            break;
        default:
            return;
    }
    // JSON bauen
    char buffer[1024];
    FILE *f = fmemopen(buffer, sizeof(buffer), "w");
    fprintf(f, "[%d,[", type);
    const char *nodeName, *elementName;
    for (uint32_t i = 0; ; ++i) {
        nodeName = node(i);
        if (!nodeName) break;
        fprintf(f, (i == 0) ? "[\"%s\",[" : ",[\"%s\",[", nodeName);
        for (uint32_t j = 0; ; ++j) {
            elementName = element(i, j);
            if (!elementName) break;
            fprintf(f, (j == 0) ? "\"%s\"" : ",\"%s\"", elementName);
        }
        fputs("]]", f);
    }
    fputs("]]", f);
    remote_messageSend(buffer, ftell(f));
    fclose(f);
}

static void remote_messageSend(char *message, size_t length) {
    if (remote.connected && remote.ws) {
        if (cgiWebsocketSend(&remote.httpd.httpdInstance, remote.ws, message, length, WEBSOCK_FLAG_NONE) <= 0) {
            cgiWebsocketClose(&remote.httpd.httpdInstance, remote.ws, 1011); // Beende mit "Internal Error"
            remote.ws = NULL; // Sendefehler, Websocket inaktiv setzen
        }
    }
}

static void remote_wsReceive(Websock *ws, char *data, int len, int flags) {
    if (ws == remote.ws) {
        remote.lastContact = xTaskGetTickCount();
        remote.timeoutPending = false;
        remote_messageProcess(data, len);
    }
}

static void remote_wsConnect(Websock *ws) {
    // als aktiver ws speichern
    remote.ws = ws;
    ++remote.connected;
    // Callbacks registrieren
	ws->recvCb = remote_wsReceive;
    ws->closeCb = remote_wsDisconnect;
    // Hallo senden
	cgiWebsocketSend(&remote.httpd.httpdInstance, ws, "quadro2", 7, WEBSOCK_FLAG_NONE);
    // Event publizieren
    pvPublishUint(xRemote, REMOTE_PV_CONNECTIONS, remote.connected);
}

static void remote_wsDisconnect(Websock *ws) {
    // inaktiv setzen
    if (remote.ws == ws) remote.ws = NULL;
    --remote.connected;
    // pv-Registrierungen in Intercom löschen
    intercom_pvUnsubscribeAll(xRemote);
    // Event publizieren
    pvPublishUint(xRemote, REMOTE_PV_CONNECTIONS, remote.connected);
}

CgiStatus remote_sendEmbedded(HttpdConnData *connData) {
    const char *fileStart = connData->cgiArg;
    const char *fileEnd = (const char*) connData->cgiArg2 - 1; // Null-Terminator wird nicht gebraucht
    uint32_t *sentLength = (uint32_t*) &connData->cgiData;
    uint32_t fileLength = (uint32_t)fileEnd - (uint32_t)fileStart;
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
    if (remote.logLevel) {
        bool shouldForward = false;
        switch (format[0]) {
            case ('E'):
                shouldForward = true;
                break;
            case ('W'):
                shouldForward = (remote.logLevel >= ESP_LOG_WARN);
                break;
            case ('I'):
                shouldForward = (remote.logLevel >= ESP_LOG_INFO);
                break;
            case ('D'):
                shouldForward = (remote.logLevel >= ESP_LOG_DEBUG);
                break;
            case ('V'):
                shouldForward = (remote.logLevel == ESP_LOG_VERBOSE);
                break;
            default:
                shouldForward = false;
                break;
        }
        if (shouldForward) {
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
            remote_messageSend(buffer, length);
        }
    }
    return remote.defaultLog(format, arguments);
}
