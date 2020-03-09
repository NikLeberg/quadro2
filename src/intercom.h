/*
 * File: interComm.h
 * ----------------------------
 * Author: Niklaus Leuenberger
 * Date:   2020-02-04
 * ----------------------------
 * Standartisierte Schnittstellen zwischen den Modulen.
 * Jedes Modul hat eine Queue des Typs event_t welches von Verschiedenen Quellen mit
 * Events gefüllt werden kann:
 * 
 * COMMAND:
 *  - Befehl ausführen
 * 
 * SETTING:
 *  - kein wirklicher Input, sondern der Wert wird direkt geändert
 *  - permanente Einstellungen
 * 
 * PARAMETER:
 *  - kein wirklicher Input, sondern der Wert wird direkt geändert
 *  - nicht permanenter Wert
 * 
 * PV:
 *  - Prozessvariable
 *  - Task bietet Variablen an
 *  - Task der die Werte aktualisiert informiert hiermit registrierte andere Tasks
 *  - sozusagen Publish / Subscribe Funktionalität
 *  - reine Events per VALUE_TYPE_NONE übermittelbar
 * 
 * INTERNAL:
 *  - für Taskinterne Events
 * 
 */


/** Externe Abhängigkeiten **/

#include "esp_system.h"
#include "nvs.h"
#include "nvs_flash.h"


/** Interne Abhängigkeiten **/


/** Variablendeklaration **/

typedef enum {
    EVENT_COMMAND = 0,
    EVENT_SETTING,
    EVENT_PARAMETER,
    EVENT_PV,
    EVENT_INTERNAL
} event_type_t;

typedef struct {
    event_type_t type;
    void *data;
} event_t;

typedef enum {
    VALUE_TYPE_NONE,
    VALUE_TYPE_UINT,
    VALUE_TYPE_INT,
    VALUE_TYPE_FLOAT
} value_type_t;

typedef union {
    uint32_t ui;
    int32_t i;
    float f;
} value_t;


/*
 * Types: Befehle
 * ----------------------------
 * Variabeltypen zur Definition von Befehlen im Sinne von:
 * 
 * // static command_t sensors_commands[SENSORS_COMMAND_MAX] = {
 * //     COMMAND("setHome"),
 * //     ...
 * // }
 * 
 * Intern ansprechbar per Enumerator resp. command_t[x].
 * Extern ansprechbar per String.
 */

typedef struct {
	const char *name;
} command_t;

#define COMMAND(name)  {(name)}

typedef struct command_list_s {
    const char *task;
    QueueHandle_t owner;
    command_t *settings;
    size_t length;
    struct command_list_s *next;
} command_list_t;

#define COMMAND_LIST(task, commands, length)    command_list_t commands##_list = {task, NULL, commands, length, NULL};

#define commandRegister(queue, commands)        intercom_commandRegister(queue, &(commands##_list))


/*
 * Types: Einstellungen
 * ----------------------------
 * Variabeltypen zur Definition von Einstellungen im Sinne von:
 * 
 * // static setting_t sensors_settings[SENSORS_SETTING_MAX] = {
 * //     SETTING("fuseZ_errorAcceleration", &test, SETTING_TYPE_FLOAT),
 * //     ...
 * // }
 * 
 * Intern ansprechbar per Enumerator resp. setting_t[x].
 * Extern ansprechbar per String.
 */

typedef struct {
	const char *name;
    void *address;
    value_type_t type;
} setting_t;

#define SETTING(name, address, type)        {(name), (void *)(address), (type)}

typedef struct setting_list_s {
    const char *task;
    QueueHandle_t owner;
    setting_t *settings;
    size_t length;
    struct setting_list_s *next;
} setting_list_t;

#define SETTING_LIST(task, settings, length)    setting_list_t settings##_list = {task, NULL, settings, length, NULL};

#define settingRegister(queue, settings)        intercom_settingRegister(queue, &(settings##_list))


/*
 * Types: Parameter
 * ----------------------------
 * Variabeltypen zur Definition von Parametern im Sinne von:
 * 
 * // static parameter_t sensors_parameters[SENSORS_PARAMETERS_MAX] = {
 * //     PARAMETER("fuseZ_errorAcceleration", &test, PARAMETER_TYPE_FLOAT),
 * //     ...
 * // }
 * 
 * Intern ansprechbar per Enumerator resp. parameter_t[x].
 * Extern ansprechbar per String.
 */

typedef struct {
	const char *name;
    void *address;
    value_type_t type;
} parameter_t;

#define PARAMETER(name, address, type)  {(name), (void *)(address), (type)}


/*
 * Types: Prozessvariabel
 * ----------------------------
 * Variabeltypen zur Definition von Pvs
 * 
 * Intern ansprechbar per Enumerator resp. pv_t[x].
 * Extern ansprechbar per String.
 */

#define INTERCOM_PV_MAX_SUBSCRIBERS 3

typedef struct {
	const char *name;
    value_type_t type;
    value_t value;
    QueueHandle_t subscribers[INTERCOM_PV_MAX_SUBSCRIBERS];
} pv_t;

#define PV(name, type)  {(name), (type), 0, {NULL}}

typedef struct pv_list_s {
    const char *task;
    QueueHandle_t publisher;
    pv_t *pvs;
    size_t length;
    struct pv_list_s *next;
} pv_list_t;

#define PV_LIST(task, pvs, length)  pv_list_t pvs##_list = {task, NULL, pvs, length, NULL};

#define pvRegister(queue, pvs)      intercom_pvRegister(queue, &(pvs##_list))

typedef struct {
    QueueHandle_t publisher;
    uint32_t pv;
    value_type_t type;
    value_t value;
} pv_event_t;


/** To be made Private **/

static struct {
    command_list_t *commandHead;
    setting_list_t *settingHead;
    pv_list_t *pvHead;
} intercom;

bool intercom_init() {
    if (nvs_flash_init()) {
        if (nvs_flash_erase() || nvs_flash_init()) return true;
    }
    return false;
}

void intercom_commandRegister(QueueHandle_t owner, command_list_t *list) {
    // an Linked-List anhängen
    list->owner = owner;
    command_list_t *last = intercom.commandHead;
    while (last) {
        last = last->next;
    }
    last->next = list;
}

void intercom_settingRegister(QueueHandle_t owner, setting_list_t *list) {
    // an Linked-List anhängen
    list->owner = owner;
    setting_list_t *last = intercom.settingHead;
    while (last) {
        last = last->next;
    }
    last->next = list;
    // Einstellung aus NVS laden
    nvs_handle nvs;
    if (nvs_open(list->task, NVS_READWRITE, &nvs)) return;
    for (size_t i = 0; i < list->length; ++i) {
        esp_err_t ret = nvs_get_u32(nvs, list->settings[i].name, list->settings[i].address);
        if (ret == ESP_ERR_NVS_NOT_FOUND) {
            if (nvs_set_u32(nvs, list->settings[i].name, *(uint32_t*)(list->settings[i].address))) return;
            if (nvs_commit(nvs)) return;
        } else if (ret) return;
    }
}

void intercom_pvRegister(QueueHandle_t publisher, pv_list_t *list) {
    // an Linked-List anhängen
    list->publisher = publisher;
    pv_list_t *last = intercom.pvHead;
    while (last) {
        last = last->next;
    }
    last->next = list;
}

pv_t* intercom_pvGet(QueueHandle_t publisher, uint32_t pvNum) {
    // Publisher suchen
    pv_list_t *node = intercom.pvHead;
    while (true) {
        if (!node) return NULL; // Publisher nicht vorhanden
        if (node->publisher != publisher) break;
        node = node->next;
    }
    // Prüfen
    if (node->length < pvNum) return NULL; // PV nicht vorhanden
    return &node->pvs[pvNum];
}

bool intercom_pvSubscribe(QueueHandle_t subscriber, QueueHandle_t publisher, uint32_t pvNum) {
    pv_t *pv = intercom_pvGet(publisher, pvNum);
    if (!pv) return true;
    // Subscriber speichern
    for (uint8_t i = 0; i <= INTERCOM_PV_MAX_SUBSCRIBERS; ++i) {
        if (i == INTERCOM_PV_MAX_SUBSCRIBERS) return true; // kein Platz verfügbar
        if (pv->subscribers[i]) continue;
        pv->subscribers[i] = subscriber;
    }
    return false;
}

void intercom_pvPublish(QueueHandle_t publisher, uint32_t pvNum) {
    pv_t *pv = intercom_pvGet(publisher, pvNum);
    if (!pv) return true;
    // an alle Subscriber senden
    for (uint8_t i = 0; i < INTERCOM_PV_MAX_SUBSCRIBERS; ++i) {
        if (pv->subscribers[i]) break;
        xQueueSendToBack(pv->subscribers[i], 0, 0); // ToDo!
    }
}
