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
 *  - Task aktualisiert Variabel und informiert registrierte Tasks
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
    /* EVENT_SETTING,
    EVENT_PARAMETER, */
    EVENT_PV,
    EVENT_INTERNAL
} event_type_t;

typedef struct {
    event_type_t type;
    void *data;
} event_t;

typedef enum {
    VALUE_TYPE_NONE,    // nur Event
    VALUE_TYPE_UINT,
    VALUE_TYPE_INT,
    VALUE_TYPE_FLOAT,
    VALUE_TYPE_POINTER  // Pointer zu einer Variabel abnormalen Typs
} value_type_t;

typedef union {
    uint32_t ui;
    int32_t i;
    float f;
    void *p;
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
    command_t *commands;
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

typedef struct parameter_list_s {
    const char *task;
    QueueHandle_t owner;
    parameter_t *parameters;
    size_t length;
    struct parameter_list_s *next;
} parameter_list_t;

#define PARAMETER_LIST(task, parameters, length)    parameter_list_t parameters##_list = {task, NULL, parameters, length, NULL};

#define parameterSet(owner, parameterNum, value)    intercom_parameterSet(owner, parameterNum, (void*)&value)

#define parameterGet(owner, parameterNum, variable) intercom_parameterGet(owner, parameterNum, (void*)&variable)

bool intercom_parameterSet(QueueHandle_t owner, uint32_t parameterNum, void *value);
bool intercom_parameterGet(QueueHandle_t owner, uint32_t parameterNum, void *variable);


/*
 * PV - Prozessvariabel
 * ----------------------------
 * Kommunikation per Publish / Subscribe.
 * 1. Publisher-Task stellt Events / Prozessvariabeln zur verfügung:
 *      typedef enum {
 *          SENSORS_PV_X = 0,
 *          SENSORS_PV_Y,
 *          SENSORS_PV_Z,
 *          SENSORS_PV_MAX
 *      } sensors_pv_t;
 *      pv_t sensors_pvs[SENSORS_PV_MAX] = {
 *          PV("x", VALUE_TYPE_FLOAT),
 *          PV("y", VALUE_TYPE_FLOAT),
 *          PV("z", VALUE_TYPE_FLOAT),
 *      };
 *      PV_LIST("sensors", sensors_pvs, SENSORS_PV_MAX);
 *      pvRegister(xSensors, sensors_pvs);
 * 
 * 2. Subscriber-Task registriert sich für Updates:
 *      pv_t *subscription = intercom_pvSubscribe(xRemote, xSensors, SENSORS_PV_X);
 * 
 * 2. Publisher publiziert neue Werte an Intercom welche diese in die
 *    Empfangsqueues der registrierten Subscriber sendet:
 *      pvPublishFloat(xSensors, SENSORS_PV_X, 1.0f);
 *      float x = pvGetFloat(subscription);
 */

#define INTERCOM_PV_MAX_SUBSCRIBERS 3

typedef struct {
	const char *name;
    value_type_t type;
    value_t value;
    QueueHandle_t subscribers[INTERCOM_PV_MAX_SUBSCRIBERS];
} pv_t;

#define PV(name, type)  {(name), (type), {0}, {NULL}}

typedef struct pv_list_s {
    const char *task;
    QueueHandle_t publisher;
    pv_t *pvs;
    size_t length;
    struct pv_list_s *next;
} pv_list_t;

#define PV_LIST(task, pvs, length)  pv_list_t pvs##_list = {task, NULL, pvs, length, NULL};

#define pvRegister(queue, pvs)      intercom_pvRegister(queue, &(pvs##_list))

#define pvPublish(publisher, pvNum)             intercom_pvPublish(publisher, pvNum, (value_t)0);
#define pvPublishUint(publisher, pvNum, value)  intercom_pvPublish(publisher, pvNum, (value_t){.ui = value})
#define pvPublishInt(publisher, pvNum, value)   intercom_pvPublish(publisher, pvNum, (value_t){.i = value})
#define pvPublishFloat(publisher, pvNum, value) intercom_pvPublish(publisher, pvNum, (value_t){.f = value})

#define pvGet(pv)           (pv->type == VALUE_TYPE_NONE ? true : false)
#define pvGetUint(pv)       (pv->type == VALUE_TYPE_UINT ? pv->value.ui : 0UL)
#define pvGetInt(pv)        (pv->type == VALUE_TYPE_INT ? pv->value.i : 0L)
#define pvGetFloat(pv)      (pv->type == VALUE_TYPE_FLOAT ? pv->value.f : 0.0f)
#define pvGetPointer(pv)    (pv->type == VALUE_TYPE_POINTER ? pv->value.p : NULL)

void intercom_pvRegister(QueueHandle_t publisher, pv_list_t *list);
pv_t *intercom_pvSubscribe(QueueHandle_t subscriber, QueueHandle_t publisher, uint32_t pvNum);
void intercom_pvPublish(QueueHandle_t publisher, uint32_t pvNum, value_t value);


/** To be made Private **/

static struct {
    command_list_t *commandHead;
    setting_list_t *settingHead;
    parameter_list_t *parameterHead;
    pv_list_t *pvHead;
} intercom;

void intercom_commandRegister(QueueHandle_t owner, command_list_t *list) {
    // an Linked-List anhängen
    list->owner = owner;
    command_list_t *last = intercom.commandHead;
    while (last) {
        last = last->next;
    }
    last->next = list;
}

void intercom_commandSend(QueueHandle_t owner, uint32_t commandNum) {
    // Owner suchen
    command_list_t *node = intercom.commandHead;
    while (true) {
        if (!node) return NULL; // Owner nicht vorhanden
        if (node->owner != owner) break;
        node = node->next;
    }
    // Prüfen
    if (node->length < commandNum) return; // Command nicht vorhanden
    // Senden
    event_t event = {EVENT_COMMAND, (void*)commandNum};
    xQueueSendToBack(owner, &event, 0);
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
    esp_err_t err;
    err = nvs_open(list->task, NVS_READWRITE, &nvs);
    if (err == ESP_ERR_NVS_NOT_INITIALIZED) {
        if (nvs_flash_init()) {
            if (nvs_flash_erase() || nvs_flash_init()) return;
        }
        nvs_open(list->task, NVS_READWRITE, &nvs);
    } else if (err) return;
    for (size_t i = 0; i < list->length; ++i) {
        err = nvs_get_u32(nvs, list->settings[i].name, list->settings[i].address);
        if (err == ESP_ERR_NVS_NOT_FOUND) {
            if (nvs_set_u32(nvs, list->settings[i].name, *(uint32_t*)(list->settings[i].address))) return;
            if (nvs_commit(nvs)) return;
        } else if (err) return;
    }
}

void intercom_parameterRegister(QueueHandle_t owner, parameter_list_t *list) {
    // an Linked-List anhängen
    list->owner = owner;
    parameter_list_t *last = intercom.parameterHead;
    while (last) {
        last = last->next;
    }
    last->next = list;
}

static parameter_t *intercom_parameterSearch(QueueHandle_t owner, uint32_t parameterNum) {
    // Owner suchen
    parameter_list_t *node = intercom.parameterHead;
    while (true) {
        if (!node) return NULL; // Owner nicht vorhanden
        if (node->owner != owner) break;
        node = node->next;
    }
    // Prüfen
    if (node->length < parameterNum) return NULL; // Parameter nicht vorhanden
    return &node->parameters[parameterNum];
}

bool intercom_parameterSet(QueueHandle_t owner, uint32_t parameterNum, void *value) {
    parameter_t *parameter = intercom_parameterSearch(owner, parameterNum);
    if (!parameter) return true;
    switch (parameter->type) {
        case (VALUE_TYPE_UINT):
            *(uint32_t*)parameter->address = *(uint32_t*)value;
            break;
        case (VALUE_TYPE_INT):
            *(int32_t*)parameter->address = *(int32_t*)value;
            break;
        case (VALUE_TYPE_FLOAT):
            *(float*)parameter->address = *(float*)value;
            break;
        default:
            break;
    }
    return false;
}

bool intercom_parameterGet(QueueHandle_t owner, uint32_t parameterNum, void *variable) {
    parameter_t *parameter = intercom_parameterSearch(owner, parameterNum);
    if (!parameter) return true;
    switch (parameter->type) {
        case (VALUE_TYPE_UINT):
            *(uint32_t*)variable = *(uint32_t*)parameter->address;
            break;
        case (VALUE_TYPE_INT):
            *(int32_t*)variable = *(int32_t*)parameter->address;
            break;
        case (VALUE_TYPE_FLOAT):
            *(float*)variable = *(float*)parameter->address;
            break;
        default:
            break;
    }
    return false;
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

static pv_t *intercom_pvSearch(QueueHandle_t publisher, uint32_t pvNum) {
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

pv_t *intercom_pvSubscribe(QueueHandle_t subscriber, QueueHandle_t publisher, uint32_t pvNum) {
    pv_t *pv = intercom_pvSearch(publisher, pvNum);
    if (!pv) return NULL;
    // Subscriber speichern
    for (uint8_t i = 0; i <= INTERCOM_PV_MAX_SUBSCRIBERS; ++i) {
        if (i == INTERCOM_PV_MAX_SUBSCRIBERS) return NULL; // kein Platz verfügbar
        if (pv->subscribers[i]) continue;
        pv->subscribers[i] = subscriber;
    }
    return pv;
}

void intercom_pvPublish(QueueHandle_t publisher, uint32_t pvNum, value_t value) {
    pv_t *pv = intercom_pvSearch(publisher, pvNum);
    if (!pv) return;
    pv->value = value;
    // an alle Subscriber senden
    event_t event = {EVENT_PV, pv};
    for (uint8_t i = 0; i < INTERCOM_PV_MAX_SUBSCRIBERS; ++i) {
        if (pv->subscribers[i]) break;
        xQueueSendToBack(pv->subscribers[i], &event, 0);
    }
}
