/*
 * File: interComm.c
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

#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include <string.h>
#include <stdio.h>
#include "nvs.h"
#include "nvs_flash.h"
#include "esp_log.h"


/** Interne Abhängigkeiten **/

#include "intercom.h"


/** Variablendeklaration **/

static struct {
    command_list_t *commandHead;
    setting_list_t *settingHead;
    parameter_list_t *parameterHead;
    pv_list_t *pvHead;
} intercom;


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

void intercom_commandRegister(QueueHandle_t owner, command_list_t *list) {
    // an Linked-List anhängen
    list->owner = owner;
    command_list_t *last = intercom.commandHead;
    if (last) {
        while (last->next) {
            last = last->next;
        }
        last->next = list;
    } else { // erstes
        intercom.commandHead = list;
    }
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

// sensors:00(setHome:00,resetFusion:01,setAltToGps:02,)
char* intercom_commandList() {
    // Grösse ermitteln
    size_t strLen = 1; // '\0'
    command_list_t *node = intercom.commandHead;
    while (node) {
        strLen += 5; // _:00(_)
        strLen += strlen(node->task);
        for (size_t i = 0; i < node->length; ++i) {
            strLen += strlen(node->commands[i].name);
            strLen += 4; // _:00,
        }
        node = node->next;
    }
    char *str = malloc(strLen);
    // Zusammensetzen
    FILE *f = fmemopen(str, strLen, "w");
    size_t nodeCount = 0;
    node = intercom.commandHead;
    while (node) {
        fprintf(f, "%s:%02d(", node->task, nodeCount);
        for (size_t i = 0; i < node->length; ++i) {
            fprintf(f, "%s:%02d,", node->commands[i].name, i);
        }
        fprintf(f, ")");
        node = node->next;
        ++nodeCount;
    }
    fclose(f);
    return str;
}


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

void intercom_settingRegister(QueueHandle_t owner, setting_list_t *list) {
    // an Linked-List anhängen
    list->owner = owner;
    setting_list_t *last = intercom.settingHead;
    if (last) {
        while (last->next) {
            last = last->next;
        }
        last->next = list;
    } else { // erstes
        intercom.settingHead = list;
    }
    // Einstellung aus NVS laden
    nvs_handle nvs;
    esp_err_t err;
    err = nvs_open(list->task, NVS_READWRITE, &nvs);
    if (err == ESP_ERR_NVS_NOT_INITIALIZED) {
        esp_log_level_set("nvs", ESP_LOG_INFO);
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

void intercom_parameterRegister(QueueHandle_t owner, parameter_list_t *list) {
    // an Linked-List anhängen
    list->owner = owner;
    parameter_list_t *last = intercom.parameterHead;
    if (last) {
        while (last->next) {
            last = last->next;
        }
        last->next = list;
    } else { // erstes
        intercom.parameterHead = list;
    }
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

void intercom_pvRegister(QueueHandle_t publisher, pv_list_t *list) {
    // an Linked-List anhängen
    list->publisher = publisher;
    pv_list_t *last = intercom.pvHead;
    if (last) {
        while (last->next) {
            last = last->next;
        }
        last->next = list;
    } else { // erstes
        intercom.pvHead = list;
    }
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
