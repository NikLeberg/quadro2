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

static command_list_t *intercom_commandSearchOwner(QueueHandle_t owner) {
    // Owner suchen
    command_list_t *node = intercom.commandHead;
    while (true) {
        if (!node) return NULL; // Owner nicht vorhanden
        if (node->owner == owner) break;
        node = node->next;
    }
    return node;
}

static command_list_t *intercom_commandSearchOwner2(uint32_t ownerNum) {
    // Owner suchen
    command_list_t *node = intercom.commandHead;
    if (!node) return NULL;
    while (ownerNum--) {
        node = node->next;
        if (!node) return NULL; // Owner nicht vorhanden
    }
    return node;
}

void intercom_commandSend(QueueHandle_t owner, uint32_t commandNum) {
    command_list_t *node = intercom_commandSearchOwner(owner);
    if (!node || commandNum >= node->length) return; // Command nicht vorhanden
    event_t event = {EVENT_COMMAND, (void*)commandNum};
    xQueueSendToBack(owner, &event, 0);
    // Log
    ESP_LOGD("intercom", "An '%s' wurde Befehl '%s' gesendet.", node->task, node->commands[commandNum].name);
}

void intercom_commandSend2(uint32_t ownerNum, uint32_t commandNum) {
    command_list_t *node = intercom_commandSearchOwner2(ownerNum);
    if (node) intercom_commandSend(node->owner, commandNum);
}

const char* intercom_commandNameOwner(uint32_t ownerNum) {
    command_list_t *node = intercom_commandSearchOwner2(ownerNum);
    if (!node) return NULL;
    return node->task;
}

const char* intercom_commandNameCommand(uint32_t ownerNum, uint32_t commandNum) {
    command_list_t *node = intercom_commandSearchOwner2(ownerNum);
    if (!node || commandNum >= node->length) return NULL; // Command nicht vorhanden
    return node->commands[commandNum].name;
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
    if (err == ESP_ERR_NVS_NOT_INITIALIZED) { // lazy init
        esp_log_level_set("nvs", ESP_LOG_INFO);
        if (nvs_flash_init()) {
            nvs_flash_erase();
            if (nvs_flash_init()) return;
        }
        if (nvs_open(list->task, NVS_READWRITE, &nvs)) return;
    } else if (err) return;
    for (size_t i = 0; i < list->length; ++i) {
        err = nvs_get_u32(nvs, list->settings[i].name, list->settings[i].address);
        if (err == ESP_ERR_NVS_NOT_FOUND) {
            if (nvs_set_u32(nvs, list->settings[i].name, *(uint32_t*)(list->settings[i].address))) break;
            if (nvs_commit(nvs)) break;
        } else if (err) break;
    }
    nvs_close(nvs);
}

static setting_list_t *intercom_settingSearchOwner(QueueHandle_t owner) {
    // Owner suchen
    setting_list_t *node = intercom.settingHead;
    while (true) {
        if (!node) return NULL; // Owner nicht vorhanden
        if (node->owner == owner) break;
        node = node->next;
    }
    return node;
}

static setting_list_t *intercom_settingSearchOwner2(uint32_t ownerNum) {
    // Owner suchen
    setting_list_t *node = intercom.settingHead;
    if (!node) return NULL;
    while (ownerNum--) {
        node = node->next;
        if (!node) return NULL; // Owner nicht vorhanden
    }
    return node;
}

value_type_t intercom_settingType(QueueHandle_t owner, uint32_t settingNum) {
    setting_list_t *node = intercom_settingSearchOwner(owner);
    if (!node || settingNum >= node->length) return VALUE_TYPE_NONE; // Einstellung nicht vorhanden
    return node->settings[settingNum].type;
}

value_type_t intercom_settingType2(uint32_t ownerNum, uint32_t settingNum) {
    setting_list_t *node = intercom_settingSearchOwner2(ownerNum);
    if (!node || settingNum >= node->length) {
        return VALUE_TYPE_NONE; // Einstellung nicht vorhanden
    }
    return node->settings[settingNum].type;
}

bool intercom_settingSet(QueueHandle_t owner, uint32_t settingNum, value_t *value) {
    setting_list_t *node = intercom_settingSearchOwner(owner);
    if (!node || settingNum >= node->length) return true; // Einstellung nicht vorhanden
    setting_t *setting = &node->settings[settingNum];
    switch (setting->type) {
        case (VALUE_TYPE_UINT):
            *(uint32_t*)setting->address = value->ui;
            break;
        case (VALUE_TYPE_INT):
            *(int32_t*)setting->address = value->i;
            break;
        case (VALUE_TYPE_FLOAT):
            *(float*)setting->address = value->f;
            break;
        default:
            return true;
    }
    // in NVS aktualisieren
    nvs_handle nvs;
    if (!nvs_open(node->task, NVS_READWRITE, &nvs)) {
        if (nvs_set_u32(nvs, setting->name, value->ui) || nvs_commit(nvs)) return true;
        nvs_close(nvs);
    }
    return false;
}

bool intercom_settingSet2(uint32_t ownerNum, uint32_t settingNum, value_t *value) {
    setting_list_t *node = intercom_settingSearchOwner2(ownerNum);
    if (node) return intercom_settingSet(node->owner, settingNum, value);
    else return true;
}

bool intercom_settingGet(QueueHandle_t owner, uint32_t settingNum, value_t *value) {
    setting_list_t *node = intercom_settingSearchOwner(owner);
    if (!node || settingNum >= node->length) return true; // Einstellung nicht vorhanden
    setting_t *setting = &node->settings[settingNum];
    switch (setting->type) {
        case (VALUE_TYPE_UINT):
            value->ui = *(uint32_t*)setting->address;
            break;
        case (VALUE_TYPE_INT):
            value->i = *(int32_t*)setting->address;
            break;
        case (VALUE_TYPE_FLOAT):
            value->f = *(float*)setting->address;
            break;
        default:
            return true;
    }
    return false;
}

bool intercom_settingGet2(uint32_t ownerNum, uint32_t settingNum, value_t *value) {
    setting_list_t *node = intercom_settingSearchOwner2(ownerNum);
    if (node) return intercom_settingGet(node->owner, settingNum, value);
    else return true;
}

const char* intercom_settingNameOwner(uint32_t ownerNum) {
    setting_list_t *node = intercom_settingSearchOwner2(ownerNum);
    if (!node) return NULL;
    return node->task;
}

const char* intercom_settingNameSetting(uint32_t ownerNum, uint32_t settingNum) {
    setting_list_t *node = intercom_settingSearchOwner2(ownerNum);
    if (!node || settingNum >= node->length) return NULL; // Einstellung nicht vorhanden
    return node->settings[settingNum].name;
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

static parameter_list_t *intercom_parameterSearchOwner(QueueHandle_t owner) {
    // Owner suchen
    parameter_list_t *node = intercom.parameterHead;
    while (true) {
        if (!node) return NULL; // Owner nicht vorhanden
        if (node->owner == owner) break;
        node = node->next;
    }
    return node;
}

static parameter_list_t *intercom_parameterSearchOwner2(uint32_t ownerNum) {
    // Owner suchen
    parameter_list_t *node = intercom.parameterHead;
    if (!node) return NULL;
    while (ownerNum--) {
        node = node->next;
        if (!node) return NULL; // Owner nicht vorhanden
    }
    return node;
}

value_type_t intercom_parameterType(QueueHandle_t owner, uint32_t parameterNum) {
    parameter_list_t *node = intercom_parameterSearchOwner(owner);
    if (!node || parameterNum >= node->length) return VALUE_TYPE_NONE; // Parameter nicht vorhanden
    return node->parameters[parameterNum].type;
}

value_type_t intercom_parameterType2(uint32_t ownerNum, uint32_t parameterNum) {
    parameter_list_t *node = intercom_parameterSearchOwner2(ownerNum);
    if (!node || parameterNum >= node->length) {
        return VALUE_TYPE_NONE; // Parameter nicht vorhanden
    }
    return node->parameters[parameterNum].type;
}

bool intercom_parameterSet(QueueHandle_t owner, uint32_t parameterNum, value_t *value) {
    parameter_list_t *node = intercom_parameterSearchOwner(owner);
    if (!node || parameterNum >= node->length) return true; // Parameter nicht vorhanden
    parameter_t *parameter = &node->parameters[parameterNum];
    switch (parameter->type) {
        case (VALUE_TYPE_UINT):
            *(uint32_t*)parameter->address = value->ui;
            break;
        case (VALUE_TYPE_INT):
            *(int32_t*)parameter->address = value->i;
            break;
        case (VALUE_TYPE_FLOAT):
            *(float*)parameter->address = value->f;
            break;
        default:
            return true;
    }
    return false;
}

bool intercom_parameterSet2(uint32_t ownerNum, uint32_t parameterNum, value_t *value) {
    parameter_list_t *node = intercom_parameterSearchOwner2(ownerNum);
    if (node) return intercom_parameterSet(node->owner, parameterNum, value);
    else return true;
}

bool intercom_parameterGet(QueueHandle_t owner, uint32_t parameterNum, value_t *value) {
    parameter_list_t *node = intercom_parameterSearchOwner(owner);
    if (!node || parameterNum >= node->length) return true; // Parameter nicht vorhanden
    parameter_t *parameter = &node->parameters[parameterNum];
    switch (parameter->type) {
        case (VALUE_TYPE_UINT):
            value->ui = *(uint32_t*)parameter->address;
            break;
        case (VALUE_TYPE_INT):
            value->i = *(int32_t*)parameter->address;
            break;
        case (VALUE_TYPE_FLOAT):
            value->f = *(float*)parameter->address;
            break;
        default:
            return true;
    }
    return false;
}

bool intercom_parameterGet2(uint32_t ownerNum, uint32_t parameterNum, value_t *value) {
    parameter_list_t *node = intercom_parameterSearchOwner2(ownerNum);
    if (node) return intercom_parameterGet(node->owner, parameterNum, value);
    else return true;
}

const char* intercom_parameterNameOwner(uint32_t ownerNum) {
    parameter_list_t *node = intercom_parameterSearchOwner2(ownerNum);
    if (!node) return NULL;
    return node->task;
}

const char* intercom_parameterNameParameter(uint32_t ownerNum, uint32_t parameterNum) {
    parameter_list_t *node = intercom_parameterSearchOwner2(ownerNum);
    if (!node || parameterNum >= node->length) return NULL; // Einstellung nicht vorhanden
    return node->parameters[parameterNum].name;
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

static pv_list_t *intercom_pvSearchPublisher(QueueHandle_t publisher) {
    // Publisher suchen
    pv_list_t *node = intercom.pvHead;
    while (true) {
        if (!node) return NULL; // Publisher nicht vorhanden
        if (node->publisher == publisher) break;
        node = node->next;
    }
    return node;
}

static pv_list_t *intercom_pvSearchPublisher2(uint32_t publisherNum) {
    // Publisher suchen
    pv_list_t *node = intercom.pvHead;
    if (!node) return NULL;
    while (publisherNum--) {
        node = node->next;
        if (!node) return NULL; // Publisher nicht vorhanden
    }
    return node;
}

pv_t *intercom_pvSubscribe(QueueHandle_t subscriber, QueueHandle_t publisher, uint32_t pvNum) {
    pv_list_t *node = intercom_pvSearchPublisher(publisher);
    if (!node || pvNum >= node->length) return NULL; // Pv nicht vorhanden
    pv_t *pv = &node->pvs[pvNum];
    // Subscriber speichern
    for (uint8_t i = 0; i <= INTERCOM_PV_MAX_SUBSCRIBERS; ++i) {
        if (i == INTERCOM_PV_MAX_SUBSCRIBERS) return NULL; // kein Platz verfügbar
        if (pv->subscribers[i] == subscriber) { // bereits registriert, löschen
            pv->subscribers[i] = NULL;
            return NULL;
        }
        if (pv->subscribers[i]) continue;
        pv->subscribers[i] = subscriber;
        break;
    }
    return pv;
}

pv_t *intercom_pvSubscribe2(QueueHandle_t subscriber, uint32_t publisherNum, uint32_t pvNum) {
    pv_list_t *node = intercom_pvSearchPublisher2(publisherNum);
    if (node) return intercom_pvSubscribe(subscriber, node->publisher, pvNum);
    else return NULL;
}

void intercom_pvUnsubscribeAll(QueueHandle_t subscriber) {
    // über Publisher iterieren
    pv_list_t *node = intercom.pvHead;
    while (true) {
        if (!node) return; // am Ende angelangt
        // über PVs iterieren
        for (uint8_t i = 0; i < node->length; ++i) {
            pv_t *pv = &node->pvs[i];
            // über subscriber iterieren
            for (uint8_t j = 0; j < INTERCOM_PV_MAX_SUBSCRIBERS; ++j) {
                if (pv->subscribers[j] == subscriber) { // registriert, löschen
                    pv->subscribers[j] = NULL;
                    continue;
                }
            }
        }
        node = node->next;
    }
}

void intercom_pvPublish(QueueHandle_t publisher, uint32_t pvNum, value_t value) {
    pv_list_t *node = intercom_pvSearchPublisher(publisher);
    if (!node || pvNum >= node->length) return; // Pv nicht vorhanden
    pv_t *pv = &node->pvs[pvNum];
    pv->value = value;
    // an alle Subscriber senden
    event_t event = {EVENT_PV, pv};
    for (uint8_t i = 0; i < INTERCOM_PV_MAX_SUBSCRIBERS; ++i) {
        if (!pv->subscribers[i]) break;
        xQueueSendToBack(pv->subscribers[i], &event, 0);
    }
}

const char* intercom_pvNamePublisher(uint32_t publisherNum) {
    pv_list_t *node = intercom_pvSearchPublisher2(publisherNum);
    if (!node) return NULL;
    return node->task;
}

const char* intercom_pvNamePv(uint32_t publisherNum, uint32_t pvNum) {
    pv_list_t *node = intercom_pvSearchPublisher2(publisherNum);
    if (!node || pvNum >= node->length) return NULL; // Pv nicht vorhanden
    return node->pvs[pvNum].name;
}

bool intercom_pvIndex(pv_t *pv, uint32_t *subscriberNum, uint32_t *pvNum) {
    pv_list_t *node = intercom.pvHead;
    for (*subscriberNum = 0; ; ++(*subscriberNum)) {
        if (!node) return true; // nicht gefunden
        for (*pvNum = 0; *pvNum < node->length; ++(*pvNum)) {
            if (&node->pvs[*pvNum] == pv) return false; // gefunden
        }
        node = node->next;
    }
}
