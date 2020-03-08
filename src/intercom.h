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

typedef bool (command_function_t)(void);

typedef struct {
	const char *name;
} command_t;

#define COMMAND(name)  {(name)}


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

#define SETTING(name, address, type)  {(name), (void *)(address), (type)}


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
    QueueHandle_t subscribers[INTERCOM_PV_MAX_SUBSCRIBERS];
} pv_t;

#define PV(name, type)  {(name), (type), {NULL}}

typedef struct pv_list_s {
    const char *task;
    QueueHandle_t publisher;
    pv_t *pvs;
    struct pv_list_s *next;
} pv_list_t;

#define PVLIST(task, pvs)   pv_list_t pvs##_list = {task, NULL, pvs, NULL};

#define pvRegister(queue, pvs)  intercom_pvRegister(queue, &(pvs##_list))


/** To be made Private **/

static struct {
    pv_list_t *pvHead;
} intercom;

void intercom_pvRegister(QueueHandle_t publisher, pv_list_t *list) {
    list->publisher = publisher;
    pv_list_t *last = intercom.pvHead;
    while (last) {
        last = last->next;
    }
    last->next = list;
}
