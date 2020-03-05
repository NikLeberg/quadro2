/*
 * File: interComm.h
 * ----------------------------
 * Author: Niklaus Leuenberger
 * Date:   2020-02-04
 * ----------------------------
 * Standartisierte Schnittstellen zwischen den Modulen.
 * 
 * Befehle / Parameter / Einstellungen
 */


/** Externe Abhängigkeiten **/

#include "esp_system.h"


/** Interne Abhängigkeiten **/


/** Variablendeklaration **/

typedef uint32_t event_type_t;

typedef struct {
    event_type_t type;
    void *data;
} event_t;

#define EVENT_COMMAND       0
#define EVENT_PARAMETER     1


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

typedef enum {
    SETTING_TYPE_UINT,
    SETTING_TYPE_INT,
    SETTING_TYPE_FLOAT
} setting_type_t;

typedef struct {
	const char *name;
    void *address;
    setting_type_t type;
} setting_t;

#define SETTING(name, address, type)  {(name), (void *)(address), (type)}

// ToDo: LinkedList in dem die Arrays mit Taskverknüpfungen registriert werden