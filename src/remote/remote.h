/*
 * File: remote.h
 * ----------------------------
 * Author: Niklaus Leuenberger
 * Date:   2019-12-13
 * ----------------------------
 * Implementiert eine Remote Steuerung per Webbrowser & Verwaltung für persistente Einstellungen.
 * Ordner www/ enthält index.html und ? für die Webkonnektivität, diese werden in das Binary-File eingebettet:
 *    -> board_build.embed_txtfiles https://docs.platformio.org/en/latest/platforms/espressif32.html#embedding-binary-data
 * Webbrowser <-> quadro2 kommuniziert über eine persistente Websocket-Verbindung.
 */


#pragma once


/** Externe Abhängigkeiten **/

#include "esp_system.h"
#include <stdbool.h>
#include <string.h>


/** Einstellungen **/

#define REMOTE_SETTINGS_USE_NVS 1
#define REMOTE_CONNECTION_COUNT (10U)


/** Variablendeklaration **/

enum setting_type_t {
    SETTING_TYPE_UINT,
    SETTING_TYPE_INT,
    SETTING_TYPE_FLOAT
};

struct setting_t {
    enum setting_type_t type;
    union {
        uint32_t ui;
        int32_t i;
        float f;
    } value;
};

typedef bool (remote_settingUpdate_t)(void *cookie, struct setting_t *setting); // return true um Änderung abzulehnen


/*
 * Function: remote_init
 * ----------------------------
 * Startet TCP/IP-Stack und verbindet mit WLAN.
 * Websocket muss noch nicht geöffnet sein.
 * 
 * char* ssid: Name des zu verbindenden WLAN
 * char* pw: PW des zu verbindenden WLAN
 *
 * returns: false -> Erfolg, true -> Error
 */
bool remote_init(char* ssid, char* pw);

/*
 * Function: remote_settingRegister
 * ----------------------------
 * Registriert eine Einstellung und speichert diese persistent in NVS.
 * Vorgenommene Veränderungen werden dem Callback mitgeteilt.
 * 
 * const char* taskTag: Taskname dem die Einstellung gehört
 * const char* settingTag: Name der Einstellung
 * remote_settingUpdate_t *updateCallback: Callback der über Veränderungen informiert wird
 * void *updateCallbackCookie: Cookie das dem Callback gegeben wird, bsp. zur Unterscheidung der Einstellungen in einem Switch
 * struct setting_t *setting: zu verwaltende Einstellung
 *
 * returns: Handle der Einstellung bei Erfolg, sonst NULL
 */
bool remote_settingRegister(const char* taskTag, const char* settingTag, remote_settingUpdate_t *updateCallback, void *updateCallbackCookie, struct setting_t *setting);
