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

#define REMOTE_CONNECTION_COUNT (10U)   // Anzahl gleichzeitiger Verbindungen
#define REMOTE_START_AFTER_STOP 0       // Nach WLAN Error automatisch Verbindung neustarten
#define REMOTE_RESET_AFTER_STOP 1       // Nach WLAN Error Neustarten


/** Variablendeklaration **/


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
