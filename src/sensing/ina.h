/*
 * File: ina.h
 * ----------------------------
 * Author: Niklaus Leuenberger
 * Date:   2020-05-11
 * ----------------------------
 * Spannungs- und Strom Sensor INA219 von Texas Instruments.
 * Nur Spannungsmessung implementiert.
 */


#pragma once


/** Externe Abhängigkeiten **/

#include "esp_system.h"


/** Einstellungen **/


/*
 * Function: ina_init
 * ----------------------------
 * Initialisiert Sensor und startet zyklisches Update.
 * 
 * uint8_t address: I2C Adresse
 * uint32_t *rate: Datenrate, reagiert bei Änderung automatisch
 *
 * returns: false -> Erfolg, true -> Error
 */
bool ina_init(uint8_t address, uint32_t *rate);
