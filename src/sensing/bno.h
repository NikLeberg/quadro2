/*
 * File: bno.h
 * ----------------------------
 * Author: Niklaus Leuenberger
 * Date:   2020-02-09
 * ----------------------------
 * Public API für BNO080 Treiber.
 */


#pragma once


/** Externe Abhängigkeiten **/

#include "esp_system.h"
#include "driver/gpio.h"
#include "sensor_types.h"


/** Einstellungen **/

#define BNO_DATA_RATE_QUAT_US       20000   // 20 ms
#define BNO_DATA_RATE_ACCEL_US      40000   // 40 ms
#define BNO_DATA_RATE_PRESSURE_US   100000  // 100 ms
#define BNO_STARTUP_WAIT_MS         1000    // 1 s


/*
 * Function: bno_init
 * ----------------------------
 * Initialisiert Sensor, installiert Hintergrundtask und blockiert währenddem.
 *
 * uint8_t bnoAddr: BNO I2C Adresse
 * gpio_num_t bnoInterrupt: BNO Interrupt Pin (Data ready)
 * gpio_num_t bnoReset: BNO Reset Pin
 *
 * returns: false -> Erfolg, true -> Error
 */
bool bno_init(uint8_t address, gpio_num_t interruptPin, gpio_num_t resetPin);

/*
 * Function: bno_toWorldFrame
 * ----------------------------
 * Rotiere Vektor anhand Orientierung (in Form eines Quaternions)
 * vom Lokalen Referenzsystem in Weltkoordinaten.
 *
 * struct vector_t *vector: zu rotierender Vektor
 * orientation_t *quaternion: Quaternion für Rotation, bei NULL wird interner Quaternion benutzt
 */
void bno_toWorldFrame(vector_t *vector, orientation_t *quaternion);

/*
 * Function: bno_toLocalFrame
 * ----------------------------
 * Rotiere Vektor anhand Orientierung (in Form eines Quaternions)
 * von Weltkoordinaten in Lokales Referenzsystem.
 *
 * struct vector_t *vector: zu rotierender Vektor
 * orientation_t *quaternion: Quaternion für Rotation, bei NULL wird interner Quaternion benutzt
 */
void bno_toLocalFrame(vector_t *vector, orientation_t *quaternion);

/*
 * Function: bno_toEuler
 * ----------------------------
 * Rechne Quaternion der Orientierung in Eulerwinkel Roll (x), Pitch (y), Heading (z) um.
 *
 * struct vector_t *vector: resultierende Eulerwinkel
 * orientation_t *quaternion: Quaternion für Umrechnung, bei NULL wird interner Quaternion benutzt
 */
void bno_toEuler(vector_t *euler, orientation_t *quaternion);
