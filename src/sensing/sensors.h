/*
 * File: sensors.h
 * ----------------------------
 * Author: Niklaus Leuenberger
 * Date:   2019-11-28
 * ----------------------------
 * Implementiert die einzelnen Sensoren und erweitert ggf. deren Funktionen
 *    BNO080:     - Beschleunigung
 *                - Rotation
 *                - Magnetisch Nord
 *                > lineare Beschleunigung
 *                > Orientierung
 *                > uvm.
 *    HC-SR04:    - Abstand zum Boden per Ultraschall
 *    BN-220:     - GPS, GLONASS, BeiDou, SBAS, Galileo
 * 
 * Alle Sensoren sind in ihrem eigenen Task implementiert.
 * Neue Sensordaten werden dem Sensor-Task mitgeteilt welcher den absoluten Status des Systems
 * aktualisiert, Sensor-Fusion betreibt und an registrierte Handler weiterleitet.
 *
 * Höhe z:
 *  - lineare Beschleunigung (Worldframe) doppelt integriert über Zeit
 *  - Ultraschall
 *  - GPS Höhe über Meer tariert auf Startposition
 *  - Drucksensor tariert auf Startposition
 */


#pragma once


/** Externe Abhängigkeiten **/

#include "esp_system.h"
#include "driver/gpio.h"


/** Einstellungen **/

#define SENSORS_TIMEOUT_MS                  2000

#define SENSORS_FUSE_Z_ERROR_ACCELERATION   0.35f   // +/- 0.35 m/s2
#define SENSORS_FUSE_Z_ERROR_ULTRASONIC     0.005f  // +/- 5 mm
#define SENSORS_FUSE_Z_ERROR_BAROMETER      0.3f    // +/- 30 cm
#define SENSORS_FUSE_Z_ERROR_GPS            1.0f    // +/- 1 m
#define SENSORS_FUSE_Z_X_VEL_LIMIT          100.0f  // +/- 100 m/s


/*
 * Function: sensors_init
 * ----------------------------
 * Konfiguriert einzelne Sensoren und deren Hintergrundtasks und startet Verwaltungstask.
 *
 * gpio_num_t scl: I2C-Bus Clock-Pin
 * gpio_num_t sda: I2C-Bus Daten-Pin
 * uint8_t bnoAddr: BNO I2C Adresse
 * gpio_num_t bnoInterrupt: BNO Interrupt Pin (Data ready)
 * gpio_num_t bnoReset: BNO Reset Pin
 * gpio_num_t ultTrigger: Ultraschall Trigger Pin
 * gpio_num_t ultEcho: Ultraschall Echo Pin
 * gpio_num_t gpsRxPin: Host RX GPS TX Pin
 * gpio_num_t gpsTxPin: Host TX GPS RX Pin
 *
 * returns: false -> Erfolg, true -> Error
 */
bool sensors_init(gpio_num_t scl, gpio_num_t sda,
                  uint8_t bnoAddr, gpio_num_t bnoInterrupt, gpio_num_t bnoReset,
                  gpio_num_t ultTrigger, gpio_num_t ultEcho,
                  gpio_num_t gpsRxPin, gpio_num_t gpsTxPin);

/*
 * Function: sensors_setHome
 * ----------------------------
 * Setze aktuelle Position als Homepunkt aller Sensoren.
 */
void sensors_setHome();
