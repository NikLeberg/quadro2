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
 *    BN-880Q:    - GPS, GLONASS, BeiDou, SBAS, Galileo
 * 
 * Alle Sensoren sind in ihrem eigenen Task implementiert.
 * Neue Sensordaten werden dem Sensor-Task mitgeteilt welcher den absoluten Status des Systems
 * aktualisiert, Sensor-Fusion betreibt und an registrierte Handler weiterleitet.
 * 
 * Systemstatus: (alle Werte im ENU Koordinatensystem)
 *  - Orientierung
 *  - geschätzte Position
 *  - geschätzte Geschwindigkeit
 *
 * Höhe z:
 *  - lineare z Beschleunigung (Worldframe) doppelt integriert über Zeit
 *  - Ultraschall
 *  - GPS Höhe über Meer tariert auf Startposition
 *  - Drucksensor tariert auf Startposition
 * 
 * Position y:
 *  - lineare y Beschleunigung (Worldframe) doppelt integriert über Zeit
 *  - GPS y-Positionskomponente
 * 
 * Position x:
 *  - lineare x Beschleunigung (Worldframe) doppelt integriert über Zeit
 *  - GPS x-Positionskomponente
 */


#pragma once


/** Externe Abhängigkeiten **/

#include "esp_system.h"
#include "driver/gpio.h"


/** Compiler Einstellungen **/


/** Befehle **/

typedef enum {
    SENSORS_COMMAND_SET_HOME = 0,
    SENSORS_COMMAND_SET_ALTIMETER_TO_GPS,
    SENSORS_COMMAND_RESET_FUSION,
    SENSORS_COMMAND_RESET_QUEUE,
    SENSORS_COMMAND_UPDATE_RATE,
    SENSORS_COMMAND_MAX
} sensors_command_t;


/** Einstellungen **/

typedef enum {
    SENSOR_SETTING_TIMEOUT = 0,
    // Datenrate
    SENSOR_SETTING_RATE_FAST,
    SENSOR_SETTING_RATE_MEDIUM,
    SENSOR_SETTING_RATE_SLOW,
    // Fuse Z
    SENSORS_SETTING_FUSE_Z_ERROR_ACCELERATION,
    SENSORS_SETTING_FUSE_Z_ERROR_ULTRASONIC,
    SENSORS_SETTING_FUSE_Z_ERROR_BAROMETER,
    SENSORS_SETTING_FUSE_Z_ERROR_GPS,
    SENSORS_SETTING_FUSE_Z_LIMIT_VEL,
    // Fuse Y
    SENSORS_SETTING_FUSE_Y_ERROR_ACCELERATION,
    SENSORS_SETTING_FUSE_Y_ERROR_GPS,
    SENSORS_SETTING_FUSE_Y_ERROR_VELOCITY,
    SENSORS_SETTING_FUSE_Y_LIMIT_VEL,
    // Fuse X
    SENSORS_SETTING_FUSE_X_ERROR_ACCELERATION,
    SENSORS_SETTING_FUSE_X_ERROR_GPS,
    SENSORS_SETTING_FUSE_X_ERROR_VELOCITY,
    SENSORS_SETTING_FUSE_X_LIMIT_VEL,
    // Spannungslimits
    SENSORS_SETTING_VOLTAGE_WARNING,
    SENSORS_SETTING_VOLTAGE_LOW,
    // Flow Skalierung
    SENSORS_SETTING_FLOW_SCALE_X,
    SENSORS_SETTING_FLOW_SCALE_Y,
    SENSORS_SETTING_MAX
} sensors_setting_t;


/** Parameter **/


/** Prozessvariablen **/

typedef enum {
    SENSORS_PV_TIMEOUT = 0,
    SENSORS_PV_ORIENTATION,
    SENSORS_PV_X,
    SENSORS_PV_VX,
    SENSORS_PV_Y,
    SENSORS_PV_VY,
    SENSORS_PV_Z,
    SENSORS_PV_VZ,
    SENSORS_PV_VOLTAGE,
    SENSORS_PV_VOLTAGE_WARN,
    SENSORS_PV_VOLTAGE_LOW,
    SENSORS_PV_FLOW_X,
    SENSORS_PV_FLOW_Y,
    SENSORS_PV_GYRO_X,
    SENSORS_PV_GYRO_Y,
    SENSORS_PV_GYRO_Z,
    SENSORS_PV_MAX
} sensors_pv_t;


/*
 * Function: sensors_init
 * ----------------------------
 * Konfiguriert einzelne Sensoren und deren Hintergrundtasks und startet Verwaltungstask.
 *
 * gpio_num_t scl: I2C-Bus Clock-Pin
 * gpio_num_t sda: I2C-Bus Daten-Pin
 * uint8_t bnoAddress: BNO I2C Adresse
 * gpio_num_t bnoInterrupt: BNO Interrupt Pin (Data ready)
 * gpio_num_t bnoReset: BNO Reset Pin
 * gpio_num_t flowRxPin: Rx Pin des Optischen Fluss & Lidar
 * gpio_num_t gpsRxPin: Host RX GPS TX Pin
 * gpio_num_t gpsTxPin: Host TX GPS RX Pin
 * uint8_t inaAddress: INA I2C Adresse
 *
 * returns: false -> Erfolg, true -> Error
 */
bool sensors_init(gpio_num_t scl, gpio_num_t sda,
                  uint8_t bnoAddress, gpio_num_t bnoInterrupt, gpio_num_t bnoReset,
                  gpio_num_t flowRxPin,                                             // Optischer Fluss & Lidar
                  gpio_num_t gpsRxPin, gpio_num_t gpsTxPin,
                  uint8_t inaAddress);
