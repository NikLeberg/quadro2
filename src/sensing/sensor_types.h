/*
 * File: sensor_types.h
 * ----------------------------
 * Author: Niklaus Leuenberger
 * Date:   2020-01-09
 * ----------------------------
 *   Warteschlangen Input aller Sensortypen
 */


#pragma once

/** Externe Abhängigkeiten **/
#include "sh2.h"

/** Interne Abhängigkeiten **/


/** Variablendeklaration **/

enum sensors_input_type_t {
    SENSORS_ACCELERATION = SH2_LINEAR_ACCELERATION, // m/s^2
    SENSORS_ORIENTATION = SH2_ROTATION_VECTOR, // Quaternion
    SENSORS_PRESSURE = SH2_PRESSURE, // ?
    SENSORS_ALTIMETER, // umgerechneter Druck Pa -> m
    SENSORS_ULTRASONIC // m
};

struct sensors_input_t {
    enum sensors_input_type_t type;
    int64_t timestamp;
    union {
        struct { // generischer Vektor
            float x, y, z;
        } vector;
        float distance; // generischer Distanz
    };
};
