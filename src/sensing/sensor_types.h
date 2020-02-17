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

#include "sh2_SensorValue.h"


/** Interne Abhängigkeiten **/


/** Variablendeklaration **/

enum sensors_input_type_t {
    SENSORS_ACCELERATION, // m/s^2
    SENSORS_ORIENTATION, // Quaternion
    SENSORS_ALTIMETER, // umgerechneter Druck Pa -> m
    SENSORS_ULTRASONIC, // m
    SENSORS_POSITION, // GPS Fix lat/lon/alt
    SENSORS_GROUNDSPEED // m/s
};

struct vector_t { // generischer Vektor
    union {
        struct {
            float x, y, z;
        };
        float v[3];
    };
};

struct orientation_t { // Orientierung
    float i, j, k;
    float real;
};

struct sensors_input_t {
    enum sensors_input_type_t type;
    int64_t timestamp;
    union {
        struct vector_t vector;
        struct orientation_t orientation;
        float distance; // generische Distanz
    };
    float accuracy; // Genauigkeit der Daten
};
