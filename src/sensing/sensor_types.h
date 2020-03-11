/*
 * File: sensor_types.h
 * ----------------------------
 * Author: Niklaus Leuenberger
 * Date:   2020-01-09
 * ----------------------------
 * Warteschlangen Events "EVENT_INTERNAL" aller Sensortypen
 */


#pragma once


/** Externe Abhängigkeiten **/


/** Interne Abhängigkeiten **/


/** Variablendeklaration **/

typedef enum {
    SENSORS_ACCELERATION, // m/s^2
    SENSORS_ORIENTATION, // Quaternion
    SENSORS_ALTIMETER, // umgerechneter Druck Pa -> m
    SENSORS_ULTRASONIC, // m
    SENSORS_POSITION, // GPS Fix lat/lon/alt
    SENSORS_GROUNDSPEED, // m/s
    SENSORS_MAX
} sensors_event_type_t;

typedef struct { // generischer Vektor
    union {
        struct {
            float x, y, z;
        };
        float v[3];
    };
} vector_t;

typedef struct { // Orientierung
    float i, j, k;
    float real;
} orientation_t;

typedef struct {
    sensors_event_type_t type;
    int64_t timestamp;
    union {
        vector_t vector;
        orientation_t orientation;
    };
    float accuracy; // Genauigkeit der Daten
} sensors_event_t;
