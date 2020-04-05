/*
 * File: control.h
 * ----------------------------
 * Author: Niklaus Leuenberger
 * Date:   2020-04-01
 * ----------------------------
 * Implementiert die Flugregelung und Ansteuerung der Motoren.
 * Sollvorgaben sind je nach Modi:
 *  - Sollposition x, y, z + Heading
 *  - Sollgeschwindigkeit vx, vy, vz + Heading
 *  - Sollorientierung in 3D als Eulerwinkel
 */


/** Externe Abhängigkeiten **/

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "esp_log.h"
#include "driver/ledc.h"


/** Interne Abhängigkeiten **/

#include "intercom.h"
#include "resources.h"
// motors
// pid?
// actions?
// modes?
#include "sensing/sensor_types.h"
#include "sensing/sensors.h"
#include "sensing/bno.h"
#include "control.h"


/** Variablendeklaration **/

#ifndef M_PI
    #define M_PI 3.14159265358979323846
#endif

typedef void (*pvHandlerFunc_t)(pv_t*);

typedef struct {
    float Kp;
    float Ki;
    float Kd;
    float band;
    float integralError;
    float prevError;
    TickType_t lastTick;
} control_pid_t;

typedef enum {
    DIRECTION_X = 0,
    DIRECTION_Y,
    DIRECTION_Z,
    DIRECTION_MAX
} control_directions_t;

typedef enum {
    AXIS_ROLL = 0,
    AXIS_PITCH,
    AXIS_HEADING,
    AXIS_MAX
} control_axes_t;

typedef enum {
    MOTOR_FRONT_LEFT = 0,
    MOTOR_FRONT_RIGHT,
    MOTOR_BACK_LEFT,
    MOTOR_BACK_RIGHT,
    MOTOR_MAX
} control_motors_t;

struct control_t {
    bool armed;
    float throttle;

    struct {
        vector_t euler;
        bool headingRate;
        vector_t velocity;
        vector_t position;
    } setpoints;

    struct {
        control_pid_t stabilize[AXIS_MAX];
        control_pid_t direction[DIRECTION_MAX];
    } pids;
};
static struct control_t control;

static command_t control_commands[CONTROL_COMMAND_MAX] = {
    COMMAND("disarm"),
    COMMAND("arm"),
    COMMAND("resetStabilizePID")
};
static COMMAND_LIST("control", control_commands, CONTROL_COMMAND_MAX);

static setting_t control_settings[CONTROL_SETTING_MAX] = {
    SETTING("xStabilizeKp",     &control.pids.stabilize[AXIS_ROLL].Kp,      VALUE_TYPE_FLOAT),
    SETTING("xStabilizeKi",     &control.pids.stabilize[AXIS_ROLL].Ki,      VALUE_TYPE_FLOAT),
    SETTING("xStabilizeKd",     &control.pids.stabilize[AXIS_ROLL].Kd,      VALUE_TYPE_FLOAT),
    SETTING("xStabilizeBand",   &control.pids.stabilize[AXIS_ROLL].band,    VALUE_TYPE_FLOAT),
    
    SETTING("yStabilizeKp",     &control.pids.stabilize[AXIS_PITCH].Kp,     VALUE_TYPE_FLOAT),
    SETTING("yStabilizeKi",     &control.pids.stabilize[AXIS_PITCH].Ki,     VALUE_TYPE_FLOAT),
    SETTING("yStabilizeKd",     &control.pids.stabilize[AXIS_PITCH].Kd,     VALUE_TYPE_FLOAT),
    SETTING("yStabilizeBand",   &control.pids.stabilize[AXIS_PITCH].band,   VALUE_TYPE_FLOAT),
    
    SETTING("zStabilizeKp",     &control.pids.stabilize[AXIS_HEADING].Kp,   VALUE_TYPE_FLOAT),
    SETTING("zStabilizeKi",     &control.pids.stabilize[AXIS_HEADING].Ki,   VALUE_TYPE_FLOAT),
    SETTING("zStabilizeKd",     &control.pids.stabilize[AXIS_HEADING].Kd,   VALUE_TYPE_FLOAT),
    SETTING("zStabilizeBand",   &control.pids.stabilize[AXIS_HEADING].band, VALUE_TYPE_FLOAT),
};
static SETTING_LIST("control", control_settings, CONTROL_SETTING_MAX);

static parameter_t control_parameters[CONTROL_PARAMETER_MAX] = {
    PARAMETER("throttle",       &control.throttle,              VALUE_TYPE_FLOAT),
    PARAMETER("roll",           &control.setpoints.euler.x,     VALUE_TYPE_FLOAT),
    PARAMETER("pitch",          &control.setpoints.euler.y,     VALUE_TYPE_FLOAT),
    PARAMETER("heading",        &control.setpoints.euler.z,     VALUE_TYPE_FLOAT),
    PARAMETER("headingRate",    &control.setpoints.headingRate, VALUE_TYPE_UINT),
    PARAMETER("vx",             &control.setpoints.velocity.x,  VALUE_TYPE_FLOAT),
    PARAMETER("vy",             &control.setpoints.velocity.y,  VALUE_TYPE_FLOAT),
    PARAMETER("vz",             &control.setpoints.velocity.z,  VALUE_TYPE_FLOAT),
    PARAMETER("x",              &control.setpoints.position.x,  VALUE_TYPE_FLOAT),
    PARAMETER("y",              &control.setpoints.position.y,  VALUE_TYPE_FLOAT),
    PARAMETER("z",              &control.setpoints.position.z,  VALUE_TYPE_FLOAT),
};
static PARAMETER_LIST("control", control_parameters, CONTROL_PARAMETER_MAX);

static pv_t control_pvs[CONTROL_PV_MAX] = {
    PV("armed",             VALUE_TYPE_UINT),
    PV("dutyFrontLeft",     VALUE_TYPE_UINT),
    PV("dutyFrontRight",    VALUE_TYPE_UINT),
    PV("dutyBackLeft",      VALUE_TYPE_UINT),
    PV("dutyBackRight",     VALUE_TYPE_UINT),
    PV("roll",              VALUE_TYPE_FLOAT),
    PV("pitch",             VALUE_TYPE_FLOAT),
    PV("heading",           VALUE_TYPE_FLOAT)
};
static PV_LIST("control", control_pvs, CONTROL_PV_MAX);


/** Private Functions **/

/*
 * Function: control_task
 * ----------------------------
 * Haupttask. Verwaltet alle eingehenden Intercom-Events und bedient Regler.
 *
 * void* arg: Dummy für FreeRTOS
 */
void control_task(void* arg);

/*
 * Function: control_processCommand
 * ----------------------------
 * Verarbeitet von Intercom gesendete Befehle gemäss control_command_t.
 *
 * control_command_t command: auszuführender Befehl
 */
static void control_processCommand(control_command_t command);

/*
 * Function: control_pidSet
 * ----------------------------
 * Setzt Gains des PID Reglers.
 * 
 * control_pid_t *pid: PID-Container der eingestellt werden soll
 */
static void control_pidSet(control_pid_t *pid, float Kp, float Ki, float Kd, float band);

/*
 * Function: control_pidCalculate
 * ----------------------------
 * Berechnet PID Regler.
 * ToDo: Integral Windup eliminieren.
 * 
 * control_pid_t *pid: PID-Container mit gespeicherten Zuständen und Gains
 * float setpoint: Sollwert
 * float feedback: Istwert
 * TickType_t tick: aktueller Tick
 * 
 * returns: gain im Bereich von - 1.0 bis + 1.0
 */
static float control_pidCalculate(control_pid_t *pid, float setpoint, float feedback, TickType_t tick);

/*
 * Function: control_pidReset
 * ----------------------------
 * Setzt Interal und Derivat des Reglers zurück.
 *
 * control_pid_t *pid: PID-Container der zurückgesetzt werden soll
 */
static void control_pidReset(control_pid_t *pid);

/*
 * Function: control_stabilize
 * ----------------------------
 * Stabilisiert auf gewünschte Eulerwinkel mittles PID-Regler.
 *
 * float throttle: grundlegender Throttle
 * float setpoints[AXIS_MAX]: Sollwert in Eulerwinkel Roll, Pitch, Heading
 * float eulers[AXIS_MAX]: Istwert in Eulerwinkel Roll, Pitch, Heading
 */
static void control_stabilize(float throttle, vector_t setpoint, vector_t euler);

/*
 * Function: control_motorsThrottle
 * ----------------------------
 * Setzt Throttle der Motoren per LEDC Hardware.
 * Grundlegendes PWM-Signal erlaubt Updates mit max. 50 Hz.
 * ToDo: PWM-Signal auf Schubkraft linearisieren. (Waagenmessung & Batterieleistung)
 *
 * float throttles[4]: Throttle der einzelnen Motoren von 0.0 bis 1.0
 */
static void control_motorsThrottle(float throttle[4]);


/** Implementierung **/

bool control_init(gpio_num_t motorFrontLeft, gpio_num_t motorFrontRight,
                  gpio_num_t motorBackLeft, gpio_num_t motorBackRight) {
    // Intercom-Queue erstellen
    xControl = xQueueCreate(16, sizeof(event_t));
    // an Intercom anbinden
    commandRegister(xControl, control_commands);
    settingRegister(xControl, control_settings);
    parameterRegister(xControl, control_parameters);
    pvRegister(xControl, control_pvs);
    // Motoren initialisieren
    bool ret = false;
    ESP_LOGD("control", "Motors init");
    // ret = ;
    ESP_LOGD("control", "Motors %s", ret ? "error" : "ok");
    // ...
    // für PV-Empfang registrieren
    // installiere task
    if (xTaskCreate(&control_task, "control", 1 * 1024, NULL, xControl_PRIORITY, NULL) != pdTRUE) return true;
    return ret;
}


/* Haupttask */

void control_task(void* arg) {
    // Variablen
    event_t event;
    TickType_t timeout = 0;
    // Loop
    while (true) {
        if (xQueueReceive(xControl, &event, 1) == pdTRUE) {
            switch (event.type) {
                case (EVENT_COMMAND): // Befehl erhalten
                    control_processCommand((control_command_t)event.data);
                    break;
                case (EVENT_PV): // Istwert-Änderung
                    break;
                case (EVENT_INTERNAL):
                default:
                    // ungültig
                    break;
            }
            // lösche wenn Platz gering wird
            if (uxQueueSpacesAvailable(xControl) <= 1) {
                xQueueReset(xControl);
                ESP_LOGE("control", "queue reset!");
            }
        }
        if (timeout) --timeout;
        else {
            timeout = 500 / portTICK_PERIOD_MS;
            vector_t euler;
            bno_toEuler(&euler, NULL);
            pvPublishFloat(xControl, CONTROL_PV_ROLL, euler.x * (180.0f / M_PI));
            pvPublishFloat(xControl, CONTROL_PV_PITCH, euler.y * (180.0f / M_PI));
            pvPublishFloat(xControl, CONTROL_PV_HEADING, euler.z * (180.0f / M_PI));
            control_stabilize(control.throttle, control.setpoints.euler, euler);
        }
    }
}

static void control_processCommand(control_command_t command) {
    switch (command) {
        case (CONTROL_COMMAND_DISARM):
            control.armed = false;
            pvPublishUint(xControl, CONTROL_PV_ARMED, 0);
            control_processCommand(CONTROL_RESET_STABILIZE_PID);
            break;
        case (CONTROL_COMMAND_ARM):
            control.armed = true;
            pvPublishUint(xControl, CONTROL_PV_ARMED, 1);
            break;
        case (CONTROL_RESET_STABILIZE_PID):
            control_pidReset(&control.pids.stabilize[AXIS_ROLL]);
            control_pidReset(&control.pids.stabilize[AXIS_PITCH]);
            control_pidReset(&control.pids.stabilize[AXIS_HEADING]);
            break;
        default:
            break;
    }
    return;
}

static void control_pidSet(control_pid_t *pid, float Kp, float Ki, float Kd, float band) {
    pid->Kp = Kp;
    pid->Ki = Ki;
    pid->Kd = Kd;
    pid->band = band;
}

static float control_pidCalculate(control_pid_t *pid, float setpoint, float feedback, TickType_t tick) {
    float gain = 0.0f;
    float error = setpoint - feedback;
    TickType_t deltaT = tick - pid->lastTick;
    pid->lastTick = tick;
    if (pid->Kp) {
        gain += pid->Kp * error;
    }
    if (pid->Ki) {
        pid->integralError += error * deltaT;
        gain += pid->Ki * pid->integralError;
    }
    if (pid->Kd) {
        error *= deltaT;
        gain += pid->Kd * (pid->prevError - error);
        pid->prevError = error;
    }
    if (gain > pid->band) gain = pid->band;
    else if (gain < -pid->band) gain = -pid->band;
    return gain;
}

static void control_pidReset(control_pid_t *pid) {
    pid->integralError = 0.0f;
    pid->prevError = 0.0f;
}

static void control_position(vector_t setpoint, vector_t position) {
    // setpoint der geschwindigkeit rechnen,
    // sollposition, istposition
    // Beschleunigen, bremsen macht alles PID, oder lieber PI
    // in die drei Achsenkomponenten aufteilen? x, y, z?
}

static void control_direction(float throttle, vector_t setpoint, vector_t velocity) {
    // float throttle, woher kommt es, sollte dies nicht hir errechnet werden?
    bno_toWorldFrame(&setpoint, NULL);
    // rechne pids
    TickType_t tick = xTaskGetTickCount();
    vector_t gain; // - 45.0 bis + 45.0
    for (control_directions_t i = 0; i < 3 ; ++i) { // x, y, z
        gain.v[i] = control_pidCalculate(&control.pids.direction[i], setpoint.v[i], velocity.v[i], tick);
    }
    bno_toLocalFrame(&gain, NULL);
    // auf neue Eulerwinkel stabilisieren
    vector_t euler;
    bno_toEuler(&euler, NULL);
    control_stabilize(throttle, gain, euler);
}

static void control_stabilize(float throttle, vector_t setpoint, vector_t euler) {
    // rechne pids
    TickType_t tick = xTaskGetTickCount();
    vector_t gain; // - 1.0 bis + 1.0
    for (control_axes_t i = 0; i < 3 ; ++i) { // roll, pitch, heading
        gain.v[i] = control_pidCalculate(&control.pids.stabilize[i], setpoint.v[i], euler.v[i], tick);
    }
    // throttle rechnen
    float throttles[MOTOR_MAX];
    throttles[MOTOR_FRONT_LEFT] =   gain.x - gain.y + gain.z + throttle;
    throttles[MOTOR_FRONT_RIGHT] = -gain.x - gain.y - gain.z + throttle;
    throttles[MOTOR_BACK_LEFT]  =   gain.x + gain.y - gain.z + throttle;
    throttles[MOTOR_BACK_RIGHT] =  -gain.x + gain.y + gain.z + throttle;
    control_motorsThrottle(throttles);
}

static void control_motorsThrottle(float throttle[4]) {
    uint32_t duty;
    for (control_motors_t i = 0; i < 4; ++i) {
        if (control.armed) {
            if (throttle[i] > 1.0f) throttle[i] = 1.0f;
            else if (throttle[i] < 0.0f) throttle[i] = 0.0f;
            duty = throttle[i] * (2000 - 1000) + 1000;
        } else {
            duty = 1000;
        }
        ledc_set_duty(LEDC_LOW_SPEED_MODE, i, duty);
        ledc_update_duty(LEDC_LOW_SPEED_MODE, i);
        pvPublishUint(xControl, CONTROL_PV_DUTY_FRONT_LEFT + i, duty);
    }
}
