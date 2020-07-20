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
#include "sensing/sensor_types.h"
#include "sensing/sensors.h"
#include "sensing/bno.h"
#include "remote/remote.h" // Intercom-Events
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
    bool throttleBoost;

    float maxRollPitch;

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
    COMMAND("resetStabilizePID"),
    COMMAND("resetQueue")
};
static COMMAND_LIST("control", control_commands, CONTROL_COMMAND_MAX);

static setting_t control_settings[CONTROL_SETTING_MAX] = {
    SETTING("maxRollPitch",     &control.maxRollPitch,                      VALUE_TYPE_FLOAT),

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

    SETTING("throttleBoost",    &control.throttleBoost,                     VALUE_TYPE_UINT)
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
    PARAMETER("z",              &control.setpoints.position.z,  VALUE_TYPE_FLOAT)
};
static PARAMETER_LIST("control", control_parameters, CONTROL_PARAMETER_MAX);

static pv_t control_pvs[CONTROL_PV_MAX] = {
    PV("armed",         VALUE_TYPE_UINT),
    PV("frontLeft",     VALUE_TYPE_FLOAT),
    PV("frontRight",    VALUE_TYPE_FLOAT),
    PV("backLeft",      VALUE_TYPE_FLOAT),
    PV("backRight",     VALUE_TYPE_FLOAT),
    PV("maxThrottle",   VALUE_TYPE_NONE),
    PV("minThrottle",   VALUE_TYPE_NONE),
    PV("roll",          VALUE_TYPE_FLOAT),
    PV("pitch",         VALUE_TYPE_FLOAT),
    PV("heading",       VALUE_TYPE_FLOAT),
    PV("xOut",          VALUE_TYPE_FLOAT),
    PV("yOut",          VALUE_TYPE_FLOAT),
    PV("zOut",          VALUE_TYPE_FLOAT),
    PV("rate",          VALUE_TYPE_UINT)
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
 * Function: control_pidCalculate
 * ----------------------------
 * Berechnet PID Regler.
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
 */
static void control_stabilize();

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

/*
 * Function: control_motorsBoost
 * ----------------------------
 * Erhöht geforderter Throttle sodass ein äquivalenter Schub wie bei voller Batterie resultiert.
 *
 * float throttle[4]: Throttle der geboosted werden soll
 */
static void control_motorsBoost(float throttle[4]);


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
    // LEDC als Motortreiber initialisieren
    ESP_LOGD("control", "Motors init");
    bool ret = false;
    ledc_timer_config_t ledcConfig = {
        speed_mode:         LEDC_HIGH_SPEED_MODE,
        {duty_resolution:   LEDC_TIMER_18_BIT},
        timer_num:          LEDC_TIMER_0,
        freq_hz:            CONTROL_MOTOR_FREQUENCY
    };
    ret = ledc_timer_config(&ledcConfig);
    ledc_channel_config_t ledcChannel = {
        speed_mode: LEDC_HIGH_SPEED_MODE,
        intr_type:  LEDC_INTR_DISABLE,
        timer_sel:  LEDC_TIMER_0,
        duty:       CONTROL_MOTOR_DUTY_MIN,
        hpoint:     0
    };
    ledcChannel.channel = LEDC_CHANNEL_0;
    ledcChannel.gpio_num = motorFrontLeft;
    ret |= ledc_channel_config(&ledcChannel);
    ledcChannel.channel = LEDC_CHANNEL_1;
    ledcChannel.gpio_num = motorFrontRight;
    ret |= ledc_channel_config(&ledcChannel);
    ledcChannel.channel = LEDC_CHANNEL_2;
    ledcChannel.gpio_num = motorBackLeft;
    ret |= ledc_channel_config(&ledcChannel);
    ledcChannel.channel = LEDC_CHANNEL_3;
    ledcChannel.gpio_num = motorBackRight;
    ret |= ledc_channel_config(&ledcChannel);
    ESP_LOGD("control", "Motors %s", ret ? "error" : "ok");
    // installiere task
    if (xTaskCreate(&control_task, "control", 3 * 1024, NULL, xControl_PRIORITY, NULL) != pdTRUE) return true;
    return ret;
}


/* Haupttask */

void control_task(void* arg) {
    // Variablen
    event_t event;
    // PVs empfangen
    pv_t *pvRemoteConnection = intercom_pvSubscribe(xControl, xRemote, REMOTE_PV_CONNECTIONS, 0); // UInt
    pv_t *pvRemoteTimeout = intercom_pvSubscribe(xControl, xRemote, REMOTE_PV_TIMEOUT, 0); // Event
    pv_t *pvRemoteState = intercom_pvSubscribe(xControl, xRemote, REMOTE_PV_STATE_ERROR, 0); // Event
    pv_t *pvSensorsOrientation = intercom_pvSubscribe(xControl, xSensors, SENSORS_PV_ORIENTATION, 0); // Event
    pv_t *pvSensorsTimeout = intercom_pvSubscribe(xControl, xSensors, SENSORS_PV_TIMEOUT, 0);
    // Loop
    while (true) {
        xQueueReceive(xControl, &event, portMAX_DELAY);
        switch (event.type) {
            case (EVENT_COMMAND): // Befehl erhalten
                control_processCommand((control_command_t)event.data);
                break;
            case (EVENT_PV): { // Istwert-Änderung
                pv_t *pv = event.data;
                if (pv == pvSensorsOrientation) control_stabilize();
                else if (!control.armed) break;
                else if (pv == pvRemoteTimeout) {
                    ESP_LOGD("control", "got timeout");
                    control_processCommand(CONTROL_COMMAND_DISARM);
                } else if (pv == pvRemoteConnection) {
                    ESP_LOGD("control", "got con");
                    control_processCommand(CONTROL_COMMAND_DISARM);
                } else if (pv == pvRemoteState) {
                    ESP_LOGD("control", "got state");
                    control_processCommand(CONTROL_COMMAND_DISARM);
                } else if (pv == pvSensorsTimeout) {
                    if (pvGetUint(pv) & (0x1 << SENSORS_ORIENTATION)) {
                        ESP_LOGD("control", "got sens timeout");
                        control_processCommand(CONTROL_COMMAND_DISARM);
                    }
                } else {
                    ESP_LOGD("control", "got unknown %p", pv);
                    control_processCommand(CONTROL_COMMAND_DISARM);
                }
                break;
            }
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
}

static void control_processCommand(control_command_t command) {
    switch (command) {
        case (CONTROL_COMMAND_DISARM):
            control.armed = false;
            float throttle[4];
            control_motorsThrottle(throttle);
            pvPublishUint(xControl, CONTROL_PV_ARMED, 0);
            control_processCommand(CONTROL_COMMAND_RESET_STABILIZE_PID);
            break;
        case (CONTROL_COMMAND_ARM):
            control.armed = true;
            pvPublishUint(xControl, CONTROL_PV_ARMED, 1);
            break;
        case (CONTROL_COMMAND_RESET_STABILIZE_PID):
            control_pidReset(&control.pids.stabilize[AXIS_ROLL]);
            control_pidReset(&control.pids.stabilize[AXIS_PITCH]);
            control_pidReset(&control.pids.stabilize[AXIS_HEADING]);
            break;
        case (CONTROL_COMMAND_RESET_QUEUE):
            xQueueReset(xControl);
        default:
            break;
    }
    return;
}

static float control_pidCalculate(control_pid_t *pid, float setpoint, float feedback, TickType_t tick) {
    float gain;
    float error = setpoint - feedback;
    TickType_t deltaT = tick - pid->lastTick;
    pid->lastTick = tick;
    gain = pid->Kp * error; // P
    if (pid->Kd) { // D
        gain += pid->Kd * (error - pid->prevError) / deltaT;
        pid->prevError = error;
    }
    if (pid->Ki) { // I
        error *= pid->Ki * deltaT;
        pid->integralError += error;
        gain += pid->integralError;
        if (gain > pid->band || gain < -pid->band) { // anti-windup
            pid->integralError -= error;
        }
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

static void control_direction(vector_t setpoint, vector_t velocity) {
    // float throttle, woher kommt es, sollte dies nicht hier errechnet werden?
    bno_toWorldFrame(&setpoint, NULL);
    // rechne pids
    TickType_t tick = xTaskGetTickCount();
    vector_t gain; // - 45.0 bis + 45.0
    for (control_directions_t i = 0; i < 3 ; ++i) { // x, y, z
        gain.v[i] = control_pidCalculate(&control.pids.direction[i], setpoint.v[i], velocity.v[i], tick);
    }
    bno_toLocalFrame(&gain, NULL);
    // x, y verschiebt Sollwinkel Roll, Pitch
    float maxAngle = control.maxRollPitch / 2.0f; // Hälfte des sicheren Winkels
    if ((gain.x > maxAngle) || (gain.x < -maxAngle)) {
        gain.x = 0.0f;
    }
    if ((gain.y > maxAngle) || (gain.y < -maxAngle)) {
        gain.y = 0.0f;
    }
    control.setpoints.euler.x = gain.x;
    control.setpoints.euler.y = gain.y;
    // z pusht throttle
    control.throttle += gain.z;
    if (gain.z > 1.0f) gain.z = 1.0f;
    else if (gain.z < 0.0f) gain.z = 0.0f;
    control_stabilize();
}

static void control_stabilize() {
    vector_t euler; // aktuelle Orientierung (Istwert)
    bno_toEuler(&euler, NULL);
    pvPublishFloat(xControl, CONTROL_PV_ROLL, euler.x);
    pvPublishFloat(xControl, CONTROL_PV_PITCH, euler.y);
    pvPublishFloat(xControl, CONTROL_PV_HEADING, euler.z);
    // Loop Rate
    static int64_t lastTime = 0;
    int64_t now = esp_timer_get_time();
    pvPublishUint(xControl, CONTROL_PV_RATE, (uint32_t)now - lastTime);
    lastTime = now;
    // Sicherheit
    if (!control.armed) return;
    if ((euler.x > control.maxRollPitch) || (euler.x < -control.maxRollPitch)
     || (euler.y > control.maxRollPitch) || (euler.y < -control.maxRollPitch)) {
        ESP_LOGD("control", "max angle!");
        control_processCommand(CONTROL_COMMAND_DISARM);
    }
    // rechne pids
    TickType_t tick = xTaskGetTickCount();
    vector_t gain;
    for (control_axes_t i = 0; i < 3; ++i) { // roll, pitch, heading
        gain.v[i] = control_pidCalculate(&control.pids.stabilize[i], control.setpoints.euler.v[i], euler.v[i], tick);
        pvPublishFloat(xControl, CONTROL_PV_OUT_X + i, gain.v[i]);
    }
    // throttle mixen
    gain.y /= 0.775f; // Korrektur für wide-X Frame gemäss https://www.iforce2d.net/mixercalc/
    float throttles[MOTOR_MAX]; //  roll     pitch    heading
    throttles[MOTOR_FRONT_LEFT] =   gain.x - gain.y + gain.z;
    throttles[MOTOR_FRONT_RIGHT] = -gain.x - gain.y - gain.z;
    throttles[MOTOR_BACK_LEFT]  =   gain.x + gain.y - gain.z;
    throttles[MOTOR_BACK_RIGHT] =  -gain.x + gain.y + gain.z;
    // nach oben beschränken (remanent auf throttle wirkend) wenn auf einem Motor > 1.0 verlangt wäre
    for (control_motors_t i = 0 ; i < MOTOR_MAX; ++i) {
        if (throttles[i] * control.throttle + control.throttle > 1.0f) {
            control.throttle = 1.0f / (throttles[i] + 1.0f);
            pvPublish(xControl, CONTROL_PV_THROTTLE_MAX);
        }
    }
    for (control_motors_t i = 0 ; i < MOTOR_MAX; ++i) {
        throttles[i] *= control.throttle;
        throttles[i] += control.throttle;
    }
    // tiefen throttle heben (nur temporär auf throttle wirkend) wenn auf einem Motor < 0.0 verlangt wäre
    if (control.throttleBoost) {
        float negThrottle = 0.0f;
        for (control_motors_t i = 0 ; i < MOTOR_MAX; ++i) {
            if (throttles[i] < 0.0f && throttles[i] < negThrottle) {
                negThrottle = throttles[i];
                pvPublish(xControl, CONTROL_PV_THROTTLE_MIN);
            }
        }
        for (control_motors_t i = 0 ; i < MOTOR_MAX; ++i) {
            throttles[i] += -negThrottle;
        }
    } // https://docs.px4.io/v1.9.0/en/config_mc/pid_tuning_guide_multicopter.html
    control_motorsThrottle(throttles);
}

static void control_motorsThrottle(float throttle[4]) {
    uint32_t duty;
    for (control_motors_t i = 0; i < MOTOR_MAX; ++i) {
        control_motorsBoost(&throttle[i]);
        if (throttle[i] > 1.0f) throttle[i] = 1.0f;
        else if (throttle[i] < 0.0f) throttle[i] = 0.0f;
        if (control.armed) {
            duty = throttle[i] * (CONTROL_MOTOR_DUTY_MAX - CONTROL_MOTOR_DUTY_MIN_SPIN) + CONTROL_MOTOR_DUTY_MIN_SPIN;
        } else {
            duty = CONTROL_MOTOR_DUTY_MIN;
        }
        ledc_set_duty(LEDC_HIGH_SPEED_MODE, i, duty);
        ledc_update_duty(LEDC_HIGH_SPEED_MODE, i);
        pvPublishFloat(xControl, CONTROL_PV_THROTTLE_FRONT_LEFT + i, throttle[i]);
    }
}

static void control_motorsBoost(float throttle[4]) {
    ;
}
