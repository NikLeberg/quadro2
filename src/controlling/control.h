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
 *  - Sollorientierung in 3D
 */


#pragma once


/** Externe Abhängigkeiten **/

#include "esp_system.h"
#include "driver/gpio.h"


/** Compiler Einstellungen **/

#define CONTROL_MOTOR_FREQUENCY 200
#define CONTROL_MOTOR_DUTY_MIN  ((0x1 << LEDC_TIMER_18_BIT) - 1) / (1000 / CONTROL_MOTOR_FREQUENCY)
#define CONTROL_MOTOR_DUTY_MAX  CONTROL_MOTOR_DUTY_MIN * 2


/** Befehle **/

typedef enum {
    CONTROL_COMMAND_DISARM = 0,
    CONTROL_COMMAND_ARM,
    CONTROL_COMMAND_RESET_STABILIZE_PID,
    CONTROL_COMMAND_RESET_QUEUE,
    CONTROL_COMMAND_MAX
} control_command_t;


/** Einstellungen **/

typedef enum {
    CONTROL_SETTING_ROLL_PITCH_MAX = 0,
    CONTROL_SETTING_STABILIZE_X_KP,
    CONTROL_SETTING_STABILIZE_X_KI,
    CONTROL_SETTING_STABILIZE_X_KD,
    CONTROL_SETTING_STABILIZE_X_BAND,
    CONTROL_SETTING_STABILIZE_Y_KP,
    CONTROL_SETTING_STABILIZE_Y_KI,
    CONTROL_SETTING_STABILIZE_Y_KD,
    CONTROL_SETTING_STABILIZE_Y_BAND,
    CONTROL_SETTING_STABILIZE_Z_KP,
    CONTROL_SETTING_STABILIZE_Z_KI,
    CONTROL_SETTING_STABILIZE_Z_KD,
    CONTROL_SETTING_STABILIZE_Z_BAND,
    CONTROL_SETTING_MAX
} control_setting_t;


/** Parameter **/

typedef enum {
    CONTROL_PARAMETER_THROTTLE = 0,
    CONTROL_PARAMETER_SETPOINT_ROLL,
    CONTROL_PARAMETER_SETPOINT_PITCH,
    CONTROL_PARAMETER_SETPOINT_HEADING,
    CONTROL_PARAMETER_SETPOINT_HEADINGRATE,
    CONTROL_PARAMETER_SETPOINT_VX,
    CONTROL_PARAMETER_SETPOINT_VY,
    CONTROL_PARAMETER_SETPOINT_VZ,
    CONTROL_PARAMETER_SETPOINT_X,
    CONTROL_PARAMETER_SETPOINT_Y,
    CONTROL_PARAMETER_SETPOINT_Z,
    CONTROL_PARAMETER_THROTTLE_OVERRIDE,
    CONTROL_PARAMETER_MAX
} control_parameter_t;


/** Prozessvariablen **/

typedef enum {
    CONTROL_PV_ARMED = 0,
    CONTROL_PV_THROTTLE_FRONT_LEFT,
    CONTROL_PV_THROTTLE_FRONT_RIGHT,
    CONTROL_PV_THROTTLE_BACK_LEFT,
    CONTROL_PV_THROTTLE_BACK_RIGHT,
    CONTROL_PV_THROTTLE_MAX,
    CONTROL_PV_ROLL,
    CONTROL_PV_PITCH,
    CONTROL_PV_HEADING,
    CONTROL_PV_OUT_X,
    CONTROL_PV_OUT_Y,
    CONTROL_PV_OUT_Z,
    CONTROL_PV_RATE,
    CONTROL_PV_MAX
} control_pv_t;


/*
 * Function: control_init
 * ----------------------------
 * Konfiguriert die Regler und Motoren und startet Verwaltungstask.
 *
 * gpio_num_t motorFrontLeft: Motorpin von vorne links, dreht CW
 * gpio_num_t motorFrontRight: Motorpin von vorne rechts, dreht CCW
 * gpio_num_t motorBackLeft: Motorpin von hinten links, dreht CW
 * gpio_num_t motorBackRight: Motorpin von hinten rechts, dreht CCW
 *
 * returns: false -> Erfolg, true -> Error
 */
bool control_init(gpio_num_t motorFrontLeft, gpio_num_t motorFrontRight,
                  gpio_num_t motorBackLeft, gpio_num_t motorBackRight);
