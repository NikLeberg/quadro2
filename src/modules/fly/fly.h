/*
 * File:          /fly.h
 * Project:       quadro2
 * Created Date:  2019-12-04
 * Author:        Niklaus Ruben Leuenberger
 * -----
 * Description:   Kaskadenregelung des Fluges.
 * 				  Sollgeschwindigkeiten -> Sollwinkel -> Motoransteuerung
 */

/**
 * External dependencies
 */
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"

/**
 * Internal dependencies
 */
#include "ressources.h"
//#include "direction.h"
//#include "axes.h"
//#include "motors.h"

#define FLY_FEEDBACK_VELOCITY_INTERVALL (40U) // 40 ms
#define FLY_FEEDBACK_EULER_INTERVALL	(20U) // 20 ms
#define FLY_FEEDBACK_JITTER				(1U)  // 1 Tick maximale Abweichung

#define FLY_FEEDBACK_VELOCITY_MAX_TICKS	((FLY_FEEDBACK_VELOCITY_INTERVALL / portTICK_RATE_MS) + FLY_FEEDBACK_JITTER)
#define FLY_FEEDBACK_EULER_MAX_TICKS	((FLY_FEEDBACK_EULER_INTERVALL / portTICK_RATE_MS) + FLY_FEEDBACK_JITTER)

#define FLY_DIRECTION_MAX				(60UL) // 60° Maximal

enum fly_event_type_t {
	FLY_EVENT_SETPOINT,				// neue Sollwerte
	FLY_EVENT_FEEDBACK_EULER,		// neuer Istwert Eulerwinkel
	FLY_EVENT_FEEDBACK_VELOCITY		// neuer Istwert lineare Beschleunigung
};

enum fly_velocity_direction_t {
	FLY_V_X,
	FLY_V_Y,
	FLY_V_Z,
	FLY_V_MAX
};

enum fly_euler_axis_t {
	FLY_ROLL_X,
	FLY_PITCH_Y,
	FLY_YAW_Z,
	FLY_EULER_MAX
};

enum fly_state_t {
	FLY_STATE_OK,
	FLY_STATE_FATAL,
	FLY_STATE_DATA_TOO_LATE,
	FLY_STATE_MAX
};

struct fly_direction_setpoint_t {
	int16_t v[FLY_V_MAX]; 	// mm/s
	int8_t rotate;			// grad/s
	bool armed;
};

struct fly_axes_setpoint_t {
	int16_t e[FLY_EULER_MAX];
	uint16_t throttle;
	bool armed;
};

struct fly_velocity_t {
	int16_t v[FLY_V_MAX];	// mm/s (in horizontaler Ebene?)
	uint32_t tick;
};

struct fly_euler_t {
	int16_t e[FLY_EULER_MAX];
	uint32_t tick;
};

struct fly_event_t {
    enum fly_event_type_t type;
	union {
		struct fly_direction_setpoint_t	setpoint;
		struct fly_velocity_t	velocity;
		struct fly_euler_t		euler;
	};
};

struct fly_pid_t {
	int32_t Kp, Ki, Kd;
	int32_t min, max;
	int32_t lastError;
	int32_t integral;
};

struct fly_direction_t {
    struct fly_direction_setpoint_t setpoint;
    struct fly_velocity_t feedback;
    struct fly_pid_t pid[FLY_V_MAX];
};

struct fly_axes_t {
    struct fly_axes_setpoint_t setpoint;
    struct fly_euler_t feedback;
    struct fly_pid_t pid[FLY_EULER_MAX];
};

static struct fly_t {
    struct fly_direction_t direction;
    struct fly_axes_t axes;
};
static struct fly_t fly;

/**
 * Prototypes
 */
bool fly_init(struct fly_pid_t axis[3], struct fly_pid_t direction[3]);
void fly_task(void* arg);
static void fly_stateSet(enum fly_state_t state);
static int16_t fly_pidStep(struct fly_pid_t* pid, int16_t setpoint, int16_t feedback);
static void fly_pidReset(struct fly_pid_t* pid);
static void fly_directionArm(bool arm);
static void fly_directionStep();
static void fly_axesArm(bool arm);
static void fly_axesStep();

bool fly_init(struct fly_pid_t axis[3], struct fly_pid_t direction[3]) {
	// PIDs übernehmen
	for (uint8_t i = 0; i < FLY_EULER_MAX; ++i) {
		fly.axes.pid[i] = axis[i];
		fly.axes.pid[i].min = 0;
		fly.axes.pid[i].max = USHRT_MAX;
	}
	for (uint8_t i = 0; i < FLY_V_MAX; ++i) {
		fly.direction.pid[i] = direction[i];
	}
	fly.direction.pid[FLY_V_X].max = (FLY_DIRECTION_MAX << 8);
	fly.direction.pid[FLY_V_X].min = -(FLY_DIRECTION_MAX << 8);
	fly.direction.pid[FLY_V_Y].max = (FLY_DIRECTION_MAX << 8);
	fly.direction.pid[FLY_V_Y].min = -(FLY_DIRECTION_MAX << 8);
	fly.direction.pid[FLY_V_Z].max = (1000 << 8);
	fly.direction.pid[FLY_V_Z].min = -(1000 << 8);
	// Queue + Task erstellen
	if (!xFlyInputQueue) {
		xFlyInputQueue = xQueueCreate(3, sizeof(struct fly_event_t));
		if (!xFlyInputQueue) return true;
	}
	if (!xFlyTaskHandle) {
		if (xTaskCreate(&fly_task, "fly", 2 * 1024, NULL, FLY_TASK_PRIORITY, &xFlyTaskHandle) != pdTRUE) return true;
	}
	return false;
}

void fly_task(void* arg) {
	struct fly_event_t event;
	while (true) {
		xQueueReceive(xFlyInputQueue, &event, portMAX_DELAY);
		switch (event.type) {
			case (FLY_EVENT_SETPOINT): {
				if (fly.direction.setpoint.armed != event.setpoint.armed) {
					fly_directionArm(event.setpoint.armed);
					fly_axesArm(event.setpoint.armed);
				}
				fly.direction.setpoint = event.setpoint;
				break;
			}
			case (FLY_EVENT_FEEDBACK_VELOCITY): {
				if (fly.direction.feedback.tick > FLY_FEEDBACK_VELOCITY_MAX_TICKS) {
					fly_stateSet(FLY_STATE_DATA_TOO_LATE);
				}
                fly.direction.feedback = event.velocity;
				fly_directionStep();
				break;
			}
			case (FLY_EVENT_FEEDBACK_EULER): {
				if (fly.axes.feedback.tick > FLY_FEEDBACK_EULER_MAX_TICKS) {
					fly_stateSet(FLY_STATE_DATA_TOO_LATE);
				}
				fly_axesStep();
				break;
			}
		}
	}
}

static void fly_stateSet(enum fly_state_t state) {
	xTaskNotify(xFlyTaskHandle, ((1UL) << state), eSetBits);
}

static int16_t fly_pidStep(struct fly_pid_t* pid, int16_t setpoint, int16_t feedback) {
    int16_t output = 0;
    int16_t error = setpoint - feedback;
    if (pid->Kp != 0) {
        output += error * pid->Kp;
    }
    if (pid->Ki != 0) {
        pid->integral += error * pid->Ki;
        output += pid->integral;
    }
    if (pid->Kd != 0) {
        output += (pid->lastError - error) * pid->Kd;
        pid->lastError = error;
    }
    if (output > pid->max) output = pid->max;
    else if (output < pid->min) output = pid->min;
    return output;
}

static void fly_pidReset(struct fly_pid_t* pid) {
    pid->integral = 0;
    pid->lastError = 0;
}

static void fly_directionArm(bool arm) {
    if (!arm) {
        for(uint8_t i = 0; i < FLY_V_MAX; ++i) {
            fly_pidReset(fly.direction.pid+i);
        }
    }
}

static void fly_directionStep() {
	fly.axes.setpoint.e[FLY_PITCH_Y] =
		-fly_pidStep(&fly.direction.pid[FLY_V_X], fly.direction.setpoint.v[FLY_V_X], fly.direction.feedback.v[FLY_V_X]);
	fly.axes.setpoint.e[FLY_ROLL_X] =
		fly_pidStep(&fly.direction.pid[FLY_V_Y], fly.direction.setpoint.v[FLY_V_Y], fly.direction.feedback.v[FLY_V_Y]);
	fly.axes.setpoint.throttle +=
		fly_pidStep(&fly.direction.pid[FLY_V_Z], fly.direction.setpoint.v[FLY_V_Z], fly.direction.feedback.v[FLY_V_Z]);
	fly.axes.setpoint.e[FLY_YAW_Z] += fly.direction.setpoint.rotate;
}

static void fly_axesArm(bool arm) {
    if (!arm) {
        for(uint8_t i = 0; i < FLY_EULER_MAX; ++i) {
            fly_pidReset(fly.axes.pid+i);
        }
    }
}

static void fly_axesStep() {
	uint16_t mLeftFront, mLeftBack, mRightFront, mRightBack;
	mLeftFront = 0;
	if (mLeftFront > (USHRT_MAX - fly.axes.setpoint.throttle)) {
		fly.axes.setpoint.throttle = USHRT_MAX - mLeftFront;
	}
	if (mLeftBack > (USHRT_MAX - fly.axes.setpoint.throttle)) {
		fly.axes.setpoint.throttle = USHRT_MAX - mLeftBack;
	}
	if (mRightFront > (USHRT_MAX - fly.axes.setpoint.throttle)) {
		fly.axes.setpoint.throttle = USHRT_MAX - mRightFront;
	}
	if (mRightBack > (USHRT_MAX - fly.axes.setpoint.throttle)) {
		fly.axes.setpoint.throttle = USHRT_MAX - mRightBack;
	}
	mLeftFront += fly.axes.setpoint.throttle;
	mLeftBack += fly.axes.setpoint.throttle;
	mRightFront += fly.axes.setpoint.throttle;
	mRightBack += fly.axes.setpoint.throttle;
}
