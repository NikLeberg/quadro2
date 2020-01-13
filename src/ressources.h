/*
 * File:          /ressources.h
 * Project:       quadro2
 * Created Date:  2019-12-04
 * Author:        Niklaus Ruben Leuenberger
 * -----
 * Description:   Globale Ressourcen- und Task-Handles zur Interprozesskommunikation.
 *                Handles müssen von den Modulen initialisiert werden.
 */

/**
 * External dependencies
 */
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "freertos/semphr.h"


#define FLY_TASK_PRIORITY (10U)
TaskHandle_t xFlyTask = NULL;
QueueHandle_t xFlyInput = NULL;
