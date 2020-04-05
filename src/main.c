/*
 * File: main.c
 * ----------------------------
 * Author: Niklaus Leuenberger
 * Date:   2020-03-05
 * ----------------------------
 * Startup aller Module
 */

/*
    Module:
    - Sensors
        - BNO
        - Ultrschall
        - GPS
    - Remote
        - Smartphone
        - Website
            - Konfiguration
            - Befehle / Aktionen
    - Fly
        - Motorregelung & -ansteuerung
    - Info
        - Statusanzeige

    Erzwungener Systemtakt von 20ms / 50 Hz durch PWM-Ansteuerung der ESCs?
    alter BNO hatte 100 Hz Updaterate
    neuer hat 400 Hz! <3

    // Pinout
    https://cdn.instructables.com/FOL/YWLI/JEOILQ5U/FOLYWLIJEOILQ5U.LARGE.jpg?auto=webp&frame=1&width=1024&fit=bounds

*/

/** Externe Abhängigkeiten **/
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/uart.h"
#include "driver/gpio.h"
#include <stdio.h>
#include "esp_log.h"

/** Interne Abhängigkeiten **/
#include "intercom.h"
#include "sensing/sensors.h"
#include "remote/remote.h"
#include "controlling/control.h"
#include "info/info.h"

// Pins
#define I2C_SCL             GPIO_NUM_25
#define I2C_SDA             GPIO_NUM_33
#define BNO_INTERRUPT       GPIO_NUM_35
#define BNO_RESET           GPIO_NUM_32
#define ULTRASONIC_TRIGGER  GPIO_NUM_26
#define ULTRASONIC_ECHO     GPIO_NUM_27
#define GPS_RX_HOST_TX      GPIO_NUM_17
#define GPS_TX_HOST_RX      GPIO_NUM_16
#define LED_I2C             GPIO_NUM_22

// Bugs:
// - Sensors X rechnet noch nicht mit Geschwindigkeit vom GPS
// - libesphttpd besser portieren
// - lib\ringbuf\ringbuf.c führt oft zu Watchdog Timeout im ISR, aber nur jeder "zweiter" Start

typedef enum {
    MAIN_PV_TICKS,
    MAIN_PV_MAX
} main_pv_t;

TickType_t currentTick;

pv_t main_pvs[MAIN_PV_MAX] = {
    PV("tick", VALUE_TYPE_UINT)
};
PV_LIST("main", main_pvs, MAIN_PV_MAX);


void app_main(void* arg) {
    // Setup
    esp_log_level_set("*", ESP_LOG_VERBOSE);
    ESP_LOGI("quadro2", "Version: %s - %s", __DATE__, __TIME__);

    pvRegister(1, main_pvs);

    bool ret = false;
    ESP_LOGI("quadro2", "Starte Sensorik...");
    ret = sensors_init(I2C_SCL, I2C_SDA,
                       0x4A, BNO_INTERRUPT, BNO_RESET,
                       ULTRASONIC_TRIGGER, ULTRASONIC_ECHO,
                       GPS_TX_HOST_RX, GPS_RX_HOST_TX);
    ESP_LOGI("quadro2", "Status Sensorik: %s", ret ? "Error" : "Ok");

    ESP_LOGI("quadro2", "Starte Remote...");
    ret = remote_init("OnePlus 5", "Testing1234");
    ESP_LOGI("quadro2", "Status Remote: %s", ret ? "Error" : "Ok");

    ESP_LOGI("quadro2", "Starte Control...");
    ret = control_init(0, 0, 0, 0);
    ESP_LOGI("quadro2", "Status Control: %s", ret ? "Error" : "Ok");

    // ESP_LOGI("quadro2", "Starte Info...");
    // ret = info_init(LED_I2C, 0x00);
    // ESP_LOGI("quadro2", "Status Info: %s", ret ? "Error" : "Ok");

    ESP_LOGI("quadro2", "Start dauerte: %lld", esp_timer_get_time());

    // Main Loop
    while (true) {
        vTaskDelay(2000 / portTICK_PERIOD_MS);
        currentTick = xTaskGetTickCount() * portTICK_PERIOD_MS;
        pvPublishUint(1, MAIN_PV_TICKS, currentTick);
        // intercom_commandSend(xSensors, SENSORS_COMMAND_SET_HOME);
        // vTaskDelay(portMAX_DELAY);
    }
}
