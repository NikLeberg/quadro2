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


    3.0V - 4.2V per LiPo-Zelle
    9.0V - 12.6V

    Motorprofil:
    Volt	12.6	10.8	9	
    0.03	16	    10	    7	
    0.1	    44	    27	    22	
    0.2	    79	    56	    43	
    0.3	    107	    80	    62
    0.4	    145	    106	    62	
    0.5	    175	    128	    62	
    0.6	    211	    160		
    0.7	    250	    190		<
    0.8	    	    190		
    0.9	    	    197		
    1				
        >6A	?	<9V	
    // Info
    http://esp32.net/
    // Schema
    http://esp32.net/images/Ai-Thinker/NodeMCU-32S/Ai-Thinker_NodeMCU-32S_DiagramSchematic.png
    // Pinout
    https://cdn.instructables.com/FOL/YWLI/JEOILQ5U/FOLYWLIJEOILQ5U.LARGE.jpg?auto=webp&frame=1&width=1024&fit=bounds

*/

/** Externe Abhängigkeiten **/
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/uart.h"
#include "driver/gpio.h"
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
#define ULTRASONIC_TRIGGER  GPIO_NUM_26 // über Wiederstand-Spannungsteiler
#define ULTRASONIC_ECHO     GPIO_NUM_27
#define GPS_RX_HOST_TX      GPIO_NUM_17 // über 100 Ohm Wiederstand
#define GPS_TX_HOST_RX      GPIO_NUM_16
#define LED_I2C             GPIO_NUM_22
#define MOTOR_FRONT_LEFT    GPIO_NUM_4
#define MOTOR_FRONT_RIGHT   GPIO_NUM_0
#define MOTOR_BACK_LEFT     GPIO_NUM_2
#define MOTOR_BACK_RIGHT    GPIO_NUM_15

// Bugs:
// - Sensors X rechnet noch nicht mit Geschwindigkeit vom GPS
// - libesphttpd besser portieren
// - gps ubx Prüfsumme wird nicht geprüft
// - sensor timeout
// - rateUpdate von Ultraschall ungetestet
// - stack overflow in remote (nach längerer Zeit) / esphttpd (nach Website Refresh)

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

    pvRegister((QueueHandle_t)1, main_pvs);

    bool ret = false;
    ESP_LOGI("quadro2", "Starte Sensorik...");
    ret = sensors_init(I2C_SCL, I2C_SDA,
                       0x4A, BNO_INTERRUPT, BNO_RESET,
                       ULTRASONIC_TRIGGER, ULTRASONIC_ECHO,
                       GPS_TX_HOST_RX, GPS_RX_HOST_TX,
                       0x40);
    ESP_LOGI("quadro2", "Status Sensorik: %s", ret ? "Error" : "Ok");

    ESP_LOGI("quadro2", "Starte Remote...");
    ret = remote_init("OnePlus 8", "594cbd28ee05");
    ESP_LOGI("quadro2", "Status Remote: %s", ret ? "Error" : "Ok");

    ESP_LOGI("quadro2", "Starte Control...");
    ret = control_init(MOTOR_FRONT_LEFT, MOTOR_FRONT_RIGHT, MOTOR_BACK_LEFT, MOTOR_BACK_RIGHT);
    ESP_LOGI("quadro2", "Status Control: %s", ret ? "Error" : "Ok");

    // ESP_LOGI("quadro2", "Starte Info...");
    // ret = info_init(LED_I2C, 0x00);
    // ESP_LOGI("quadro2", "Status Info: %s", ret ? "Error" : "Ok");

    ESP_LOGI("quadro2", "Start dauerte: %lld", esp_timer_get_time());

    vTaskPrioritySet(NULL, 1);

    // Main Loop
    while (true) {
        vTaskDelay(2000 / portTICK_PERIOD_MS);
        currentTick = xTaskGetTickCount() * portTICK_PERIOD_MS;
        pvPublishUint((QueueHandle_t)1, MAIN_PV_TICKS, currentTick);
        // Stats
        // ESP_LOGD("stats", "Queue Load:");
        // ESP_LOGD("sensors", "%d von 16", uxQueueMessagesWaiting(xSensors));
        // ESP_LOGD("bno", "%d von 4", uxQueueMessagesWaiting(xBno));
        // ESP_LOGD("remote", "%d von 32", uxQueueMessagesWaiting(xRemote));
        // ESP_LOGD("control", "%d von 16", uxQueueMessagesWaiting(xControl));
    }
}
