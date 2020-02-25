
/*
    Hardware -> Software -> Hardware

    Sensorik -+
    Commands -+-> Software -> Motorregler -> Hardware

    Events:
     - neue Sensordaten
     - Steuerbefehle

    Software läuft
    - neue Sensordaten
    - 
    Orientierung
    Lineare Beschleunigung
    GPS
    Ultraschall
    Altimeter


    Module:
    - Inputs
        <-> Sensoren
        <-- Datenverarbeitung
    - Commander
        <-- Manuelle Steuerbefehle
        <-- Automatische Steuerbefehle
        --> Statusanzeige / Kontrolle
        <-> Konfiguration
    - Controler
        --> Motorregelung
    - Outputs
        --> Motoransteuerung

    Offene Fragen:
    - Ist Outputs als Modul qualifiziert oder Teil vom Controler?
    - Wie werden die Module konfiguriert resp. entsprechend aktualisiert?
    - Wie kommunizieren die Module untereinander?
        --> Module bieten alle ähnliches Interface, ein Linkermodul verlinkt alles

    Erzwungener Systemtakt von 20ms / 50 Hz durch PWM-Ansteuerung der ESCs?
    alter BNO hatte 100 Hz Updaterate
    neuer hat 400 Hz! <3

    Jedes Modul ist ein Prozess
    - Konfig wird geparst als Singleton, global erreichbar
    - bei Konfig Änderungen wird ein bit gesetzt zu welchem alle module bestätigen müssen und ihre internen einstellungen aktualisieren
    Nein:
    - bei Änderungen wird configUpdate(class Config c, time_t t) - Funktion des Moduls (übernommen aus Basis-Klasse) vom Loop aufgerufen
    - jede Variable ist ein Scruct {type_t variablentyp, var_t variable, bool wasUpdated}
    - wird eine Variable von Extern geändert wird wasUpdated auf 1 gesetzt.

    Oder :
    1 - Config-Singleton wird verändert
    2 - Laufendes Modul / Prozess wird aufgefordert alle laufenden Prozesse abzuschliessen
    3 - Währenddem wird neues Modul mit neuen Einstellungen instanziert (aber im Pre-Run Modus gehalten)
    4 - Altes Modul wird gestoppt, neues gestartet.
    5 - Altes Modul entsorgen

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
#define LOG_LOCAL_LEVEL ESP_LOG_VERBOSE
#include "./sensing/sensors.h"
#include "./remote/remote.h"
#include "./info/info.h"

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



void app_main(void* arg){
    // Setup
    esp_log_level_set("*", ESP_LOG_VERBOSE);
    ESP_LOGI("quadro2", "Version: %s - %s", __DATE__, __TIME__);

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

    // ESP_LOGI("quadro2", "Starte Info...");
    // ret = info_init(LED_I2C, 0x00);
    // ESP_LOGI("quadro2", "Status Info: %s", ret ? "Error" : "Ok");

    ESP_LOGI("quadro2", "Start dauerte: %lld", esp_timer_get_time());

    // Main Loop
    while (true) {
        vTaskDelay(1000 / portTICK_PERIOD_MS);
        sensors_setHome();
        vTaskDelay(portMAX_DELAY);
    }
}