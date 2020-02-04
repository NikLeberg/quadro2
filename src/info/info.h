/*
 * File: info.h
 * ----------------------------
 * Author: Niklaus Leuenberger
 * Date:   2020-01-28
 * ----------------------------
 *   Implementiert die Ansteuerung für WS2812B 24xLED-Ring und ein I2C Display um Statusinformationen anzuzeigen.
 *   Nutzt I2S Peripherie um WS2812B-Protokoll zu imitieren.
 *   - Pro LED müssen 3 Bytes gesendet werden. Grün | Rot | Blau
 *   - I2S Tx-FIFO hat eine Tiefe von 64 und eine Breite von 32 Bits.
 *   - bei Update wird die interne Abbildung ins WS-Protokoll gewandelt.
 *   - Low: -100-, High: -110- mit per Bit 0,4 us
 * 
 *   Lose nach https://github.com/FastLED/FastLED/blob/master/platforms/esp/32/clockless_i2s_esp32.h epfunden.
 */


#pragma once

/** Externe Abhängigkeiten **/
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include <stdio.h>
#include <stdbool.h>
#include "esp_log.h"
#include "driver/gpio.h"
#include "driver/i2s.h"

/** Interne Abhängigkeiten **/
#include "resources.h"
#include "sensing/i2c.h"


/** Variablendeklaration **/

#define WS_I2S I2S_NUM_0
#define WS_SAMPLE_RATE 800000
#define WS_LED_NUM 24

enum info_input_type_t {
    INFO_DUMMY
};

struct info_input_t {
    enum info_input_type_t type;
    union {
        ;
    };
};

struct info_ws_led_t {
    union {
        struct {
            uint8_t g, r, b;
        };
        uint8_t grb[3];
    };
};

struct info_t {
    struct info_ws_led_t leds[WS_LED_NUM];
};
struct info_t info;

static volatile i2s_dev_t* I2S[I2S_NUM_MAX] = {&I2S0, &I2S1};


/** Public Functions **/

/*
 * Function: info_init
 * ----------------------------
 *   Initialisiert interne Konfiguration und startet Haupttask.
 *
 *   gpio_num_t dataOut: Datenleitung zu LED-Ring (an LED di)
 *   uint8_t addr: I2C Display Adresse
 *
 *   returns: false bei Erfolg, sonst true 
 */
bool info_init(gpio_num_t di, uint8_t bnoAddr);


/** Private Functions **/

/*
 * Function: info_task
 * ----------------------------
 *   Haupttask. Verwaltet alle Inputs und stellt diese auf dem geeigneten Medium dar.
 *
 *   void* arg: Dummy für FreeRTOS
 */
void info_task(void* arg);

// ToDo
static bool info_ws_update();
static inline void info_ws_setRGB(uint8_t ledNum, uint8_t red, uint8_t green, uint8_t blue);
static bool info_ws_init(gpio_num_t dataOut);
static void IRAM_ATTR info_ws_interrupt(void* arg);


/** Implementierung **/

bool info_init(gpio_num_t dataOut, uint8_t addr) {
    // Input-Queue erstellen
    xInfo_input = xQueueCreate(2, sizeof(struct info_input_t));
    // Display initialisieren
    // info_display_init();
    // I2S initialisieren
    if (info_ws_init(dataOut)) return true;
    // installiere task
    if (xTaskCreate(&info_task, "info", 3 * 1024, NULL, xInfo_PRIORITY, &xInfo_handle) != pdTRUE) return true;
    return false;
}

/* Haupttask */

void info_task(void* arg) {
    // Variablen
    struct info_input_t input;
    uint8_t i = 0, j = 0;
    // Loop
    for (uint8_t i = 0; i < WS_LED_NUM; ++i) {
        info_ws_setRGB(i, 126, 0, 0);
    }
    while (true) {
        vTaskDelay(1);
        info_ws_update();
        for (uint8_t i = 0; i < WS_LED_NUM; ++i) {
            //info_ws_setRGB(i, (esp_random() % 256) / 2, 0, 0);
            info_ws_setRGB(i, 0, 0, 0);
        }
        ++j;
        if (j > WS_LED_NUM - 1) j = 0;
        info_ws_setRGB(j, 0x1 << 1, 0, 0);
        info_ws_setRGB((j + 1) % 24, 0x1 << 3, 0, 0);
        info_ws_setRGB((j + 2) % 24, 0x1 << 5, 0, 0);
        info_ws_setRGB((j + 3) % 24, 0x1 << 7, 0, 0);
        info_ws_setRGB(j + 8, 0, 0x1 << 1, 0);
        info_ws_setRGB((j + 8 + 1) % 24, 0, 0x1 << 3, 0);
        info_ws_setRGB((j + 8 + 2) % 24, 0, 0x1 << 5, 0);
        info_ws_setRGB((j + 8 + 3) % 24, 0, 0x1 << 7, 0);
        info_ws_setRGB(j + 16, 0, 0, 0x1 << 1);
        info_ws_setRGB((j + 16 + 1) % 24, 0, 0, 0x1 << 3);
        info_ws_setRGB((j + 16 + 2) % 24, 0, 0, 0x1 << 5);
        info_ws_setRGB((j + 16 + 3) % 24, 0, 0, 0x1 << 7);
        continue;

        continue;

        if (xQueueReceive(xInfo_input, &input, 5000 / portTICK_PERIOD_MS) == pdTRUE) {
            switch (input.type) {
                default:
                    break;
            }
        } else {
            ESP_LOGD("info", "%llu,online", esp_timer_get_time());
        }
    }
}

static bool info_ws_init(gpio_num_t dataOut) {
    // Modul aktivieren
    if (WS_I2S == I2S_NUM_1) periph_module_enable(PERIPH_I2S1_MODULE);
    else periph_module_enable(PERIPH_I2S0_MODULE);
    // Data-Out Pin
    PIN_FUNC_SELECT(GPIO_PIN_MUX_REG[dataOut], PIN_FUNC_GPIO);
    if (gpio_set_direction(dataOut, GPIO_MODE_OUTPUT)) return true;
    if (gpio_set_pull_mode(dataOut, GPIO_PULLDOWN_ONLY)) return true;
    gpio_matrix_out(dataOut, (WS_I2S == I2S_NUM_0) ? I2S0O_DATA_OUT23_IDX : I2S1O_DATA_OUT23_IDX, 0, 0);
    // Reset
    I2S[WS_I2S]->conf.tx_start = 0;
    I2S[WS_I2S]->conf.tx_reset = 1;
    I2S[WS_I2S]->conf.tx_reset = 0;
    I2S[WS_I2S]->conf.tx_fifo_reset = 1;
    I2S[WS_I2S]->conf.tx_fifo_reset = 0;
    // Master Mode
    I2S[WS_I2S]->conf.tx_msb_right = 1;
    I2S[WS_I2S]->conf.tx_mono = 0;
    I2S[WS_I2S]->conf.tx_short_sync = 0;
    I2S[WS_I2S]->conf.tx_msb_shift = 0;
    I2S[WS_I2S]->conf.tx_right_first = 1;
    I2S[WS_I2S]->conf.tx_slave_mod = 0;
    // Sample Rate
    I2S[WS_I2S]->sample_rate_conf.val = 0;
    I2S[WS_I2S]->sample_rate_conf.tx_bits_mod = 32;
    I2S[WS_I2S]->sample_rate_conf.tx_bck_div_num = 1;
    // Clock
    I2S[WS_I2S]->clkm_conf.val = 0;
    I2S[WS_I2S]->clkm_conf.clka_en = 0; //?
    // 80 MHz / 64 = 1.25 Mhz = ~ 0.8us Clock. (gilt für Zeit für zwei Bits)
    I2S[WS_I2S]->clkm_conf.clkm_div_a = 0; //?
    I2S[WS_I2S]->clkm_conf.clkm_div_b = 1;
    I2S[WS_I2S]->clkm_conf.clkm_div_num = 64;
    // 1 / (80 MHz / ?) = 0.8 / 1000 / 1000
    I2S[WS_I2S]->clkm_conf.clk_en = 1;
    // FIFO (ohne DMA)
    I2S[WS_I2S]->fifo_conf.val = 0;
    I2S[WS_I2S]->fifo_conf.tx_fifo_mod_force_en = 1;
    I2S[WS_I2S]->fifo_conf.tx_fifo_mod = 2; // 32 Bit 2-Kanalig, aber WS interessiert uns nicht
    I2S[WS_I2S]->fifo_conf.tx_data_num = 32;
    // Modes
    I2S[WS_I2S]->conf1.val = 0;
    I2S[WS_I2S]->conf1.tx_stop_en = 0; //?
    I2S[WS_I2S]->conf1.tx_pcm_bypass = 1;
    I2S[WS_I2S]->conf_chan.val = 0;
    I2S[WS_I2S]->conf_chan.tx_chan_mod = 0; // 2-Kanalig, Links nicht nach Rechts spiegeln
    // kein Timingcheck
    I2S[WS_I2S]->timing.val = 0;

    // Master starten
    I2S[WS_I2S]->conf.tx_start = 1;

    // Test 1
    I2S[WS_I2S]->conf2.lcd_en = 0; // Paraleller LCD Modi deaktivieren
    
    // Interrupt registrieren
    if (esp_intr_alloc(ETS_I2S0_INTR_SOURCE + WS_I2S, ESP_INTR_FLAG_IRAM, info_ws_interrupt, NULL, NULL)) return true;

    return false;
}

static inline void info_ws_setRGB(uint8_t ledNum, uint8_t red, uint8_t green, uint8_t blue) {
    info.leds[ledNum].r = red;
    info.leds[ledNum].g = green;
    info.leds[ledNum].b = blue;
    return;
}

static bool info_ws_update() {
    // RGB-Farben in I2S-Bitstream wandeln
    uint8_t grbStream[WS_LED_NUM * 3 * 3];
    for (uint8_t i = 0; i < WS_LED_NUM; ++i) { // jede LED
        for (uint8_t j = 0; j < 3; ++j) { // jede Farbe
            uint32_t color = 0;
            for (uint8_t k = 0; k < 8; ++k) { // jedes Farbbit
                color <<= 3;
                if ((info.leds[i].grb[j] >> (7 - k)) & 0x1) {
                    color |= 0b110; // 1 -> 110
                } else {
                    color |= 0b100; // 0 -> 100
                }
            }
            grbStream[(i * 9) + (j * 3)] = color >> 16;
            grbStream[(i * 9) + (j * 3) + 1] = color >> 8;
            grbStream[(i * 9) + (j * 3) + 2] = color;
        }
    }
    // FIFO mit Bitstream füllen
    uint16_t pos = 0;
    uint32_t buffer = 0;
    for (; pos < sizeof(grbStream); pos = pos + 4) {
        I2S[WS_I2S]->reserved_0 = grbStream[pos] << 24 | grbStream[pos + 1] << 16 | grbStream[pos + 2] << 8 | grbStream[pos + 3];
    }
    pos -= 4;
    switch (sizeof(grbStream) - pos) {
        default:
        case (0):
            break;
        case (1):
            I2S[WS_I2S]->reserved_0 = grbStream[pos] << 24;
            break;
        case (2):
            I2S[WS_I2S]->reserved_0 = grbStream[pos] << 24 | grbStream[pos + 1] << 16;
            break;
        case (3):
            I2S[WS_I2S]->reserved_0 = grbStream[pos] << 24 | grbStream[pos + 1] << 16 | grbStream[pos + 2] << 8;
            break;
    }
    // TX start
    I2S[WS_I2S]->int_ena.tx_rempty = 1;

    return false;
}

static void IRAM_ATTR info_ws_interrupt(void* arg) {
    if (I2S[WS_I2S]->int_st.tx_rempty) {
        I2S[WS_I2S]->int_ena.tx_rempty = 0; // Interrupt deaktivieren
        //I2S[WS_I2S]->conf.tx_start = 0; // TX beenden
        for (uint32_t i = 0; i < 64; ++i) { // FIFO überschreiben damit nicht nochmal gesendet wird
            I2S[WS_I2S]->reserved_0 = 0;
        }
    }
    I2S[WS_I2S]->int_clr.val = I2S[WS_I2S]->int_st.val; // Interrupts zurücksetzen 
}
