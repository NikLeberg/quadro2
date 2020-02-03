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
        uint8_t g, r, b;
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
static bool info_ws_tx(uint32_t color, bool final);
static bool info_ws_update();
static inline uint32_t info_ws_getColor(uint8_t ledNum, uint8_t color);
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
    // i2s_config_t i2s_config = {
    //     .mode = ((i2s_mode_t) (I2S_MODE_MASTER | I2S_MODE_TX)),
    //     .sample_rate = WS_SAMPLE_RATE,
    //     .bits_per_sample = I2S_BITS_PER_SAMPLE_32BIT,
    //     .channel_format = I2S_CHANNEL_FMT_ONLY_RIGHT,
    //     .communication_format = ((i2s_comm_format_t) (I2S_COMM_FORMAT_I2S | I2S_COMM_FORMAT_I2S_MSB)),
    //     .intr_alloc_flags = ESP_INTR_FLAG_LEVEL1,
    //     .dma_buf_count = 2,
    //     .dma_buf_len = 128,
    //     .use_apll = false
    // };
    // i2s_pin_config_t pin_config = {
    //     .bck_io_num = GPIO_NUM_23,
    //     .ws_io_num = I2S_PIN_NO_CHANGE,
    //     .data_out_num = dataOut,
    //     .data_in_num = I2S_PIN_NO_CHANGE
    // };
    // i2s_driver_install(WS_I2S, &i2s_config, 0, NULL);
    //i2s_set_pin(WS_I2S, &pin_config);
    // installiere task
    if (xTaskCreate(&info_task, "info", 2 * 1024, NULL, xInfo_PRIORITY, &xInfo_handle) != pdTRUE) return true;
    return false;
}

/* Haupttask */

void info_task(void* arg) {
    // Variablen
    struct info_input_t input;
    uint32_t data0 = 0b10101110001110001110001110001110;
    uint32_t data1 = 0b10100001110001110001110001110000;
    // Loop
    while (true) {
        vTaskDelay(1000);
        //info_ws_update();
        //continue;
        // DEBUG
        ESP_LOGV("info", "pre");
        ESP_LOGV("info", "I2S_CONF_REG          %#x", I2S[WS_I2S]->conf.val);
        ESP_LOGV("info", "I2S_CONF1_REG         %#x", I2S[WS_I2S]->conf1.val);
        ESP_LOGV("info", "I2S_CONF2_REG         %#x", I2S[WS_I2S]->conf2.val);
        ESP_LOGV("info", "I2S_TIMING_REG        %#x", I2S[WS_I2S]->timing.val);
        ESP_LOGV("info", "I2S_FIFO_CONF_REG     %#x", I2S[WS_I2S]->fifo_conf.val);
        ESP_LOGV("info", "I2S_CONF_SINGLE_DATA_REG %#x", I2S[WS_I2S]->conf_single_data);
        ESP_LOGV("info", "I2S_CONF_CHAN_REG     %#x", I2S[WS_I2S]->conf_chan.val);
        ESP_LOGV("info", "I2S_LC_HUNG_CONF_REG  %#x", I2S[WS_I2S]->lc_hung_conf.val);
        ESP_LOGV("info", "I2S_CLKM_CONF_REG     %#x", I2S[WS_I2S]->clkm_conf.val);
        ESP_LOGV("info", "I2S_SAMPLE_RATE_CONF_REG %#x", I2S[WS_I2S]->sample_rate_conf.val);
        ESP_LOGV("info", "I2S_PD_CONF_REG       %#x", I2S[WS_I2S]->pd_conf.val);
        ESP_LOGV("info", "I2S_CLKM_CONF_REG     %#x", I2S[WS_I2S]->clkm_conf.val);
        ESP_LOGV("info", "I2S_STATE_REG I2S     %#x", I2S[WS_I2S]->state.val);
        ESP_LOGV("info", "I2S_LC_CONF_REG I2S   %#x", I2S[WS_I2S]->lc_conf.val);
        ESP_LOGV("info", "I2S_LC_STATE1_REG     %#x", I2S[WS_I2S]->lc_state1);
        ESP_LOGV("info", "I2S_PDM_CONF_REG      %#x", I2S[WS_I2S]->pdm_conf.val);

        ESP_LOGV("info", "%u-write %u fifo %u", 0, data0, I2S[WS_I2S]->reserved_0);
        I2S[WS_I2S]->reserved_0 = data0;
        ESP_LOGV("info", "%u-write %u fifo %u", 1, data1, I2S[WS_I2S]->reserved_0);
        I2S[WS_I2S]->reserved_0 = data1;
        // TX start
        I2S[WS_I2S]->conf.tx_start = 1;
        while (!I2S[WS_I2S]->state.tx_idle); // warte auf Ende
        I2S[WS_I2S]->conf.tx_start = 0;

        // DEBUG
        ESP_LOGV("info", "apre");
        ESP_LOGV("info", "I2S_CONF_REG          %#x", I2S[WS_I2S]->conf.val);
        ESP_LOGV("info", "I2S_CONF1_REG         %#x", I2S[WS_I2S]->conf1.val);
        ESP_LOGV("info", "I2S_CONF2_REG         %#x", I2S[WS_I2S]->conf2.val);
        ESP_LOGV("info", "I2S_TIMING_REG        %#x", I2S[WS_I2S]->timing.val);
        ESP_LOGV("info", "I2S_FIFO_CONF_REG     %#x", I2S[WS_I2S]->fifo_conf.val);
        ESP_LOGV("info", "I2S_CONF_SINGLE_DATA_REG %#x", I2S[WS_I2S]->conf_single_data);
        ESP_LOGV("info", "I2S_CONF_CHAN_REG     %#x", I2S[WS_I2S]->conf_chan.val);
        ESP_LOGV("info", "I2S_LC_HUNG_CONF_REG  %#x", I2S[WS_I2S]->lc_hung_conf.val);
        ESP_LOGV("info", "I2S_CLKM_CONF_REG     %#x", I2S[WS_I2S]->clkm_conf.val);
        ESP_LOGV("info", "I2S_SAMPLE_RATE_CONF_REG %#x", I2S[WS_I2S]->sample_rate_conf.val);
        ESP_LOGV("info", "I2S_PD_CONF_REG       %#x", I2S[WS_I2S]->pd_conf.val);
        ESP_LOGV("info", "I2S_CLKM_CONF_REG     %#x", I2S[WS_I2S]->clkm_conf.val);
        ESP_LOGV("info", "I2S_STATE_REG I2S     %#x", I2S[WS_I2S]->state.val);
        ESP_LOGV("info", "I2S_LC_CONF_REG I2S   %#x", I2S[WS_I2S]->lc_conf.val);
        ESP_LOGV("info", "I2S_LC_STATE1_REG     %#x", I2S[WS_I2S]->lc_state1);
        ESP_LOGV("info", "I2S_PDM_CONF_REG      %#x", I2S[WS_I2S]->pdm_conf.val);
        // Tx start
        //I2S[WS_I2S]->conf.tx_start = 1;
        //while (I2S[WS_I2S]->state.tx_idle); // warte auf Start
        //while (!I2S[WS_I2S]->state.tx_idle); // warte auf Ende
        //I2S[WS_I2S]->conf.tx_start = 0;
        //ESP_LOGV("info", "TX done");
        //I2S[WS_I2S]->conf.tx_start = 0;
        //I2S[WS_I2S]->conf.tx_fifo_reset = 1;
        //I2S[WS_I2S]->conf.tx_fifo_reset = 0;
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
    //I2S[WS_I2S]->conf.tx_start = 1;

    // Test 1
    I2S[WS_I2S]->conf2.lcd_en = 0; // Paraleller LCD Modi deaktivieren
    //I2S[WS_I2S]->conf1.tx_stop_en = 1; // TX = 0 mach FIFO leer, force deak, data_num = 0, master immer an

    
    // DEBUG
    ESP_LOGV("info", "I2S_CONF_REG          %#x", I2S[WS_I2S]->conf.val);
    ESP_LOGV("info", "I2S_CONF1_REG         %#x", I2S[WS_I2S]->conf1.val);
    ESP_LOGV("info", "I2S_CONF2_REG         %#x", I2S[WS_I2S]->conf2.val);
    ESP_LOGV("info", "I2S_TIMING_REG        %#x", I2S[WS_I2S]->timing.val);
    ESP_LOGV("info", "I2S_FIFO_CONF_REG     %#x", I2S[WS_I2S]->fifo_conf.val);
    ESP_LOGV("info", "I2S_CONF_SINGLE_DATA_REG %#x", I2S[WS_I2S]->conf_single_data);
    ESP_LOGV("info", "I2S_CONF_CHAN_REG     %#x", I2S[WS_I2S]->conf_chan.val);
    ESP_LOGV("info", "I2S_LC_HUNG_CONF_REG  %#x", I2S[WS_I2S]->lc_hung_conf.val);
    ESP_LOGV("info", "I2S_CLKM_CONF_REG     %#x", I2S[WS_I2S]->clkm_conf.val);
    ESP_LOGV("info", "I2S_SAMPLE_RATE_CONF_REG %#x", I2S[WS_I2S]->sample_rate_conf.val);
    ESP_LOGV("info", "I2S_PD_CONF_REG       %#x", I2S[WS_I2S]->pd_conf.val);
    ESP_LOGV("info", "I2S_CLKM_CONF_REG     %#x", I2S[WS_I2S]->clkm_conf.val);
    ESP_LOGV("info", "I2S_STATE_REG I2S     %#x", I2S[WS_I2S]->state.val);
    ESP_LOGV("info", "I2S_LC_CONF_REG I2S   %#x", I2S[WS_I2S]->lc_conf.val);
    ESP_LOGV("info", "I2S_LC_STATE1_REG     %#x", I2S[WS_I2S]->lc_state1);
    ESP_LOGV("info", "I2S_PDM_CONF_REG      %#x", I2S[WS_I2S]->pdm_conf.val);
    
    // Interrupt registrieren
    //if (esp_intr_alloc(ETS_I2S0_INTR_SOURCE + WS_I2S, ESP_INTR_FLAG_IRAM, info_ws_interrupt, NULL, NULL)) return true;
    //I2S[WS_I2S]->int_ena.tx_rempty = 1;

    return false;
}

static inline void info_ws_setRGB(uint8_t ledNum, uint8_t red, uint8_t green, uint8_t blue) {
    info.leds[ledNum].r = red;
    info.leds[ledNum].g = green;
    info.leds[ledNum].b = blue;
    return;
}

// "Upsampling" von 8 Bit RGB-Wert auf 24 Bit 0 -> 100, 1 -> 110
static inline uint32_t info_ws_getColor(uint8_t ledNum, uint8_t grb) {
    uint32_t color = 0;
    for (uint8_t i = 0; i < 8; ++i) {
        color = color << 3;
        if ((info.leds[ledNum].grb[grb] >> (7 - i)) & 0x1) {
            color |= 0b110;
        } else {
            color |= 0b100;
        }
    }
    ESP_LOGV("info", "LED-%u, %c %u > %u", ledNum, (grb == 0) ? 'g' : (grb == 1) ? 'r' : 'b', info.leds[ledNum].grb[grb], color);
    return color;
}

static bool info_ws_update() {
    // FIFO Reset
    I2S[WS_I2S]->conf.tx_fifo_reset = 1;
    I2S[WS_I2S]->conf.tx_fifo_reset = 0;
    // FIFO mit aktuellem Zustand füllen
    uint32_t color = 0;
    // for (uint8_t i = 0; i < 1; ++i) {
    for (uint8_t i = 0; i < WS_LED_NUM; ++i) {
        for (uint8_t j = 0; j < 3; ++j) {
            //color = info_ws_getColor(i, j);
            color = 0b00000000110110110110110110110110;
            if (info_ws_tx(color, false)) return true;
        }
    }
    if (info_ws_tx(0, true)) return true;
    // Tx start
    I2S[WS_I2S]->conf.tx_start = 1;
    while (!I2S[WS_I2S]->state.tx_idle) {
        ets_delay_us(1000);
    }
    ESP_LOGV("info", "TX done");
    I2S[WS_I2S]->conf.tx_start = 0;
    return false;
}

// schreibe die 3 Bytes an Farben in Zwischenspeicher, in FIFO schieben sobald 4 Bytes bereit
static bool info_ws_tx(uint32_t color, bool final) {
    static uint8_t position;
    static uint64_t buffer;
    if (I2S[WS_I2S]->int_st.tx_wfull) return true; // FIFO voll
    ESP_LOGV("info", "TX %#llx", buffer);
    buffer |= ((uint64_t) (color & 0x00ffffff)) << (8 * (5 - position));
    ESP_LOGV("info", "TX %u %#x %#llx", position, (uint32_t) (color & 0x00ffffff), buffer);
    // 0x00924924 - alles 0
    // 0x0000000000924924, p:0
    // 0x9249240000000000, p:3
    // 0x9249249249240000, p:3
    position += 3;
    if (position >= 4) {
        // schreibe in tx-fifo
        ESP_LOGV("info", "FIFO %#x", (uint32_t) (buffer >> 32));
        I2S[WS_I2S]->reserved_0 = (buffer >> 32);
        buffer <<= 32;
        position -= 4;
    }
    if (final) {
        if (buffer & 0x00000000ffffffff) { // wenn Fragmente noch übrig
            if (info_ws_tx(0, false)) return true;
        }
        position = 0;
        buffer = 0;
    }
    return false;
}

static void IRAM_ATTR info_ws_interrupt(void* arg) {
    if (I2S[WS_I2S]->int_st.tx_rempty) {
        I2S[WS_I2S]->conf.tx_start = 0; // TX beenden
    }
    I2S[WS_I2S]->int_clr.val = I2S[WS_I2S]->int_st.val; // Interrupts löschen 
}
