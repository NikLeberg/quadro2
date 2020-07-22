/*
 * File: uart.c
 * ----------------------------
 * Author: Niklaus Leuenberger
 * Date:   2020-07-20
 * ----------------------------
 * UART Treiber für simplerere Handhabung gegenüber idf Treiber.
 */


/** Externe Abhängigkeiten **/

#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"


/** Interne Abhängigkeiten **/

#include "uart.h"


/** Variablendeklaration **/

extern uart_dev_t UART0;
extern uart_dev_t UART1;
extern uart_dev_t UART2;
DRAM_ATTR uart_dev_t* const UART[UART_NUM_MAX] = {&UART0, &UART1, &UART2};
QueueHandle_t rxQueues[UART_NUM_MAX]; // Queues die bei rx mit Zeitstempeln gefüllt wird
int64_t rxDelaysUs[UART_NUM_MAX]; // Zeit die zum Empfang von einem Byte benötigt wird


/*
 * Function: uart_interrupt
 * ----------------------------
 * ISR. Ausgeführt bei UART-Events. Setzt UART zurück bei Fehler oder entsperrt Semaphoren.
 *
 * void* arg: [(uart_port_t) uartNum]: entsprechende UART-Nummer
 */
static void uart_interrupt(void* arg);


/** Implementierung **/

bool uart_init(uart_port_t uartNum, gpio_num_t txPin, gpio_num_t rxPin, uint32_t baud_rate, QueueHandle_t rxTimestamp) {
    uart_dev_t *uart = UART[uartNum];
    // Aktivieren
    periph_module_enable(uartNum + 1);
    // Data-Bits
    uart->conf0.bit_num = UART_DATA_8_BITS;
    // Parity
    uart->conf0.parity_en = 0;
    // Stop-Bits
    uart->conf0.stop_bit_num = UART_STOP_BITS_1;
    // Flow-Control
    uart->conf0.tx_flow_en = 0;
    uart->conf1.rx_flow_en = 0;
    // fehlerhafte Frames nicht im FIFO speichern
    uart->conf0.err_wr_mask = 1;
    // Clock
    uart->conf0.tick_ref_always_on = 1;
    // Pins
    if (uart_set_pin(uartNum, txPin, rxPin, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE)) return true;
    // Baud
    uart_baud(uartNum, baud_rate);
    // Interrupt Handler registrieren
    if (rxTimestamp) {
        if (esp_intr_alloc(ETS_UART0_INTR_SOURCE + uartNum, 0, uart_interrupt, (void*)uartNum, NULL)) return true;
        // Interrupts aktivieren
        uart->int_clr.val = UART_INTR_MASK;
        uart->conf1.rx_tout_thrhd = 10; // Interrupt nach Ende eines Frames
        uart->conf1.rx_tout_en = 1;
        uart->conf1.rxfifo_full_thrhd = 120; // Interrupt kurz vor rx-FIFO Überlauf damit dieser noch ohne Verlust geleert werden kann
        uart->int_ena.val = UART_RXFIFO_FULL_INT_ENA_M;
        rxQueues[uartNum] = rxTimestamp;
    }
    return false;
}

bool uart_baud(uart_port_t uartNum, uint32_t baud_rate) {
    uint32_t clk_div = ((APB_CLK_FREQ << 4) / baud_rate);
    if (clk_div < 16) return true;
    else {
        UART[uartNum]->clk_div.div_int = clk_div >> 4;
        UART[uartNum]->clk_div.div_frag = clk_div & 0xf;
    }
    // rx Delay berechnen
    // 10 Bits per 1 Byte / baud = Zeit in Sekunden
    rxDelaysUs[uartNum] = 10.0 / baud_rate / 1000.0 / 1000.0;
    return false;
}

IRAM_ATTR void uart_rxFifoReset(uart_port_t uartNum) {
    uart_dev_t *uart = UART[uartNum];
    while (uart->status.rxfifo_cnt) {
        (volatile void) uart->fifo.rw_byte;
    }
}

IRAM_ATTR void uart_rxInterrupt(uart_port_t uartNum, bool enabled) {
    uart_dev_t *uart = UART[uartNum];
    uart->int_ena.rxfifo_tout = enabled;
    uart->int_ena.rxfifo_full = enabled;
}

uint8_t uart_txAvailable(uart_port_t uartNum) {
    uart_dev_t *uart = UART[uartNum];
    return (UART_FIFO_LEN - uart->status.txfifo_cnt);
}

void uart_write(uart_port_t uartNum, uint8_t value) {
    uart_dev_t *uart = UART[uartNum];
    uart->fifo.rw_byte = value;
}

uint8_t uart_rxAvailable(uart_port_t uartNum) {
    uart_dev_t *uart = UART[uartNum];
    return uart->status.rxfifo_cnt;
}

uint8_t uart_read(uart_port_t uartNum) {
    uart_dev_t *uart = UART[uartNum];
    return uart->fifo.rw_byte;
}

IRAM_ATTR static void uart_interrupt(void* arg) {
    uart_port_t uartNum = (uart_port_t)arg;
    uart_dev_t *uart = UART[uartNum];
    BaseType_t woken = pdFALSE;
    uint32_t status = uart->int_st.val;
    uart->int_clr.val = status; // aktive Interrupts zurücksetzen
    if (status & UART_RXFIFO_TOUT_INT_ST_M || // rx-Frame erhalten
        status & UART_RXFIFO_FULL_INT_ST_M) { // rx-FIFO bald voll
        uart_rxInterrupt(uartNum, false); // rx-Interrupts deaktivieren
        if (rxQueues[uartNum]) {
            int64_t timestamp = esp_timer_get_time();
            timestamp -= uart->status.rxfifo_cnt * rxDelaysUs[uartNum]; // ToDo: Zeit für TOUT Interrupt-generierung abziehen
            xQueueSendFromISR(rxQueues[uartNum], &timestamp, &woken); // rx beendet, entsperren
        }
    } else if (status & UART_RXFIFO_OVF_INT_ST_M) { // rx-FIFO Überlauf
        uart_rxFifoReset(uartNum); // FIFO zurücksetzten
    }
    if (woken == pdTRUE) portYIELD_FROM_ISR();
}
