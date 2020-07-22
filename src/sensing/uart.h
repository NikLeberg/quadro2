/*
 * File: uart.h
 * ----------------------------
 * Author: Niklaus Leuenberger
 * Date:   2020-07-20
 * ----------------------------
 * UART Treiber für simplerere Handhabung gegenüber idf Treiber.
 */


#pragma once


/** Externe Abhängigkeiten **/

#include "driver/uart.h" // uart_reg.h uart_struct.h


/** Einstellungen **/


/*
 * Function: uart_init
 * ----------------------------
 * Aktiviere eigener UART Treiber auf den gegebenen Pins und setze Baud.
 *
 * uart_port_t uartNum: entsprechender UART
 * gpio_num_t txPin: Host Data-Out, Receiver Data-In, oder UART_PIN_NO_CHANGE
 * gpio_num_t rxPin: Host Data-In, Receiver Data-Out, oder UART_PIN_NO_CHANGE
 * uint32_t baud_rate: Baud e.g. 9600
 * QueueHandle_t rxTimestamp: Queue mit mind. grösse 1 die mit int64_t des Empfangszeitpunkts gefüllt wird
 * 
 * returns: false -> Erfolg, true -> Error
 */
bool uart_init(uart_port_t uartNum, gpio_num_t txPin, gpio_num_t rxPin, uint32_t baud_rate, QueueHandle_t rxTimestamp);

/*
 * Function: uart_baud
 * ----------------------------
 * Setze UART Baudrate.
 *
 * uart_port_t uartNum: entsprechender UART
 * uint32_t baud_rate: Baud e.g. 9600
 * 
 * returns: false -> Erfolg, true -> Error, Baud zu hoch.
 */
bool uart_baud(uart_port_t uartNum, uint32_t baud_rate);

/*
 * Function: uart_rxFifoReset
 * ----------------------------
 * Leere UART FIFO.
 * 
 * uart_port_t uartNum: entsprechender UART
 */
IRAM_ATTR void uart_rxFifoReset(uart_port_t uartNum);

/*
 * Function: uart_rxInterrupt
 * ----------------------------
 * Aktiviere RX Timeout und RX FIFO Überlauf als Interrupts.
 * Interrupt wird im ISR automatisch deaktiviert.
 * 
 * uart_port_t uartNum: entsprechender UART
 * bool enabled: True -> aktiviere Intr, False -> Deaktiviere Intr
 */
IRAM_ATTR void uart_rxInterrupt(uart_port_t uartNum, bool enabled);

/*
 * Function: uart_txAvailable
 * ----------------------------
 * Gibt verfügbarer Platz in Bytes im tx-FIFO zurück.
 *
 * uart_port_t uartNum: entsprechender UART
 *
 * returns: Verbleibender Platz an Bytes im tx-FIFO
 */
uint8_t uart_txAvailable(uart_port_t uartNum);

/*
 * Function: uart_write
 * ----------------------------
 * Schreibe Byte in UART.
 *
 * uart_port_t uartNum: entsprechender UART
 * uint8_t value: zu sendender Wert
 */
void uart_write(uart_port_t uartNum, uint8_t value);

/*
 * Function: uart_rxAvailable
 * ----------------------------
 * Gibt Anzahl der verfügbaren Bytes im rx-FIFO zurück.
 *
 * uart_port_t uartNum: entsprechender UART
 *
 * returns: Anzahl Bytes im rx-FIFO
 */
uint8_t uart_rxAvailable(uart_port_t uartNum);

/*
 * Function: uart_read
 * ----------------------------
 * Lese Byte aus UART. Wenn FIFO leer ist wird 0 zurückgegeben.
 *
 * uart_port_t uartNum: entsprechender UART
 *
 * returns: Byte auf rx-FIFO
 */
uint8_t uart_read(uart_port_t uartNum);
