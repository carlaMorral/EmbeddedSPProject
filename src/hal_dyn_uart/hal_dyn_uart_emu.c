/*
 * hal_dyn_uart_emu.c
 */

#include <stdio.h>
#include <stdint.h>
#include <unistd.h>

#include "hal_dyn_uart_emu.h"
#include "fake_msp.h"
#include "../dyn/dyn_frames.h"
#include "../dyn_test/b_queue.h"
#include "../main/main.h"

volatile uint8_t UCA2TXBUF = 0;
volatile uint8_t UCA2RXBUF = 0;
volatile uint16_t UCA2IFG = UCTXIFG;
volatile uint16_t UCA2STATW = 0;

// Configuració de la línia de dades Half Duplex: recepció
void Sentit_Dades_Rx_emu(void) {
    //printf("\nHAL_DYN_UART: Changed direction to RX\n");
}

// Configuració de la línia de dades Half Duplex: transmissió
void Sentit_Dades_Tx_emu(void) {
    //printf("\nHAL_DYN_UART: Changed direction to TX\n");
}

void TxUAC2_emu(byte bTxdData) {
    QUEUE_RET ret = QUEUE_ERR;
    UCA2IFG &= ~UCTXIFG; // Buffer de transmisión = lleno
    UCA2STATW |= UCBUSY; // Línea = ocupada
    while (ret != QUEUE_OK) {
        ret = queue_push(bTxdData, &q_tx);
    }
    usleep(20);
}

/**
 * Read a byte from the thread-safe queue and place it inside the Statuspacket of RxReturn
 */
void rx_uart_byte_emu(struct RxReturn *respuesta) {
    QUEUE_RET ret = QUEUE_ERR;
    while (ret != QUEUE_OK) {
        ret = queue_pop(&(respuesta->StatusPacket[respuesta->idx]), &q_rx);
    }
    respuesta->idx++;
    usleep(20);
}
