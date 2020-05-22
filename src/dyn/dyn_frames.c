/*
 * dyn_frames.c
 *
 * Dynamixel framing layer
 */

#include <unistd.h>
#include <stdio.h>

#include "dyn_frames.h"
#include "dyn_instr.h"
#include "../hal_dyn_uart/hal_dyn_uart_emu.h"
#include "../hal_dyn_uart/fake_msp.h"

#define f_TxUAC2 TxUAC2_emu
#define f_Sentit_Dades_Tx Sentit_Dades_Tx_emu
#define f_Sentit_Dades_Rx Sentit_Dades_Rx_emu
#define f_rx_uart_byte rx_uart_byte_emu


// TxPacket() 3 paràmetres: ID del Dynamixel, Mida dels paràmetres, Instruction byte. Torna la mida del "Return packet"
byte TxPacket(byte bID, byte bParameterLength, byte bInstruction, const byte *Parameters) {
    byte bCount, bCheckSum, bPacketLength;
    byte TxBuffer[32];

    // Per seguretat (dels motors): impedim un intent d'escriptura a una direcció <= 0x05
    if ((Parameters[0] <= 5) && (bInstruction == DYN_INSTR__WRITE)) {
        printf("ERROR: Intent d'escriptura a una direcció no permesa.\n");
        return 0;
    }

    // Línia de dades en Tx: transmetre
    f_Sentit_Dades_Tx();

    TxBuffer[0] = 0xff; // Primers 2 bytes que indiquen inici de trama FF, FF.
    TxBuffer[1] = 0xff;
    TxBuffer[2] = bID; // ID del mòdul al que volem enviar el missatge
    TxBuffer[3] = bParameterLength + 2; // Length(Parameter,Instruction,Checksum)
    TxBuffer[4] = bInstruction; // Instrucció que enviem al mòdul

    // Comencem a generar la trama que hem d’enviar
    for (bCount = 0; bCount < bParameterLength; bCount++) {
        TxBuffer[bCount + 5] = Parameters[bCount];
    }
    bCheckSum = 0;
    bPacketLength = bParameterLength + 4 + 2;

    // Càlcul del checksum
    for (bCount = 2; bCount < bPacketLength - 1; bCount++) {
        bCheckSum += TxBuffer[bCount];
    }

    // Escriu el Checksum (complement a 1)
    TxBuffer[bCount] = ~bCheckSum;

    // Aquest bucle és el que envia la trama al Mòdul Robot
    for (bCount = 0; bCount < bPacketLength; bCount++) {
        while (!(UCA2IFG & UCTXIFG)) {}
        f_TxUAC2(TxBuffer[bCount]);
    }

    // Espera fins que s’ha transmès el últim byte
    while ((UCA2STATW & UCBUSY)) {}

    return (bPacketLength);
}


// RxPacket(): retorna una estructura "Status packet"
struct RxReturn RxPacket(void) {

    // Inicialitzem el status packet
    struct RxReturn respuesta;
    byte bCount, bCheckSum = 0;

    respuesta.time_out = false;
    respuesta.tx_err = false;
    respuesta.idx = 0; // Inicialitzem l'índex d'escriptura del status packet

    // Posem la línia de dades en Rx: el mòdul Dynamixel envia la resposta
    f_Sentit_Dades_Rx();

    //f_Activa_Timer_TimeOut();

    // Llegim els 4 primers paràmetres
    for (bCount = 0; bCount < 4; bCount++) {
        f_rx_uart_byte(&respuesta);
    }

    // Continua llegint la resta de bytes del Status Packet
    if (!respuesta.time_out) {
        for (bCount = 0; bCount < respuesta.StatusPacket[3]; bCount++) {
            f_rx_uart_byte(&respuesta);
        }
    }

    // Calculem i comprovem el checksum del status packet
    if (!respuesta.time_out) {

        // Calculem el checksum
        for (bCount = 2; bCount < respuesta.idx - 1; bCount++) {
            bCheckSum += respuesta.StatusPacket[bCount];
        }
        bCheckSum = ~bCheckSum; // Complement a 1

        // Comparem el checksum rebut amb el calculat
        if (respuesta.StatusPacket[respuesta.idx - 1] != bCheckSum) {
            respuesta.tx_err = true;
        }
    }

    return respuesta;
}


/**
 * Perform a full read/write transaction
 *
 * This function will send an instruction frame to the dynamixel module
 * and the following status frame
 *
 * @param[in] bID Id of the dynamixel module
 * @param[in] bParameterLength Number of parameters to send
 * @param[in] bInstruction Instruction type of the frame
 * @param[in] Parametros Parameters of the TX frame
 * @return Returns a RxReturn struct with the information of the reply
 */
struct RxReturn RxTxPacket(byte bID, byte bParameterLength, byte bInstruction, const byte *Parameters) {
    struct RxReturn respuesta;
    if (TxPacket(bID, bParameterLength, bInstruction, Parameters) == 0) {
        respuesta.tx_err = true;
    } else {
        respuesta = RxPacket();
    }
    return respuesta;
}
