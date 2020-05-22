/*
 * dyn_frames.h
 */

#ifndef DYN_FRAMES_H
#define DYN_FRAMES_H

#include <stdint.h>
#include <stdbool.h>

typedef uint8_t byte;

typedef struct RxReturn {
    byte StatusPacket[32];
    uint8_t idx;
    bool time_out;
    bool tx_err;
} RxReturn;

byte TxPacket(byte bID, byte bParameterLength, byte bInstruction, const byte *Parameters);

struct RxReturn RxPacket(void);

struct RxReturn RxTxPacket(byte bID, byte bParameterLength, byte bInstruction, const byte *Parameters);

#endif /* DYN_FRAMES_H */
