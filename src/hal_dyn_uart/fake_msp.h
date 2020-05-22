/*
 * fake_msp.h
 */

#ifndef FAKE_MSP_H
#define FAKE_MSP_H

#include <stdbool.h>
#include <stdint.h>

//#define UCA2STATW true
//#define UCBUSY false

#define UCBUSY true
#define UCTXIFG true

extern volatile uint8_t UCA2TXBUF;
extern volatile uint8_t UCA2RXBUF;
extern volatile uint16_t UCA2IFG;
extern volatile uint16_t UCA2STATW;

#endif /* FAKE_MSP_H */
