/*
 * joystick.h
 */

#ifndef JOYSTICK_H
#define JOYSTICK_H

#include <stdint.h>

enum {
    Ninguno, Up, Down, Left, Right, Center, Sw1, Sw2, Otro, Quit
};

void Get_estado(uint8_t *estado, uint8_t *anterior);

void Set_estado_anterior(uint8_t valor);

void Set_estado_actual(uint8_t valor);

_Noreturn void *joystick_emu(void *vargp);

#endif /* JOYSTICK_H */
