/*
 * posicion.c
 */

#include <stdio.h>
#include <stdbool.h>
#include <math.h>

#include "posicion.h"
#include "../dyn_test/movement_simulator.h"
#include "../main/main.h"

#define TIPO_DATOS_HAB uint32_t

#define WORLD__N_BYTES sizeof(TIPO_DATOS_HAB)
#define WORLD__N_BITS (WORLD__N_BYTES << 3)
#define WORLD__MAX_2POW ((int) log2(WORLD__N_BYTES * 8))
#define Y_LEN LARGO
#define X_LEN (ANCHO >> WORLD__MAX_2POW)
#define X__MAX_2POW ((int) log2(X_LEN))

#define MAX_SENSOR_RANGE 255

/**
 * Función para transformar las coordenadas espaciales (x,y) de un punto en coordenadas
 * del bit correspondiente en el bloque de memoria que representa la habitación.
 *
 * Entrada:
 * coordenadas (x,y) = (columna, fila) del punto dentro de la habitación.
 *
 * Resultado:
 * índice del bloque de memoria en el que se encuentra el punto, y número
 * del bit dentro de este bloque que corresponde al punto.
 */
void coordenadas(uint16_t x, uint16_t y, uint32_t *p_offset, uint8_t *p_bit) {
    uint32_t p = 0;
    uint8_t b = 0;
    // En que fila me encuentro?
    // Cada incremento de "y", corresponde a un salto de 1 fila de 128 bloques
    p = y << X__MAX_2POW; // INT(y*128)
    // Dentro de la fila, nos hemos movido hasta el bloque INT(x/32)
    p += x >> WORLD__MAX_2POW;
    // Lo que queda de esta división nos da el bit dentro del bloque
    // 31 - x - INT(x/32)*32
    b = (WORLD__N_BITS - 1) - (x - ((x >> WORLD__MAX_2POW) << WORLD__MAX_2POW));

    //printf("Conversion: %u, %d", p, b);
    *p_offset = p;
    *p_bit = b;
}

/**
 * Función para determinar si un punto concreto (x,y) pertenece, o no, a un obstáculo o a una pared
 */
uint8_t obstaculo(uint16_t x, uint16_t y, const uint32_t *mundo) {
    uint32_t offset = 0;
    uint8_t bit = 0;
    coordenadas(x, y, &offset, &bit);
    if ((mundo[offset] & (1 << bit)))
        return true;
    return false;
}

void sensor_distance(uint16_t x0, uint16_t y0, float theta, const uint32_t *world, uint8_t *sensor_data, uint8_t dbg_msg) {
    float dx, dy; // Incrementos del vector de desplazamiento en la dirección de un sensor
    float modulo = 0.0; // Módulo del vector de desplazamiento en la dirección de un sensor
    float x = 0.0, y = 0.0; // Componentes del vector de desplazamiento en la dirección de un sensor
    uint16_t indice = 0; // Distancia al obstáculo
    uint8_t u8_mod;

    dx = cos(theta);
    dy = sin(theta);

#if DEBUG_LEVEL > 3
    printf("\n");
#endif

    while ((modulo < 255) && !(obstaculo((uint16_t) (x0 + x), (uint16_t) (y0 + y), world))) {
        x += dx;
        y += dy;
        modulo = sqrt(x * x + y * y);
        indice++;
    }

    if (modulo > 255) {
        u8_mod = 255;
    } else {
        u8_mod = (uint8_t) round(modulo);
    }
    *sensor_data = u8_mod;

#if DEBUG_LEVEL > 3
    if (dbg_msg == 0) {
        printf("Robot en (%d, %d, %.3f rad), obstaculo a la izquierda en %dmm\n", x0, y0, theta, indice);
    } else if (dbg_msg == 1) {
        printf("Robot en (%d, %d, %.3f rad), obstaculo delante en %dmm\n", x0, y0, theta, indice);
    } else if (dbg_msg == 2) {
        printf("Robot en (%d, %d, %.3f rad), obstaculo a la derecha en %dmm\n", x0, y0, theta, indice);
    }
#endif
}

void distance(_robot_pos_t *robot_pos, uint8_t *izq, uint8_t *centro, uint8_t *der) {
    uint16_t x0; // Posición del bloque de sensores
    uint16_t y0; // Posición del bloque de sensores
    float theta; // Orientación del sensor central
    float theta_l, theta_r; // Orientación de los sensores izquierdo y derecho

    x0 = robot_pos->x; // Posición del bloque de sensores = Posición del robot
    y0 = robot_pos->y; // Posición del bloque de sensores = Posición del robot
    theta = robot_pos->theta; // Orientación del sensor central, paralela a la del robot
    theta_l = theta + M_PI / 2;
    theta_r = theta - M_PI / 2;

    // Sensor central
    sensor_distance(x0, y0, theta, robot_pos->world, centro, 1);

    // Sensor izquierda
    sensor_distance(x0, y0, theta_l, robot_pos->world, izq, 0);

    // Sensor derecha
    sensor_distance(x0, y0, theta_r, robot_pos->world, der, 2);
}
