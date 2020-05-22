/*
 * posicion.h
 */

#ifndef POSICION_H
#define POSICION_H

#include <stdint.h>
#include <stdbool.h>
#include "../dyn_test/movement_simulator.h"

#define LARGO 4096
#define ANCHO 4096

void coordenadas(uint16_t x, uint16_t y, uint32_t *p_offset, uint8_t *p_bit);

uint8_t obstaculo(uint16_t x, uint16_t y, const uint32_t *mundo);

void
sensor_distance(uint16_t x0, uint16_t y0, float theta, const uint32_t *world, uint8_t *sensor_data, uint8_t dbg_msg);

void distance(_robot_pos_t *robot_pos, uint8_t *izq, uint8_t *centro, uint8_t *der);

#endif /* POSICION_H */
