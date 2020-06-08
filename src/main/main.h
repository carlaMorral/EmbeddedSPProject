/*
 * main.h
 */

#ifndef MAIN_H
#define MAIN_H

#include <stdint.h>
#include <stdbool.h>

/*
 * Set project wide debug level (>)
 *   1.
 *   2. Store movement positions
 *   3. Print distance information
 *   4.
 */
#define DEBUG_LEVEL 3

// Tests function
void execute_P4_tests(void);

// Robot autonomous movement function
void robot_autonomous_movement(uint8_t,uint8_t,uint8_t);

//Find wall
void buscar_paret(uint8_t left, uint8_t center, uint8_t right, int* paret);

void aproparse_paret(uint8_t center, bool* paret_trobada);

#endif /* MAIN_H */
