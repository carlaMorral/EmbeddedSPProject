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
#define DEBUG_LEVEL 4

#define WALL_LEFT 0
#define WALL_CENTER 1
#define WALL_RIGHT 2

// Tests function
void execute_P4_tests(void);

// Robot autonomous movement functions
void robot_autonomous_movement_left(void);
void robot_autonomous_movement_right(void);

// Find wall functions
void robot_search_wall(void);
void robot_approach_wall(void);

#endif /* MAIN_H */
