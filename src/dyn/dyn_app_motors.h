/*
 * dyn_app_motors.h
 */

#ifndef DYN_APP_MOTORS_H
#define DYN_APP_MOTORS_H

#include <stdint.h>
#include <stdbool.h>

#define ID_MOTOR_LEFT 1
#define ID_MOTOR_RIGHT 2

#define MAX_SPEED 0x3FF // 1023
#define DIRECTION_LEFT 0 // CCW
#define DIRECTION_RIGHT 1 // CW

int dyn_led_control(uint8_t id, bool val);

int dyn_led_read(uint8_t id, uint8_t *val);

int dyn_setMode_EndlessTurn(uint8_t id);

bool dyn_isMode_EndlessTurn(uint8_t id);

int dyn_moveWheel(uint8_t id, bool direction, unsigned int speed);

int dyn_read_GoalSpeed(uint8_t id, bool *direction, unsigned int *speed);

int dyn_stop(void);

int dyn_moveForward(unsigned int speed);

int dyn_moveBackward(unsigned int speed);

int dyn_turnLeft(unsigned int speed);

int dyn_turnRight(unsigned int speed);

int dyn_turnLeft_onSelf(unsigned int speed);

int dyn_turnRight_onSelf(unsigned int speed);

#endif /* DYN_APP_MOTORS_H */
