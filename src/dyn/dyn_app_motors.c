/*
 * dyn_app_motors.c
 *
 * High-level functions for dynamixel motors
 */

#include <stdint.h>
#include <stdbool.h>

#include "dyn_app_motors.h"
#include "dyn_instr.h"

/**
 * Turn on or off the LED of a given dynamixel module
 *
 * @param[in] id Id of the dynamixel module
 * @param[in] val Boolean value for the LED objective state
 */
int dyn_led_control(uint8_t id, bool val) {
    return dyn_write_byte(id, DYN_REG__LED, (uint8_t) val);
}

/**
 * Read the current LED status
 *
 * @param[in] id Id of the dynamixel module
 * @param[out] val Current LED status
 */
int dyn_led_read(uint8_t id, uint8_t *val) {
    return dyn_read_byte(id, DYN_REG__LED, val);
}

/**
 * Funció que permet que la roda pugui girar contínuament
 */
int dyn_setMode_EndlessTurn(uint8_t id) {
    // CW_ANGLE_LIMIT_L, CW_ANGLE_LIMIT_H, CCW_ANGLE_LIMIT_L, CCW_ANGLE_LIMIT_H
    uint8_t val[4] = {0, 0, 0, 0};
    return dyn_write(id, DYN_REG__CW_ANGLE_LIMIT_L, val, 4);
}

/**
 * Move the wheel at a certain speed and direction
 */
int dyn_moveWheel(uint8_t id, bool direction, unsigned int speed) {
    if (speed > MAX_SPEED) {
        speed = MAX_SPEED;
    }
    uint8_t mov_speed_L = speed & 0xFF;
    uint8_t mov_speed_H = ((direction << 2) & 0x04) | ((speed >> 8) & 0x03);
    uint8_t val[2] = {mov_speed_L, mov_speed_H};
    return dyn_write(id, DYN_REG__MOV_SPEED_L, val, 2);
}


/*
 * Robot movement functions (It has 2 wheels)
 */

/**
 * Stop the robot
 */
int dyn_stop(void) {
    int l = dyn_moveWheel(ID_MOTOR_LEFT, 0, 0);
    int r = dyn_moveWheel(ID_MOTOR_RIGHT, 0, 0);
    return (l == 0 && r == 0) ? 0 : 1;
}

/**
 * Move the robot forward
 */
int dyn_moveForward(unsigned int speed) {
    int l = dyn_moveWheel(ID_MOTOR_LEFT, DIRECTION_LEFT, speed);
    int r = dyn_moveWheel(ID_MOTOR_RIGHT, DIRECTION_RIGHT, speed);
    return (l == 0 && r == 0) ? 0 : 1;
}

/**
 * Move the robot backward
 */
int dyn_moveBackward(unsigned int speed) {
    int l = dyn_moveWheel(ID_MOTOR_LEFT, DIRECTION_RIGHT, speed);
    int r = dyn_moveWheel(ID_MOTOR_RIGHT, DIRECTION_LEFT, speed);
    return (l == 0 && r == 0) ? 0 : 1;
}

/**
 * Turn to the left
 */
int dyn_turnLeft(unsigned int speed) {
    int l = dyn_moveWheel(ID_MOTOR_LEFT, DIRECTION_RIGHT, 0);
    int r = dyn_moveWheel(ID_MOTOR_RIGHT, DIRECTION_RIGHT, speed);
    return (l == 0 && r == 0) ? 0 : 1;
}

/**
 * Turn to the right
 */
int dyn_turnRight(unsigned int speed) {
    int l = dyn_moveWheel(ID_MOTOR_LEFT, DIRECTION_LEFT, speed);
    int r = dyn_moveWheel(ID_MOTOR_RIGHT, DIRECTION_LEFT, 0);
    return (l == 0 && r == 0) ? 0 : 1;
}

/**
 * Self Turn to the left
 */
int dyn_turnLeft_onSelf(unsigned int speed) {
    int l = dyn_moveWheel(ID_MOTOR_LEFT, DIRECTION_RIGHT, speed);
    int r = dyn_moveWheel(ID_MOTOR_RIGHT, DIRECTION_RIGHT, speed);
    return (l == 0 && r == 0) ? 0 : 1;
}

/**
 * Self Turn to the right
 */
int dyn_turnRight_onSelf(unsigned int speed) {
    int l = dyn_moveWheel(ID_MOTOR_LEFT, DIRECTION_LEFT, speed);
    int r = dyn_moveWheel(ID_MOTOR_RIGHT, DIRECTION_LEFT, speed);
    return (l == 0 && r == 0) ? 0 : 1;
}


/*
 * Test functions
 */

/**
 * Funció que comprova si el EndlessTurn està activat
 */
bool dyn_isMode_EndlessTurn(uint8_t id) {
    uint8_t val[4];
    if (!dyn_read(id, DYN_REG__CW_ANGLE_LIMIT_L, val, 4)) {
        return (val[0] | val[1] | val[2] | val[3]) == 0x00;
    }
    return false;
}

/**
 * Read direction and speed values
 */
int dyn_read_GoalSpeed(uint8_t id, bool *direction, unsigned int *speed) {
    uint8_t val[2]; // mov_speed_L, mov_speed_H
    if (!dyn_read(id, DYN_REG__MOV_SPEED_L, val, 2)) {
        *speed = (val[0] + (val[1] << 8)) & 0x3FF;
        *direction = val[1] >> 2;
        return 0;
    }
    return 1;
}
