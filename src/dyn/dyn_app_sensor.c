/*
 * dyn_app_sensor.c
 *
 * High-level functions for dynamixel sensor
 */

#include <stdint.h>

#include "dyn_app_sensor.h"
#include "dyn_instr.h"

/**
 * Returns the center distance using the infrared sensor data
 *
 * @param[in] id Id of the dynamixel module
 * @param[in] val current IR center status
 */
int dyn_distance_wall_center(uint8_t id, uint8_t *val) {
    return dyn_read_byte(id, DYN_REG__IR_CENTER, val);
}

/**
 * Returns the left distance using the infrared sensor data
 *
 * @param[in] id Id of the dynamixel module
 * @param[in] val current IR left status
 */
int dyn_distance_wall_left(uint8_t id, uint8_t *val) {
    return dyn_read_byte(id, DYN_REG__IR_LEFT, val);
}

/**
 * Returns the right distance using the infrared sensor data
 *
 * @param[in] id Id of the dynamixel module
 * @param[in] val current IR right status
 */
int dyn_distance_wall_right(uint8_t id, uint8_t *val) {
    return dyn_read_byte(id, DYN_REG__IR_RIGHT, val);
}
