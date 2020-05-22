/*
 * dyn_app_sensor.h
 */

#ifndef DYN_APP_SENSOR_H
#define DYN_APP_SENSOR_H

#include <stdint.h>

#define ID_SENSOR 3

int dyn_distance_wall_center(uint8_t id, uint8_t *val);

int dyn_distance_wall_left(uint8_t id, uint8_t *val);

int dyn_distance_wall_right(uint8_t id, uint8_t *val);

#endif /* DYN_APP_SENSOR_H */
