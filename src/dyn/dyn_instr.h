/*
 * dyn_instr.h
 */

#ifndef DYN_INSTR_H
#define DYN_INSTR_H

#include <stdint.h>
#include <stdbool.h>

typedef enum _dyn_instr_type {
    DYN_INSTR__READ = 2,
    DYN_INSTR__WRITE = 3,
} DYN_INSTR_t;

typedef enum _dyn_reg_type {
    DYN_REG__CW_ANGLE_LIMIT_L = 0x06,
    DYN_REG__CW_ANGLE_LIMIT_H = 0x07,
    DYN_REG__CCW_ANGLE_LIMIT_L = 0x08,
    DYN_REG__CCW_ANGLE_LIMIT_H = 0x09,
    DYN_REG__LED = 0x19,
    DYN_REG__MOV_SPEED_L = 0x20,
    DYN_REG__MOV_SPEED_H = 0x21,
    DYN_REG__IR_LEFT = 0x1A,
    DYN_REG__IR_CENTER = 0x1B,
    DYN_REG__IR_RIGHT = 0x1C,
    DYN_REG__MAX = 0x32,
} DYN_REG_t;

int dyn_write_byte(uint8_t module_id, DYN_REG_t reg_addr, uint8_t reg_write_val);

int dyn_read_byte(uint8_t module_id, DYN_REG_t reg_addr, uint8_t *reg_read_val);

int dyn_write(uint8_t module_id, DYN_REG_t reg_addr, const uint8_t *val, uint8_t len);

int dyn_read(uint8_t module_id, DYN_REG_t reg_addr, uint8_t *val, uint8_t len);

#endif /* DYN_INSTR_H */
