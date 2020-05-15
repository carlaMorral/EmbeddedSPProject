cmake_minimum_required(VERSION 3.10)
project(ProjectePAE C)

set(CMAKE_C_STANDARD 11)
set(CMAKE_C_FLAGS "-pthread")

include_directories(src)

add_executable(ProjectePAE
        dyn/dyn_app_sensor.c dyn/dyn_app_sensor.h
        dyn/dyn_frames.c dyn/dyn_frames.h
        dyn/dyn_instr.c dyn/dyn_instr.h
        dyn/dyn_app_common.c dyn/dyn_app_common.h
        dyn_test/b_queue.c dyn_test/b_queue.h
        dyn_test/dyn_emu.c dyn_test/dyn_emu.h
        dyn_test/movement_simulator.c dyn_test/movement_simulator.h
        hal_dyn_uart/fake_msp.h
        hal_dyn_uart/hal_dyn_uart_emu.c hal_dyn_uart/hal_dyn_uart_emu.h
        joystick_emu/joystick.c joystick_emu/joystick.h
        Posicion/posicion.c Posicion/posicion.h
        main.c main.h)