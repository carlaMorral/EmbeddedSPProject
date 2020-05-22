cmake_minimum_required(VERSION 3.10)
project(ProjectePAE C)

set(CMAKE_C_STANDARD 11)
set(CMAKE_C_FLAGS "-pthread -lm")

include_directories(src)

add_executable(ProjectePAE
        src/dyn/dyn_app_motors.c src/dyn/dyn_app_motors.h
        src/dyn/dyn_app_sensor.c src/dyn/dyn_app_sensor.h
        src/dyn/dyn_frames.c src/dyn/dyn_frames.h
        src/dyn/dyn_instr.c src/dyn/dyn_instr.h
        src/dyn_test/b_queue.c src/dyn_test/b_queue.h
        src/dyn_test/dyn_emu.c src/dyn_test/dyn_emu.h
        src/dyn_test/movement_simulator.c src/dyn_test/movement_simulator.h
        src/hal_dyn_uart/fake_msp.h
        src/hal_dyn_uart/hal_dyn_uart_emu.c src/hal_dyn_uart/hal_dyn_uart_emu.h
        src/joystick_emu/joystick.c src/joystick_emu/joystick.h
        src/simulator/habitacion_001.h
        src/simulator/posicion.c src/simulator/posicion.h
        src/main/main.c src/main/main.h
        )

target_link_libraries(ProjectePAE m)
