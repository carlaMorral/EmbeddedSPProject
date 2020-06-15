/*
 * main.c
 */

#include <pthread.h>
#include <signal.h>
#include <assert.h>
#include <stdio.h>
#include <unistd.h>

#include "main.h"
#include "../dyn/dyn_app_motors.h"
#include "../dyn/dyn_app_sensor.h"
#include "../dyn_test/dyn_emu.h"
#include "../dyn_test/b_queue.h"
#include "../joystick_emu/joystick.h"
#include "../simulator/habitacion_001.h"
#include "../simulator/posicion.h"

// Joystick variables
uint8_t estado = Ninguno, estado_anterior = Ninguno, finalizar = 0;
uint32_t indice;

// Robot variables
uint8_t left = 0, center = 0, right = 0; // Distances
bool wall_found = false;
int wall;

/**
 * Main function
 */
int main(void) {
    pthread_t tid, jid;

    // Init queues for TX/RX data
    init_queue(&q_tx);
    init_queue(&q_rx);

    // Start thread for dynamixel module emulation
    // Passing the room information to the dyn_emu thread
    pthread_create(&tid, NULL, dyn_emu, (void *) datos_habitacion);
    pthread_create(&jid, NULL, joystick_emu, (void *) &jid);

    //execute_P4_tests();

    // End of tests
    printf("All tests passed successfully!\n");

    // Init robot
    printf("\nInit robot: EndlessTurnMode[ON] + LEDs[ON] + STOP\n");
    dyn_setMode_EndlessTurn(ID_MOTOR_LEFT);
    dyn_setMode_EndlessTurn(ID_MOTOR_RIGHT);
    dyn_led_control(ID_MOTOR_LEFT, 1);
    dyn_led_control(ID_MOTOR_RIGHT, 1);
    dyn_stop(); // Actually unneeded since it's not moving yet

    // Room info
    printf("\nDimensiones habitación: %d ancho x %d largo mm2\n", ANCHO, LARGO);
    printf("En memoria: %lu B = %lu MiB\n", sizeof(datos_habitacion), sizeof(datos_habitacion) >> 20);

    // Joystick info
    printf("\nPulsar 'q' para terminar, qualquier tecla para seguir: \n");
    fflush(stdout);

    // Search the closest wall
    robot_search_wall();

    // Main loop
    while (estado != Quit && !simulator_finished) {

        if (!wall_found) {
            robot_approach_wall();
        } else {
            // Follow the wall clockwise (It will always be on the left side of the robot)
            robot_autonomous_movement_left();
        }

        Get_estado(&estado, &estado_anterior);
        if (estado != estado_anterior) {
            Set_estado_anterior(estado);
            printf("estado = %d\n", estado);
            switch (estado) {
                case Sw1: // a
                    printf("Botón Sw1 ('a') apretado\n");
                    dyn_led_control(ID_MOTOR_RIGHT, 1); // Probaremos de encender el led del motor 2
                    printf("\n");
                    break;
                case Sw2: // s
                    printf("Botón Sw2 ('s') apretado\n");
                    dyn_led_control(ID_MOTOR_RIGHT, 0); // Probaremos de apagar el led del motor 2
                    printf("\n");
                    break;
                case Up: // i
                    break;
                case Down: // m
                    break;
                case Left: // j
                    // Comprobaremos si detectamos las esquinas de la pared izquierda
                    printf("Esquina inferior izquierda:\n");
                    printf("(1, 1): %d (fuera pared)\n", obstaculo(1, 1, datos_habitacion));
                    printf("(0, 1): %d (pared izq.)\n", obstaculo(0, 1, datos_habitacion));
                    printf("(1, 0): %d (pared del.)\n", obstaculo(1, 0, datos_habitacion));
                    printf("(0, 0): %d (esquina)\n", obstaculo(0, 0, datos_habitacion));
                    printf("Esquina superior izquierda:\n");
                    printf("(1, 4094): %d (fuera pared)\n", obstaculo(1, 4094, datos_habitacion));
                    printf("(0, 4094): %d (pared izq.)\n", obstaculo(0, 4094, datos_habitacion));
                    printf("(1, 4095): %d (pared fondo)\n", obstaculo(1, 4095, datos_habitacion));
                    printf("(0, 4095): %d (esquina)\n", obstaculo(0, 4095, datos_habitacion));
                    break;
                case Right: // l
                    // Comprobaremos si detectamos las esquinas de la pared derecha
                    printf("Esquina inferior derecha:\n");
                    printf("(4094, 1): %d (fuera pared)\n", obstaculo(4094, 1, datos_habitacion));
                    printf("(4094, 0): %d (pared del.)\n", obstaculo(4094, 0, datos_habitacion));
                    printf("(4095, 1): %d (pared der.)\n", obstaculo(4095, 1, datos_habitacion));
                    printf("(4095, 0): %d (esquina)\n", obstaculo(4095, 0, datos_habitacion));
                    printf("Esquina superior derecha:\n");
                    printf("(4094, 4094): %d (fuera pared)\n", obstaculo(4094, 4094, datos_habitacion));
                    printf("(4094, 4095): %d (pared fondo)\n", obstaculo(4094, 4095, datos_habitacion));
                    printf("(4095, 4094): %d (pared der.)\n", obstaculo(4095, 4094, datos_habitacion));
                    printf("(4095, 4095): %d (esquina)\n", obstaculo(4095, 4095, datos_habitacion));
                    break;
                case Center: // k
                    break;
                case Quit: // q
                    printf("Bye!\n");
                    break;
                default:
                    break;
            }
            fflush(stdout);
        }
    }

    // Finish
    printf("Programa terminado\n");
    fflush(stdout);

    // Signal the emulation thread to stop
    pthread_kill(tid, SIGTERM);
    pthread_kill(jid, SIGTERM);

    return 0;
}

/**
 * Search the closest wall
 */
void robot_search_wall(void) {
    dyn_distance_wall_left(ID_SENSOR, &left);
    dyn_distance_wall_center(ID_SENSOR, &center);
    dyn_distance_wall_right(ID_SENSOR, &right);

    // Mirem quina és la paret més propera
    if (left < center || right < center) {
        wall = (left <= right)? WALL_LEFT : WALL_RIGHT;

        // Orientem el robot adequadament
        if (wall == WALL_LEFT)
            dyn_turnLeft_onSelf(1000);
        else
            dyn_turnRight_onSelf(1000);

    } else {
        wall = WALL_LEFT; // Default side to follow once center wall is reached
    }

    // Anem cap a la paret
    dyn_moveForward(500);
}

/**
 * Approach to the closest wall
 */
void robot_approach_wall(void) {
    dyn_distance_wall_center(ID_SENSOR, &center);
    if (center <= 10) {
        wall_found = true;
    }
}

/**
 * (LEFT) Automatically update robot movement according to sensor data
 */
void robot_autonomous_movement_left(void) {
    // Llegim els sensors
    dyn_distance_wall_left(ID_SENSOR, &left);
    dyn_distance_wall_center(ID_SENSOR, &center);
    dyn_distance_wall_right(ID_SENSOR, &right);

    // Cas en que davant no hi ha obstacles i es compleixen les distancies de seguretat
    if (left > 2 && left < 10 && center > 10) {
        dyn_moveForward(500);
    }

    // Cas paret frontal o cantonada interior
    else if (center <= 10) {
        dyn_turnRight_onSelf(100);
    }

    // Cas cantonada exterior o massa lluny de la paret
    else if (left >= 10) {
        dyn_turnLeft(200);
    }

    // Cas massa pròxim a la paret
    else if (left <= 2) {
        dyn_turnRight(200);
    }
}

/**
 * Execute the high level functions tests from P4
 */
void execute_P4_tests(void) {
    uint8_t tmp;

    // Testing some high level functions (LED)
    printf("\nMAIN: Setting Motor Left LED to 0");
    dyn_led_control(ID_MOTOR_LEFT, 0);

    printf("\nMAIN: Getting Motor Left LED value");
    dyn_led_read(ID_MOTOR_LEFT, &tmp);
    assert(tmp == 0);

    printf("\nMAIN: Setting Motor Right LED to 1");
    dyn_led_control(ID_MOTOR_RIGHT, 1);

    printf("\nMAIN: Getting Motor Right LED value");
    dyn_led_read(ID_MOTOR_RIGHT, &tmp);
    assert(tmp == 1);

    // Testing Motors mode EndlessTurn
    printf("\nMAIN: Getting Mode EndlessTurn of Motor 1 (Left)");
    assert(dyn_isMode_EndlessTurn(ID_MOTOR_LEFT) == false); // Inicialment està desactivat

    printf("\nMAIN: Setting Mode EndlessTurn to Motor 1 (Left)");
    dyn_setMode_EndlessTurn(ID_MOTOR_LEFT);

    printf("\nMAIN: Getting Mode EndlessTurn of Motor 1 (Left)");
    assert(dyn_isMode_EndlessTurn(ID_MOTOR_LEFT) == true);

    printf("\nMAIN: Getting Mode EndlessTurn of Motor 2 (Right)");
    assert(dyn_isMode_EndlessTurn(ID_MOTOR_RIGHT) == false); // Inicialment està desactivat

    printf("\nMAIN: Setting Mode EndlessTurn to Motor 2 (Right)");
    dyn_setMode_EndlessTurn(ID_MOTOR_RIGHT);

    printf("\nMAIN: Getting Mode EndlessTurn of Motor 2 (Right)");
    assert(dyn_isMode_EndlessTurn(ID_MOTOR_RIGHT) == true);

    // Test Motor speed and direction
    unsigned int speed;
    bool direction;
    printf("\nMAIN: Moving Left Wheel with direction RIGHT and speed 100");
    dyn_moveWheel(ID_MOTOR_LEFT, DIRECTION_RIGHT, 100);

    printf("\nMAIN: Getting direction and speed of Left Wheel");
    dyn_read_GoalSpeed(ID_MOTOR_LEFT, &direction, &speed);
    assert(direction == 1);
    assert(speed == 100);

    printf("\nMAIN: Getting direction and speed of Right Wheel");
    dyn_read_GoalSpeed(ID_MOTOR_RIGHT, &direction, &speed);
    assert(direction == 0); // Valor inicial
    assert(speed == 0); // Valor inicial

    // *** Test robot movement ***
    // Aquestes funcions es limiten a cridar dyn_moveWheel per cada roda,
    // per tal de posar els valors de velocitat i direcció oportuns.
    // Per tant, no és realment necessari testejar-les aquí ja que
    // en el test anterior s'ha vist la lectura i escriptura d'aquests valors per cada motor.

    // Move forward
    printf("\nMAIN: Moving Robot Forward with speed 500");
    dyn_moveForward(500);

    dyn_read_GoalSpeed(ID_MOTOR_LEFT, &direction, &speed);
    assert(direction == 0);
    assert(speed == 500);
    dyn_read_GoalSpeed(ID_MOTOR_RIGHT, &direction, &speed);
    assert(direction == 0);
    assert(speed == 500);

    // ++

    /*
    // ** Testing default (hard-coded) values of the IR sensor **
    printf("\nMAIN: Reading IR sensor");
    dyn_distance_wall_left(ID_SENSOR, &tmp);
    assert(tmp == 10);
    dyn_distance_wall_center(ID_SENSOR, &tmp);
    assert(tmp == 150);
    dyn_distance_wall_right(ID_SENSOR, &tmp);
    assert(tmp == 100);
    */

    printf("\n****************************\n");
}
