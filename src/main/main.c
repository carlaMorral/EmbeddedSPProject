/*
 * main.c
 */

#include <pthread.h>
#include <signal.h>
#include <assert.h>
#include <stdio.h>

#include "main.h"
#include "../dyn/dyn_app_motors.h"
#include "../dyn/dyn_app_sensor.h"
#include "../dyn_test/dyn_emu.h"
#include "../dyn_test/b_queue.h"
#include "../joystick_emu/joystick.h"
#include "../simulator/habitacion_001.h"
#include "../simulator/posicion.h"

uint8_t estado = Ninguno, estado_anterior = Ninguno, finalizar = 0;
uint32_t indice;

/**
 * Main function
 */
int main(void) {
    pthread_t tid, jid;
    uint8_t left = 0, center = 0, right = 0;

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
    dyn_stop();

    // Room info
    printf("\nDimensiones habitación: %d ancho x %d largo mm2\n", ANCHO, LARGO);
    printf("En memoria: %lu B = %lu MiB\n", sizeof(datos_habitacion), sizeof(datos_habitacion) >> 20);

    // Joystick info
    printf("\nPulsar 'q' para terminar, qualquier tecla para seguir: \n");
    fflush(stdout);

    // Main loop
    while (estado != Quit) {
        if (simulator_finished) {
            break;
        } else {
            robot_autonomous_movement(left, center, right);
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
 * Automatically update robot movement according to sensor data
 */
void robot_autonomous_movement(uint8_t left, uint8_t center, uint8_t right) {
    // TODO: algorithm implementation
    // read sensor data, get obstacle, max/min distance
    // if/elseif/switch conditions to call the appropriate movement function
    dyn_distance_wall_left(3,&left);
    dyn_distance_wall_center(3,&center);
    dyn_distance_wall_right(3,&right);


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
    // Per tant, no és realment necessari testejar-les totes ja que
    // en el test anterior s'ha vist la lectura i escriptura d'aquests valors per cada motor.

    // Tot i així, veiem algunes ...

    // Move forward
    printf("\nMAIN: Moving Robot Forward with speed 500");
    dyn_moveForward(500);

    dyn_read_GoalSpeed(ID_MOTOR_LEFT, &direction, &speed);
    assert(direction == 0);
    assert(speed == 500);
    dyn_read_GoalSpeed(ID_MOTOR_RIGHT, &direction, &speed);
    assert(direction == 1);
    assert(speed == 500);

    // Move backward
    printf("\nMAIN: Moving Robot Backward with speed 500");
    dyn_moveBackward(500);

    dyn_read_GoalSpeed(ID_MOTOR_LEFT, &direction, &speed);
    assert(direction == 1);
    assert(speed == 500);
    dyn_read_GoalSpeed(ID_MOTOR_RIGHT, &direction, &speed);
    assert(direction == 0);
    assert(speed == 500);

    // Self turn to the right
    printf("\nMAIN: Self turning Robot to the Right with speed 800");
    dyn_turnRight_onSelf(800);

    dyn_read_GoalSpeed(ID_MOTOR_LEFT, &direction, &speed);
    assert(direction == 0);
    assert(speed == 800);
    dyn_read_GoalSpeed(ID_MOTOR_RIGHT, &direction, &speed);
    assert(direction == 0);
    assert(speed == 800);

    // Self turn to the left
    printf("\nMAIN: Self turning Robot to the Left with speed 800");
    dyn_turnLeft_onSelf(800);

    dyn_read_GoalSpeed(ID_MOTOR_LEFT, &direction, &speed);
    assert(direction == 1);
    assert(speed == 800);
    dyn_read_GoalSpeed(ID_MOTOR_RIGHT, &direction, &speed);
    assert(direction == 1);
    assert(speed == 800);

    // Turn left
    printf("\nMAIN: Moving Robot to the Left with speed 200");
    dyn_turnLeft(200);

    dyn_read_GoalSpeed(ID_MOTOR_LEFT, &direction, &speed);
    assert(direction == 1);
    assert(speed == 0);
    dyn_read_GoalSpeed(ID_MOTOR_RIGHT, &direction, &speed);
    assert(direction == 1);
    assert(speed == 200);

    // ++ turn right, stop

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
