#ifndef ARDUINO_H_INCLUDE
#define ARDUINO_H_INCLUDE

#include <Arduino.h>
#include <Servo.h>

#endif  // ARDUINO_H_INCLUDE


//Digital Pins
#define PIN_MAX_VEL 45 //22
#define PIN_REVERSE 47 //23
#define PIN_QUAD_ON 49 //24

//PWM Pins
#define ACCEL_PWM 8

//Servo
#define PIN_SERVOFB 12 //9
#define PIN_SERVOBB 13 //10

//IBT 2
//#define enaLPWM 10
#define enaRPWM 9 //11
#define RPWM 11 //12
#define LPWM 10 //13


// Funciones

// Inicializa los pines de control del QUAD
void initQuad();

// Freno delantero Quad
void Front_Brakes(int ch2value);

// Frenos trasero Quad
void Back_Brakes(int ch2value);

// Acelerador Quad
void generatePWM(int ch3value);

void change_MaxVel(int ch2value, int new_MaxVel, bool& maxVel_schmitt_trigger);

void Quad_Reverse(int ch6Value);

bool Quad_ON(int ch5Value);

void turn_right();

void turn_left();

void turn_standby();