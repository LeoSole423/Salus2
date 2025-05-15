#ifndef ARDUINO_H_INCLUDE
#define ARDUINO_H_INCLUDE

#include <Arduino.h>

#endif  // ARDUINO_H_INCLUDE

// Constantes del filtro
const int ORDER = 10; // Orden del filtro (mayor es más rápido pero menos eficiente)
const float ALPHA = 0.1; // Parámetro de ajuste del filtro (0 a 1)


void initFilter();

int getFilteredValue(int value);
