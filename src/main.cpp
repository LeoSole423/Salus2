#include <Arduino.h>
#include "ACS712.h"
#include "PID_v1.h"
#include <Servo.h>

// Librería RC
#include "../include/fs_ia6.h"
#include "../include/quad.h"
#include "../include/sma_filter.h"


#define STEERING_THRESHOLD 15000 // Threshold para bloquear el volante

enum Estado {
  READY,
  BLOQ
};
Estado SteeringState = READY;

// Definir constantes de PID
const double Pk1 = 4.5;
const double Ik1 = 0.0;
const double Dk1 = 0.0;
double Setpoint1, Input1, Output1; // Variables para el PID

PID PID1(&Input1, &Output1, &Setpoint1, Pk1, Ik1, Dk1, DIRECT); // Configuración PID

// ACS712 (sensor de corriente)
ACS712 ACS(A0, 5.0, 1023, 66);
int mA = 0; // Almacena el valor de corriente

// Objeto servo para controlar frenos
Servo myServo;

// Definir pines
#define CH1 2
#define CH2 3
#define CH3 4
#define CH4 5
#define CH5 6
#define CH6 7
#define Current_Sensor A0
#define AnglePot A1  // Pin para el potenciómetro del ángulo

// Variables del Quad
int new_MaxVel = 1;
bool maxvel_funcition_aux = true;

// Variables de canal
int ch1Value, ch2Value, ch3Value, ch4Value, ch5Value, ch6Value;
int sma_ch1, sma_ch2, sma_ch3, sma_ch4, sma_ch5, sma_ch6;
int anglePot, angle;

// Declaración de funciones
void checkSteeringStatus();
void SteeringBloqLogic();
void printInfo();

//
// ******************* setup() *******************
void setup() {
  Serial.begin(115200);
  Serial3.begin(115200); // Inicializa la UART3

  
  // Configuración del PID
  PID1.SetMode(AUTOMATIC);
  PID1.SetOutputLimits(-255, 255);
  PID1.SetSampleTime(30);

  pinMode(AnglePot, INPUT); // Potenciómetro

  initQuad();
  ACS.autoMidPoint(); // Calibrar ACS712
}

//
// ******************* loop() *******************
void loop() {

  ch1Value = readChannel(CH1, -85, 85, 0);

  ch3Value = readChannel(CH3, -100, 100, 0);

  generatePWM(ch3Value);
  //change_MaxVel(sma_ch2, new_MaxVel, maxvel_funcition_aux);
  //Quad_Reverse(sma_ch6);
  //Front_Brakes(sma_ch2);
  //Back_Brakes(sma_ch2);

  // Control PID para el giro
  anglePot = analogRead(AnglePot);
  Setpoint1 = map(ch1Value, -100, 100, 30, 170);
  Input1 = map(anglePot, 0, 1023, 0, 270);
  PID1.Compute();

  // Chequear estado y aplicar lógica de giro
  checkSteeringStatus();
  SteeringBloqLogic();
  

  // Imprimir información para debug en Serial
  printInfo();

  delay(30); // Retardo para el siguiente ciclo
}

//
// ******************* Funciones Auxiliares *******************

// Chequear el estado del Steering (sin modificaciones)
void checkSteeringStatus() {
  mA = ACS.mA_DC();
  SteeringState = (abs(mA) > STEERING_THRESHOLD) ? BLOQ : READY;
  
  // Giro centrado
  if (abs(Output1) < 10) {
    digitalWrite(enaRPWM, LOW);
  } else if (abs(Output1) > 10) {
    digitalWrite(enaRPWM, HIGH);
  }
}

// Lógica de bloqueo de Steering (sin modificaciones)
void SteeringBloqLogic() {
  if (SteeringState == READY) {
    if (Output1 > 0) { // Girar derecha
      analogWrite(RPWM, Output1);
      analogWrite(LPWM, 0);
    } else if (Output1 < 0) { // Girar izquierda
      analogWrite(RPWM, 0);
      analogWrite(LPWM, abs(Output1));
    }
  } else {
    Serial.println("\nBLOQUEADO\n");
    analogWrite(RPWM, 0);
    analogWrite(LPWM, 0);
  }
}

// Imprimir información agrupada (para debug)
void printInfo() {
  String info = "";
  info += " | Current: " + String(mA);
  info += " | Angulo: " + String(Input1);
  info += "\nDatos Giro: \n";
  info += " | PID Output: " + String(Output1);
  info += " | EstadoGiro: " + String(SteeringState);
  info += "\nReceptor: \n";
  info += " | Ch1: " + String(ch1Value);
  info += " | Ch3: " + String(ch3Value);
  Serial.println(info);
}
