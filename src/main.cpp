#include <Arduino.h>
#include "ACS712.h"
#include "PID_v1.h"
#include <Servo.h>

// Librería RC
#include "../include/fs_ia6.h"
#include "../include/quad.h"
#include "../include/sma_filter.h"

// Definir el tamaño máximo de la ventana para el SMA
#define MAX_WINDOW_SIZE 50
#define NUM_SIGNALS 6            // Número de señales paralelas
#define STEERING_THRESHOLD 15000 // Threshold para bloquear el volante

// Estructura para almacenar los datos de cada señal
struct SMAData {
  int values[MAX_WINDOW_SIZE];
  int index;
  int sum;
  int count;
};

// Crear un array de estructuras para las 6 señales
SMAData signals[NUM_SIGNALS];

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

// Crear objeto iBus
IBusBM ibus;

// Variables de canal
int ch1Value, ch2Value, ch3Value, ch4Value, ch5Value, ch6Value;
int sma_ch1, sma_ch2, sma_ch3, sma_ch4, sma_ch5, sma_ch6;
int anglePot, angle;

// Variables para control por UART3
// Los comandos recibidos deberán tener:
// Giro: -100 a 100, Acelerador: 0 a 255 y Frenos: 0 (falso) o 1 (verdadero)
int uartSteering = 0;
int uartThrottle = 0;
bool uartBrake = false;

// Flag para indicar que se ha solicitado telemetría
bool telemetryRequested = false;

// Declaración de funciones
void initSMA();
int SMAFilter(int signalIndex, int newValue, int windowSize);
void checkSteeringStatus();
void SteeringBloqLogic();
void printInfo();
void parseUARTCommand(String cmd);
void sendTelemetry(int steering, int throttle, bool brake);

//
// ******************* setup() *******************
void setup() {
  Serial.begin(115200);
  Serial3.begin(115200); // Inicializa la UART3

  initSMA();

  // Configuración del PID
  PID1.SetMode(AUTOMATIC);
  PID1.SetOutputLimits(-255, 255);
  PID1.SetSampleTime(30);

  initFS_IA6(CH1, CH2, CH3, CH4, CH5, CH6);
  pinMode(AnglePot, INPUT); // Potenciómetro

  initQuad();
  ACS.autoMidPoint(); // Calibrar ACS712
}

//
// ******************* loop() *******************
void loop() {
  // Leer valores de los canales del control remoto
  ch1Value = readChannel(CH1, -100, 100, 0);
  ch2Value = readChannel(CH2, -100, 100, 0);
  ch3Value = readChannel(CH3, -100, 100, -100);
  ch4Value = readChannel(CH4, -100, 100, 0);
  ch5Value = readChannel(CH5, 0, 100, 0);
  ch6Value = readChannel(CH6, 0, 100, 0);

  // Aplicar filtro SMA a cada canal
  sma_ch1 = SMAFilter(0, ch1Value, 3);
  sma_ch2 = SMAFilter(1, ch2Value, 3);
  sma_ch3 = SMAFilter(2, ch3Value, 5);
  sma_ch4 = SMAFilter(3, ch4Value, 5);
  sma_ch5 = SMAFilter(4, ch5Value, 15);
  sma_ch6 = SMAFilter(5, ch6Value, 8);

  // Determinar el modo de control vía UART: activado si ch6 (filtrado) > 66
  bool modoUART = (sma_ch6 > 66);

  // Procesar comandos recibidos por UART3 (para control o telemetría)
  if (Serial3.available() > 0) {
    String cmd = Serial3.readStringUntil('\n');
    cmd.trim();
    if (cmd.equals("GET_TELEMETRY")) {
      telemetryRequested = true;
    } else {
      // Si no es el comando de telemetría, se asume que es de control
      if (modoUART) {
        parseUARTCommand(cmd);
      }
    }
  }

  // Determinar los valores activos de control según el modo
  // controlGiro: de -100 a 100
  // controlAcelerador: de 0 a 255
  // controlFrenos: verdadero (aplicados) o falso
  int controlGiro;
  int controlAcelerador;
  bool controlFrenos;
  
  if (modoUART) {
    // Se utilizan los comandos recibidos por UART3
    controlGiro = uartSteering;
    controlAcelerador = uartThrottle;
    controlFrenos = uartBrake;
  } else {
    // Se utilizan las señales del control remoto
    controlGiro = sma_ch1;
    controlAcelerador = sma_ch3;
    controlFrenos = (sma_ch2 < -10);
  }

  // Solo se ejecuta el control si el vehículo está “encendido”
  if (Quad_ON(sma_ch5)) {
    // Control del acelerador y frenos
    if (modoUART) {
      generatePWM(controlAcelerador);
      if (controlFrenos) {
        Front_Brakes(-60);
        Back_Brakes(-60);
      } else {
        Front_Brakes(0);
        Back_Brakes(0);
      }
      // En modo UART se omiten funciones propias del remoto (change_MaxVel, Quad_Reverse, etc.)
    } else {
      generatePWM(sma_ch3);
      change_MaxVel(sma_ch2, new_MaxVel, maxvel_funcition_aux);
      Quad_Reverse(sma_ch6);
      Front_Brakes(sma_ch2);
      Back_Brakes(sma_ch2);
    }

    // Control PID para el giro
    anglePot = analogRead(AnglePot);
    angle = map(anglePot, 0, 1023, 0, 270);
    if (modoUART) {
      Setpoint1 = map(controlGiro, -100, 100, 30, 170);
    } else {
      Setpoint1 = map(sma_ch1, -100, 100, 30, 170);
    }
    Input1 = map(anglePot, 0, 1023, 0, 270);
    PID1.Compute();

    // Chequear estado y aplicar lógica de giro
    checkSteeringStatus();
    SteeringBloqLogic();
  }

  // Enviar telemetría por UART3 solo si se ha recibido el comando de solicitud
  if (telemetryRequested) {
    sendTelemetry(controlGiro, controlAcelerador, controlFrenos);
    telemetryRequested = false;
  }

  // Imprimir información para debug en Serial
  printInfo();

  delay(30); // Retardo para el siguiente ciclo
}

//
// ******************* Funciones Auxiliares *******************

// Inicializar filtro SMA
void initSMA() {
  for (int i = 0; i < NUM_SIGNALS; i++) {
    signals[i].index = 0;
    signals[i].sum = 0;
    signals[i].count = 0;
    for (int j = 0; j < MAX_WINDOW_SIZE; j++) {
      signals[i].values[j] = 0;
    }
  }
}

// Filtrar señal con SMA
int SMAFilter(int signalIndex, int newValue, int windowSize) {
  if (windowSize > MAX_WINDOW_SIZE) {
    windowSize = MAX_WINDOW_SIZE;
  }
  signals[signalIndex].sum -= signals[signalIndex].values[signals[signalIndex].index];
  signals[signalIndex].values[signals[signalIndex].index] = newValue;
  signals[signalIndex].sum += newValue;
  signals[signalIndex].index = (signals[signalIndex].index + 1) % windowSize;
  if (signals[signalIndex].count < windowSize) {
    signals[signalIndex].count++;
  }
  return signals[signalIndex].sum / signals[signalIndex].count;
}

// Función para parsear comandos de control recibidos por UART3
// Se espera un formato: "giro,acelerador,freno" (por ejemplo: "50,200,1")
void parseUARTCommand(String cmd) {
  int pos1 = cmd.indexOf(',');
  int pos2 = cmd.indexOf(',', pos1 + 1);
  if (pos1 == -1 || pos2 == -1) return;
  
  String strGiro = cmd.substring(0, pos1);
  String strAcel = cmd.substring(pos1 + 1, pos2);
  String strFreno = cmd.substring(pos2 + 1);
  
  uartSteering = strGiro.toInt();
  uartThrottle = strAcel.toInt();
  uartBrake = (strFreno.toInt() != 0);
}

// Función para enviar telemetría por UART3
// Se envía una línea con el siguiente formato: giro,acelerador,freno
void sendTelemetry(int steering, int throttle, bool brake) {
  String telemetry = String(steering) + "_" + String(throttle) + "_" + (brake ? "1" : "0");
  Serial3.println(telemetry);
}

// Chequear el estado del Steering (sin modificaciones)
void checkSteeringStatus() {
  mA = ACS.mA_DC();
  SteeringState = (abs(mA) > STEERING_THRESHOLD) ? BLOQ : READY;
  
  // Giro centrado
  if (abs(Output1) < 10 && Quad_ON(sma_ch5) == true) {
    digitalWrite(enaRPWM, LOW);
  } else if (abs(Output1) > 10 && Quad_ON(sma_ch5)) {
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
  info += " | Angulo: " + String(angle);
  info += "\nDatos Giro: \n";
  info += " | PID Output: " + String(Output1);
  info += " | EstadoGiro: " + String(SteeringState);
  info += "\nReceptor: \n";
  info += " | Ch1: " + String(sma_ch1);
  info += " | Ch2: " + String(sma_ch2);
  info += " | Ch3: " + String(sma_ch3);
  info += " | Ch4: " + String(sma_ch4);
  info += " | Ch5: " + String(sma_ch5);
  info += " | Ch6: " + String(sma_ch6);
  Serial.println(info);
}
