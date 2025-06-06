#include "../include/quad.h"

// Se crea un objeto Servo para controlar los frenos
Servo ServoFB;
Servo ServoBB;

void initQuad(){
    //IBT_2 Setup
    pinMode(RPWM, OUTPUT);
    pinMode(LPWM, OUTPUT);
    pinMode(enaRPWM, OUTPUT);
    //pinMode(enaLPWM, OUTPUT);

    digitalWrite(enaRPWM,LOW);
    //digitalWrite(enaLPWM,LOW);
    analogWrite(RPWM, 0);
    analogWrite(LPWM, 0);

    // Digital Pins Setup
    pinMode(PIN_MAX_VEL,OUTPUT);
    pinMode(PIN_REVERSE,OUTPUT);
    pinMode(PIN_QUAD_ON, OUTPUT);

    digitalWrite(PIN_MAX_VEL,true);
    digitalWrite(PIN_REVERSE,true);
    digitalWrite(PIN_QUAD_ON,false);

    //PWM Pins Setup
    pinMode(ACCEL_PWM, OUTPUT);

    analogWrite(ACCEL_PWM, 0);

    //Servo setup
    ServoFB.attach(PIN_SERVOFB);  // Se vincula el objeto Servo al pin del Arduino    
    ServoBB.attach(PIN_SERVOBB);

    ServoFB.write(180);
    ServoBB.write(21);
}

// Freno delantero Quad
void Front_Brakes(int ch2value){
  if(ch2value < -50){
    ServoFB.write(160);
  }
  else{
    ServoFB.write(180);
  }

  // int pwmBrake = map (ch2value,-20,-100,180,21);
  // pwmBrake = constrain(pwmBrake, 21, 180);
  // ServoFB.write(pwmBrake);  // Ángulo 0 grados
  // Serial.print("\nFrenoD: ");
  // Serial.println(pwmBrake);
}

// Frenos trasero Quad
void Back_Brakes(int ch2value){
  if(ch2value < -50){
    ServoBB.write(110);
  }
  else{
    ServoBB.write(21);
  }

  // int pwmBrake = map (ch2value,-20,-100,0,170);
  // pwmBrake = constrain(pwmBrake, 0, 170);
  // ServoBB.write(pwmBrake);  // Ángulo 0 grados
  // Serial.print("\nFreno T: ");
  // Serial.println(pwmBrake);
}

// Acelerador Quad
// Función para generar un pulso PWM
void generatePWM(int ch3value) {
  static bool reverseState = false; // Solo visible dentro de la función, pero persiste
  int pwmValue = 40;
  const int pwmMin = 40;
  const int pwmMax = 150;
  const int deadZone = 10;
  bool newReverseState = reverseState;

  if (ch3value > deadZone) {
    newReverseState = true;
    pwmValue = map(ch3value, deadZone, 100, pwmMin, pwmMax);
  }
  else if (ch3value < -deadZone) {
    newReverseState = false;
    pwmValue = map(-ch3value, deadZone, 100, pwmMin, pwmMax);
  }
  else
  {
    newReverseState = true;
  }

  if (newReverseState != reverseState) {
    digitalWrite(PIN_REVERSE, newReverseState ? HIGH : LOW);
    reverseState = newReverseState;
    delay(1000);
  }

  analogWrite(ACCEL_PWM, pwmValue);
}

void change_MaxVel(int ch2value, int new_MaxVel, bool& maxVel_schmitt_trigger){
  if(ch2value > 50){
    if(maxVel_schmitt_trigger == 1){
      maxVel_schmitt_trigger = 0;
      switch (new_MaxVel)
      {
      case 1:
        new_MaxVel = 2;
        break;
      case 2:
        new_MaxVel = 3;
        break;
      case 3:
        new_MaxVel = 4;
        break;
      case 4:
        new_MaxVel = 1;
        break;
      default:
        new_MaxVel = 1;
        break;
      }
      digitalWrite(PIN_MAX_VEL,false);
      delay(50);
      digitalWrite(PIN_MAX_VEL,true);
      Serial.print("\nCambio: ");
      Serial.print(new_MaxVel);
    }
    
  }
  else if(ch2value <= 30){
    maxVel_schmitt_trigger = 1;
    digitalWrite(PIN_MAX_VEL,true);
  }
}

void Quad_Reverse(int ch6Value){
  if(ch6Value > 33 && ch6Value < 66)
  {
    digitalWrite(PIN_REVERSE,false);
  }
  else{
    digitalWrite(PIN_REVERSE, true);
  }
}

bool Quad_ON(int ch5Value){
  if(ch5Value > 10)
  {
    digitalWrite(PIN_QUAD_ON,true);
    digitalWrite(enaRPWM, HIGH);
    //digitalWrite(enaLPWM, HIGH);
    return true;
  }
  else{
    digitalWrite(PIN_QUAD_ON, false);
    digitalWrite(enaRPWM, LOW);
    //digitalWrite(enaLPWM, LOW);
    return false;
  }
}

void turn_right(){
  analogWrite(RPWM, 100);
  analogWrite(LPWM, 0); 
}

void turn_left(){
  analogWrite(RPWM, 0);
  analogWrite(LPWM, 100); 
}

void turn_standby(){
  digitalWrite(RPWM, 0);
  digitalWrite(LPWM, 0);
}