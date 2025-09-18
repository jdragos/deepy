#include <Arduino.h>
#include <PS4Controller.h>
#include <ESP32Servo.h>
// Servo MG995/MG996R
#define SERVO1_PIN 25 // modifică dacă folosești alt pin
#define SERVO2_PIN 26 // modifică dacă folosești alt pin
Servo servo1;
Servo servo2;

#define IN1 16
#define IN2 17
#define IN3 18
#define IN4 19

int motor1Speed = 0;
int motor2Speed = 0;
int motor3Speed = 0;
int motor4Speed = 0;

const int pwmFreq = 1000;
const int pwmResolution = 8;

void setup() {
  Serial.begin(115200);

  // Atașăm servo-urile și le poziționăm la 180 de grade (stânga maxim)
  servo1.attach(SERVO1_PIN);
  servo2.attach(SERVO2_PIN);
  servo1.write(180);
  servo2.write(180);
  delay(500); // așteaptă să ajungă la poziție

  // Inițializăm pinii pentru L298N
  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  pinMode(IN3, OUTPUT);
  pinMode(IN4, OUTPUT);

  // Configurăm canalele PWM
  ledcSetup(0, pwmFreq, pwmResolution);
  ledcSetup(1, pwmFreq, pwmResolution);
  ledcSetup(2, pwmFreq, pwmResolution);
  ledcSetup(3, pwmFreq, pwmResolution);

  ledcAttachPin(IN1, 0);
  ledcAttachPin(IN2, 1);
  ledcAttachPin(IN3, 2);
  ledcAttachPin(IN4, 3);


  // Conectare PS4 controller
  if (!PS4.begin()) {
    Serial.println("Nu s-a putut conecta la PS4 controller!");
    while (1);
  }
  Serial.println("Controller conectat!");
}

void loop() {
  // Citim datele de la stick-ul stanga
  int joystickX = PS4.LStickX(); // -128 ... 127
  int joystickY = PS4.LStickY(); // -128 ... 127

  int speedX = map(joystickX, -128, 127, -255, 255);
  int speedY = map(joystickY, -128, 127, -255, 255);

  if (speedY > 0) {
    motor1Speed = speedY;
    motor2Speed = speedY;
    motor3Speed = speedY;
    motor4Speed = speedY;
  } else if (speedY < 0) {
    motor1Speed = speedY;
    motor2Speed = speedY;
    motor3Speed = speedY;
    motor4Speed = speedY;
  }

  if (speedX > 0) {
    motor1Speed = motor1Speed * 0.8;
    motor4Speed = motor4Speed * 0.8;
  } else if (speedX < 0) {
    motor2Speed = motor2Speed * 0.8;
    motor3Speed = motor3Speed * 0.8;
  }

  motor1Speed = constrain(motor1Speed, -255, 255);
  motor2Speed = constrain(motor2Speed, -255, 255);
  motor3Speed = constrain(motor3Speed, -255, 255);
  motor4Speed = constrain(motor4Speed, -255, 255);

  ledcWrite(0, motor1Speed > 0 ? motor1Speed : 0);
  ledcWrite(1, motor1Speed < 0 ? -motor1Speed : 0);
  ledcWrite(2, motor3Speed > 0 ? motor3Speed : 0);
  ledcWrite(3, motor3Speed < 0 ? -motor3Speed : 0);

  delay(20);
}
