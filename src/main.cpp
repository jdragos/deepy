#define SERVO_PIVOT2_R_PIN 32
#define SERVO_PIVOT2_L_PIN 33
Servo ServoPivot2R;
Servo ServoPivot2L;
int servoPivot2Angle = 180;
int servoPivot2Target = 180;
unsigned long lastServo2Update = 0;
const int servo2Step = 2;
const int servo2Delay = 10;
// Double tap logic
unsigned long lastSquarePress = 0;
unsigned long lastCirclePress = 0;
const unsigned long doubleTapWindow = 350; // ms
#include <Arduino.h>
#include <PS4Controller.h>
#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>
#include <ESP32Servo.h>


// Servo-uri pivot
#define SERVO_PIVOT_R_PIN 25
#define SERVO_PIVOT_L_PIN 26
Servo ServoPivot1R;
Servo ServoPivot1L;
int servoPivotAngle = 180; // poziția curentă (0-180)
int servoPivotTarget = 180; // ținta spre care se duce lent
unsigned long lastServoUpdate = 0;
const int servoStep = 2; // pas de mișcare lentă
const int servoDelay = 10; // ms între pași

// PCA9685 pentru 6 servo-uri
Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver();
#define SERVOMIN  150 // minim puls servo (ajustează la nevoie)
#define SERVOMAX  600 // maxim puls servo (ajustează la nevoie)


#define IN1 16
#define IN2 17
#define IN3 18
#define IN4 19
#define IN5 27 // pin nou pentru motor 5
#define IN6 14 // pin nou pentru motor 5


int motor1Speed = 0;
int motor2Speed = 0;
int motor3Speed = 0;
int motor4Speed = 0;
int motor5Speed = 0; // motor suplimentar

const int pwmFreq = 1000;
const int pwmResolution = 8;

void setup() {
  // Atașează servo-urile pivot 2 și le poziționează la 180° (perpendicular cu solul)
  ServoPivot2R.attach(SERVO_PIVOT2_R_PIN);
  ServoPivot2L.attach(SERVO_PIVOT2_L_PIN);
  ServoPivot2L.write(180);
  ServoPivot2R.write(0);
  servoPivot2Angle = 180;
  servoPivot2Target = 180;
  delay(500);
  // Atașează servo-urile pivot și le poziționează la 180° (maxim jos)
  ServoPivot1R.attach(SERVO_PIVOT_R_PIN);
  ServoPivot1L.attach(SERVO_PIVOT_L_PIN);
  ServoPivot1L.write(180);
  ServoPivot1R.write(0);
  servoPivotAngle = 180;
  delay(500);
  Serial.begin(115200);

  // Inițializare PCA9685 și servo-uri la 180 de grade (stânga maxim)
  pwm.begin();
  pwm.setPWMFreq(50);  // frecvență standard pentru servo
  for (uint8_t i = 0; i < 6; i++) {
    pwm.setPWM(i, 0, SERVOMAX); // 180°
  }
  delay(500); // așteaptă să ajungă la poziție


  // Inițializăm pinii pentru L298N
  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  pinMode(IN3, OUTPUT);
  pinMode(IN4, OUTPUT);
  pinMode(IN5, OUTPUT);
  pinMode(IN6, OUTPUT);


  // Configurăm canalele PWM
  ledcSetup(0, pwmFreq, pwmResolution);
  ledcSetup(1, pwmFreq, pwmResolution);
  ledcSetup(2, pwmFreq, pwmResolution);
  ledcSetup(3, pwmFreq, pwmResolution);
  ledcSetup(4, pwmFreq, pwmResolution); // pentru motor 5
  ledcSetup(5, pwmFreq, pwmResolution); // pentru motor 5

  ledcAttachPin(IN1, 0);
  ledcAttachPin(IN2, 1);
  ledcAttachPin(IN3, 2);
  ledcAttachPin(IN4, 3);
  ledcAttachPin(IN5, 4);
  ledcAttachPin(IN6, 5);


  // Conectare PS4 controller
  if (!PS4.begin()) {
    Serial.println("Nu s-a putut conecta la PS4 controller!");
    while (1);
  }
  Serial.println("Controller conectat!");
}

void loop() {
  // Control servo-uri pivot 2 cu butoane și double tap
  static bool squareLast = false, circleLast = false;
  bool squareNow = PS4.Square();
  bool circleNow = PS4.Circle();
  unsigned long now = millis();
  // Double tap pe Square
  if (squareNow && !squareLast) {
    if (now - lastSquarePress < doubleTapWindow) {
      servoPivot2Angle = 90;
      servoPivot2Target = 90;
      ServoPivot2L.write(servoPivot2Angle);
      ServoPivot2R.write(180 - servoPivot2Angle);
    }
    lastSquarePress = now;
  }
  // Double tap pe Circle
  if (circleNow && !circleLast) {
    if (now - lastCirclePress < doubleTapWindow) {
      servoPivot2Angle = 180;
      servoPivot2Target = 180;
  ServoPivot2L.write(servoPivot2Angle);
  ServoPivot2R.write(180 - servoPivot2Angle);
    }
    lastCirclePress = now;
  }
  // Apăsare simplă: mișcare lentă
  if (squareNow && !circleNow) {
    servoPivot2Target = 0;
  } else if (circleNow && !squareNow) {
    servoPivot2Target = 180;
  }
  // Mișcare lentă spre țintă (dacă nu e shortcut)
  if (!(squareNow && (now - lastSquarePress < doubleTapWindow)) && !(circleNow && (now - lastCirclePress < doubleTapWindow))) {
    if (now - lastServo2Update > servo2Delay) {
      if (servoPivot2Angle < servoPivot2Target) {
        servoPivot2Angle = min(servoPivot2Angle + servo2Step, servoPivot2Target);
      } else if (servoPivot2Angle > servoPivot2Target) {
        servoPivot2Angle = max(servoPivot2Angle - servo2Step, servoPivot2Target);
      }
  ServoPivot2L.write(servoPivot2Angle);
  ServoPivot2R.write(180 - servoPivot2Angle);
      lastServo2Update = now;
    }
  }
  squareLast = squareNow;
  circleLast = circleNow;
  // Shortcut rapid: săgeată dreapta = 180°, săgeată stânga = 90°
  if (PS4.Right()) {
    servoPivotAngle = 180;
    servoPivotTarget = 180;
  ServoPivot1L.write(servoPivotAngle);
  ServoPivot1R.write(180 - servoPivotAngle);
  } else if (PS4.Left()) {
    servoPivotAngle = 90;
    servoPivotTarget = 90;
  ServoPivot1L.write(servoPivotAngle);
  ServoPivot1R.write(180 - servoPivotAngle);
  }
  // Mișcare lentă/precisă cu X/Triunghi
  else if (PS4.Triangle()) {
    servoPivotTarget = 180;
  } else if (PS4.Cross()) {
    servoPivotTarget = 0;
  }
  // Mișcare lentă spre țintă (doar dacă nu e shortcut)
  if (!PS4.Right() && !PS4.Left()) {
    unsigned long now = millis();
    if (now - lastServoUpdate > servoDelay) {
      if (servoPivotAngle < servoPivotTarget) {
        servoPivotAngle = min(servoPivotAngle + servoStep, servoPivotTarget);
      } else if (servoPivotAngle > servoPivotTarget) {
        servoPivotAngle = max(servoPivotAngle - servoStep, servoPivotTarget);
      }
  ServoPivot1L.write(servoPivotAngle);
  ServoPivot1R.write(180 - servoPivotAngle);
      lastServoUpdate = now;
    }
  }
  // Citim datele de la stick-uri
  int lX = PS4.LStickX(); // -128 ... 127 (stanga/dreapta)
  int lY = PS4.LStickY(); // -128 ... 127 (fata/spate)
  int rY = PS4.RStickY(); // -128 ... 127 (motor suplimentar)

  // Control sasiu ca un volan (fata/spate + stanga/dreapta proportional)
  int v = map(lY, -128, 127, -255, 255); // viteza fata/spate
  int w = map(lX, -128, 127, -255, 255); // rotatie stanga/dreapta

  // Algoritm tip "diferential drive" pentru volan
  motor1Speed = constrain(v + w, -255, 255); // stanga fata
  motor2Speed = constrain(v - w, -255, 255); // dreapta fata
  motor3Speed = constrain(v + w, -255, 255); // stanga spate
  motor4Speed = constrain(v - w, -255, 255); // dreapta spate

  // Control motor suplimentar cu joystick dreapta (sus = dreapta, jos = stanga)
  motor5Speed = map(rY, -128, 127, -255, 255); // -255 = stanga, 255 = dreapta
  motor5Speed = constrain(motor5Speed, -255, 255);

  // Comenzi PWM pentru motoare sasiu
  ledcWrite(0, motor1Speed > 0 ? motor1Speed : 0);
  ledcWrite(1, motor1Speed < 0 ? -motor1Speed : 0);
  ledcWrite(2, motor3Speed > 0 ? motor3Speed : 0);
  ledcWrite(3, motor3Speed < 0 ? -motor3Speed : 0);

  // Comenzi PWM pentru motor suplimentar
  ledcWrite(4, motor5Speed > 0 ? motor5Speed : 0); // IN5
  ledcWrite(5, motor5Speed < 0 ? -motor5Speed : 0); // IN6

  delay(20);
}
