#include <Arduino.h>
#include <PS4Controller.h>
#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>

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
