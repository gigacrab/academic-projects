#include <Arduino.h>
#include <LiquidCrystal.h>

LiquidCrystal lcd(8, 9, 4, 5, 6, 7); // RS, E, D4, D5, D6, D7

const int ENA = 11;
const int IN1 = 13;
const int IN2 = 12;

const int ENB = 3;
const int IN3 = 2;
const int IN4 = 1;

typedef struct {
  byte speed;
  struct {
    byte forward;
    byte backward;
  } direction;
} motor; 

motor motorA, motorB;

int now;
int interval = 10*1000; // 10 seconds

void setSpeed(byte speed){
  motorA.speed = speed;
  motorB.speed = speed;
  analogWrite(ENA, motorA.speed);
  analogWrite(ENB, motorB.speed);
}

void goBackward(){
  motorA.direction.backward = HIGH;
  motorA.direction.forward = LOW;
  motorB.direction.backward = HIGH;
  motorB.direction.forward = LOW;
  digitalWrite(IN1, motorA.direction.forward);
  digitalWrite(IN2, motorA.direction.backward);
  digitalWrite(IN3, motorB.direction.forward);
  digitalWrite(IN4, motorB.direction.backward);
}

void goForward(){
  motorA.direction.backward = LOW;
  motorA.direction.forward = HIGH;
  motorB.direction.backward = LOW;
  motorB.direction.forward = HIGH;
  digitalWrite(IN1, motorA.direction.forward);
  digitalWrite(IN2, motorA.direction.backward);
  digitalWrite(IN3, motorB.direction.forward);
  digitalWrite(IN4, motorB.direction.backward);
}

void stop(){
  setSpeed(0);
  motorA.direction.backward = LOW;
  motorA.direction.forward = LOW;
  motorB.direction.backward = LOW;
  motorB.direction.forward = LOW;
}

void setup() {
  lcd.begin(16, 2);

  pinMode(ENA, OUTPUT);
  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  pinMode(ENB, OUTPUT);
  pinMode(IN3, OUTPUT);
  pinMode(IN4, OUTPUT);
}

void loop() {
  now = millis();
  if (now <= interval){
    setSpeed(100);
    goForward();
  } else {
    stop();
  }
  lcd.setCursor(0, 0);
  lcd.print(String(now/1000.0));
}