#include <Arduino.h>
#include <LiquidCrystal.h>
#include <math.h>

LiquidCrystal lcd(8, 9, 4, 5, 6, 7); // RS, E, D4, D5, D6, D7

const int ENA = 11;
const int IN1 = 13;
const int IN2 = 12;

const int ENB = 3;
const int IN3 = A1;
const int IN4 = A2;

const int ENCODER = 2;

typedef struct {
  byte speed;
  struct {
    boolean forward;
    boolean backward;
  } direction;
} motor; 

motor motorA, motorB;

unsigned long timeNow;
unsigned long timeElapsed;
unsigned long timeStart;
unsigned int timeInterval = 10*1000; // 10 seconds

unsigned long prevSample;
unsigned int samplePeriod = 100;
int sampleCount = 0;
int pulseCount = 0;

boolean start;

unsigned long T1, T2, T;
bool MeasDone;
int totalRPM;
int distance;

void INT0_ISR(void){
  if(MeasDone)
  {
    T2 = micros();
    T = (T2 - T1 > 500) ? T2 - T1 : T;
    if (T > 500) pulseCount++;
    MeasDone = 0;
  }
  else
  {
    T1 = micros();
    MeasDone = 1;
  }
}

void setSpeed(byte speedA, byte speedB){
  motorA.speed = speedA;
  motorB.speed = speedB;
  analogWrite(ENA, motorA.speed);
  analogWrite(ENB, motorB.speed);
}

void go(String direction){
  boolean value1 = LOW;
  boolean value2 = LOW;
  if (direction == "forward"){
    value1 = LOW;
    value2 = HIGH;
  } else if (direction == "backward"){
    value1 = HIGH;
    value2 = LOW;
  } else if (direction == "stop"){
    setSpeed(0, 0);
  }
  motorA.direction.backward = motorB.direction.backward = value1;
  motorB.direction.forward = motorA.direction.forward = value2;
  digitalWrite(IN1, motorA.direction.forward);
  digitalWrite(IN2, motorA.direction.backward);
  digitalWrite(IN3, motorB.direction.forward);
  digitalWrite(IN4, motorB.direction.backward);
}

void setup() {
  lcd.begin(16, 2);
  Serial.begin(9600);

  pinMode(ENA, OUTPUT);
  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  pinMode(ENB, OUTPUT);
  pinMode(IN3, OUTPUT);
  pinMode(IN4, OUTPUT);

  pinMode(ENCODER, INPUT);
  attachInterrupt(digitalPinToInterrupt(ENCODER), INT0_ISR, RISING);

  lcd.print("Select to start!");
}

void loop() {
  if (analogRead(A0) > 600 && analogRead(A0) < 800){
    timeStart = millis();
    lcd.clear();
    lcd.setCursor(0, 0);
    lcd.print("Moving forward!");

    while(start){
      timeNow = millis();
      timeElapsed = timeNow - timeStart;
      if (timeElapsed <= timeInterval){
        setSpeed(107, 150);
        go("forward");
      } else {
        timeStart = millis();
        go("stop");
        start = false;

      }
      lcd.setCursor(0, 1);
      lcd.print("Time: " + String(timeElapsed/1000.0));
    }
    start = true;
    lcd.setCursor(0, 0);
    lcd.print("Select to start!");
    lcd.setCursor(0, 1);
    lcd.print("Distance: ");
    //distance = totalRPM / sampleCount * M_PI * 3.2 * 2 / 6;
    Serial.print(totalRPM);
    Serial.print(sampleCount);
    lcd.print(distance);
    totalRPM = 0;
    sampleCount = 0;
  }
}