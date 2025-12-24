#include <Arduino.h>
#include <LiquidCrystal.h>
#include <math.h>

void go(int, int);

LiquidCrystal lcd(8, 9, 4, 5, 6, 7); // RS, E, D4, D5, D6, D7

const int ENA = 11;
const int IN1 = 13;
const int IN2 = 12;

const int ENB = 3;
const int IN3 = A1;
const int IN4 = A2;

const int IR_L = A4;
const int IR_M = A5;
const int IR_R = A3;

float Kp = 35;  
float Ki = 0.05;
float Kd = 5;

unsigned long millisNow, millisElapsed, millisStart;

boolean start;

int distance;

int defaultSpeedL = 255;
int defaultSpeedR = 255;

int currentSpeedL;
int currentSpeedR;

enum dir {STRAIGHT, LEFT, RIGHT};

unsigned long stopMillis;
boolean stop;

void setup() {
  lcd.begin(16, 2);
  Serial.begin(9600);

  pinMode(ENA, OUTPUT);
  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  pinMode(ENB, OUTPUT);
  pinMode(IN3, OUTPUT);
  pinMode(IN4, OUTPUT);

  pinMode(IR_R, INPUT);
  pinMode(IR_M, INPUT);
  pinMode(IR_L, INPUT);

  lcd.print("Select to start!");
}

void loop() {
  enum dir lastDir = STRAIGHT;
  float error = 0;
  int lastError = 0;
  int P = 0;
  int I = 0;
  int D = 0;
  if (analogRead(A0) > 600 && analogRead(A0) < 800){
    millisStart = millis();
    lcd.clear();
    lcd.setCursor(0, 0);
    lcd.print("Moving forward!");
    start = true;

    while(start){
      millisNow = millis();
      millisElapsed = millisNow - millisStart;
      
      boolean L = digitalRead(IR_L);
      boolean M = !digitalRead(IR_M);  
      boolean R = digitalRead(IR_R);

      byte pattern = (L << 2) | (M << 1) | R;

      switch(pattern){
        // drift left
        case 0b001:
          error = -0.5;
          lastDir = LEFT;
          break;
        // turn left
        case 0b011:
          error = -1;
          lastDir = LEFT;
          break;
        // drift right
        case 0b100:
          error = 0.5;
          lastDir = RIGHT;
          break;
        // go straight
        case 0b101:
          error = 0;
          lastDir = STRAIGHT;       
        // turn right
        case 0b110:
          error = 1;
          lastDir = RIGHT;
          break;
        // last dir
        case 0b111:
          switch(lastDir){
            case STRAIGHT:
              error = 0;
              break;
            case LEFT:
              error = -12;
              break;
            case RIGHT:
              error = 12;
              break;
          }
          break;
      }

      P = error;
      I = I + error;
      D = error - lastError;
      lastError = error;

      float motorSpeed = (Kp * P) + (Ki * I) + (Kd * D);
      int motorSpeedL = defaultSpeedL + motorSpeed;
      int motorSpeedR = defaultSpeedR - motorSpeed;
    
      if ((~pattern & 0b101) == 5) {
        if (stop && millisNow - stopMillis > 40){
          motorSpeedL = 0;
          motorSpeedR = 0;
          start = false;
        } else if (!stop) {
          stop = true;
          stopMillis = millisNow;
        }
      } else stop = false;

      if(pattern == 0b111 && lastDir == STRAIGHT) go(255, 255); else go(motorSpeedL, motorSpeedR);

      lcd.setCursor(0, 1);  
      lcd.print("Time: " + String(millisElapsed/1000.0));
    }
    start = true;
    lcd.clear();
    lcd.setCursor(0, 0);
    lcd.print("Select to start!");
  }
}

void go(int speedL, int speedR){ 
  digitalWrite(IN1, (speedR <= 0)? LOW : HIGH);
  digitalWrite(IN2, (speedR >= 0)? LOW : HIGH);

  digitalWrite(IN3, (speedL <= 0)? LOW : HIGH);
  digitalWrite(IN4, (speedL >= 0)? LOW : HIGH);

  // Apply speed, constraining to the 0-255 PWM range
  analogWrite(ENB, constrain(abs(speedL), 0, 255));
  analogWrite(ENA, constrain(abs(speedR), 0, 255));
}