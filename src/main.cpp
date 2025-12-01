// 3 ir sensor
// pid

#include <Arduino.h>
#include <LiquidCrystal.h>
#include <math.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>

void go(int, int);
float kalmanFilter(float newAngle, float newRate, float dt, float &angle, float &bias);

LiquidCrystal lcd(8, 9, 4, 5, 6, 7); // RS, E, D4, D5, D6, D7
Adafruit_MPU6050 mpu;

float x_angle, y_angle;
float x_bias, y_bias;
float P[2][2] = {{1, 0}, {0, 1}};

float q_angle = 0.01;
float q_bias = 0.01;
float r_measure = 0.01;

const int ENA = 11;
const int IN1 = 13;
const int IN2 = 12;

const int ENB = 3;
const int IN3 = A1;
const int IN4 = A2;

const int IR_L = A5;
//const int IR_M = 1;
const int IR_R = A4;

unsigned long millisNow, millisElapsed, millisStart;

boolean start;

int distance;

int defaultSpeedL = 255; // 107
int defaultSpeedR = 255; // 150

int currentSpeedL;
int currentSpeedR;

enum dir {STRAIGHT, LEFT, RIGHT};

unsigned long stopMillis;
boolean stop;

void setup() {
  // hardware initializations
  lcd.begin(16, 2);
  Serial.begin(9600);

  if (!mpu.begin()){
    Serial.println("MPU6050 not found!");
    while (1) delay(10);
  }

  mpu.setAccelerometerRange(MPU6050_RANGE_8_G);
  mpu.setGyroRange(MPU6050_RANGE_500_DEG);
  mpu.setFilterBandwidth(MPU6050_BAND_10_HZ);

  // allowing settings to take effect
  delay(100);

  pinMode(ENA, OUTPUT);
  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  pinMode(ENB, OUTPUT);
  pinMode(IN3, OUTPUT);
  pinMode(IN4, OUTPUT);

  pinMode(IR_R, INPUT);
  //pinMode(IR_M, INPUT);
  pinMode(IR_L, INPUT);

  lcd.print("Select to start!");
}

void loop() {
  enum dir lastDir = STRAIGHT;
  if (analogRead(A0) > 600 && analogRead(A0) < 800){
    lcd.clear();
    lcd.setCursor(0, 0);
    lcd.print("Moving forward!");

    while(start){
      sensors_event_t a, g, temp;
      mpu.getEvent(&a, &g, &temp);

      static unsigned long prevTime = millis();
      float dt = (millis() - prevTime) / 1000.0;
      prevTime = millis();
      float filteredX = kalmanFilter(a.acceleration.x, g.gyro.x, dt, x_angle, x_bias);
      float filteredY = kalmanFilter(a.acceleration.y, g.gyro.y, dt, y_angle, y_bias);

      Serial.println(String(filteredX) + " " + String(filteredY));

      boolean L = digitalRead(IR_L);  
      boolean R = digitalRead(IR_R);
      
      byte pattern = (L << 1) | R;

      switch(pattern){
        
        case 0b11:
          switch(lastDir){
            case STRAIGHT:
              currentSpeedL = defaultSpeedL; 
              currentSpeedR = defaultSpeedR;
              lastDir = STRAIGHT;
              break;
            case LEFT:
              currentSpeedL = -150;
              currentSpeedR = 255;
              break;
            case RIGHT:
              currentSpeedL = 255;
              currentSpeedR = -150;
              break;
          }
          break;
        case 0b10:
          currentSpeedL = 255;
          currentSpeedR = 200;
          lastDir = RIGHT;
          break;
        case 0b01:
          currentSpeedL = 200;
          currentSpeedR = 255;
          lastDir = LEFT;
          break;
      }
      if (pattern == 0b00) {
        if (stop && millisNow - stopMillis > 40){
          currentSpeedL = 0;
          currentSpeedR = 0;
          start = false;
        } else if (!stop) {
          stop = true;
          stopMillis = millisNow;
        }
      } else stop = false;

      go(currentSpeedL, currentSpeedR);
    }
    start = true;
    lcd.setCursor(0, 0);
    //lcd.print("Select to start!");
    //lcd.setCursor(0, 1);
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

float kalmanFilter(float newAngle, float newRate, float dt, float &angle, float &bias) {
  float rate = newRate - bias;  // Remove bias from gyroscope rate
  angle += dt * rate;  // Estimate new angle
  // Update estimation error covariance
  P[0][0] += dt * (dt * P[1][1] - P[0][1] - P[1][0] + q_angle);
  P[0][1] -= dt * P[1][1];
  P[1][0] -= dt * P[1][1];
  P[1][1] += q_bias * dt;
  // Compute Kalman gain
  float S = P[0][0] + r_measure;
  float K[2] = { P[0][0] / S, P[1][0] / S };
  // Update estimates with measurement
  float y = newAngle - angle;
  angle += K[0] * y;
  bias += K[1] * y;
  // Update error covariance matrix
  P[0][0] -= K[0] * P[0][0];
  P[0][1] -= K[0] * P[0][1];
  P[1][0] -= K[1] * P[0][0];
  P[1][1] -= K[1] * P[0][1];
  return angle;  // Return the filtered angle
}