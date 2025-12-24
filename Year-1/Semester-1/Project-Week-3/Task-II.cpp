#include <Arduino.h>
#include <LiquidCrystal.h>
#include <math.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>

void go(int, int);
float kalmanFilter(float newAngle, float newRate, float dt, float &angle, float &bias);

LiquidCrystal lcd(8, 9, 4, 5, 6, 7); // RS, E, D4, D5, D6, D7
Adafruit_MPU6050 mpu;

float filteredPitch;
float gyroXBias = 0.12;
float gyroYBias = -0.05;
float pitchBias;
float P[2][2] = {{1, 0}, {0, 1}};

// tuning parameters
float q_angle = 0.001;
float q_bias = 0.003;
float r_measure = 0.03;

int pitchOffset = -4;

const int ENA = 11;
const int IN1 = 13;
const int IN2 = 12;

const int ENB = 3;
const int IN3 = A1;
const int IN4 = A2;

const int IR_L = 2;
//const int IR_M = 1;
const int IR_R = A3;

unsigned long millisNow, millisPrev;
unsigned long millisSample;

int defaultSpeedL = 255; // 107
int defaultSpeedR = 255; // 150

int currentSpeedL;
int currentSpeedR;

void setup() {
  lcd.begin(16, 2);
  Serial.begin(9600);

  if (!mpu.begin()){
    Serial.println("MPU6050 not found!");
    while (1) delay(10);
  }

  mpu.setAccelerometerRange(MPU6050_RANGE_8_G);
  mpu.setGyroRange(MPU6050_RANGE_500_DEG);
  mpu.setFilterBandwidth(MPU6050_BAND_10_HZ);

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
  if (analogRead(A0) > 600 && analogRead(A0) < 800){
    bool goingUp = false;
    bool circled = false;
    float startYaw = 0;
    float maxAngle = 0;
    float yaw = 0;
    float pitch = 0;
    lcd.clear();
    go(255, 255);
    delay(500);

    while(1){
      millisNow = millis();

      sensors_event_t a, g, temp;
      mpu.getEvent(&a, &g, &temp);

      float dt = (millisNow - millisPrev) / 1000.0;
      millisPrev = millisNow;

      pitch = atan2(a.acceleration.z, sqrt(pow(a.acceleration.y, 2) + pow(a.acceleration.x, 2))) / M_PI * 180 + pitchOffset;
      float correctGyroY = (round(g.gyro.y * 100)/100.0 + gyroYBias) * 180 / M_PI ;
      filteredPitch = kalmanFilter(pitch, correctGyroY, dt, filteredPitch, pitchBias);

      if (millisNow - millisSample > 100) {
        Serial.println("Time: " + String(millisNow) + ", Pitch: " + String(pitch) + ", Filtered Pitch: " + String(filteredPitch));
        millisSample = millisNow;
      }

      float correctedGyroX = (round(g.gyro.x * 100)/100.0 + gyroXBias) * dt * 180 / M_PI;
      yaw += correctedGyroX;

      lcd.setCursor(0, 0);
      lcd.print("Yaw: " + String(yaw));

      lcd.setCursor(0, 1);
      lcd.print("Ramp Angle: " + String(maxAngle));
      
      if (pitch > maxAngle) maxAngle = pitch;
      if (pitch > 20) {
        goingUp = true;
        //Serial.println("going up state");
      } else if (goingUp && pitch < 10){
        static unsigned long millisStart = millisNow;
        if (millisNow - millisStart < 4000){
          go(0, 0);
          startYaw = yaw;
          //Serial.println("stopping state" + String(yaw) + " " + String(startYaw));
        } else {
          if (!circled && yaw < startYaw + 330){
            go (-150, 255);
            //Serial.println("circling state" + String(yaw) + " " + String(startYaw));
          } 
          else {
            circled = true;
            go(107, 150);
            //Serial.println("done state");
          }
        }
      } else {
        go (255, 255);
        //Serial.println("going straight");
      }
    }
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