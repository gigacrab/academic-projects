#include <Arduino.h>
#include <LiquidCrystal.h>
#include <math.h>
#include <PinChangeInterrupt.h>

void INT0_ISR(void);
void go(int, int);

LiquidCrystal lcd(8, 9, 4, 5, 6, 7); // RS, E, D4, D5, D6, D7

const int ENA = 11;
const int IN1 = 13;
const int IN2 = 12;

const int ENB = 3;
const int IN3 = A1;
const int IN4 = A2;

const int ENCODER_R = 2;
const int ENCODER_L = A3;

const int IR_L = A5;
//const int IR_M = 1;
const int IR_R = A4;

volatile uint8_t previousPortCState = 0;

unsigned long millisNow, millisElapsed, millisStart;
unsigned int millisInterval = 10*1000; // 10 seconds

unsigned long microNowRight, prevSampleRight, prevMicroRight;
int pulseCountRight;

unsigned long microNowLeft, prevSampleLeft, prevMicroLeft;
int pulseCountLeft;

boolean start;

int distance;

int defaultSpeedL = 100; // 107
int defaultSpeedR = 100; // 150

enum dir {STRAIGHT, LEFT, RIGHT};

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
  //pinMode(IR_M, INPUT);
  pinMode(IR_L, INPUT);

  pinMode(ENCODER_R, INPUT);
  pinMode(A3, INPUT);
  attachInterrupt(digitalPinToInterrupt(ENCODER_R), INT0_ISR, RISING);
  PCICR |= (1 << PCIE1); // Enable PCINT1 group (for pin A0-A5)
  PCMSK1 |= (1 << PCINT11); // Enable PCINT11 (for pin A3)
  previousPortCState = PINC; // Store initial state of PORTC

  lcd.print("Select to start!");
}

void loop() {
  enum dir lastDir = STRAIGHT;
  if (analogRead(A0) > 600 && analogRead(A0) < 800){
    millisStart = millis();
    lcd.clear();
    lcd.setCursor(0, 0);
    lcd.print("Moving forward!");

    while(start){
      millisNow = millis();
      millisElapsed = millisNow - millisStart;
      
      /*boolean L = digitalRead(IR_L);
      boolean M = digitalRead(IR_M);  
      boolean R = digitalRead(IR_R);

      byte pattern = (L << 2) | (M << 1) | R;

      switch(pattern){
        case 0b001:
          go(defaultSpeedL / 2, motorspeedR);
          break;
        case 0b011:
          go(-defaultSpeedL, motorspeedR);
          break;
        case 0b100:
          go(defaultSpeedL, motorspeedR / 2);
          break;
        case 0b101:
          go(defaultSpeedL, motorspeedR);
          break;       
        case 0b110:
          go(defaultSpeedL, -motorspeedR);
          break;
        default:
          go(0, 0);
      }*/

      boolean L = digitalRead(IR_L);  
      boolean R = digitalRead(IR_R);
      
      byte pattern = (L << 1) | R;

      switch(pattern){
        case 0b11:
          switch(lastDir){
            case STRAIGHT:
              go(defaultSpeedL, defaultSpeedR);
              lastDir = STRAIGHT;
              break;
            case LEFT:
              go(-150, 255);
              break;
            case RIGHT:
              go(255, -150);
              break;
          }
          break;
        case 0b10:
          go(255, 200);
          lastDir = RIGHT;
          break;
        case 0b01:
          go(200, 255);
          lastDir = LEFT;
          break;
        case 0b00:
          go(0, 0);
          lastDir = STRAIGHT;
          start = false;
          break;
      }

      lcd.setCursor(0, 1);  
      lcd.print("Time: " + String(millisElapsed/1000.0));
    }
    start = true;
    lcd.setCursor(0, 0);
    //lcd.print("Select to start!");
    //lcd.setCursor(0, 1);
    lcd.print("Distance: ");
    distance = (pulseCountRight + pulseCountLeft) / 2 * M_PI * 6.5 / 20;
    Serial.println(pulseCountLeft);
    Serial.println(pulseCountRight);
    lcd.print(distance);
    lcd.print("cm   ");
    pulseCountRight = 0;
    pulseCountLeft = 0;
  }
}

ISR(PCINT1_vect){
  uint8_t currentPortCState = PINC;

  // Rising edge: previous = 0, current = 1
  if (!(previousPortCState & (1 << 3)) && (currentPortCState  & (1 << 3))) {
    microNowLeft = micros();

    if ((microNowLeft - prevMicroLeft) > 0) {
        prevSampleLeft = microNowLeft - prevMicroLeft;
        prevMicroLeft = microNowLeft;
        pulseCountLeft++;
    }
  }
  previousPortCState = currentPortCState;
}

void INT0_ISR(void){
  microNowRight = micros();
  if (microNowRight - prevMicroRight > 0){
    prevSampleRight = microNowRight - prevMicroRight; 
    prevMicroRight = microNowRight;
    pulseCountRight++;
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
