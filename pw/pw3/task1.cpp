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

const int IR_L = A2;
//const int IR_M = 1;
const int IR_R = A4;

volatile uint8_t previousPortCState = 0;

unsigned long millisNow, millisElapsed, millisStart, millisMarker;
unsigned int millisInterval = 10*1000; // 10 seconds

unsigned long microNowRight, prevSampleRight, prevMicroRight;
int pulseCountRight;

unsigned long microNowLeft, prevSampleLeft, prevMicroLeft;
int pulseCountLeft;

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
  bool takenMarker = false;
  bool done = false;
  bool distanceB = false;
  
  if (analogRead(A0) > 600 && analogRead(A0) < 800){
    millisStart = millis();
    lcd.clear();

    while(start){
      millisNow = millis();
      millisElapsed = millisNow - millisStart;
      if(!distanceB) distance = (pulseCountRight + pulseCountLeft) / 2 * M_PI * 6.5 / 20;

      lcd.setCursor(0, 0);
      lcd.print("Distance: " + String(distance) + "cm   ");
      lcd.setCursor(0, 1);  
      lcd.print("Time: " + String(millisElapsed/1000.0));
      if (!done && !takenMarker) {millisMarker = millisNow;} 
      if (!done && distance >= 110 && !takenMarker) {
        millisMarker = millisNow;
        takenMarker = true;
      } else if (takenMarker && millisNow - millisMarker < 2000){
        distance = 110;
        lcd.setCursor(0, 0);
        lcd.print("Distance: " + String(distance) + "cm   ");
        go(0, 0);
        done = true;
        distanceB = true;
      } else {
        distanceB = false;
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
    }
    start = true;
    lcd.setCursor(0, 0);
    pulseCountRight = 0;
    pulseCountLeft = 0;
  }
}

ISR(PCINT1_vect){
  uint8_t currentPortCState = PINC;

  // Rising edge: previous = 0, current = 1
  if (!(previousPortCState & (1 << 3)) && (currentPortCState  & (1 << 3))) {
    pulseCountLeft += (currentSpeedL > 0)? 1 : -1;
  }
  previousPortCState = currentPortCState;
}

void INT0_ISR(void){
  pulseCountRight += (currentSpeedR > 0)? 1 : -1;
}

void go(int speedL, int speedR){ 
  digitalWrite(IN1, (speedR <= 0)? LOW : HIGH);
  digitalWrite(IN2, (speedR >= 0)? LOW : HIGH);

  digitalWrite(IN3, (speedL <= 0)? LOW : HIGH);
  digitalWrite(IN4, (speedL >= 0)? LOW : HIGH);

  analogWrite(ENB, constrain(abs(speedL), 0, 255));
  analogWrite(ENA, constrain(abs(speedR), 0, 255));
}