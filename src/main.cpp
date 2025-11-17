#include <Arduino.h>
#include <LiquidCrystal.h>
#include <math.h>
#include <PinChangeInterrupt.h>

void INT0_ISR(void);
void setSpeed(byte, byte);
void go(String);

LiquidCrystal lcd(8, 9, 4, 5, 6, 7); // RS, E, D4, D5, D6, D7

const int ENA = 11;
const int IN1 = 13;
const int IN2 = 12;

const int ENB = 3;
const int IN3 = A1;
const int IN4 = A2;

const int ENCODER_RIGHT = 2;
const int ENCODER_LEFT = A3;

volatile uint8_t previousPortCState = 0;

typedef struct {
  byte speed;
  struct {
    boolean forward;
    boolean backward;
  } direction;
} motor; 

motor motorA, motorB;

unsigned long millisNow;
unsigned long millisElapsed;
unsigned long millisStart;
unsigned int millisInterval = 10*1000; // 10 seconds

unsigned long microNowRight;
unsigned int prevSampleRight;
unsigned long prevMicroRight;
int pulseCountRight;

unsigned long microNowLeft;
unsigned int prevSampleLeft;
unsigned long prevMicroLeft;
int pulseCountLeft;

boolean start;

int distance;

void setup() {
  lcd.begin(16, 2);
  Serial.begin(9600);

  pinMode(ENA, OUTPUT);
  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  pinMode(ENB, OUTPUT);
  pinMode(IN3, OUTPUT);
  pinMode(IN4, OUTPUT);

  pinMode(ENCODER_RIGHT, INPUT);
  pinMode(A3, INPUT);
  attachInterrupt(digitalPinToInterrupt(ENCODER_RIGHT), INT0_ISR, RISING);

  // Enable PCINT1 group (for pin A0-A5)
  PCICR |= (1 << PCIE1);

  // Enable PCINT11 (for pin A3)
  PCMSK1 |= (1 << PCINT11);

  // Store initial state of PORTC
  previousPortCState = PINC;

  lcd.print("Select to start!");
}

void loop() {
  if (analogRead(A0) > 600 && analogRead(A0) < 800){
    millisStart = millis();
    lcd.clear();
    lcd.setCursor(0, 0);
    lcd.print("Moving forward!");

    while(start){
      millisNow = millis();
      millisElapsed = millisNow - millisStart;
      if (millisElapsed <= millisInterval){
        setSpeed(107, 150);
        go("forward");
      } else {
        go("stop");
        start = false;
      }
      lcd.setCursor(0, 1);
      lcd.print("Time: " + String(millisElapsed/1000.0));
    }
    start = true;
    lcd.setCursor(0, 0);
    lcd.print("Select to start!");
    lcd.setCursor(0, 1);
    lcd.print("Distance: ");
    distance = (pulseCountRight + pulseCountLeft) / 2 * M_PI * 6.5 / 20;
    lcd.print(distance);
    pulseCountRight = 0;
    pulseCountLeft = 0;
  }
}

ISR(PCINT1_vect){
  uint8_t currentPortCState = PINC;

  // Rising edge: previous = 0, current = 1
  if (!(previousPortCState & (1 << 3)) && (currentPortCState  & (1 << 3))) {
    microNowLeft = micros();

    if ((microNowLeft - prevMicroLeft) * 10 > prevSampleLeft * 6) {
        prevSampleLeft = microNowLeft - prevMicroLeft;
        prevMicroLeft = microNowLeft;
        pulseCountLeft++;
    }
  }
  previousPortCState = currentPortCState;
}

void INT0_ISR(void){
  microNowRight = micros();
  if (microNowRight - prevMicroRight > prevSampleRight * 0.6){
    prevSampleRight = microNowRight - prevMicroRight; 
    prevMicroRight = microNowRight;
    pulseCountRight++;
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