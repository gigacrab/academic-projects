#include <Arduino.h>
#include <LiquidCrystal.h>

void go(int, int);
float detectDistance();

const int ENA = 11;
const int IN1 = 13;
const int IN2 = 12;

const int ENB = 3;
const int IN3 = A1;
const int IN4 = A2;

const int TRIG = A5;
const int ECHO = A4;

LiquidCrystal lcd(8, 9, 4, 5, 6, 7); // RS, E, D4, D5, D6, D7

boolean start = false;

int defaultSpeedL = 107; // 107
int defaultSpeedR = 150; // 150

int currentSpeedL;
int currentSpeedR;

void setup() {
  Serial.begin(9600);
  lcd.begin(16, 2);
  lcd.setCursor(0, 0);
  lcd.print("Group E11");
  lcd.setCursor(0, 1);
  lcd.print("Task 3");

  pinMode(ENA, OUTPUT);
  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  pinMode(ENB, OUTPUT);
  pinMode(IN3, OUTPUT);
  pinMode(IN4, OUTPUT);

  pinMode(TRIG, OUTPUT);
  pinMode(ECHO, INPUT);
}

void loop() {
  if (analogRead(A0) > 600 && analogRead(A0) < 800){
		start = !start;
	}
	while(start){
    lcd.setCursor(0, 0);
    float distance = detectDistance();
    Serial.println(distance);
		if (distance <= 30.0){
			lcd.print("Obstacle!      ");
			currentSpeedL = -defaultSpeedL;
			currentSpeedR = defaultSpeedR;
		} else {
			lcd.print("Moving forward!");
			currentSpeedL = defaultSpeedL;
			currentSpeedR = defaultSpeedR;
		}
		go(currentSpeedL, currentSpeedR);
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

// detects distance in centimeters
float detectDistance(){
	digitalWrite(TRIG, LOW);  
	delayMicroseconds(2);  
	digitalWrite(TRIG, HIGH);  
	delayMicroseconds(10);  
	digitalWrite(TRIG, LOW);

	float duration = pulseIn(ECHO, HIGH);
	float distance = (duration * .0343) / 2;
  return distance;
  delay(100);
}