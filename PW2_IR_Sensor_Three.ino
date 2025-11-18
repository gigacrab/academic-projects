int motorSpeedL = 100;  // Adjust Left Motor Speed 
int motorSpeedR = 100; // Adjust Right Motor Speed

// --- Pin Definitions --- //
// IR Sensors
const byte IR_Sensor_Left = A5;
const byte IR_Sensor_Middle = A3;
const byte IR_Sensor_Right = A4;

// Right Motor
const byte ENA = 11; // RM_Speed (PWM)
const byte IN1 = 13; // RM_FW
const byte IN2 = 12; // RM_BW

// Left Motor
const byte ENB = 3;  // LM_Speed (PWM)
const byte IN3 = A1; // LM_FW
const byte IN4 = A2; // LM_BW

void setup() {
  pinMode(ENA, OUTPUT);
  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  pinMode(IN3, OUTPUT);
  pinMode(IN4, OUTPUT);
  pinMode(ENB, OUTPUT);
  pinMode(IR_Sensor_Left, INPUT);
  pinMode(IR_Sensor_Middle, INPUT);
  pinMode(IR_Sensor_Right, INPUT);
  rotatemotor(0, 0);
}

void loop() {
  int L = digitalRead(IR_Sensor_Left);
  int M = digitalRead(IR_Sensor_Middle);
  int R = digitalRead(IR_Sensor_Right);

  // Case 1: (H, L, H) - Go straight
  if (L == HIGH && M == LOW && R == HIGH) {
    rotatemotor(motorSpeedL, motorSpeedR);
  }

  // Case 2: (H, L, L) - Drifting left
  else if (L == HIGH && M == LOW && R == LOW) {
    rotatemotor(255, 30);
  }

  // Case 3: (L, L, H) - Drifting right
  else if (L == LOW && M == LOW && R == HIGH) {
    rotatemotor(30, 255);
  }

  // Case 4: (H, H, L) - 90 Degree Turn to the Right
  else if (L == HIGH && M == HIGH && R == LOW) {
    rotatemotor(255, -150);
  }

  // Case 5: (L, H, H) - 90 Degree Turn to the Left
  else if (L == LOW && M == HIGH && R == HIGH) {
    rotatemotor(-150, 255);
  }
  
  // Case 6: (H, H, H) - Stop moving
  else if (L == HIGH && M == HIGH && R == HIGH) {
    rotatemotors(0, 0); 
  }

  // Case 7: (L, L, L) - Stop moving
  else {
    rotatemotor(0, 0);
  }
}

void rotatemotor(int LeftMotorSpeed, int RightMotorSpeed) {
  // --- Left Motor --- //
  if (LeftMotorSpeed < 0) { // Backward
    digitalWrite(IN3, LOW);
    digitalWrite(IN4, HIGH); // LM_BW
  }
  else if (LeftMotorSpeed > 0) { // Forward
    digitalWrite(IN3, HIGH); // LM_FW
    digitalWrite(IN4, LOW);
  }
  else { // Stop
    digitalWrite(IN3, LOW);
    digitalWrite(IN4, LOW);
  }

  // --- Right Motor --- //
  if (RightMotorSpeed < 0) { // Backward
    digitalWrite(IN1, LOW);
    digitalWrite(IN2, HIGH); // RM_BW
  }
  else if (RightMotorSpeed > 0) { // Forward
    digitalWrite(IN1, HIGH); // RM_FW
    digitalWrite(IN2, LOW);
  }
  else { // Stop
    digitalWrite(IN1, LOW);
    digitalWrite(IN2, LOW);
  }

  // --- Speed --- //
  analogWrite(ENB, constrain(abs(LeftMotorSpeed), 0, 255));
  analogWrite(ENA, constrain(abs(RightMotorSpeed), 0, 255));
}