#include <IBusBM.h> // Include iBUS library

IBusBM ibus; // Initialize iBUS object

// Pin assignments for Motor Driver
// Motor 1 - Front Motor 
#define ENA 6
#define INA1 5
#define INA2 4

// Motor 2 - back Motor 
#define ENB 3
#define INB1 2
#define INB2 7

// Control parameters
int yAxis, xAxis;

void setup() {
  ibus.begin(Serial);
  
  // Motor driver pins
  pinMode(ENA, OUTPUT);
  pinMode(INA1, OUTPUT);
  pinMode(INA2, OUTPUT);
  pinMode(ENB, OUTPUT);
  pinMode(INB1, OUTPUT);
  pinMode(INB2, OUTPUT);
  
  Serial.begin(115200);
  stopMotors();
}

void loop() {
  // Check for valid iBUS signal
  if (ibus.readChannel(0) == -1) { 
    stopMotors();  // Stop the motors if no signal is received
    return;  // Exit the loop early
  }
  
  // Read and map control channels
  int forwardBackward = ibus.readChannel(1);
  yAxis = map(forwardBackward, 1057, 2000, -255, 255);
  yAxis -= 15; // Adjusting range
  
  int leftRight = ibus.readChannel(3);
  xAxis = map(leftRight, 1000, 1991, -255, 255);
  xAxis += 3; // Adjusting range
  
  Serial.print("Y: "); Serial.print(yAxis); 
  Serial.print(" X: "); Serial.println(xAxis);
  
  // Movement logic
  if (yAxis > 5) {
    forward(yAxis);
  }
  else if (yAxis < -5) {
    backward(yAxis);
  }
  else if (xAxis > 5) {
    turnRight(xAxis);
  }
  else if (xAxis < -5) {
    turnLeft(xAxis);
  }
  else {
    stopMotors();  // If no input is detected, stop the motors
  }
}

void forward(int speed) {
  speed = constrain(map(speed, 5, 240, 0, 200), 0, 200);
  
  // Both motors forward at same speed
  controlMotor(ENA, INA1, INA2, speed);   // Front motor
  controlMotor(ENB, INB1, INB2, speed);   // Rear motor
  
  Serial.print("Front: "); Serial.print(speed);
  Serial.print(" Rear: "); Serial.println(speed);
}

void backward(int speed) {
  speed = constrain(map(speed, -270, -5, 200, 0), 0, 200);
  
  // Both motors backward at same speed
  controlMotor(ENA, INA1, INA2, -speed);  // Front motor
  controlMotor(ENB, INB1, INB2, -speed);  // Rear motor
  
  Serial.print("Front: "); Serial.print(-speed);
  Serial.print(" Rear: "); Serial.println(-speed);
}

void turnRight(int turnSpeed) {
  int speed = constrain(map(turnSpeed, 5, 258, 0, 150), 0, 150);
  
  controlMotor(ENA, INA1, INA2, speed);   // Front forward
  controlMotor(ENB, INB1, INB2, -speed);  // Rear backward
  
  Serial.print("Front: "); Serial.print(speed);
  Serial.print(" Rear: "); Serial.println(-speed);
}

void turnLeft(int turnSpeed) {
  int speed = constrain(map(turnSpeed, -5, -252, 0, 150), 0, 150);
  
  controlMotor(ENA, INA1, INA2, -speed);  // Front backward
  controlMotor(ENB, INB1, INB2, speed);   // Rear forward
  
  Serial.print("Front: "); Serial.print(-speed);
  Serial.print(" Rear: "); Serial.println(speed);
}

void stopMotors() {
  controlMotor(ENA, INA1, INA2, 0);  // Stop front motor
  controlMotor(ENB, INB1, INB2, 0);  // Stop rear motor
  Serial.println("Motors Stopped");
}

void controlMotor(int EN, int IN1, int IN2, int speed) {
  if (speed > 0) {
    digitalWrite(IN1, HIGH);
    digitalWrite(IN2, LOW);
  } else if (speed < 0) {
    digitalWrite(IN1, LOW);
    digitalWrite(IN2, HIGH);
  } else {
    digitalWrite(IN1, LOW);
    digitalWrite(IN2, LOW);
  }
  analogWrite(EN, abs(speed));
}
