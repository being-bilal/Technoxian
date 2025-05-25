#include <Arduino.h>

// ------------------------------------------------motor data-----------------------------------------
#define LEFT_MOTOR_PWM 18 // left motor 
#define LEFT_MOTOR_DIR 19
#define RIGHT_MOTOR_PWM 21 // right motor
#define RIGHT_MOTOR_DIR 22

int baseSpeed = 100; // Base speed 
int maxSpeed = 200;  // Max speed

// ----------------------------------------------Sensor data-------------------------------------------
#define SENSOR_COUNT 12
int sensorPins[SENSOR_COUNT] = {A0, A1, A2, A3, A4, A5, 2, 3, 8, 9, 10, 11}; 
int sensorValues[SENSOR_COUNT];
int sensorWeights[SENSOR_COUNT] = {-55, -45, -35, -25, -15, -5, 5, 15, 25, 35, 45, 55}; // Sensor Weight (distance between sensors)

// -----------------------------------------------PID Data---------------------------------------------
float Kp = 0.0;
float Ki = 0.0;
float Kd = 0.0;

float error = 0;
float previousError = 0;
float integral = 0;
int maxIntegral = 100;
float pidOutput = 0;


void setup(){
  Serial.begin(115200);
  SetupMotors();
  SetupSensors();
  Serial.println("Motor and Sensor Setup Complete, Starting the bot...");
}

void PrintToSerialPlotter() {
  Serial.print("Error:");
  Serial.print(error);
  Serial.print(",");
  
  Serial.print("PID:");
  Serial.print(pidOutput);
  Serial.print(",");
  
  Serial.print("Left_Motor:");
  Serial.print(constrain(baseSpeed - pidOutput, 0, maxSpeed));
  Serial.print(",");
  
  Serial.print("Right_Motor:");
  Serial.print(constrain(baseSpeed + pidOutput, 0, maxSpeed));
  Serial.print(",");
  
  Serial.print("Base_Speed:");
  Serial.print(baseSpeed);
  
  Serial.println();
}

void loop(){
  ReadSensors();
  CalculateMotorSpeeds();
  PrintToSerialPlotter();
  delay(10);
}

// motor pin setup 
void SetupMotors(){
  pinMode(LEFT_MOTOR_PWM, OUTPUT);
  pinMode(LEFT_MOTOR_DIR, OUTPUT);
  pinMode(RIGHT_MOTOR_PWM, OUTPUT);
  pinMode(RIGHT_MOTOR_DIR, OUTPUT);
}

// IR pin setup
void SetupSensors(){
  for(int i = 0; i < SENSOR_COUNT; i++){
     pinMode(sensorPins[i], INPUT);
  }
}

// Reading senor vaalues
void ReadSensors(){
  for(int i = 0; i < SENSOR_COUNT; i++){
  sensorValues[i] = digitalRead(sensorPins[i]);
  }
}

// Error Calculation 
// Calculating the error of the follower by finding the relative distance from the center using the weighted sum of the sensors
float CalculateError(){
  int activeSensors = 0;
  float weightedSum = 0;
  for(int i = 0; i < SENSOR_COUNT; i++){ 
    if(sensorValues[i] == 0){                // Assuming sensors give 0 on black
        weightedSum += sensorWeights[i];
        activeSensors++;}}
        
  if(activeSensors == 0){                   // No sensors are active and the bot has lost the path
    return 0;
  }else{
    return (weightedSum / activeSensors); 
  } 
}

// PID implementation 
// Applying PID using the relative position from the center 
void CalculatePID() {
  error = CalculateError(); 
  integral += error;
  integral = constrain(integral, -maxIntegral, maxIntegral); // contraining the integral error so it doesnt get to large over time
  float derivative = error - previousError;
  
  pidOutput = (Kp * error) + (Ki * integral) + (Kd * derivative); 
  // pidOutput is positive == robot is to the right of the line
  // pidOutput is negative == robot is to the left of the line
  previousError = error;
}


// Calculating motor speeds based on PID output  
void CalculateMotorSpeeds() {
  CalculatePID();
  // adjusting motor speeds 
  int leftMotorSpeed = baseSpeed - pidOutput; 
  int rightMotorSpeed = baseSpeed + pidOutput;
  
  // Constraining speeds between 0 and maxSpeed
  leftMotorSpeed = constrain(leftMotorSpeed, 0, maxSpeed);
  rightMotorSpeed = constrain(rightMotorSpeed, 0, maxSpeed);
  
  SetMotorSpeeds(leftMotorSpeed, rightMotorSpeed);
}



void SetMotorSpeeds(int leftSpeed, int rightSpeed) {
  // Left motor 
  digitalWrite(LEFT_MOTOR_DIR, HIGH);  // Forward direction
  analogWrite(LEFT_MOTOR_PWM, leftSpeed);
  
  // Right motor  
  digitalWrite(RIGHT_MOTOR_DIR, HIGH);  // Forward direction
  analogWrite(RIGHT_MOTOR_PWM, rightSpeed);
  }
