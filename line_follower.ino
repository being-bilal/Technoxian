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
// 12 sensors == Left side (2), Center (8), Right side (2)
int sensorWeights[SENSOR_COUNT] = {0, 0, -44, -31, -19, -6, 6, 19, 31, 44, 0, 0}; // distance between sensors == 12.50mm 

bool onLeftEdge = (sensorValues[0] == 0 || sensorValues[1] == 0);    // 0 = black, 1 = white
bool onRightEdge = (sensorValues[10] == 0 || sensorValues[11] == 0);
unsigned long lineLostStartTime = 0;
bool lineCurrentlyLost = false;

// -----------------------------------------------PID Data---------------------------------------------
float Kp = 0.0;
float Ki = 0.0;
float Kd = 0.0;

float error = 0.0;
float previousError = 0.0;
float integral = 0.0;
float maxIntegral = 100.0;
float pidOutput = 0.0;

// ------------------------------------------------SetUp------------------------------------------------
void setup(){
  Serial.begin(115200);
  SetupMotors();
  SetupSensors();
  Serial.println("Motor and Sensor Setup Complete, Starting the bot...");
}

// ----------------------------------------------Debugging-----------------------------------------------
// Visualising the values using Serial plotter
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

// visualize all sensors as a bar chart
void PrintSensorArray() {
  Serial.print("Sensors: ");
  for(int i = 0; i < SENSOR_COUNT; i++) {
    Serial.print("S");
    Serial.print(i);
    Serial.print(":");
    Serial.print(sensorValues[i] * 100); // Scale up for visibility
    if(i < SENSOR_COUNT - 1) Serial.print(",");
  }
  Serial.println();
}

// Printing values on serial monitor
void PrintToSerialMonitor() {
  Serial.println("========================================================");
  
  Serial.print("Time: ");
  Serial.print(millis());
  Serial.println(" ms");
  Serial.println("--------------------------------------------------------");
  
  Serial.println("SENSOR ARRAY:");
  Serial.print("  Pins: ");
  for (int i = 0; i < SENSOR_COUNT; i++) {
    if (sensorPins[i] < 10) Serial.print("  ");
    else if (sensorPins[i] < 100) Serial.print(" ");
    Serial.print(sensorPins[i]);
    Serial.print(" ");
  }
  Serial.println();
  
  Serial.print("Values: ");
  for (int i = 0; i < SENSOR_COUNT; i++) {
    // manual %3d formatting for sensorValues[i] (0 or 1)
    if (sensorValues[i] < 10) Serial.print("  ");
    else if (sensorValues[i] < 100) Serial.print(" ");
    Serial.print(sensorValues[i]);
    Serial.print(" ");
  }
  Serial.println();
  
  Serial.print("Visual: ");
  for (int i = 0; i < SENSOR_COUNT; i++) {
    Serial.print(sensorValues[i] ? "█" : "░");
    Serial.print(" ");
  }
  Serial.println();
  
  Serial.println("-------------------------------------------------------");
  Serial.print("Line Position: ");
  // Print float with 2 decimals manually
  float linePos = CalculateError();
  int linePosInt = (int)linePos;
  int linePosFrac = abs((int)(linePos * 100)) % 100;
  if (linePos < 0 && linePosInt == 0) Serial.print("-"); // handle negative zero
  Serial.print(linePosInt);
  Serial.print(".");
  if (linePosFrac < 10) Serial.print("0");
  Serial.println(linePosFrac);
  
  Serial.print("Error:         ");
  // Print error float with 2 decimals manually
  int errorInt = (int)error;
  int errorFrac = abs((int)(error * 100)) % 100;
  if (error < 0 && errorInt == 0) Serial.print("-");
  Serial.print(errorInt);
  Serial.print(".");
  if (errorFrac < 10) Serial.print("0");
  Serial.print(errorFrac);
  Serial.print("  |");
  
  int errorBar = map(constrain(error, -50, 50), -50, 50, 0, 20);
  for (int i = 0; i < 21; i++) {
    if (i == 10) Serial.print("|");
    else if (i == errorBar) Serial.print("*");
    else Serial.print("-");
  }
  Serial.println("|");
  
  Serial.println("------------------------------------------------------");
  
  Serial.print("PID: Kp=");
  Serial.print(Kp, 3);
  Serial.print("  Ki=");
  Serial.print(Ki, 3);
  Serial.print("  Kd=");
  Serial.println(Kd, 3);
  
  Serial.print("     Error=");
  Serial.print(error, 2);
  Serial.print("  Integral=");
  Serial.print(integral, 2);
  Serial.print("  Derivative=");
  Serial.println(error - previousError, 2);
  
  Serial.print("     Output: ");
  Serial.println(pidOutput, 2);
  
  Serial.println("------------------------------------------------------");
  
  int leftSpeed = constrain(baseSpeed - pidOutput, 0, maxSpeed);
  int rightSpeed = constrain(baseSpeed + pidOutput, 0, maxSpeed);
  
  Serial.print("MOTORS:  Base Speed = ");
  Serial.println(baseSpeed);
  
  Serial.print("         Left  Motor: ");
  if (leftSpeed < 10) Serial.print("  ");
  else if (leftSpeed < 100) Serial.print(" ");
  Serial.print(leftSpeed);
  Serial.print(" ");
  PrintSpeedBar(leftSpeed, maxSpeed);
  
  Serial.print("         Right Motor: ");
  if (rightSpeed < 10) Serial.print("  ");
  else if (rightSpeed < 100) Serial.print(" ");
  Serial.print(rightSpeed);
  Serial.print(" ");
  PrintSpeedBar(rightSpeed, maxSpeed);
  
  Serial.println("-----------------------------------------------");
  Serial.print("STATUS: ");
  if (abs(error) < 5) {
    Serial.print("✓ ON LINE");
  } else if (error > 25) {
    Serial.print("→ TURNING RIGHT");
  } else if (error < -25) {
    Serial.print("← TURNING LEFT");
  } else {
    Serial.print("~ ADJUSTING");
  }
  Serial.println();
  
  Serial.println("======================================================");
  Serial.println();
}

// Helper function to print speed bars
void PrintSpeedBar(int speed, int maxSpeed) {
  Serial.print("[");
  int bars = map(speed, 0, maxSpeed, 0, 20);
  for(int i = 0; i < 20; i++) {
    if(i < bars) Serial.print("█");
    else Serial.print("░");
  }
  Serial.println("]");
}

----------------------------------------------------Main Logic---------------------------------------------
void loop(){
  ReadSensors();
  CalculateMotorSpeeds();
  //PrintToSerialPlotter();
  //PrintSensorArray();
  //PrintToSerialMonitor();

  // Line Lost (stopping the bot if the line is lost for more than 5 seconds)
  if (lineCurrentlyLost) {
    if (lineLostStartTime == 0) { // checking if this is the first moment we noticed the line is lost
      lineLostStartTime = millis();
    }else if (millis() - lineLostStartTime >= 5000) { 
      SetMotorSpeeds(0, 0);  // Stop the motors
    }
  } else { // Line is not lost 
    lineLostStartTime = 0; 
  }

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
  for(int i = 2; i < (SENSOR_COUNT-2); i++){ 
    if(sensorValues[i] == 0){                // sensors give 0 on black
        weightedSum += sensorWeights[i];
        activeSensors++;}}
        
  // Line lost Conditions 
  if (activeSensors == 0) {
      if (onLeftEdge) {                      // Bot is on left edge
          // hard left turn
          SetMotorSpeeds(baseSpeed / 2, baseSpeed);
      } else if (onRightEdge) {              // Bot is on right edge
          // Line detected far right → hard right turn
          SetMotorSpeeds(baseSpeed, baseSpeed / 2);
      } else {
          // Bot lost line completely
          lineCurrentlyLost = true;
          if (previousError > 0) {          // previous error would be positive if the bot left the line from right
              SetMotorSpeeds(baseSpeed / 2, baseSpeed); 
          } else {
              SetMotorSpeeds(baseSpeed, baseSpeed / 2);}
      }
      return 0;
  }

  lineCurrentlyLost = false;
  // If center sensors are active, proceed with normal PID control
  return (weightedSum / activeSensors); // relative position from the center 
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
 

// Setting the speed of the motors 
void SetMotorSpeeds(int leftSpeed, int rightSpeed) {
  // Left motor 
  digitalWrite(LEFT_MOTOR_DIR, HIGH);  
  analogWrite(LEFT_MOTOR_PWM, leftSpeed);
  
  // Right motor  
  digitalWrite(RIGHT_MOTOR_DIR, HIGH); 
  analogWrite(RIGHT_MOTOR_PWM, rightSpeed);
  }
  
