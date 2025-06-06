#include <Arduino.h>
#include <SparkFun_TB6612.h>

// ------------------------------------------------TB6612 Motor Driver-----------------------------------------
// Motor 1 pins
#define AIN1 18
#define AIN2 19
#define PWMA 22
// Motor 2 pins
#define BIN1 20
#define BIN2 21
#define PWMB 23
// Standby pin 
#define STBY 17 // motor driver ko enable aur disable karne ke liye 

// Create motor objects
Motor leftMotor(PWMA, AIN1, AIN2, 1, STBY);   // offset for left motor
Motor rightMotor(PWMB, BIN1, BIN2, 1, STBY);  // offset for right motor

int baseSpeed = 100; // Base speed 
int maxSpeed = 200;  // Max speed

// ----------------------------------------------Multiplexer & Sensor data-----------------------------------
#define S0 2
#define S1 3
#define S2 4
#define S3 5
#define SIG_PIN A0  // Analog pin to read multiplexer output

#define SENSOR_COUNT 12
// Multiplexer channels for each sensor (0-15 available)
int sensorChannels[SENSOR_COUNT] = {0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11};
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
  Serial.print(constrain(baseSpeed - pidOutput, -maxSpeed, maxSpeed));
  Serial.print(",");
  
  Serial.print("Right_Motor:");
  Serial.print(constrain(baseSpeed + pidOutput, -maxSpeed, maxSpeed));
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
  Serial.print("  Channels: ");
  for (int i = 0; i < SENSOR_COUNT; i++) {
    if (sensorChannels[i] < 10) Serial.print("  ");
    else if (sensorChannels[i] < 100) Serial.print(" ");
    Serial.print(sensorChannels[i]);
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
  
  int leftSpeed = constrain(baseSpeed - pidOutput, -maxSpeed, maxSpeed);
  int rightSpeed = constrain(baseSpeed + pidOutput, -maxSpeed, maxSpeed);
  
  Serial.print("MOTORS:  Base Speed = ");
  Serial.println(baseSpeed);
  
  Serial.print("         Left  Motor: ");
  if (abs(leftSpeed) < 10) Serial.print("  ");
  else if (abs(leftSpeed) < 100) Serial.print(" ");
  Serial.print(leftSpeed);
  Serial.print(" ");
  PrintSpeedBar(abs(leftSpeed), maxSpeed);
  
  Serial.print("         Right Motor: ");
  if (abs(rightSpeed) < 10) Serial.print("  ");
  else if (abs(rightSpeed) < 100) Serial.print(" ");
  Serial.print(rightSpeed);
  Serial.print(" ");
  PrintSpeedBar(abs(rightSpeed), maxSpeed);
  
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

//----------------------------------------------------Main Logic---------------------------------------------
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

// TB6612 motor driver setup 
void SetupMotors(){
  pinMode(STBY, OUTPUT);
  digitalWrite(STBY, HIGH); // Enable motor driver
}

// Multiplexer pin setup
void SetupSensors(){
  pinMode(S0, OUTPUT);
  pinMode(S1, OUTPUT);
  pinMode(S2, OUTPUT);
  pinMode(S3, OUTPUT);
  pinMode(SIG_PIN, INPUT);
}

// Multiplexer channel selection
void SelectChannel(int channel) {
  digitalWrite(S0, bitRead(channel, 0));  
  digitalWrite(S1, bitRead(channel, 1));
  digitalWrite(S2, bitRead(channel, 2));
  digitalWrite(S3, bitRead(channel, 3));  
  delayMicroseconds(10); // Small delay for signal settling
}

// Reading sensor values through multiplexer
void ReadSensors(){
  for(int i = 0; i < SENSOR_COUNT; i++){
    SelectChannel(sensorChannels[i]);
    int analogValue = analogRead(SIG_PIN);
    // Convert analog reading to digital (adjust threshold)
    // Assuming white surface gives high reading, black line gives low reading
    sensorValues[i] = (analogValue > 512) ? 1 : 0;  // Threshold at mid-point
  }
  
  // Update edge detection variables
  onLeftEdge = (sensorValues[0] == 0 || sensorValues[1] == 0);
  onRightEdge = (sensorValues[10] == 0 || sensorValues[11] == 0);
}

// Error Calculation 
// Calculating the error of the follower by finding the relative distance from the center using the weighted sum of the sensors
float CalculateError(){
  int activeSensors = 0;
  float weightedSum = 0;
  for(int i = 2; i < (SENSOR_COUNT-2); i++){ 
    if(sensorValues[i] == 0){                // sensors give 0 on black
        weightedSum += sensorWeights[i];
        activeSensors++;
    }
  }
        
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
              SetMotorSpeeds(baseSpeed, baseSpeed / 2);
          }
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
  integral = constrain(integral, -maxIntegral, maxIntegral); // constraining the integral error so it doesn't get too large over time
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
  
  // Constraining speeds between -maxSpeed and maxSpeed (TB6612 supports reverse)
  leftMotorSpeed = constrain(leftMotorSpeed, -maxSpeed, maxSpeed);
  rightMotorSpeed = constrain(rightMotorSpeed, -maxSpeed, maxSpeed);
 
  SetMotorSpeeds(leftMotorSpeed, rightMotorSpeed);
}

// Setting the speed of the motors using TB6612 library
void SetMotorSpeeds(int leftSpeed, int rightSpeed) {
  leftMotor.drive(leftSpeed); // drive() function is part of the SparkFun_TB6612 motor driver library 
  rightMotor.drive(rightSpeed);
}
