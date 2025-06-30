// --- IR sensor multiplexer ---
const int SIG_PIN = 15;
const int S0 = 18;
const int S1 = 5;
const int S2 = 4;
const int S3 = 2;

// --- Motor driver pins (TB6612FNG) ---
const int AIN1 = 13;
const int AIN2 = 12;
const int PWMA = 25;

const int BIN1 = 14;
const int BIN2 = 27;
const int PWMB = 26;

const int STBY = 19;

int sensorValues[16];
//int threshold = 3350;  // Black line gives values above this
int minVals[16];
int maxVals[16];
int normalizedVals[16];
int lastValidError = 0;  


// PID constants (tune these!)
float Kp = 15;
float Ki = 0;
float Kd = 10;

// PID variables
int lastError = 0;
float integral = 0;

void setup() {
  Serial.begin(115200);

  pinMode(S0, OUTPUT);
  pinMode(S1, OUTPUT);
  pinMode(S2, OUTPUT);
  pinMode(S3, OUTPUT);

  pinMode(AIN1, OUTPUT);
  pinMode(AIN2, OUTPUT);
  pinMode(PWMA, OUTPUT);

  pinMode(BIN1, OUTPUT);
  pinMode(BIN2, OUTPUT);
  pinMode(PWMB, OUTPUT);

  pinMode(STBY, OUTPUT);
  digitalWrite(STBY, HIGH);

  // Callibrating
  Serial.println("Place robot on track. Calibration starts in:");
  for (int i = 5; i > 0; i--) {
      Serial.print(i);
      Serial.println("...");
      delay(1000);
  }
  
  for (int i = 0; i < 16; i++) {
    minVals[i] = 4095; 
    maxVals[i] = 0;
  }

  Serial.println("Calibrating sensors...");
  unsigned long startTime = millis();
  while (millis() - startTime < 4000) {
    for (int i = 0; i < 16; i++) {
      setMuxChannel(i);
      delayMicroseconds(200);
      int val = analogRead(SIG_PIN);
  
      if (val < minVals[i]) minVals[i] = val;
      if (val > maxVals[i]) maxVals[i] = val;
  
      // Print which sensor is being read
      Serial.print("Calibrating Sensor ");
      Serial.print(i);
      Serial.print(": Value = ");
      Serial.println(val);
    }
    delay(50); // slow down the print rate
   }

  Serial.println("Calibration done.");
  Serial.println("Final calibrated min/max values:");
  for (int i = 0; i < 16; i++) {
    Serial.print("Sensor ");
    Serial.print(i);
    Serial.print(" => Min: ");
    Serial.print(minVals[i]);
    Serial.print(" | Max: ");
    Serial.println(maxVals[i]);
  }
}

void loop() {
  readSensors();

  int position = getLinePosition();

  if (position == -1) {
  Serial.println("Line lost! Rotating to search...");

  int recoverySpeed = 120;
  if (lastValidError < 0) {
    moveMotors(-recoverySpeed, recoverySpeed);
  } else {
    moveMotors(recoverySpeed, -recoverySpeed);
  }

  delay(200); 
  return;
  }

  int baseSpeed = 180;

  // PID calculations
  int error = position - 8;
  lastValidError = error;  // Save for recovery
  integral += error;
  int derivative = error - lastError;
  lastError = error;

  int correction = Kp * error + Ki * integral + Kd * derivative;

  int leftSpeed = constrain(baseSpeed - correction, -255, 255);
  int rightSpeed = constrain(baseSpeed + correction, -255, 255);

  moveMotors(leftSpeed, rightSpeed);
  printSensorData(position, leftSpeed, rightSpeed);

  delay(10);
}

void readSensors() {
    for (int i = 0; i < 16; i++) {
      setMuxChannel(i);
      delayMicroseconds(200);
      int val = analogRead(SIG_PIN);
  
      // Avoid division by zero
      if (maxVals[i] - minVals[i] == 0) {
        normalizedVals[i] = 0;
      } else {
        normalizedVals[i] = map(val, minVals[i], maxVals[i], 0, 1000);
      }
  
      sensorValues[i] = normalizedVals[i]; // use normalized value in PID
  }
}

int getLinePosition() {
  long weightedSum = 0;
  int activeCount = 0;

  for (int i = 0; i < 16; i++) {
    if (sensorValues[i] > 500) {
      weightedSum += i;
      activeCount++;
    }
  }

  if (activeCount == 0) return -1;

  return weightedSum / activeCount;
}

void setMuxChannel(int channel) {
  digitalWrite(S0, bitRead(channel, 0));
  digitalWrite(S1, bitRead(channel, 1));
  digitalWrite(S2, bitRead(channel, 2));
  digitalWrite(S3, bitRead(channel, 3));
}

void moveMotors(int leftSpeed, int rightSpeed) {
  // Left Motor
  if (leftSpeed >= 0) {
    digitalWrite(AIN1, HIGH);
    digitalWrite(AIN2, LOW);
  } else {
    digitalWrite(AIN1, LOW);
    digitalWrite(AIN2, HIGH);
    leftSpeed = -leftSpeed;
  }
  analogWrite(PWMA, leftSpeed);

  // Right Motor
  if (rightSpeed >= 0) {
    digitalWrite(BIN1, HIGH);
    digitalWrite(BIN2, LOW);
  } else {
    digitalWrite(BIN1, LOW);
    digitalWrite(BIN2, HIGH);
    rightSpeed = -rightSpeed;
  }
  analogWrite(PWMB, rightSpeed);
}

void stopMotors() {
  analogWrite(PWMA, 0);
  analogWrite(PWMB, 0);
}

void printSensorData(int position, int leftSpeed, int rightSpeed) {
  Serial.print("IR: ");
  for (int i = 0; i < 16; i++) {
    Serial.print(sensorValues[i]);
    Serial.print(i == 15 ? " | " : ", ");
  }

  Serial.print("Pos: ");
  Serial.print(position);
  Serial.print(" | L: ");
  Serial.print(leftSpeed);
  Serial.print(" R: ");
  Serial.println(rightSpeed);
  delay(300);
}