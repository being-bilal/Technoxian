#include <WiFi.h>
#include <WebServer.h>
#include <EEPROM.h>

// WiFi credentials
const char* ssid = "Shiraz iPhone";
const char* password = "Shiraz1234";


// Create WebServer object on port 80
WebServer server(80);

// --- IR sensor multiplexer ---
const int SIG_PIN = 34;
const int S0 = 18;
const int S1 = 5;
const int S2 = 4;
const int S3 = 2;

// --- Motor driver pins (TB6612FNG) ---
const int AIN1 = 14;
const int AIN2 = 17;
const int PWMA = 21;

const int BIN1 = 12;
const int BIN2 = 13;
const int PWMB = 20;

const int STBY = 19;
const int CALIBRATION_PIN = 15;  // Set this pin HIGH to trigger calibration

// EEPROM addresses for calibration data
const int EEPROM_SIZE = 512;
const int CALIBRATION_VALID_ADDR = 0;    // 1 byte - magic number to check if calibration exists
const int MIN_VALS_ADDR = 1;             // 16 * 4 = 64 bytes for min values (int = 4 bytes)
const int MAX_VALS_ADDR = 65;            // 16 * 4 = 64 bytes for max values
const int CALIBRATION_MAGIC = 0xAB;      // Magic number to identify valid calibration

int sensorValues[16];
//int threshold = 3350;  // Black line gives values above this
int minVals[16];
int maxVals[16];
int normalizedVals[16];
int lastValidError = 0;  

// PID constants (tune these!)
float Kp = 20;
float Ki = 0;
float Kd = 1;

// PID variables
int lastError = 0;
float integral = 0;

// Current system status for web interface
int currentPosition = 0;
int currentLeftSpeed = 0;
int currentRightSpeed = 0;
int currentError = 0;
float currentPIDOutput = 0;

void setup() {
  Serial.begin(115200);
  EEPROM.begin(EEPROM_SIZE);

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
  pinMode(CALIBRATION_PIN, INPUT);

  // Initialize WiFi
  WiFi.begin(ssid, password);
  Serial.print("Connecting to WiFi");
  while (WiFi.status() != WL_CONNECTED) {
    delay(1000);
    Serial.print(".");
  }
  Serial.println();
  Serial.print("WiFi connected! IP address: ");
  Serial.println(WiFi.localIP());

  // Setup web server routes
  server.on("/", handleRoot);
  server.begin();
  Serial.println("Web server started");

  // Print initial PID values
  printPIDValues();

  // Read calibration pin state 
  bool calibrationPinState = digitalRead(CALIBRATION_PIN);
  Serial.println("Reading external calibration control...");
  Serial.print("Calibration Pin State: ");
  Serial.println(calibrationPinState ? "HIGH (Calibration Mode)" : "LOW (Normal Mode)");

  if (calibrationPinState) {
    Serial.println("*** CALIBRATION MODE DETECTED ***");
    Serial.println("External pin is HIGH - Starting calibration sequence...");
    performCalibration();
  } else {
    Serial.println("*** NORMAL OPERATION MODE ***");
    Serial.println("External pin is LOW - Loading saved calibration...");
    
    if (loadCalibrationFromEEPROM()) {
      Serial.println("✓ Calibration data loaded from EEPROM successfully!");
      printCalibrationValues();
      Serial.println("Robot ready for line following!");
    } else {
      Serial.println("✗ ERROR: No saved calibration data found!");
      Serial.println("SOLUTION: Set external calibration pin HIGH and reset ESP32");
      Serial.println("Robot cannot operate without calibration data.");
      }
    }

  Serial.println("Calibration done, place bot on line");
  for (int i = 5; i > 0; i--) {
      Serial.print(i);
      Serial.println("...");
      delay(1000);
  }

}

void loop() {
  server.handleClient(); // Handle web requests
  
  readSensors();

  int position = getLinePosition();
  currentPosition = position;

  if (position == -1) {
    Serial.println("Line lost! Rotating to search...");

    int recoverySpeed = 190;
    if (lastValidError < 0) {
      moveMotors(-recoverySpeed, recoverySpeed);
      currentLeftSpeed = -recoverySpeed;
      currentRightSpeed = recoverySpeed;
    } else {
      moveMotors(recoverySpeed, -recoverySpeed);
      currentLeftSpeed = recoverySpeed;
      currentRightSpeed = -recoverySpeed;
    }

    delay(200); 
    return;
  }

  int baseSpeed = 255;

  // PID calculations
  int error = position - 8;
  currentError = error;
  lastValidError = error;  // Save for recovery
  integral += error;
  int derivative = error - lastError;
  lastError = error;

  int correction = Kp * error + Ki * integral + Kd * derivative;
  currentPIDOutput = correction;

  int leftSpeed = constrain(baseSpeed - correction, -255, 255);
  int rightSpeed = constrain(baseSpeed + correction, -255, 255);
  
  currentLeftSpeed = leftSpeed;
  currentRightSpeed = rightSpeed;

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

// Load calibration data from EEPROM
bool loadCalibrationFromEEPROM() {
  // Check if valid calibration exists
  if (EEPROM.read(CALIBRATION_VALID_ADDR) != CALIBRATION_MAGIC) {
    return false;
  }
  
  // Load min values
  for (int i = 0; i < 16; i++) {
    int addr = MIN_VALS_ADDR + (i * 4);
    minVals[i] = EEPROM.read(addr) | 
                 (EEPROM.read(addr + 1) << 8) | 
                 (EEPROM.read(addr + 2) << 16) | 
                 (EEPROM.read(addr + 3) << 24);
  }
  
  // Load max values
  for (int i = 0; i < 16; i++) {
    int addr = MAX_VALS_ADDR + (i * 4);
    maxVals[i] = EEPROM.read(addr) | 
                 (EEPROM.read(addr + 1) << 8) | 
                 (EEPROM.read(addr + 2) << 16) | 
                 (EEPROM.read(addr + 3) << 24);
  }
  
  return true;
}

// Save calibration data to EEPROM
void saveCalibrationToEEPROM() {
  // Set magic number to indicate valid calibration
  EEPROM.write(CALIBRATION_VALID_ADDR, CALIBRATION_MAGIC);
  
  // Save min values
  for (int i = 0; i < 16; i++) {
    int addr = MIN_VALS_ADDR + (i * 4);
    EEPROM.write(addr, minVals[i] & 0xFF);
    EEPROM.write(addr + 1, (minVals[i] >> 8) & 0xFF);
    EEPROM.write(addr + 2, (minVals[i] >> 16) & 0xFF);
    EEPROM.write(addr + 3, (minVals[i] >> 24) & 0xFF);
  }
  
  // Save max values
  for (int i = 0; i < 16; i++) {
    int addr = MAX_VALS_ADDR + (i * 4);
    EEPROM.write(addr, maxVals[i] & 0xFF);
    EEPROM.write(addr + 1, (maxVals[i] >> 8) & 0xFF);
    EEPROM.write(addr + 2, (maxVals[i] >> 16) & 0xFF);
    EEPROM.write(addr + 3, (maxVals[i] >> 24) & 0xFF);
  }
  
  EEPROM.commit(); // Important: commit changes to flash
  Serial.println("✓ Calibration data saved to EEPROM successfully!");
}

// Perform sensor calibration
void performCalibration() {
  stopMotors(); 
  Serial.println("╔══════════════════════════════════╗");
  Serial.println("║        CALIBRATION MODE          ║");
  Serial.println("╚══════════════════════════════════╝");
  Serial.println("Calibration starts in:");
  
  for (int i = 5; i > 0; i--) {
    Serial.print("  ");
    Serial.print(i);
    Serial.println(" seconds...");
    delay(1000);
  }
  
  // Initialize calibration arrays
  for (int i = 0; i < 16; i++) {
    minVals[i] = 4095;  // Start with max possible value
    maxVals[i] = 0;     // Start with min possible value
  }

  Serial.println("🔄 CALIBRATING SENSORS...");
  Serial.println("Move robot over line for best results!");
  Serial.println("Progress: ");

  unsigned long startTime = millis();
  int progressCounter = 0;
  
  while (millis() - startTime < 4000) { // 4 second calibration
    for (int i = 0; i < 16; i++) {
      setMuxChannel(i);
      delayMicroseconds(500);
      int val = analogRead(SIG_PIN);
  
      if (val < minVals[i]) minVals[i] = val;
      if (val > maxVals[i]) maxVals[i] = val;
    }
    
    // Show progress
    if (millis() - startTime > (progressCounter * 400)) {
      Serial.print("█");
      progressCounter++;
    }
    
    delay(50);
  }
  
  Serial.println(" DONE!");
   
  // Validate calibration quality
  bool validCalibration = true;
  int poorSensors = 0;
  
  Serial.println("\n📊 CALIBRATION ANALYSIS:");
  Serial.println("Sensor | Min  | Max  | Range | Status");
  Serial.println("-------|------|------|-------|--------");
  
  for (int i = 0; i < 16; i++) {
    int range = maxVals[i] - minVals[i];
    String status = "✓ Good";
    
    if (range < 200) { // Minimum range check
      status = "⚠ Poor";
      poorSensors++;
      validCalibration = false;
    }
    
    Serial.printf("  %2d   | %4d | %4d |  %4d | %s\n", 
                  i, minVals[i], maxVals[i], range, status.c_str());
  }
  
  Serial.println("-------|------|------|-------|--------");
  
  if (validCalibration) {
    saveCalibrationToEEPROM();
    Serial.println("✅ CALIBRATION SUCCESSFUL!");
    Serial.println("   All sensors have good range (>200 counts)");
    Serial.println("   Data saved to EEPROM");
  } else {
    Serial.println("❌ CALIBRATION WARNING!");
    Serial.printf("   %d sensors have poor range (<200 counts)\n", poorSensors);
    Serial.println("   Calibration NOT saved - please recalibrate");
  }

  Serial.println("╔══════════════════════════════════╗");
  Serial.println("║     CALIBRATION COMPLETE         ║");
  Serial.println("╚══════════════════════════════════╝");
}

void printCalibrationValues() {
  Serial.println("📋 Current Calibration Data:");
  Serial.println("Sensor | Min Value | Max Value | Range");
  Serial.println("-------|-----------|-----------|-------");
  for (int i = 0; i < 16; i++) {
    Serial.printf("  %2d   |   %4d    |   %4d    | %4d\n", 
                  i, minVals[i], maxVals[i], maxVals[i] - minVals[i]);
  }
  Serial.println("-------|-----------|-----------|-------");
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

void printPIDValues() {
  Serial.println("=== Current PID Values ===");
  Serial.print("Kp (Proportional): ");
  Serial.println(Kp, 3);
  Serial.print("Ki (Integral): ");
  Serial.println(Ki, 3);
  Serial.print("Kd (Derivative): ");
  Serial.println(Kd, 3);
  Serial.println("========================");
}

void printSensorData(int position, int leftSpeed, int rightSpeed) {
  Serial.print("IR: ");
  for (int i = 0; i < 16; i++) {
    Serial.print(sensorValues[i]);
    Serial.print(i == 15 ? " | " : ", ");
  }

  Serial.print("Pos: ");
  Serial.print(position);
  Serial.print(" | Error: ");
  Serial.print(currentError);
  Serial.print(" | PID: ");
  Serial.print(currentPIDOutput, 2);
  Serial.print(" | L: ");
  Serial.print(leftSpeed);
  Serial.print(" R: ");
  Serial.println(rightSpeed);
  delay(300);
}

// Web server handlers
void handleRoot() {
  String html = R"rawliteral(
    <!DOCTYPE html>
<html lang="en">
<head>
    <meta charset="UTF-8">
    <meta name="viewport" content="width=device-width, initial-scale=1.0, user-scalable=no">
    <title>Line Follower PID Tuner</title>
    <link rel="stylesheet" href="https://cdnjs.cloudflare.com/ajax/libs/font-awesome/6.4.0/css/all.min.css">
    <style>
        :root {
            --background: hsl(0, 0%, 100%);
            --foreground: hsl(222.2, 84%, 4.9%);
            --card: hsl(0, 0%, 100%);
            --card-foreground: hsl(222.2, 84%, 4.9%);
            --popover: hsl(0, 0%, 100%);
            --popover-foreground: hsl(222.2, 84%, 4.9%);
            --primary: hsl(221.2, 83.2%, 53.3%);
            --primary-foreground: hsl(210, 40%, 98%);
            --secondary: hsl(210, 40%, 96.1%);
            --secondary-foreground: hsl(222.2, 47.4%, 11.2%);
            --muted: hsl(210, 40%, 96.1%);
            --muted-foreground: hsl(215.4, 16.3%, 46.9%);
            --accent: hsl(210, 40%, 96.1%);
            --accent-foreground: hsl(222.2, 47.4%, 11.2%);
            --destructive: hsl(0, 84.2%, 60.2%);
            --destructive-foreground: hsl(210, 40%, 98%);
            --border: hsl(214.3, 31.8%, 91.4%);
            --input: hsl(214.3, 31.8%, 91.4%);
            --ring: hsl(221.2, 83.2%, 53.3%);
            --radius: 0.5rem;
            --success: hsl(142.1, 76.2%, 36.3%);
            --warning: hsl(47.9, 95.8%, 53.1%);
        }

        .dark {
            --background: hsl(222.2, 84%, 4.9%);
            --foreground: hsl(210, 40%, 98%);
            --card: hsl(222.2, 84%, 4.9%);
            --card-foreground: hsl(210, 40%, 98%);
            --popover: hsl(222.2, 84%, 4.9%);
            --popover-foreground: hsl(210, 40%, 98%);
            --primary: hsl(217.2, 91.2%, 59.8%);
            --primary-foreground: hsl(222.2, 47.4%, 11.2%);
            --secondary: hsl(217.2, 32.6%, 17.5%);
            --secondary-foreground: hsl(210, 40%, 98%);
            --muted: hsl(217.2, 32.6%, 17.5%);
            --muted-foreground: hsl(215, 20.2%, 65.1%);
            --accent: hsl(217.2, 32.6%, 17.5%);
            --accent-foreground: hsl(210, 40%, 98%);
            --destructive: hsl(0, 62.8%, 30.6%);
            --destructive-foreground: hsl(210, 40%, 98%);
            --border: hsl(217.2, 32.6%, 17.5%);
            --input: hsl(217.2, 32.6%, 17.5%);
            --ring: hsl(224.3, 76.3%, 48%);
        }

        * {
            margin: 0;
            padding: 0;
            box-sizing: border-box;
            -webkit-tap-highlight-color: transparent;
        }

        body {
            font-family: 'Inter', -apple-system, BlinkMacSystemFont, 'Segoe UI', Roboto, sans-serif;
            background-color: var(--background);
            color: var(--foreground);
            min-height: 100vh;
            padding: 1rem;
            font-size: 0.875rem;
            line-height: 1.5;
            transition: background-color 0.2s ease;
        }

        @media (prefers-color-scheme: dark) {
            body {
                color-scheme: dark;
            }
        }

        .container {
            max-width: 640px;
            margin: 0 auto;
        }

        .header {
            text-align: center;
            margin-bottom: 1.5rem;
            display: flex;
            flex-direction: column;
            align-items: center;
        }

        .header h1 {
            font-size: 1.5rem;
            font-weight: 600;
            margin-bottom: 0.5rem;
            display: flex;
            align-items: center;
            gap: 0.5rem;
        }

        .header p {
            color: var(--muted-foreground);
            max-width: 80%;
            margin: 0 auto;
        }

        .connection-status {
            display: flex;
            align-items: center;
            justify-content: center;
            gap: 0.5rem;
            padding: 0.75rem 1rem;
            background-color: var(--secondary);
            border-radius: var(--radius);
            margin-bottom: 1.5rem;
            border: 1px solid var(--border);
        }

        .status-dot {
            width: 0.75rem;
            height: 0.75rem;
            border-radius: 50%;
            background-color: var(--success);
            animation: pulse 2s infinite;
        }

        @keyframes pulse {
            0% { opacity: 1; }
            50% { opacity: 0.5; }
            100% { opacity: 1; }
        }

        .card {
            background-color: var(--card);
            border-radius: var(--radius);
            border: 1px solid var(--border);
            padding: 1.5rem;
            margin-bottom: 1rem;
            box-shadow: 0 1px 2px 0 rgba(0, 0, 0, 0.05);
        }

        .card-header {
            display: flex;
            align-items: center;
            gap: 0.5rem;
            margin-bottom: 1.25rem;
        }

        .card-header h2 {
            font-size: 1.125rem;
            font-weight: 600;
        }

        .slider-group {
            margin-bottom: 1.5rem;
        }

        .slider-header {
            display: flex;
            justify-content: space-between;
            align-items: center;
            margin-bottom: 0.75rem;
        }

        .slider-label {
            font-weight: 500;
            color: var(--foreground);
            display: flex;
            align-items: center;
            gap: 0.5rem;
        }

        .slider-value {
            font-family: 'Roboto Mono', monospace;
            font-weight: 500;
            background-color: var(--secondary);
            padding: 0.25rem 0.5rem;
            border-radius: calc(var(--radius) - 2px);
            font-size: 0.875rem;
        }

        .slider-container {
            position: relative;
            height: 1.5rem;
            display: flex;
            align-items: center;
        }

        .slider {
            -webkit-appearance: none;
            width: 100%;
            height: 0.375rem;
            border-radius: 0.375rem;
            background: var(--input);
            outline: none;
            margin: 0;
            padding: 0;
        }

        .slider::-webkit-slider-thumb {
            -webkit-appearance: none;
            appearance: none;
            width: 1.25rem;
            height: 1.25rem;
            border-radius: 50%;
            background: var(--primary);
            cursor: pointer;
            border: 2px solid var(--background);
            box-shadow: 0 1px 2px 0 rgba(0, 0, 0, 0.05);
            transition: transform 0.15s ease;
        }

        .slider::-webkit-slider-thumb:active {
            transform: scale(1.1);
        }

        .slider::-moz-range-thumb {
            width: 1.25rem;
            height: 1.25rem;
            border-radius: 50%;
            background: var(--primary);
            cursor: pointer;
            border: 2px solid var(--background);
            box-shadow: 0 1px 2px 0 rgba(0, 0, 0, 0.05);
        }

        .btn {
            display: inline-flex;
            align-items: center;
            justify-content: center;
            white-space: nowrap;
            border-radius: var(--radius);
            font-size: 0.875rem;
            font-weight: 500;
            transition: all 0.15s ease;
            cursor: pointer;
            user-select: none;
            touch-action: manipulation;
            height: 2.5rem;
            padding: 0 1.25rem;
            border: 1px solid transparent;
        }

        .btn-primary {
            background-color: var(--primary);
            color: var(--primary-foreground);
            width: 100%;
            height: 3rem;
            font-size: 1rem;
            font-weight: 600;
        }

        .btn-primary:hover {
            background-color: hsl(221.2, 83.2%, 48%);
        }

        .btn-primary:active {
            background-color: hsl(221.2, 83.2%, 43%);
        }

        .btn-secondary {
            background-color: var(--secondary);
            color: var(--secondary-foreground);
            border: 1px solid var(--border);
        }

        .btn-secondary:hover {
            background-color: hsl(210, 40%, 92%);
        }

        .btn-secondary:active {
            background-color: hsl(210, 40%, 88%);
        }

        .btn:disabled {
            opacity: 0.5;
            pointer-events: none;
        }

        .btn-icon {
            margin-right: 0.5rem;
        }

        .grid {
            display: grid;
            gap: 1rem;
        }

        .grid-cols-2 {
            grid-template-columns: repeat(2, 1fr);
        }

        .grid-cols-3 {
            grid-template-columns: repeat(3, 1fr);
        }

        .grid-cols-4 {
            grid-template-columns: repeat(4, 1fr);
        }

        .status-item {
            background-color: var(--secondary);
            border-radius: calc(var(--radius) - 2px);
            padding: 1rem;
            border-left: 3px solid var(--primary);
        }

        .status-label {
            font-size: 0.75rem;
            color: var(--muted-foreground);
            margin-bottom: 0.25rem;
            font-weight: 500;
        }

        .status-value {
            font-family: 'Roboto Mono', monospace;
            font-weight: 600;
            font-size: 1.125rem;
        }

        .sensor-grid {
            display: grid;
            grid-template-columns: repeat(8, 1fr);
            gap: 0.25rem;
            margin: 1rem 0;
        }

        .sensor {
            aspect-ratio: 1;
            border-radius: calc(var(--radius) - 2px);
            background-color: var(--secondary);
            display: flex;
            align-items: center;
            justify-content: center;
            font-size: 0.625rem;
            font-weight: 600;
            transition: all 0.2s ease;
            position: relative;
            overflow: hidden;
        }

        .sensor::after {
            content: '';
            position: absolute;
            inset: 0;
            background-color: var(--foreground);
            opacity: 0;
            transition: opacity 0.2s ease;
        }

        .sensor.active::after {
            opacity: 1;
        }

        .sensor span {
            position: relative;
            z-index: 1;
            color: var(--foreground);
        }

        .sensor.active span {
            color: var(--background);
        }

        .current-pid {
            background-color: var(--secondary);
            border-radius: calc(var(--radius) - 2px);
            padding: 1rem;
            margin-top: 1.5rem;
        }

        .current-pid h4 {
            font-size: 0.875rem;
            font-weight: 500;
            color: var(--muted-foreground);
            text-align: center;
            margin-bottom: 0.75rem;
        }

        .pid-item {
            text-align: center;
        }

        .pid-item label {
            font-size: 0.75rem;
            color: var(--muted-foreground);
            display: block;
            margin-bottom: 0.25rem;
        }

        .pid-item span {
            font-family: 'Roboto Mono', monospace;
            font-weight: 600;
            font-size: 0.875rem;
        }

        .presets-grid {
            display: grid;
            grid-template-columns: repeat(2, 1fr);
            gap: 0.75rem;
            margin-top: 1rem;
        }

        .badge {
            display: inline-flex;
            align-items: center;
            border-radius: 9999px;
            padding: 0.25rem 0.75rem;
            font-size: 0.75rem;
            font-weight: 500;
            background-color: var(--secondary);
            color: var(--secondary-foreground);
            border: 1px solid var(--border);
        }

        .badge-success {
            background-color: hsl(142.1, 76.2%, 36.3%);
            color: white;
            border-color: transparent;
        }

        .badge-warning {
            background-color: hsl(47.9, 95.8%, 53.1%);
            color: black;
            border-color: transparent;
        }

        .badge-destructive {
            background-color: hsl(0, 84.2%, 60.2%);
            color: white;
            border-color: transparent;
        }

        /* Dark mode adjustments */
        @media (prefers-color-scheme: dark) {
            .sensor {
                border: 1px solid var(--border);
            }
            
            .sensor.active span {
                color: white;
            }
        }

        /* Responsive adjustments */
        @media (max-width: 640px) {
            .container {
                padding: 0;
            }
            
            .card {
                padding: 1.25rem;
            }
            
            .presets-grid {
                grid-template-columns: 1fr;
            }
        }

        @media (max-width: 400px) {
            .grid-cols-4 {
                grid-template-columns: repeat(2, 1fr);
            }
        }

        /* Animation for button states */
        @keyframes buttonSuccess {
            0% { background-color: var(--primary); }
            50% { background-color: var(--success); }
            100% { background-color: var(--primary); }
        }

        .btn-success {
            animation: buttonSuccess 1s ease;
        }

        /* Tooltip for sliders */
        .slider-tooltip {
            position: absolute;
            top: -2rem;
            left: 50%;
            transform: translateX(-50%);
            background-color: var(--foreground);
            color: var(--background);
            padding: 0.25rem 0.5rem;
            border-radius: calc(var(--radius) - 2px);
            font-size: 0.75rem;
            font-weight: 500;
            opacity: 0;
            transition: opacity 0.2s ease;
            pointer-events: none;
            z-index: 10;
        }

        .slider-container:hover .slider-tooltip {
            opacity: 1;
        }
    </style>
</head>
<body>
    <div class="container">
        <div class="header">
            <h1><i class="fas fa-robot btn-icon"></i>Line Follower PID Tuner</h1>
            <p>Adjust PID parameters in real-time for optimal line following performance</p>
        </div>

        <div class="connection-status">
            <div class="status-dot"></div>
            <span id="connectionText">Connected to ESP32</span>
            <span class="badge badge-success">Live</span>
        </div>

        <div class="card">
            <div class="card-header">
                <i class="fas fa-sliders-h"></i>
                <h2>PID Parameters</h2>
            </div>
            
            <div class="slider-group">
                <div class="slider-header">
                    <span class="slider-label">
                        <i class="fas fa-chart-line btn-icon"></i>
                        Proportional (Kp)
                    </span>
                    <span class="slider-value" id="kpValue">15.000</span>
                </div>
                <div class="slider-container">
                    <span class="slider-tooltip" id="kpTooltip">15.000</span>
                    <input type="range" class="slider" id="kpSlider" min="0" max="50" step="0.1" value="15">
                </div>
            </div>

            <div class="slider-group">
                <div class="slider-header">
                    <span class="slider-label">
                        <i class="fas fa-wave-square btn-icon"></i>
                        Integral (Ki)
                    </span>
                    <span class="slider-value" id="kiValue">0.000</span>
                </div>
                <div class="slider-container">
                    <span class="slider-tooltip" id="kiTooltip">0.000</span>
                    <input type="range" class="slider" id="kiSlider" min="0" max="5" step="0.01" value="0">
                </div>
            </div>

            <div class="slider-group">
                <div class="slider-header">
                    <span class="slider-label">
                        <i class="fas fa-running btn-icon"></i>
                        Derivative (Kd)
                    </span>
                    <span class="slider-value" id="kdValue">10.000</span>
                </div>
                <div class="slider-container">
                    <span class="slider-tooltip" id="kdTooltip">10.000</span>
                    <input type="range" class="slider" id="kdSlider" min="0" max="30" step="0.1" value="10">
                </div>
            </div>

            <button class="btn btn-primary" id="updatePidBtn" onclick="sendPIDValues()">
                <i class="fas fa-paper-plane btn-icon"></i>
                Update PID Values
            </button>

            <div class="current-pid">
                <h4>Active PID Values</h4>
                <div class="grid grid-cols-3">
                    <div class="pid-item">
                        <label>Kp</label>
                        <span id="activekp">15.000</span>
                    </div>
                    <div class="pid-item">
                        <label>Ki</label>
                        <span id="activeki">0.000</span>
                    </div>
                    <div class="pid-item">
                        <label>Kd</label>
                        <span id="activekd">10.000</span>
                    </div>
                </div>
            </div>

            <div class="presets-grid">
                <button class="btn btn-secondary" onclick="setPreset('conservative')">
                    <i class="fas fa-shield-alt btn-icon"></i>
                    Conservative
                </button>
                <button class="btn btn-secondary" onclick="setPreset('balanced')">
                    <i class="fas fa-balance-scale btn-icon"></i>
                    Balanced
                </button>
                <button class="btn btn-secondary" onclick="setPreset('aggressive')">
                    <i class="fas fa-bolt btn-icon"></i>
                    Aggressive
                </button>
                <button class="btn btn-secondary" onclick="setPreset('reset')">
                    <i class="fas fa-undo btn-icon"></i>
                    Reset
                </button>
            </div>
        </div>

        <div class="card">
            <div class="card-header">
                <i class="fas fa-chart-bar"></i>
                <h2>Live Performance</h2>
            </div>
            
            <div class="grid grid-cols-2 md:grid-cols-4 gap-2">
                <div class="status-item">
                    <div class="status-label">Error</div>
                    <div class="status-value" id="currentError">0</div>
                </div>
                <div class="status-item">
                    <div class="status-label">PID Output</div>
                    <div class="status-value" id="pidOutput">0.00</div>
                </div>
                <div class="status-item">
                    <div class="status-label">Left Motor</div>
                    <div class="status-value" id="leftMotor">0</div>
                </div>
                <div class="status-item">
                    <div class="status-label">Right Motor</div>
                    <div class="status-value" id="rightMotor">0</div>
                </div>
            </div>

            <div class="sensor-grid" id="sensorArray">
                <!-- Sensors will be generated here -->
            </div>
        </div>
    </div>

    <footer style="margin: 2rem auto 0 auto; max-width: 640px; text-align: center; color: var(--muted-foreground); font-size: 0.95rem; padding: 1.5rem 0 0.5rem 0;">
        <div style="border-top: 1px solid var(--border); padding-top: 1rem;">
            <span>Made with <i class="fas fa-heart" style="color: #e25555;"></i> by <strong>AMURoboclub&apos;s Technoxian Team</strong></span><br>
            <span style="font-size: 0.9em; color: var(--muted-foreground);">Team Members: Aman Javed [Lead], Ahmad Ilyas, Ammar Bari, Syed Fawaz Ayazi, Kandarp Gupta, Supreet Chaudhary, Mohd Bilal, Zaid, Shoeb, Avyukt Soni</span>
        </div>
    </footer>

    <script>
        // PID parameters
        const pidParams = {
            kp: 15.0,
            ki: 0.0,
            kd: 10.0
        };

        // Initialize the interface
        document.addEventListener('DOMContentLoaded', function() {
            initializeSliders();
            initializeSensorDisplay();
            startStatusUpdates();
            setupTooltips();
        });

        function initializeSliders() {
            const sliders = {
                kp: document.getElementById('kpSlider'),
                ki: document.getElementById('kiSlider'),
                kd: document.getElementById('kdSlider')
            };

            Object.keys(sliders).forEach(key => {
                sliders[key].addEventListener('input', function() {
                    pidParams[key] = parseFloat(this.value);
                    updateSliderDisplay(key, pidParams[key]);
                });
            });
        }

        function updateSliderDisplay(key, value) {
            const display = document.getElementById(`${key}Value`);
            const tooltip = document.getElementById(`${key}Tooltip`);
            
            if (display) display.textContent = value.toFixed(3);
            if (tooltip) tooltip.textContent = value.toFixed(3);
        }

        function setupTooltips() {
            const sliders = document.querySelectorAll('.slider');
            
            sliders.forEach(slider => {
                slider.addEventListener('input', function() {
                    const tooltip = this.parentElement.querySelector('.slider-tooltip');
                    if (tooltip) {
                        tooltip.textContent = parseFloat(this.value).toFixed(3);
                        // Position tooltip above thumb
                        const percent = (this.value - this.min) / (this.max - this.min);
                        tooltip.style.left = `${percent * 100}%`;
                    }
                });
            });
        }

        function initializeSensorDisplay() {
            const sensorArray = document.getElementById('sensorArray');
            sensorArray.innerHTML = '';
            
            for (let i = 0; i < 16; i++) {
                const sensor = document.createElement('div');
                sensor.className = 'sensor';
                sensor.id = `sensor${i}`;
                sensor.innerHTML = `<span>${i}</span>`;
                sensorArray.appendChild(sensor);
            }
        }

        function setPreset(presetName) {
            const presets = {
                conservative: { kp: 5.0, ki: 0.0, kd: 2.0, icon: 'shield-alt', color: 'success' },
                balanced: { kp: 15.0, ki: 0.0, kd: 10.0, icon: 'balance-scale', color: 'primary' },
                aggressive: { kp: 30.0, ki: 0.5, kd: 20.0, icon: 'bolt', color: 'warning' },
                reset: { kp: 0.0, ki: 0.0, kd: 0.0, icon: 'undo', color: 'destructive' }
            };

            const preset = presets[presetName];
            if (preset) {
                pidParams.kp = preset.kp;
                pidParams.ki = preset.ki;
                pidParams.kd = preset.kd;
                
                document.getElementById('kpSlider').value = preset.kp;
                document.getElementById('kiSlider').value = preset.ki;
                document.getElementById('kdSlider').value = preset.kd;
                
                updateSliderDisplay('kp', preset.kp);
                updateSliderDisplay('ki', preset.ki);
                updateSliderDisplay('kd', preset.kd);
                
                // Visual feedback
                const btn = document.querySelector(`button[onclick="setPreset('${presetName}')"]`);
                btn.classList.add(`badge-${preset.color}`);
                setTimeout(() => btn.classList.remove(`badge-${preset.color}`), 1000);
            }
        }

        async function sendPIDValues() {
            const updateBtn = document.getElementById('updatePidBtn');
            const originalHTML = updateBtn.innerHTML;
            
            updateBtn.innerHTML = `<i class="fas fa-spinner fa-spin btn-icon"></i> Updating...`;
            updateBtn.disabled = true;
            
            try {
                const response = await fetch('/setPID', {
                    method: 'POST',
                    headers: { 'Content-Type': 'application/x-www-form-urlencoded' },
                    body: `kp=${pidParams.kp}&ki=${pidParams.ki}&kd=${pidParams.kd}`
                });
                
                if (response.ok) {
                    // Update active PID display
                    document.getElementById('activekp').textContent = pidParams.kp.toFixed(3);
                    document.getElementById('activeki').textContent = pidParams.ki.toFixed(3);
                    document.getElementById('activekd').textContent = pidParams.kd.toFixed(3);
                    
                    // Visual feedback
                    updateBtn.innerHTML = `<i class="fas fa-check btn-icon"></i> Updated!`;
                    updateBtn.classList.add('btn-success');
                    
                    await new Promise(resolve => setTimeout(resolve, 1500));
                } else {
                    throw new Error('Failed to update PID values');
                }
            } catch (error) {
                console.error('Error sending PID values:', error);
                updateBtn.innerHTML = `<i class="fas fa-times btn-icon"></i> Error`;
                updateBtn.classList.add('btn-destructive');
                
                await new Promise(resolve => setTimeout(resolve, 1500));
            } finally {
                updateBtn.innerHTML = originalHTML;
                updateBtn.disabled = false;
                updateBtn.classList.remove('btn-success', 'btn-destructive');
            }
        }

        async function updateStatus() {
            try {
                const response = await fetch('/getStatus');
                if (!response.ok) throw new Error('Network response was not ok');
                
                const data = await response.json();
                
                document.getElementById('currentError').textContent = data.error;
                document.getElementById('pidOutput').textContent = data.pidOutput.toFixed(2);
                document.getElementById('leftMotor').textContent = data.leftSpeed;
                document.getElementById('rightMotor').textContent = data.rightSpeed;
                
                // Update sensor visualization
                if (data.sensorValues) {
                    updateSensorVisualization(data.sensorValues);
                }
            } catch (error) {
                console.error('Error fetching status:', error);
            }
        }

        function updateSensorVisualization(sensorValues) {
            for (let i = 0; i < 16; i++) {
                const sensor = document.getElementById(`sensor${i}`);
                if (sensor && sensorValues && sensorValues[i] !== undefined) {
                    sensor.classList.toggle('active', sensorValues[i] > 500);
                }
            }
        }

        function startStatusUpdates() {
            setInterval(updateStatus, 200); // Update every 200ms
        }

        // Prevent zoom on double tap
        let lastTouchEnd = 0;
        document.addEventListener('touchend', function(event) {
            const now = Date.now();
            if (now - lastTouchEnd <= 300) {
                event.preventDefault();
            }
            lastTouchEnd = now;
        }, { passive: false });
    </script>
</body>
</html>
)rawliteral";
  
  server.send(200, "text/html", html);
}

// Handler for setting PID values
void handleSetPID() {
  if (server.hasArg("kp") && server.hasArg("ki") && server.hasArg("kd")) {
    Kp = server.arg("kp").toFloat();
    Ki = server.arg("ki").toFloat();
    Kd = server.arg("kd").toFloat();
    
    Serial.println("PID values updated via web interface:");
    printPIDValues();
    
    server.send(200, "text/plain", "PID values updated successfully");
  } else {
    server.send(400, "text/plain", "Missing PID parameters");
  }
}

// Handler for getting current status
void handleGetStatus() {
  String json = "{";
  json += "\"error\":" + String(currentError) + ",";
  json += "\"pidOutput\":" + String(currentPIDOutput) + ",";
  json += "\"leftSpeed\":" + String(currentLeftSpeed) + ",";
  json += "\"rightSpeed\":" + String(currentRightSpeed) + ",";
  json += "\"position\":" + String(currentPosition) + ",";
  json += "\"kp\":" + String(Kp, 3) + ",";
  json += "\"ki\":" + String(Ki, 3) + ",";
  json += "\"kd\":" + String(Kd, 3) + ",";
  json += "\"sensorValues\":[";
  
  for (int i = 0; i < 16; i++) {
    json += String(sensorValues[i]);
    if (i < 15) json += ",";
  }
  
  json += "]}";
  
  server.send(200, "application/json", json);
}
