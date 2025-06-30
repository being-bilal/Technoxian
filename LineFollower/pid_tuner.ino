#include <ESP8266WiFi.h>
#include <ESP8266WebServer.h>

// WiFi credentials
const char* ssid = "Jio 4G.";
const char* password = "SABAR#123";

// Create WebServer object on port 80
ESP8266WebServer server(80);

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

// Current system status for web interface
int currentPosition = 0;
int currentLeftSpeed = 0;
int currentRightSpeed = 0;
int currentError = 0;
float currentPIDOutput = 0;

void setup() {
  Serial.begin(9600);

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
  server.on("/setPID", handleSetPID);
  server.on("/getStatus", handleGetStatus);
  server.begin();
  Serial.println("Web server started");

  // Print initial PID values
  printPIDValues();

  // Calibrating
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
    delay(50);
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
  server.handleClient(); // Handle web requests
  
  readSensors();

  int position = getLinePosition();
  currentPosition = position;

  if (position == -1) {
    Serial.println("Line lost! Rotating to search...");

    int recoverySpeed = 120;
    if (lastValidError < 0) {
      moveMotors(-recoverySpeed, recoverySpeed);
      currentLeftSpeed = -recoverySpeed;
      currentRightSpeed = recoverySpeed;
      moveMotors(recoverySpeed, -recoverySpeed);
      currentLeftSpeed = recoverySpeed;
      currentRightSpeed = -recoverySpeed;
    } else {
      moveMotors(-recoverySpeed, recoverySpeed);
      currentLeftSpeed = -recoverySpeed;
      currentRightSpeed = recoverySpeed;
    }

    delay(200); 
    return;
  }

  int baseSpeed = 180;

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
    <style>
        * {
            margin: 0;
            padding: 0;
            box-sizing: border-box;
            -webkit-tap-highlight-color: transparent;
        }

        body {
            font-family: -apple-system, BlinkMacSystemFont, 'Segoe UI', Roboto, sans-serif;
            background: linear-gradient(135deg, #667eea 0%, #764ba2 100%);
            min-height: 100vh;
            padding: 10px;
            font-size: 16px;
            line-height: 1.5;
        }

        .container {
            max-width: 100%;
            margin: 0 auto;
            background: rgba(255, 255, 255, 0.95);
            border-radius: 20px;
            padding: 20px;
            box-shadow: 0 20px 40px rgba(0, 0, 0, 0.1);
            backdrop-filter: blur(10px);
        }

        .header {
            text-align: center;
            margin-bottom: 30px;
        }

        .header h1 {
            color: #333;
            font-size: 1.8rem;
            margin-bottom: 8px;
            background: linear-gradient(45deg, #667eea, #764ba2);
            -webkit-background-clip: text;
            -webkit-text-fill-color: transparent;
            background-clip: text;
        }

        .header p {
            color: #666;
            font-size: 0.9rem;
            padding: 0 10px;
        }

        .connection-status {
            display: flex;
            align-items: center;
            justify-content: center;
            gap: 10px;
            margin-bottom: 20px;
            padding: 10px;
            background: rgba(40, 167, 69, 0.1);
            border-radius: 10px;
        }

        .status-dot {
            width: 12px;
            height: 12px;
            border-radius: 50%;
            background: #28a745;
            animation: pulse 2s infinite;
        }

        @keyframes pulse {
            0% { opacity: 1; }
            50% { opacity: 0.5; }
            100% { opacity: 1; }
        }

        .card {
            background: white;
            border-radius: 15px;
            padding: 20px;
            margin-bottom: 20px;
            box-shadow: 0 5px 15px rgba(0, 0, 0, 0.1);
            border: 1px solid rgba(255, 255, 255, 0.2);
        }

        .card h2 {
            color: #333;
            margin-bottom: 20px;
            font-size: 1.3rem;
            display: flex;
            align-items: center;
            gap: 10px;
        }

        .slider-group {
            margin-bottom: 25px;
        }

        .slider-label {
            display: flex;
            justify-content: space-between;
            align-items: center;
            margin-bottom: 12px;
            flex-wrap: wrap;
            gap: 10px;
        }

        .slider-label span {
            font-weight: 600;
            color: #333;
            font-size: 1rem;
        }

        .value-display {
            background: linear-gradient(45deg, #667eea, #764ba2);
            color: white;
            padding: 8px 15px;
            border-radius: 20px;
            font-family: 'Courier New', monospace;
            font-weight: bold;
            min-width: 80px;
            text-align: center;
            font-size: 0.9rem;
        }

        .slider {
            width: 100%;
            height: 8px;
            border-radius: 5px;
            background: #ddd;
            outline: none;
            -webkit-appearance: none;
            transition: all 0.3s ease;
        }

        .slider::-webkit-slider-thumb {
            -webkit-appearance: none;
            appearance: none;
            width: 28px;
            height: 28px;
            border-radius: 50%;
            background: linear-gradient(45deg, #667eea, #764ba2);
            cursor: pointer;
            box-shadow: 0 4px 12px rgba(102, 126, 234, 0.4);
            transition: all 0.3s ease;
        }

        .slider::-webkit-slider-thumb:active {
            transform: scale(1.1);
        }

        .slider::-moz-range-thumb {
            width: 28px;
            height: 28px;
            border-radius: 50%;
            background: linear-gradient(45deg, #667eea, #764ba2);
            cursor: pointer;
            border: none;
            box-shadow: 0 4px 12px rgba(102, 126, 234, 0.4);
        }

        .update-pid-btn {
            background: linear-gradient(45deg, #28a745, #20c997);
            color: white;
            border: none;
            padding: 18px 30px;
            border-radius: 15px;
            cursor: pointer;
            font-weight: 700;
            font-size: 1.1rem;
            width: 100%;
            margin: 20px 0;
            transition: all 0.3s ease;
            touch-action: manipulation;
            box-shadow: 0 6px 20px rgba(40, 167, 69, 0.3);
        }

        .update-pid-btn:active {
            transform: scale(0.98);
            background: linear-gradient(45deg, #218838, #1ea383);
        }

        .quick-presets {
            display: grid;
            grid-template-columns: repeat(2, 1fr);
            gap: 10px;
            margin-top: 20px;
        }

        .preset-btn {
            background: linear-gradient(45deg, #667eea, #764ba2);
            color: white;
            border: none;
            padding: 15px 20px;
            border-radius: 12px;
            cursor: pointer;
            font-weight: 600;
            font-size: 0.9rem;
            transition: all 0.3s ease;
            touch-action: manipulation;
        }

        .preset-btn:active {
            transform: scale(0.95);
            background: linear-gradient(45deg, #5a6fd8, #6a42a0);
        }

        .status-grid {
            display: grid;
            grid-template-columns: repeat(2, 1fr);
            gap: 15px;
            margin-bottom: 20px;
        }

        .status-item {
            background: #f8f9fa;
            padding: 15px;
            border-radius: 10px;
            border-left: 4px solid #667eea;
            text-align: center;
        }

        .status-item label {
            font-weight: 600;
            color: #555;
            display: block;
            margin-bottom: 8px;
            font-size: 0.85rem;
        }

        .status-value {
            font-family: 'Courier New', monospace;
            font-size: 1.1rem;
            color: #333;
            font-weight: bold;
        }

        .sensor-section h3 {
            color: #333;
            margin-bottom: 15px;
            text-align: center;
            font-size: 1.1rem;
        }

        .sensor-visualization {
            display: grid;
            grid-template-columns: repeat(8, 1fr);
            gap: 3px;
            margin-bottom: 10px;
        }

        .sensor {
            aspect-ratio: 1;
            border-radius: 8px;
            background: #e9ecef;
            border: 2px solid #dee2e6;
            display: flex;
            align-items: center;
            justify-content: center;
            font-size: 10px;
            font-weight: bold;
            transition: all 0.3s ease;
        }

        .sensor.active {
            background: #333;
            color: white;
            border-color: #333;
            transform: scale(1.1);
        }

        .current-pid {
            background: rgba(102, 126, 234, 0.1);
            border-radius: 10px;
            padding: 15px;
            margin-top: 15px;
        }

        .current-pid h4 {
            color: #333;
            margin-bottom: 10px;
            text-align: center;
        }

        .pid-values {
            display: grid;
            grid-template-columns: repeat(3, 1fr);
            gap: 10px;
        }

        .pid-item {
            text-align: center;
            padding: 10px;
            background: white;
            border-radius: 8px;
        }

        .pid-item label {
            font-size: 0.8rem;
            color: #666;
            display: block;
        }

        .pid-item span {
            font-family: 'Courier New', monospace;
            font-weight: bold;
            color: #333;
            font-size: 0.9rem;
        }

        /* Landscape phone optimization */
        @media (max-width: 768px) and (orientation: landscape) {
            .container {
                padding: 15px;
            }
            
            .status-grid {
                grid-template-columns: repeat(4, 1fr);
                gap: 10px;
            }
            
            .status-item {
                padding: 10px;
            }
            
            .sensor-visualization {
                grid-template-columns: repeat(16, 1fr);
                gap: 2px;
            }
        }

        /* Very small screens */
        @media (max-width: 480px) {
            body {
                padding: 5px;
            }
            
            .container {
                padding: 15px;
                border-radius: 15px;
            }
            
            .header h1 {
                font-size: 1.5rem;
            }
            
            .card {
                padding: 15px;
            }
            
            .sensor {
                font-size: 8px;
            }
        }

        /* Touch improvements */
        .slider, .preset-btn, .update-pid-btn {
            touch-action: manipulation;
        }

        /* Prevent zoom on input focus */
        input, select, textarea {
            font-size: 16px;
        }
    </style>
</head>
<body>
    <div class="container">
        <div class="header">
            <h1>🤖 Line Follower PID</h1>
            <p>Real-time parameter tuning</p>
        </div>

        <div class="connection-status">
            <div class="status-dot" id="statusDot"></div>
            <span id="connectionText">Connected to ESP8266</span>
        </div>

        <div class="card">
            <h2>🎛️ PID Parameters</h2>
            
            <div class="slider-group">
                <div class="slider-label">
                    <span>Proportional (Kp)</span>
                    <div class="value-display" id="kpValue">15.000</div>
                </div>
                <input type="range" class="slider" id="kpSlider" min="0" max="50" step="0.1" value="15">
            </div>

            <div class="slider-group">
                <div class="slider-label">
                    <span>Integral (Ki)</span>
                    <div class="value-display" id="kiValue">0.000</div>
                </div>
                <input type="range" class="slider" id="kiSlider" min="0" max="5" step="0.01" value="0">
            </div>

            <div class="slider-group">
                <div class="slider-label">
                    <span>Derivative (Kd)</span>
                    <div class="value-display" id="kdValue">10.000</div>
                </div>
                <input type="range" class="slider" id="kdSlider" min="0" max="30" step="0.1" value="10">
            </div>

            <button class="update-pid-btn" id="updatePidBtn" onclick="sendPIDValues()">
                🚀 Update PID Values
            </button>

            <div class="quick-presets">
                <button class="preset-btn" onclick="setPreset('conservative')">Conservative</button>
                <button class="preset-btn" onclick="setPreset('balanced')">Balanced</button>
                <button class="preset-btn" onclick="setPreset('aggressive')">Aggressive</button>
                <button class="preset-btn" onclick="setPreset('reset')">Reset</button>
            </div>

            <div class="current-pid">
                <h4>📊 Active PID Values</h4>
                <div class="pid-values">
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
        </div>

        <div class="card">
            <h2>📊 Live Performance</h2>
            <div class="status-grid">
                <div class="status-item">
                    <label>Error</label>
                    <div class="status-value" id="currentError">0</div>
                </div>
                <div class="status-item">
                    <label>PID Output</label>
                    <div class="status-value" id="pidOutput">0.00</div>
                </div>
                <div class="status-item">
                    <label>Left Motor</label>
                    <div class="status-value" id="leftMotor">0</div>
                </div>
                <div class="status-item">
                    <label>Right Motor</label>
                    <div class="status-value" id="rightMotor">0</div>
                </div>
            </div>

            <div class="sensor-section">
                <h3>Sensor Array (0-15)</h3>
                <div class="sensor-visualization" id="sensorArray1">
                    <!-- First 8 sensors -->
                </div>
                <div class="sensor-visualization" id="sensorArray2">
                    <!-- Last 8 sensors -->
                </div>
            </div>
        </div>
    </div>

    <script>
        // PID parameters
        var pidParams = {
            kp: 15.0,
            ki: 0.0,
            kd: 10.0
        };

        // Initialize the interface
        document.addEventListener('DOMContentLoaded', function() {
            initializeSliders();
            initializeSensorDisplay();
            startStatusUpdates();
        });

        function initializeSliders() {
            var kpSlider = document.getElementById('kpSlider');
            var kiSlider = document.getElementById('kiSlider');
            var kdSlider = document.getElementById('kdSlider');

            kpSlider.addEventListener('input', function() {
                pidParams.kp = parseFloat(this.value);
                document.getElementById('kpValue').textContent = pidParams.kp.toFixed(3);
            });

            kiSlider.addEventListener('input', function() {
                pidParams.ki = parseFloat(this.value);
                document.getElementById('kiValue').textContent = pidParams.ki.toFixed(3);
            });

            kdSlider.addEventListener('input', function() {
                pidParams.kd = parseFloat(this.value);
                document.getElementById('kdValue').textContent = pidParams.kd.toFixed(3);
            });
        }

        function initializeSensorDisplay() {
            var sensorArray1 = document.getElementById('sensorArray1');
            var sensorArray2 = document.getElementById('sensorArray2');
            sensorArray1.innerHTML = '';
            sensorArray2.innerHTML = '';
            
            // First 8 sensors (0-7)
            for (var i = 0; i < 8; i++) {
                var sensor = document.createElement('div');
                sensor.className = 'sensor';
                sensor.id = 'sensor' + i;
                sensor.textContent = i;
                sensorArray1.appendChild(sensor);
            }
            
            // Last 8 sensors (8-15)
            for (var i = 8; i < 16; i++) {
                var sensor = document.createElement('div');
                sensor.className = 'sensor';
                sensor.id = 'sensor' + i;
                sensor.textContent = i;
                sensorArray2.appendChild(sensor);
            }
        }

        function setPreset(presetName) {
            var presets = {
                conservative: { kp: 5.0, ki: 0.0, kd: 2.0 },
                balanced: { kp: 15.0, ki: 0.0, kd: 10.0 },
                aggressive: { kp: 30.0, ki: 0.5, kd: 20.0 },
                reset: { kp: 0.0, ki: 0.0, kd: 0.0 }
            };

            var preset = presets[presetName];
            if (preset) {
                pidParams.kp = preset.kp;
                pidParams.ki = preset.ki;
                pidParams.kd = preset.kd;
                
                document.getElementById('kpSlider').value = preset.kp;
                document.getElementById('kiSlider').value = preset.ki;
                document.getElementById('kdSlider').value = preset.kd;
                
                document.getElementById('kpValue').textContent = preset.kp.toFixed(3);
                document.getElementById('kiValue').textContent = preset.ki.toFixed(3);
                document.getElementById('kdValue').textContent = preset.kd.toFixed(3);
            }
        }

        function sendPIDValues() {
            var updateBtn = document.getElementById('updatePidBtn');
            var originalText = updateBtn.textContent;
            updateBtn.textContent = '⏳ Updating...';
            updateBtn.disabled = true;

            fetch('/setPID', {
                method: 'POST',
                headers: {
                    'Content-Type': 'application/x-www-form-urlencoded',
                },
                body: 'kp=' + pidParams.kp + '&ki=' + pidParams.ki + '&kd=' + pidParams.kd
            })
            .then(function(response) { return response.text(); })
            .then(function(data) {
                console.log('PID values sent successfully');
                // Update active PID display
                document.getElementById('activekp').textContent = pidParams.kp.toFixed(3);
                document.getElementById('activeki').textContent = pidParams.ki.toFixed(3);
                document.getElementById('activekd').textContent = pidParams.kd.toFixed(3);
                
                updateBtn.textContent = '✅ Updated!';
                setTimeout(function() {
                    updateBtn.textContent = originalText;
                    updateBtn.disabled = false;
                }, 1500);
            })
            .catch(function(error) {
                console.error('Error sending PID values:', error);
                updateBtn.textContent = '❌ Error';
                setTimeout(function() {
                    updateBtn.textContent = originalText;
                    updateBtn.disabled = false;
                }, 1500);
            });
        }

        function updateStatus() {
            fetch('/getStatus')
            .then(function(response) { return response.json(); })
            .then(function(data) {
                document.getElementById('currentError').textContent = data.error;
                document.getElementById('pidOutput').textContent = data.pidOutput.toFixed(2);
                document.getElementById('leftMotor').textContent = data.leftSpeed;
                document.getElementById('rightMotor').textContent = data.rightSpeed;
                
                // Update sensor visualization
                updateSensorVisualization(data.sensorValues, data.position);
            })
            .catch(function(error) {
                console.error('Error fetching status:', error);
            });
        }

        function updateSensorVisualization(sensorValues, position) {
            for (var i = 0; i < 16; i++) {
                var sensor = document.getElementById('sensor' + i);
                if (sensor && sensorValues && sensorValues[i] !== undefined) {
                    // Show active sensors (above threshold)
                    var isActive = sensorValues[i] > 500;
                    if (isActive) {
                        sensor.classList.add('active');
                    } else {
                        sensor.classList.remove('active');
                    }
                }
            }
        }

        function startStatusUpdates() {
            setInterval(updateStatus, 200); // Update every 200ms
        }
        // Prevent zoom on double tap
        var lastTouchEnd = 0;
        document.addEventListener('touchend', function (event) {
            var now = (new Date()).getTime();
            if (now - lastTouchEnd <= 300) {
                event.preventDefault();
            }
            lastTouchEnd = now;
        }, false);
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
