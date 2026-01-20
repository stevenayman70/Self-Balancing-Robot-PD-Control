#include <Arduino_LSM6DS3.h>

// --- PIN DEFINITIONS ---
// Right Motor
#define PWM1_1 9  // Direction
#define PWM2_1 10 // Speed

// Left Motor
#define PWM1_2 3  // Direction
#define PWM2_2 5  // Speed

// --- PID CONSTANTS (Aggressive Tuning) ---
// Increased for stiffness. If it shakes violently, lower Kp to 35.
float Kp = 40.0; 
float Ki = 0.00; // Small amount to fix leaning over time
float Kd = 1.0;  // High damping to resist pushes

// --- VARIABLES ---
float targetAngle = 0.0; // Set dynamically in setup()
float currentAngle = 0.0;
float gyroRate = 0.0;
float error, lastError, errorSum;
unsigned long lastTime, currentTime;
float dt; 
float alpha = 0.98; // Complementary filter weight

void setup() {
  Serial.begin(115200);
  
  // 1. Initialize Pins
  pinMode(PWM1_1, OUTPUT); pinMode(PWM2_1, OUTPUT);
  pinMode(PWM1_2, OUTPUT); pinMode(PWM2_2, OUTPUT);

  // 2. Initialize IMU
  if (!IMU.begin()) {
    Serial.println("IMU Failed!");
    while (1);
  }

  // 3. AUTO-CALIBRATION ROUTINE
  // IMPORTANT: Hold the robot perfectly balanced when you press Reset!
  Serial.println("Calibrating... HOLD ROBOT UPRIGHT!");
  delay(1000); // Give you a second to steady your hand
  
  float sumAngle = 0;
  for(int i=0; i<100; i++) {
    float ax, ay, az;
    if (IMU.accelerationAvailable()) {
      IMU.readAcceleration(ax, ay, az);
      // Calculating angle based on your Y/Z axis data
      sumAngle += atan2(ay, az) * 180.0 / PI; 
    }
    delay(5);
  }
  targetAngle = sumAngle / 100.0;
  
  Serial.print("Calibration Complete. Target Angle: "); 
  Serial.println(targetAngle);
  
  lastTime = millis();
}

void loop() {
  // 1. Calculate Time Step (dt)
  currentTime = millis();
  if (currentTime - lastTime < 5) return; // Run loop every 5ms
  dt = (currentTime - lastTime) / 1000.0;
  lastTime = currentTime;

  // 2. Read Sensors
  float accX, accY, accZ;
  float gyrX, gyrY, gyrZ;

  if (IMU.accelerationAvailable() && IMU.gyroscopeAvailable()) {
    IMU.readAcceleration(accX, accY, accZ);
    IMU.readGyroscope(gyrX, gyrY, gyrZ);

    // Calculate Accelerometer Angle
    // Using atan2(Y, Z) because your data showed Y changing with tilt
    float accAngle = atan2(accY, accZ) * 180.0 / PI;
    
    // Get Gyro Rate (X-axis rotation corresponds to Y-axis tilt)
    gyroRate = gyrX; 

    // Complementary Filter: Combine stable Accel with fast Gyro
    currentAngle = alpha * (currentAngle + gyroRate * dt) + (1.0 - alpha) * accAngle;
  }

  // 3. Safety Cutoff
  // If robot falls > 40 degrees, kill motors
  if (abs(currentAngle - targetAngle) > 40) {
    stopMotors();
    errorSum = 0; 
  } else {
    // 4. PID Calculation
    error = targetAngle - currentAngle; // How far are we from "Zero"?
    
    // Integral (Sum of errors to fix small leans)
    errorSum += error * dt;
    errorSum = constrain(errorSum, -200, 200); // Prevent windup
    
    // Derivative (Rate of change - resists the push)
    float dError = (error - lastError) / dt;

    // Standard PID Output
    float pidOutput = (Kp * error) + (Ki * errorSum) + (Kd * dError);
    
    // 5. AGGRESSIVE BOOST (For 200 RPM Motors)
    // If the error is large (> 8 degrees), add a static boost to help motors catch up
    if (error > 8) pidOutput += 40;
    if (error < -8) pidOutput -= 40;

    lastError = error;

    // 6. Drive Motors
    driveMotors(pidOutput);
  }
  
  // Debugging: View this in Serial Plotter
  // We plot (current - target) so "0" is always the center line
  // Serial.print("Error:"); Serial.println(currentAngle - targetAngle);
}

// --- MOTOR FUNCTIONS ---

void driveMotors(float output) {
  // Constrain to PWM range
  int pwm = constrain(output, -255, 255);
  
  // Deadzone Compensation: 
  // Motors usually won't turn below PWM 30. We force them to at least 35.
  if (pwm > 0 && pwm < 35) pwm = 35;
  if (pwm < 0 && pwm > -35) pwm = -35;

  setSingleMotor(PWM1_1, PWM2_1, pwm); // Right
  setSingleMotor(PWM1_2, PWM2_2, pwm); // Left
}

// Your specific driver logic (Inverted PWM for Reverse)
void setSingleMotor(int dirPin, int speedPin, int speed) {
  if (speed > 0) {
    // Forward
    digitalWrite(dirPin, LOW);
    analogWrite(speedPin, speed);
  } else {
    // Backward
    digitalWrite(dirPin, HIGH);
    // Invert speed logic: 255 is slow, 0 is fast in this mode? 
    // Usually logic is: HIGH + 255 = Stopped, HIGH + 0 = Full Speed Reverse.
    // Your previous code used: 255 - abs(speed)
    int invertedSpeed = 255 - abs(speed);
    analogWrite(speedPin, invertedSpeed);
  }
}

void stopMotors() {
  digitalWrite(PWM1_1, LOW); analogWrite(PWM2_1, 0);
  digitalWrite(PWM1_2, LOW); analogWrite(PWM2_2, 0);
}