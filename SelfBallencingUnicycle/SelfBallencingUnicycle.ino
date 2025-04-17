// Adafruit Motor shield library
#include <Wire.h>
#include <AFMotor.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <math.h> // For atan2 and pow

#define SPEED_BUFFER 0
#define ANGLE_DEADZONE 0.3  // Deadzone in degrees (+/- from setpoint)

AF_DCMotor motorX(1); // Motor on M1 for X-axis correction
AF_DCMotor motorY(4); // Motor on M2 for Y-axis correction

Adafruit_MPU6050 mpu;

// PID Constants (tune these!)
double KpX = 4;
double KiX = 2.8;
double KdX = 0.22;

double KpY = 12;
double KiY = 0.02;
double KdY = 0.65;

// PID Variables
// double setpointX = 1;
// double setpointY = 0.5;
double setpointX = 0;
double setpointY = 0;

double inputX, inputY, outputX, outputY;
double errorX, errorY, lastErrorX, lastErrorY, integralX, integralY, derivativeX, derivativeY;

unsigned long lastTime;

// Add offset variables for calibration
double offsetX = 0;
double offsetY = 0;

void setup() {
  Serial.begin(115200);
  Serial.println("MPU6050 with PID Motor Control");
  Wire.begin();

  if (!mpu.begin()) {
    Serial.println("Failed to find MPU6050 chip");
    while (1) {
      delay(10);
    }
  }
  Serial.println("MPU6050 Found!");

  mpu.setAccelerometerRange(MPU6050_RANGE_8_G);
  Serial.println("MPU: Accelerometer range set to: +-8G");
  mpu.setGyroRange(MPU6050_RANGE_500_DEG);
  Serial.println("MPU: Gyro range set to: +- 500 deg/s");
  mpu.setFilterBandwidth(MPU6050_BAND_21_HZ);
  Serial.println("MPU: Filter bandwidth set to: 21 Hz");
  
  // Calibration process
  Serial.println("Calibrating sensor... Please keep the device still");
  
  // Take multiple readings to get a stable average
  const int numReadings = 100;
  double sumX = 0;
  double sumY = 0;
  
  for (int i = 0; i < numReadings; i++) {
    sensors_event_t a, g, temp;
    mpu.getEvent(&a, &g, &temp);
    
    // Calculate the angles the same way we do in the loop
    double angleX = atan2(a.acceleration.y, sqrt(pow(a.acceleration.x, 2) + pow(a.acceleration.z, 2))) * 180 / M_PI;
    double angleY = atan2(-a.acceleration.x, sqrt(pow(a.acceleration.y, 2) + pow(a.acceleration.z, 2))) * 180 / M_PI;
    
    sumX += angleX;
    sumY += angleY;
    delay(10);
  }
  
  // Calculate average offsets
  offsetX = sumX / numReadings;
  offsetY = sumY / numReadings;
  
  Serial.print("Calibration complete. X offset: ");
  Serial.print(offsetX);
  Serial.print(", Y offset: ");
  Serial.println(offsetY);
  Serial.println("");

  motorX.setSpeed(0);
  motorX.run(RELEASE);
  motorY.setSpeed(0);
  motorY.run(RELEASE);

  lastTime = millis();
}

void loop() {
  sensors_event_t a, g, temp;
  mpu.getEvent(&a, &g, &temp);

  // Calculate raw tilt angles (in degrees)
  double rawX = atan2(a.acceleration.y, sqrt(pow(a.acceleration.x, 2) + pow(a.acceleration.z, 2))) * 180 / M_PI;
  double rawY = atan2(-a.acceleration.x, sqrt(pow(a.acceleration.y, 2) + pow(a.acceleration.z, 2))) * 180 / M_PI;
  
  // Apply calibration offsets
  inputX = rawX - offsetX;
  inputY = rawY - offsetY;

  // Run the PID control for both axes
  controlPID(inputX, setpointX, KpX, KiX, KdX, errorX, lastErrorX,
            integralX, derivativeX, outputX, motorX, true);

  controlPID(inputY, setpointY, KpY, KiY, KdY, errorY, lastErrorY,
            integralY, derivativeY, outputY, motorY, false);

  // Print all PID-related values for the serial plotter
  Serial.print("X-Angle:");
  Serial.print(inputX);
  Serial.print(",");
  Serial.print("Y-Angle:");
  Serial.print(inputY);
  Serial.print(",");
  Serial.print("X-Error:");
  Serial.print(errorX);
  Serial.print(",");
  Serial.print("Y-Error:");
  Serial.print(errorY);
  Serial.print(",");
  Serial.print("X-Output:");
  Serial.print(outputX);
  Serial.print(",");
  Serial.print("Y-Output:");
  Serial.println(outputY);

  lastErrorX = errorX;
  lastErrorY = errorY;

  // Limit integral term to prevent windup
  integralX = constrain(integralX, -100, 100); // Adjust limits as needed
  integralY = constrain(integralY, -100, 100);

  delay(10); // Adjust delay as needed
}

// Modified PID control function to ensure output values are properly updated even in deadzone
void controlPID(double input, double setpoint, double Kp, double Ki, double Kd,
                double &error, double &lastError, double &integral, double &derivative,
                double &output, AF_DCMotor &motor, bool flipped) {

  unsigned long now = millis();
  double timeChange = (double)(now - lastTime);

  error = setpoint - input;
  
  integral += error * timeChange / 1000.0; // Integral over time
  derivative = (input - lastError) / (timeChange / 1000.0); // Rate of change

  output = Kp * error + Ki * integral + Kd * derivative;
  
  // Apply deadzone - if error is small enough, consider it zero
  if (abs(error) <= ANGLE_DEADZONE) {
    motor.run(RELEASE);
    motor.setSpeed(0);
    return; // Exit function but we've already calculated output for plotting
  }

  // Limit the output to the motor's speed range (0-255)
  int motorSpeed = constrain(abs(output), 0, 225);

  if (output > 0 + SPEED_BUFFER) {
    if (flipped) motor.run(FORWARD);
    else motor.run(BACKWARD);
    motor.setSpeed(motorSpeed);
  } else if (output < 0 - SPEED_BUFFER) {
    if (flipped) motor.run(BACKWARD);
    else motor.run(FORWARD);
    motor.setSpeed(motorSpeed);
  } else {
    motor.run(RELEASE);
    motor.setSpeed(0);
  }

  lastTime = now;
}