// Adafruit Motor shield library
#include <Wire.h>
#include <AFMotor.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <math.h> // For atan2 and pow

#define SPEED_BUFFER 0

AF_DCMotor motorX(1); // Motor on M1 for X-axis correction
AF_DCMotor motorY(4); // Motor on M2 for Y-axis correction

Adafruit_MPU6050 mpu;

// PID Constants (tune these!)
double KpX = 4;
double KiX = 1.8;
double KdX = 0.2;

double KpY = 0.1;
double KiY = 0.1;
double KdY = 0.000;

// PID Variables
// double setpointX = 1;
// double setpointY = 0.5;
double setpointX = 0;
double setpointY = 0;

double inputX, inputY, outputX, outputY;
double errorX, errorY, lastErrorX, lastErrorY, integralX, integralY, derivativeX, derivativeY;

unsigned long lastTime;

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

  // Calculate tilt angles (in degrees)
  inputX = atan2(a.acceleration.y, sqrt(pow(a.acceleration.x, 2) + pow(a.acceleration.z, 2))) * 180 / M_PI;
  inputY = atan2(-a.acceleration.x, sqrt(pow(a.acceleration.y, 2) + pow(a.acceleration.z, 2))) * 180 / M_PI;

  // Print the angles for the serial plotter
  Serial.print("X: "); // Identifier for Angle X
  Serial.print(" ");
  Serial.print(inputX);
  Serial.print(" ");

  Serial.print("Y: "); // Identifier for Angle Y
  Serial.print(" ");
  Serial.println(inputY);

  // Run the PID control for both axes
  controlPID(inputX, setpointX, KpX, KiX, KdX, errorX, lastErrorX,
            integralX, derivativeX, outputX, motorX, true);

  controlPID(inputY, setpointY, KpY, KiY, KdY, errorY, lastErrorY,
            integralY, derivativeY, outputY, motorY, false);

  lastErrorX = errorX;
  lastErrorY = errorY;

  // Limit integral term to prevent windup
  integralX = constrain(integralX, -100, 100); // Adjust limits as needed
  integralY = constrain(integralY, -100, 100);

  delay(10); // Adjust delay as needed
}

// PID control function
void controlPID(double input, double setpoint, double Kp, double Ki, double Kd,
                double &error, double &lastError, double &integral, double &derivative,
                double &output, AF_DCMotor &motor, bool flipped) {

  unsigned long now = millis();
  double timeChange = (double)(now - lastTime);

  error = setpoint - input;
  integral += error * timeChange / 1000.0; // Integral over time
  derivative = (input - lastError) / (timeChange / 1000.0); // Rate of change

  output = Kp * error + Ki * integral + Kd * derivative;

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