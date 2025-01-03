#include <Wire.h>
#include <MPU6050.h>
#include <QMC5883LCompass.h>
#include <AFMotor.h>

// Initialize sensors
MPU6050 mpu;
QMC5883LCompass compass;

// Initialize motors
AF_DCMotor motor1(1); // Front left
AF_DCMotor motor2(2); // Front right
AF_DCMotor motor3(3); // Back right
AF_DCMotor motor4(4); // Back left

// Motor speed control constants
int baseSpeed = 150; // Base motor speed (range: 0-255)
float targetHeading; // Desired direction

void setup() {
  Serial.begin(9600);
  Wire.begin();

  // Initialize MPU6050
  mpu.initialize();
  if (!mpu.testConnection()) {
    Serial.println("MPU6050 connection failed");
    while (1);
  }

  // Initialize HMC5883L
  compass.init();
  compass.setCalibrationOffsets(587.00, -398.00, 16383.00);
compass.setCalibrationScales(9.86, 14.55, 0.35);

  // Set target heading (straight line)
  delay(1000);
  compass.read();
  targetHeading = compass.getAzimuth();

  // Start motors
  motor1.setSpeed(baseSpeed);
  motor2.setSpeed(baseSpeed);
  motor3.setSpeed(baseSpeed);
  motor4.setSpeed(baseSpeed);
  motor1.run(FORWARD);
  motor2.run(FORWARD);
  motor3.run(FORWARD);
  motor4.run(FORWARD);
}

void loop() {
  // Read current heading from the compass
  compass.read();
  float currentHeading = compass.getAzimuth();

  // Calculate the heading error
  float error = targetHeading - currentHeading;
  if (error < -180) error += 360;
  if (error > 180) error -= 360;

  // Adjust motor speeds based on the error
  int speedAdjust = (int)(error * 0.5); // Proportional control factor
  int leftSpeed = baseSpeed - speedAdjust;
  int rightSpeed = baseSpeed + speedAdjust;

  // Constrain motor speeds to valid range
  leftSpeed = constrain(leftSpeed, 0, 255);
  rightSpeed = constrain(rightSpeed, 0, 255);

  // Update motor speeds
  motor1.setSpeed(leftSpeed);
  motor4.setSpeed(leftSpeed);
  motor2.setSpeed(rightSpeed);
  motor3.setSpeed(rightSpeed);

  // Debugging output
  Serial.print("Target Heading: ");
  Serial.print(targetHeading);
  Serial.print(" | Current Heading: ");
  Serial.print(currentHeading);
  Serial.print(" | Error: ");
  Serial.print(error);
  Serial.print(" | Left Speed: ");
  Serial.print(leftSpeed);
  Serial.print(" | Right Speed: ");
  Serial.println(rightSpeed);

  delay(100); // Loop delay
}
