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
  compass.setCalibrationOffsets(655.00, -1604.00, 16383.00);
  compass.setCalibrationScales(9.49, 3.87, 0.38);

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

void moveForward(float distance) {
  // Fix the target heading before moving
  compass.read();
  targetHeading = compass.getAzimuth();
  Serial.print("Target Heading: ");
  Serial.println(targetHeading);

  float speed = 0.5; // meters per second
  float timeToMove = distance / speed * 1000; // time in milliseconds
  unsigned long startTime = millis();

  while (millis() - startTime < timeToMove) {
    compass.read();
    float currentHeading = compass.getAzimuth();
    float error = targetHeading - currentHeading;
    if (error < -180) error += 360;
    if (error > 180) error -= 360;

    turnRobot(error);

    int leftSpeed = baseSpeed;
    int rightSpeed = baseSpeed;

    leftSpeed = constrain(leftSpeed, 0, 255);
    rightSpeed = constrain(rightSpeed, 0, 255);

    motor1.setSpeed(leftSpeed);
    motor4.setSpeed(leftSpeed);
    motor2.setSpeed(rightSpeed);
    motor3.setSpeed(rightSpeed);

    delay(100); // Loop delay
  }

  stopMotors();

  // Print the heading after moving
  compass.read();
  float afterMoveHeading = compass.getAzimuth();
  float errorAfterMove = targetHeading - afterMoveHeading;
  if (errorAfterMove < -180) errorAfterMove += 360;
  if (errorAfterMove > 180) errorAfterMove -= 360;

  Serial.print("After Move Heading: ");
  Serial.println(afterMoveHeading);
  Serial.print("Error: ");
  Serial.println(errorAfterMove);
}

void turnRobot(float targetAngle) {
  float startHeading = compass.getAzimuth();
  float targetHeading = startHeading + targetAngle;

  if (targetHeading > 360) targetHeading -= 360;
  if (targetHeading < 0) targetHeading += 360;

  while (true) {
    compass.read();
    float currentHeading = compass.getAzimuth();
    float error = targetHeading - currentHeading;

    if (abs(error) < 2) break;

    if (error > 0) {
      motor1.run(FORWARD);
      motor2.run(BACKWARD);
      motor3.run(BACKWARD);
      motor4.run(FORWARD);
    } else {
      motor1.run(BACKWARD);
      motor2.run(FORWARD);
      motor3.run(FORWARD);
      motor4.run(BACKWARD);
    }

    delay(10);
  }

  stopMotors();
}

void stopMotors() {
  motor1.run(RELEASE);
  motor2.run(RELEASE);
  motor3.run(RELEASE);
  motor4.run(RELEASE);
}

void loop() {
  moveForward(1.0); // Move forward 1 meter
  delay(1000); // Wait for 1 second
  turnRobot(90); // Turn 90 degrees right
  delay(1000); // Wait for 1 second
  moveForward(1.0); // Move forward 1 meter
  delay(1000); // Wait for 1 second
}