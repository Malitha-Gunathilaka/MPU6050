#include <Wire.h>
#include <QMC5883LCompass.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <AFMotor.h>

// Motor control objects
AF_DCMotor motor1(1);
AF_DCMotor motor2(2);
AF_DCMotor motor3(3);
AF_DCMotor motor4(4);

// Sensor objects
Adafruit_MPU6050 mpu;
QMC5883LCompass compass;

// Variables
int motorSpeed = 235;
float initialHeading = 0;

void setup() {
  Serial.begin(9600);
  setMotorSpeed(motorSpeed);

  // Initialize MPU6050
  if (!mpu.begin()) {
    Serial.println("Failed to find MPU6050 chip");
    while (1);
  }
  Serial.println("MPU6050 Initialized!");

  // Initialize Magnetometer
  compass.init();
  
  // Optionally set any calibration settings here, if supported by the library
  // For example: compass.setMode(Mode_Continuous);

  // Record initial heading
  initialHeading = getHeading();
  Serial.print("Initial Heading: ");
  Serial.println(initialHeading);
}

void loop() {
  for (int i = 0; i < 1; i++) {
  moveForward(1.0);  // Move forward 1 meter
  turnRobot(90);     // Turn 90 degrees right
  delay(1000);
  moveForward(0.5);  // Move forward 50 cm
  turnRobot(-90);    // Turn 90 degrees left
  delay(1000);
  }
}

// Get current heading using magnetometer
float getHeading() {
  int x, y, z;
  compass.read();
  x = compass.getX();
  y = compass.getY();
  z = compass.getZ();
  float heading = atan2(y, x) * 180 / PI;
  if (heading < 0) heading += 360; // Normalize to 0-360 degrees
  return heading;
}

// Move forward while maintaining the initial heading
void moveForward(float distance) {
  float speed = 0.5;                         // Speed in meters per second
  float timeToMove = (distance / speed) * 1000; // Time in milliseconds
  unsigned long startTime = millis();

  while (millis() - startTime < timeToMove) {
    float currentHeading = getHeading();
    float error = initialHeading - currentHeading;

    // Correct the heading
    if (error > 1) {
      motor1.run(BACKWARD);
      motor2.run(FORWARD);
      motor3.run(FORWARD);
      motor4.run(BACKWARD);
    } else if (error < -1) {
      motor1.run(FORWARD);
      motor2.run(BACKWARD);
      motor3.run(BACKWARD);
      motor4.run(FORWARD);
    } else {
      motor1.run(FORWARD);
      motor2.run(FORWARD);
      motor3.run(FORWARD);
      motor4.run(FORWARD);
    }

    setMotorSpeed(motorSpeed);
  }
  stopMotors();
}

// Turn the robot by a specified angle
void turnRobot(int targetAngle) {
  float currentHeading = getHeading();
  float targetHeading = currentHeading + targetAngle;
  if (targetHeading >= 360) targetHeading -= 360;
  if (targetHeading < 0) targetHeading += 360;

  while (true) {
    currentHeading = getHeading();
    float error = targetHeading - currentHeading;

    if (abs(error) < 2) break; // Stop when within 2 degrees of target

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

    setMotorSpeed(230); // Slower for precision
  }
  stopMotors();
}

// Stop all motors
void stopMotors() {
  motor1.run(RELEASE);
  motor2.run(RELEASE);
  motor3.run(RELEASE);
  motor4.run(RELEASE);
}

// Set speed for all motors
void setMotorSpeed(int speed) {
  motor1.setSpeed(speed);
  motor2.setSpeed(speed);
  motor3.setSpeed(speed);
  motor4.setSpeed(speed);
}
