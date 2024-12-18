#include <Wire.h>               // Include the Wire library for I2C communication
#include <Adafruit_MPU6050.h>   // Include the Adafruit MPU6050 library
#include <Adafruit_Sensor.h>    // Include the Adafruit Sensor library
#include <AFMotor.h>            // Include the Adafruit Motor Shield library
#include <QMC5883LCompass.h>    // Include the QMC5883L Compass library

// Create MPU6050 and Compass objects
Adafruit_MPU6050 mpu;
QMC5883LCompass compass;

// Motor control objects
AF_DCMotor motor1(1);
AF_DCMotor motor2(2);
AF_DCMotor motor3(3);
AF_DCMotor motor4(4);

// Motor speed
int speed = 235;

void setup() {
  Serial.begin(9600);  // Initialize serial communication

  // Set motor speed
  setMotorSpeed(speed);

  // Initialize MPU6050
  if (!mpu.begin()) {
    Serial.println("Failed to find MPU6050 chip");
    while (1);
  }
  Serial.println("MPU6050 Found!");

  // Configure MPU6050
  mpu.setAccelerometerRange(MPU6050_RANGE_2_G);
  mpu.setGyroRange(MPU6050_RANGE_250_DEG);
  mpu.setFilterBandwidth(MPU6050_BAND_21_HZ);

  // Initialize Compass
  compass.init();
  compass.setCalibration(-430, 430, -580, 620, -650, 600);  // Example calibration values
  Serial.println("Compass initialized.");
}

// Function to get the current compass heading
float getHeading() {
  compass.read();
  float heading = compass.getAzimuth();  // Get heading in degrees
  return heading;
}

// Function to move the robot forward while maintaining the heading
void moveForward(float distance) {
  float speed = 0.5;  // meters per second
  float timeToMove = distance / speed * 1000;  // time in milliseconds

  float targetHeading = getHeading();  // Get the initial heading
  unsigned long startTime = millis();

  motor1.run(FORWARD);
  motor2.run(FORWARD);
  motor3.run(FORWARD);
  motor4.run(FORWARD);

  while (millis() - startTime < timeToMove) {
    float currentHeading = getHeading();
    float headingError = targetHeading - currentHeading;

    // Adjust motor speeds to correct heading
    if (headingError > 2) {
      motor1.setSpeed(speed - 20);  // Reduce speed on one side
      motor2.setSpeed(speed - 20);
      motor3.setSpeed(speed);
      motor4.setSpeed(speed);
    } else if (headingError < -2) {
      motor1.setSpeed(speed);
      motor2.setSpeed(speed);
      motor3.setSpeed(speed - 20);  // Reduce speed on the other side
      motor4.setSpeed(speed - 20);
    } else {
      setMotorSpeed(speed);  // Restore normal speed
    }

    delay(50);  // Small delay to stabilize the control loop
  }

  stopMotors();  // Stop motors after moving forward
}

// Function to turn the robot by a specified angle
void turnRobot(int targetAngle) {
  sensors_event_t a, g, temp;
  mpu.getEvent(&a, &g, &temp);

  float totalAngle = 0;
  unsigned long prevTime = millis();

  while (abs(totalAngle) < abs(targetAngle)) {
    unsigned long currentTime = millis();
    float deltaTime = (currentTime - prevTime) / 1000.0;
    prevTime = currentTime;

    mpu.getEvent(&a, &g, &temp);
    float angularVelocityZ = g.gyro.z;  // Gyroscope z-axis in rad/s

    totalAngle += angularVelocityZ * deltaTime * (180 / PI);  // Convert to degrees

    Serial.print("Current Angle: ");
    Serial.println(totalAngle);

    if (targetAngle > 0) {
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
  }
  stopMotors();
}

// Function to stop all motors
void stopMotors() {
  motor1.run(RELEASE);
  motor2.run(RELEASE);
  motor3.run(RELEASE);
  motor4.run(RELEASE);
}

// Function to set speed for all motors
void setMotorSpeed(int speed) {
  motor1.setSpeed(speed);
  motor2.setSpeed(speed);
  motor3.setSpeed(speed);
  motor4.setSpeed(speed);
}

// Main loop
void loop() {
  for (int i = 0; i < 5; i++) {
    delay(1000);
    moveForward(1.0);  // Move forward 1 meter
    turnRobot(90);     // Turn 90 degrees right
    delay(1000);
    moveForward(0.3);  // Move forward 30 cm
    turnRobot(90);     // Turn 90 degrees right
    delay(1000);
    moveForward(1.0);  // Move forward 1 meter
    turnRobot(-90);    // Turn 90 degrees left
    delay(1000);
    moveForward(0.3);  // Move forward 30 cm
    turnRobot(-90);    // Turn 90 degrees left
    delay(1000);
    moveForward(1.0);  // Move forward 1 meter
    delay(1000);
  }
  delay(10000);  // Wait for 10 seconds before repeating the loop
}
