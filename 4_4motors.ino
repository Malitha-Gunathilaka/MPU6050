#include <Wire.h> // Include the Wire library for I2C communication
#include <Adafruit_MPU6050.h>   // Include the Adafruit MPU6050 library
#include <Adafruit_Sensor.h>    // Include the Adafruit Sensor library
#include <Adafruit_MPU6050.h>   // Include the Adafruit MPU6050 library
#include <AFMotor.h>    // Include the Adafruit Motor Shield library

// Create an MPU6050 object
Adafruit_MPU6050 mpu;

// Create motor objects
AF_DCMotor leftMotor1(1);  // Connect left motor to port 1
AF_DCMotor rightMotor1(2); // Connect right motor to port 2
AF_DCMotor leftMotor2(4);  // Connect left motor to port 4
AF_DCMotor rightMotor2(3); // Connect right motor to port 3


void setup() {
  Serial.begin(115200);// Initialize serial communication at 115200 baud rate

  // Initialize MPU6050
  if (!mpu.begin()) {
    Serial.println("Failed to find MPU6050 chip"); // Print error message if MPU6050 is not found
    while (1);
  }
  Serial.println("MPU6050 Found!");// Print success message if MPU6050 is found

  // Set accelerometer range (optional)
  mpu.setAccelerometerRange(MPU6050_RANGE_2_G);
  Serial.print("Accelerometer range set to: ");
  Serial.println(mpu.getAccelerometerRange());

  // Set gyroscope range (optional)
  mpu.setGyroRange(MPU6050_RANGE_250_DEG);
  Serial.print("Gyro range set to: ");
  Serial.println(mpu.getGyroRange());

  // Set filter bandwidth (optional)
  mpu.setFilterBandwidth(MPU6050_BAND_21_HZ);
  Serial.print("Filter bandwidth set to: ");
  Serial.println(mpu.getFilterBandwidth());
}

void moveForward(float distance) {
  // Assuming the robot moves at a speed of 0.5 meters per second
  float speed = 0.5; // meters per second
  float timeToMove = distance / speed * 1000; // time in milliseconds

    // Set the speed of the left and right motors to maximum
  leftMotor1.setSpeed(255);
  leftMotor1.run(FORWARD);
  rightMotor1.setSpeed(255);
  rightMotor1.run(FORWARD);
  //
  leftMotor2.setSpeed(255);
  leftMotor2.run(FORWARD);
  rightMotor2.setSpeed(255);
  rightMotor2.run(FORWARD);
// Move the robot forward for the specified distance
  delay(timeToMove);
// Stop the motors after moving forward
  stopMotors();
}

// Function to turn the robot by a specified angle
void turnRobot(int targetAngle) {
  sensors_event_t a, g, temp;
  mpu.getEvent(&a, &g, &temp);

  // Initialize the total angle rotated by the robot  
  float totalAngle = 0;
  unsigned long prevTime = millis();

// Continue turning the robot until the target angle is reached
  while (abs(totalAngle) < abs(targetAngle)) {
    unsigned long currentTime = millis();
    float deltaTime = (currentTime - prevTime) / 1000.0;
    prevTime = currentTime;

    // Get gyro data (z-axis for yaw rotation)
    mpu.getEvent(&a, &g, &temp);
    float angularVelocityZ = g.gyro.z; // Gyroscope z-axis in rad/s

    // Integrate angular velocity to calculate angle
    totalAngle += angularVelocityZ * deltaTime * (180 / PI); // Convert to degrees

    Serial.print("Current Angle: ");
    Serial.println(totalAngle);

    // Control motors to turn the robot
    if (targetAngle > 0) {
      // Right turn
      leftMotor1.setSpeed(255);
      leftMotor1.run(FORWARD);
      rightMotor1.setSpeed(255);
      rightMotor1.run(BACKWARD);
      //
      leftMotor2.setSpeed(255);
      leftMotor2.run(FORWARD);
      rightMotor2.setSpeed(255);
      rightMotor2.run(BACKWARD);
    } else {
      // Left turn
      leftMotor1.setSpeed(255);
      leftMotor1.run(BACKWARD);
      rightMotor1.setSpeed(255);
      rightMotor1.run(FORWARD);
      //
      leftMotor2.setSpeed(255);
      leftMotor2.run(BACKWARD);
      rightMotor2.setSpeed(255);
      rightMotor2.run(FORWARD);
    }
  }
  stopMotors();
}

void stopMotors() {
  leftMotor1.run(RELEASE);
  rightMotor1.run(RELEASE);
  leftMotor2.run(RELEASE);
  rightMotor2.run(RELEASE);
}
// Main code
void loop() {
  moveForward(1.0);  // Move forward 1 meter
  turnRobot(90);     // Turn 90 degrees right
  delay(1000);       // Wait for 1 second
  moveForward(0.5);  // Move forward 1 meter
  delay(1000);       // Wait for 1 second
  turnRobot(-90);    // Turn 90 degrees left
  moveForward(1.0);  // Move forward 1 meter
  delay(10000);      // Wait for 10 seconds
}