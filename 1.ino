#include <Wire.h>              // Include the Wire library for I2C communication
#include <Adafruit_MPU6050.h>  // Include the Adafruit MPU6050 library
#include <Adafruit_Sensor.h>   // Include the Adafruit Sensor library
#include <Adafruit_MPU6050.h>  // Include the Adafruit MPU6050 library
#include <AFMotor.h>           // Include the Adafruit Motor Shield library

// Create an MPU6050 object
Adafruit_MPU6050 mpu;

// Motor control constants and objects
AF_DCMotor motor1(1);
AF_DCMotor motor2(2);
AF_DCMotor motor3(3);
AF_DCMotor motor4(4);

int speed = 235;  // Default motor speed

void setup() {
  Serial.begin(9600);  // Initialize serial communication at 115200 baud rate
  setMotorSpeed(speed);
  
  // Initialize MPU6050
  if (!mpu.begin()) {
    Serial.println("Failed to find MPU6050 chip");  // Print error message if MPU6050 is not found
    while (1)
      ;
  }
  Serial.println("MPU6050 Found!");  // Print success message if MPU6050 is found

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
  float speed = 0.5;                           // meters per second
  float timeToMove = distance / speed * 1000;  // time in milliseconds
  unsigned long startTime = millis();
  unsigned long correctionInterval = 500;      // Time interval for corrections in milliseconds
  unsigned long lastCorrectionTime = millis();

  // Set the speed of the left and right motors to maximum
  motor1.run(FORWARD);
  motor2.run(FORWARD);
  motor3.run(FORWARD);
  motor4.run(FORWARD);

  // Move the robot forward for the specified distance
  while (millis() - startTime < timeToMove) {
    if (millis() - lastCorrectionTime >= correctionInterval) {
      turnRobot(0);  // Correct the orientation to 0 degrees
      lastCorrectionTime = millis();
    }
  }

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
    float angularVelocityZ = g.gyro.z;  // Gyroscope z-axis in rad/s

    // Integrate angular velocity to calculate angle
    totalAngle += angularVelocityZ * deltaTime * (180 / PI);  // Convert to degrees

    Serial.print("Current Angle: ");
    Serial.println(totalAngle);

    // Control motors to turn the robot
    if (targetAngle > 0) {
      // Right turn
      motor1.run(FORWARD);
      motor2.run(BACKWARD);
      motor3.run(BACKWARD);
      motor4.run(FORWARD);
    } else if (targetAngle < 0) {
      // Left turn
      motor1.run(BACKWARD);
      motor2.run(FORWARD);
      motor3.run(FORWARD);
      motor4.run(BACKWARD);
    } else {
      // Stop turning
      stopMotors();
      break;
    }
  }
  stopMotors();
}

void stopMotors() {
  motor1.run(RELEASE);
  motor2.run(RELEASE);
  motor3.run(RELEASE);
  motor4.run(RELEASE);
}

void setMotorSpeed(int speed) {
  motor1.setSpeed(speed);
  motor2.setSpeed(speed);
  motor3.setSpeed(speed);
  motor4.setSpeed(speed);
}


// Main code
void loop() {
  for (int i = 0; i < 5; i++) {
    delay(1000);
    moveForward(3.5);
    delay(1500);
    turnRobot(90);     // Turn 90 degrees right
    delay(1000);       // Wait for 1 second
    moveForward(0.6);  // Move forward 30 cm
    turnRobot(90);     // Turn 90 degrees right
    delay(1000);       // Wait for 1 second
    moveForward(3.5);  // Move forward 1 meter
    turnRobot(-90);    // Turn 90 degrees left
    delay(1000);       // Wait for 1 second
    moveForward(0.6);  // Move forward 30 cm
    turnRobot(-90);    // Turn 90 degrees left
    delay(1000);       // Wait for 1 second     
  }
  delay(10000);  // Wait for 10 seconds before repeating the loop
}