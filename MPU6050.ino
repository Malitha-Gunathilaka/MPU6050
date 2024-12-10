#include <Wire.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_MPU6050.h>
#include <AFMotor.h>

// Create an MPU6050 object
Adafruit_MPU6050 mpu;

// Create motor objects
AF_DCMotor leftMotor(1);  // Connect left motor to port 1
AF_DCMotor rightMotor(2); // Connect right motor to port 2

void setup() {
  Serial.begin(115200);

  // Initialize MPU6050
  if (!mpu.begin()) {
    Serial.println("Failed to find MPU6050 chip");
    while (1);
  }
  Serial.println("MPU6050 Found!");

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

  leftMotor.setSpeed(255);
  leftMotor.run(FORWARD);
  rightMotor.setSpeed(255);
  rightMotor.run(FORWARD);

  delay(timeToMove);

  stopMotors();
}

void turnRobot(int targetAngle) {
  sensors_event_t a, g, temp;
  mpu.getEvent(&a, &g, &temp);

  float totalAngle = 0;
  unsigned long prevTime = millis();

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
      leftMotor.setSpeed(255);
      leftMotor.run(FORWARD);
      rightMotor.setSpeed(255);
      rightMotor.run(BACKWARD);
    } else {
      // Left turn
      leftMotor.setSpeed(255);
      leftMotor.run(BACKWARD);
      rightMotor.setSpeed(255);
      rightMotor.run(FORWARD);
    }
  }
  stopMotors();
}

void stopMotors() {
  leftMotor.run(RELEASE);
  rightMotor.run(RELEASE);
}

void loop() {
  moveForward(1.0);  // Move forward 1 meter
  turnRobot(90);     // Turn 90 degrees right
  delay(1000);       // Wait for 1 second
  moveForward(1.0);  // Move forward 1 meter
  delay(1000);       // Wait for 1 second
}