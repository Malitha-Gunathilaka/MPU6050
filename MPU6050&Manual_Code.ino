#include <AFMotor.h>  // Adafruit Motor Shield library
#include <NewPing.h> // NewPing library for ultrasonic sensor
#include <Servo.h> // Servo library
#include <Wire.h>  // Include the Wire library for I2C communication
#include <Adafruit_MPU6050.h>  // Include the Adafruit MPU6050 library
#include <Adafruit_Sensor.h>   // Include the Adafruit Sensor library

// Create an MPU6050 object
Adafruit_MPU6050 mpu;


// Motor initialization for layout:
//    M1    M2  (Front)
//     ┌────┐         
//     │    │         // Motor 1 and 2 control the left side of the robot
//     │    │         // Motor 3 and 4 control the right side of the robot
//     └────┘
//    M4    M3  (Back)
//////////////////////

// Motor control constants and objects
AF_DCMotor motor1(1); 
AF_DCMotor motor2(2);
AF_DCMotor motor3(3);
AF_DCMotor motor4(4);

// Define pins for the ultrasonic sensor
#define TRIGGER_PIN  22
#define ECHO_PIN     23
#define MAX_DISTANCE 200

NewPing sonar(TRIGGER_PIN, ECHO_PIN, MAX_DISTANCE); // NewPing object for the ultrasonic sensor

int speed = 255; // Default motor speed
bool objectDetected = false; // Flag to track object detection

// Servo and IR sensor constants and objects
const int irSensorPin = 2;
const int servoPin = 9; 
const int boxes = 10; // Number of boxes to store shuttlecocks
const int shuttlecocksPerBox = 10;  // Number of shuttlecocks per box
const int degreePerBox = 36; // 360 degrees / 10 boxes = 36 degrees per box

int shuttlecockCount = -1; // Shuttlecock count per box
int totalShuttlecockCount = -1; // Total shuttlecock count
int boxCount = 0; // Box count to track the current box
bool shuttlecockPresent = false; // Flag to track shuttlecock detection

Servo myServo;
int currentAngle = 0;

// Automatic movement constants
const int forwardTimeLong = 1400; // Time to move forward 1 meters (1m =3s)
const int forwardTimeShort = 400; // Time to move forward 0.17 meters 
const int turnTime = 1500; // Time to turn 90 degrees 
const int numPasses = 3; // Number of passes to cover the area

bool autoMode = false; // Flag to switch between manual and automatic mode

void setup() {
  // Motor control setup
  stopMotors();
  Serial3.begin(9600); // Communication with ESP8266 using Serial3
  setMotorSpeed(speed);

  // Servo and IR sensor setup
  pinMode(irSensorPin, INPUT);
  myServo.attach(servoPin);
  myServo.write(currentAngle);
  Serial.begin(9600); // For debugging on the Serial Monitor
  Serial.println("System Initialized");

///////////////////////////////////////////////////////////////////
  
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

  // Set the speed of the left and right motors to maximum
  motor1.run(FORWARD);
  motor2.run(FORWARD);
  motor3.run(FORWARD);
  motor4.run(FORWARD);
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
    } else {
      // Left turn
      motor1.run(BACKWARD);
      motor2.run(FORWARD);
      motor3.run(FORWARD);
      motor4.run(BACKWARD);
    }
  }
  stopMotors();

  //////////////////////////////////////////////////////////////////
}

void checkShuttlecockCount() {
  int irSensorValue = digitalRead(irSensorPin);
  if (irSensorValue == HIGH && !shuttlecockPresent) {
    shuttlecockPresent = true;  // Mark shuttlecock as detected
    shuttlecockCount++;  // Increment shuttlecock count
    totalShuttlecockCount++; // Increment total shuttlecock count
    sendShuttlecockCount();  // Send updated count
    
    // Check if the current box is full
    if (shuttlecockCount == shuttlecocksPerBox) {
      shuttlecockCount = 0; // Reset the shuttlecock count for the next box
      boxCount++; // Increment the box count
      
      // Rotate the servo to the next box
      currentAngle += degreePerBox;
      if (currentAngle > 360) { // Ensure the angle doesn't exceed 360 degrees
        currentAngle = 360;
      }
      myServo.write(currentAngle);

       // Check if all boxes are full
      if (boxCount == boxes) {
        Serial.println("All boxes are full. System will halt.");
        while (true) {} // Stop the system indefinitely
      } else {
        delay(1000); // Wait for the servo to rotate to the next box
      }
    }
  } else if (irSensorValue == LOW) {
    shuttlecockPresent = false; // Reset detection state
  }
}

// Manual mode functions
void runManualMode() { 
  // Read the distance from the ultrasonic sensor
  int distance = sonar.ping_cm();
  Serial.print("Distance: ");
  Serial.println(distance);

  if (distance > 0 && distance < 5) {
    objectDetected = true;
    stopMotors();
    Serial3.println('O');
  } else {
    objectDetected = false;
  }

  if (Serial3.available() > 0) {
    String input = Serial3.readStringUntil('\n');
    Serial.print("Received command: ");
    Serial.println(input);
    
    // Trim any whitespace or newline characters from the input
    input.trim();
    Serial.print("Trimmed command: ");
    Serial.println(input);

    // Check for different commands
    if (input.startsWith("SPEED:")) {
        speed = input.substring(6).toInt();
        setMotorSpeed(speed);
        Serial.print("Speed set to: ");
        Serial.println(speed);
    } else if (input == "AUTO") {
        autoMode = true;
        Serial.println("Switched to automatic mode");
        Serial.print("Auto mode flag: ");
        Serial.println(autoMode);
    } else if (input == "MANUAL") {
        autoMode = false;
        Serial.println("Switched to manual mode");
        Serial.print("Auto mode flag: ");
        Serial.println(autoMode);
    } else {
        char command = input.charAt(0);
        Serial.print("Executing command: ");
        Serial.println(command);
        switch (command) {
            case 'F':
                if (objectDetected) {
                    Serial3.println('O');
                } else {
                    moveForward();
                }
                break;
            case 'B':
                moveBackward();
                break;
            case 'L':
                turnLeft();
                break;
            case 'R':
                turnRight();
                break;
            case 'S':
                stopMotors();
                break;
            default:
                Serial.println("Unknown command");
                break;
        }
    }
  }

  // Call shuttlecock counting function
  checkShuttlecockCount();
  delay(200);// Delay to avoid flooding the Serial Monitor
}

// Automatic mode function
void runAutoMode() {
    for (int i = 0; i < 5; i++) {
    delay(1000);
    moveForward(1.0);  // Move forward 1 meter
    turnRobot(90);     // Turn 90 degrees right
    delay(1000);       // Wait for 1 second
    moveForward(0.3);  // Move forward 30 cm
    turnRobot(90);     // Turn 90 degrees right
    delay(1000);       // Wait for 1 second
    moveForward(1.0);  // Move forward 1 meter
    turnRobot(-90);    // Turn 90 degrees left
    delay(1000);       // Wait for 1 second
    moveForward(0.3);  // Move forward 30 cm
    turnRobot(-90);    // Turn 90 degrees left
    delay(1000);       // Wait for 1 second
    moveForward(1.0);  // Move forward 1 meter
    delay(1000);       // Wait for 1 second
  }
  delay(10000);  // Wait for 10 seconds before repeating the loop
  // Stop the robot
    Serial.println("Finished covering the area. Stopping the robot.");
    stopMotors();
    autoMode = false; // Reset autoMode flag after completing the sequence
  
}

// Main loop
void loop() {
    Serial.print("Auto mode flag: ");
    Serial.println(autoMode);
    if (autoMode) {
        Serial.println("Running in automatic mode");
        runAutoMode();
    } else {
        Serial.println("Running in manual mode");
        runManualMode();
    }
}

// Motor control functions
void executeCommand(char command) {
  switch (command) {
    case 'B':
      moveBackward();
      break;
    case 'L':
      turnLeft();
      break;
    case 'R':
      turnRight();
      break;
    case 'S':
      stopMotors();
      break;
  }
}

// Motor control functions
void moveForward() {
  motor1.run(FORWARD);
  motor2.run(FORWARD);
  motor3.run(FORWARD);
  motor4.run(FORWARD);
}

void moveBackward() {
  motor1.run(BACKWARD);
  motor2.run(BACKWARD);
  motor3.run(BACKWARD);
  motor4.run(BACKWARD);
}

void turnLeft() {
  motor1.run(BACKWARD);
  motor2.run(FORWARD);
  motor3.run(FORWARD);
  motor4.run(BACKWARD);
}

void turnRight() {
  motor1.run(FORWARD);
  motor2.run(BACKWARD);
  motor3.run(BACKWARD);
  motor4.run(FORWARD);
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
// Function to send the total shuttlecock count to the ESP8266
void sendShuttlecockCount() {
  Serial3.print("TOTAL_SHUTTLE:");
  Serial3.println(totalShuttlecockCount); // Send total shuttlecock count only
}