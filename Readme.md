# MPU6050 Robot Controller

This repository contains the code to control a robot using an MPU6050 sensor and DC motors. The robot can move forward and turn based on the sensor data.

## Files

- `1.ino`: Main Arduino sketch that initializes the MPU6050 sensor and controls the robot's movements.
- `MPU6050.ino`: Additional Arduino sketch for MPU6050 sensor initialization and control.

## Setup

1. Connect the MPU6050 sensor to the Arduino.
2. Connect the left motor to port 1 and the right motor to port 2 of the motor driver.
3. Upload the `1.ino` or `MPU6050.ino` sketch to the Arduino.

## Functions

### `setup()`

Initializes the MPU6050 sensor and sets up the serial communication.

### `moveForward(float distance)`

Moves the robot forward by a specified distance in meters.

### `turnRobot(int targetAngle)`

Turns the robot by a specified angle in degrees. Positive values for right turns and negative values for left turns.

### `stopMotors()`

Stops both motors.

### `loop()`

Main loop that controls the robot's movements.

## Example Usage

```cpp
void loop() {
  moveForward(1.0);  // Move forward 1 meter
  turnRobot(90);     // Turn 90 degrees right
  delay(1000);       // Wait for 1 second
  moveForward(1.0);  // Move forward 1 meter
  delay(1000);       // Wait for 1 second
  turnRobot(-90);    // Turn 90 degrees left
  moveForward(1.0);  // Move forward 1 meter
  delay(10000);      // Wait for 10 seconds
}
```

## Dependencies
* Adafruit MPU6050 Library
* Adafruit Sensor Library
* Adafruit Motor Shield Library