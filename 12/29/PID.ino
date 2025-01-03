#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_HMC5883_U.h>

// Magnetometer setup
Adafruit_HMC5883_Unified mag = Adafruit_HMC5883_Unified(12345);

// PID constants
float Kp = 1.0;  // Proportional gain
float Ki = 0.0;  // Integral gain
float Kd = 0.5;  // Derivative gain

// PID variables
float error, previous_error = 0, cumulative_error = 0;
float pid_output;

// Motor control pins
#define LEFT_MOTOR_PWM 3
#define RIGHT_MOTOR_PWM 5

void setup() {
  Serial.begin(9600);
  if (!mag.begin()) {
    Serial.println("HMC5883L not detected");
    while (1);
  }
  pinMode(LEFT_MOTOR_PWM, OUTPUT);
  pinMode(RIGHT_MOTOR_PWM, OUTPUT);
}

void loop() {
  // Get current heading
  sensors_event_t event;
  mag.getEvent(&event);
  float heading = atan2(event.magnetic.y, event.magnetic.x);
  heading = heading * 180 / PI;  // Convert to degrees
  if (heading < 0) heading += 360;

  // Calculate PID
  float desired_heading = 0;  // Target heading
  error = desired_heading - heading;
  cumulative_error += error;
  float derivative = error - previous_error;

  pid_output = Kp * error + Ki * cumulative_error + Kd * derivative;

  // Adjust motor speeds
  int left_motor_speed = constrain(150 + pid_output, 0, 255);
  int right_motor_speed = constrain(150 - pid_output, 0, 255);

  analogWrite(LEFT_MOTOR_PWM, left_motor_speed);
  analogWrite(RIGHT_MOTOR_PWM, right_motor_speed);

  previous_error = error;

  delay(50);
}
