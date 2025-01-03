#include <Wire.h>
 
// MPU6050 I2C address
#define MPU6050_ADDR 0x68
 
// MPU6050 registers
#define MPU6050_REG_ACCEL_XOUT_H 0x3B
#define MPU6050_REG_PWR_MGMT_1   0x6B
 
// HMC5883L I2C address
#define HMC5883L_ADDR 0x1E
 
// Sensitivity scales
#define ACCEL_SCALE 16384.0
#define MAG_SCALE 0.92 // 0.92 microTesla per LSB for ±1.3 Gauss
 
// Calibration offsets
double accelOffsetX = 0, accelOffsetY = 0, accelOffsetZ = 0;
double magMinX = 0, magMaxX = 0, magMinY = 0, magMaxY = 0, magMinZ = 0, magMaxZ = 0;
 
void setup() {
  Serial.begin(115200);
  Wire.begin(); // Initialize I2C
 
  MPU6050_init();   // Initialize MPU6050
  HMC5883L_init();  // Initialize HMC5883L
 
  calibrate_MPU6050();  // Calibrate accelerometer
  calibrate_HMC5883L(); // Calibrate magnetometer
}
 
void loop() {
  double ax, ay, az;
  double mx, my, mz;
  double pitch, roll, yaw;
 
  // Read MPU6050 accelerometer data
  read_MPU6050(ax, ay, az);
 
  // Calculate pitch and roll
  pitch = atan2(-ax, sqrt(ay * ay + az * az)) * 180.0 / PI;
  roll = atan2(ay, az) * 180.0 / PI;
 
  // Read HMC5883L magnetometer data
  read_HMC5883L(mx, my, mz);
 
  // Calculate yaw from magnetometer data
  yaw = atan2(my, mx);
 
  // Adjust yaw using declination angle (replace with your location's value)
  float declinationAngle = -0.1783;  // Declination in radians for -10° 13' 
  yaw += declinationAngle;
 
  // Normalize yaw to 0-360 degrees
  if (yaw < 0) yaw += 2 * PI;
  if (yaw > 2 * PI) yaw -= 2 * PI;
 
  yaw = yaw * 180.0 / PI; // Convert to degrees
 
  // Print results
  Serial.print("Pitch: "); Serial.print(pitch); Serial.print("°  ");
  Serial.print("Roll: "); Serial.print(roll); Serial.print("°  ");
  Serial.print("Yaw: "); Serial.print(yaw); Serial.println("°");
 
  delay(200);
}
 
void MPU6050_init() {
  Wire.beginTransmission(MPU6050_ADDR);
  Wire.write(MPU6050_REG_PWR_MGMT_1);
  Wire.write(0); // Wake up MPU6050
  Wire.endTransmission(true);
}
 
void calibrate_MPU6050() {
  Serial.println("Calibrating MPU6050...");
  double ax, ay, az;
  int numSamples = 100;
 
  for (int i = 0; i < numSamples; i++) {
    read_MPU6050(ax, ay, az);
    accelOffsetX += ax;
    accelOffsetY += ay;
    accelOffsetZ += az;
    delay(10);
  }
 
  accelOffsetX /= numSamples;
  accelOffsetY /= numSamples;
  accelOffsetZ /= numSamples;
 
  // Assuming Z points up, adjust for gravity
  accelOffsetZ -= 1.0; // Gravity in g (9.8 m/s^2)
 
  Serial.println("MPU6050 Calibration Complete.");
}
 
void read_MPU6050(double &ax, double &ay, double &az) {
  Wire.beginTransmission(MPU6050_ADDR);
  Wire.write(MPU6050_REG_ACCEL_XOUT_H); // Starting register for Accel Readings
  Wire.endTransmission(false);
  Wire.requestFrom(MPU6050_ADDR, 6, true);
 
  ax = (Wire.read() << 8 | Wire.read()) / ACCEL_SCALE - accelOffsetX;
  ay = (Wire.read() << 8 | Wire.read()) / ACCEL_SCALE - accelOffsetY;
  az = (Wire.read() << 8 | Wire.read()) / ACCEL_SCALE - accelOffsetZ;
}
 
void HMC5883L_init() {
  Wire.beginTransmission(HMC5883L_ADDR);
  Wire.write(0x00); // Configuration Register A
  Wire.write(0x70); // 8-average, 15 Hz default, normal measurement
  Wire.endTransmission();
 
  Wire.beginTransmission(HMC5883L_ADDR);
  Wire.write(0x01); // Configuration Register B
  Wire.write(0x20); // Gain = 5 (±1.3 Gauss)
  Wire.endTransmission();
 
  Wire.beginTransmission(HMC5883L_ADDR);
  Wire.write(0x02); // Mode Register
  Wire.write(0x00); // Continuous measurement mode
  Wire.endTransmission();
}
 
void calibrate_HMC5883L() {
  Serial.println("Calibrating HMC5883L...");
  double mx, my, mz;
 
  magMinX = magMinY = magMinZ = 1e6;
  magMaxX = magMaxY = magMaxZ = -1e6;
 
  unsigned long startTime = millis();
  while (millis() - startTime < 10000) { // 10 seconds for calibration
    read_HMC5883L(mx, my, mz);
 
    if (mx < magMinX) magMinX = mx;
    if (mx > magMaxX) magMaxX = mx;
    if (my < magMinY) magMinY = my;
    if (my > magMaxY) magMaxY = my;
    if (mz < magMinZ) magMinZ = mz;
    if (mz > magMaxZ) magMaxZ = mz;
 
    delay(100);
  }
 
  Serial.println("HMC5883L Calibration Complete.");
}
 
void read_HMC5883L(double &mx, double &my, double &mz) {
  Wire.beginTransmission(HMC5883L_ADDR);
  Wire.write(0x03); // Starting register for magnetometer readings
  Wire.endTransmission(false);
  Wire.requestFrom(HMC5883L_ADDR, 6, true);
 
  int16_t x = Wire.read() << 8 | Wire.read();
  int16_t z = Wire.read() << 8 | Wire.read();
  int16_t y = Wire.read() << 8 | Wire.read();
 
  // Apply calibration offsets and scaling
  mx = (x - (magMinX + magMaxX) / 2) * MAG_SCALE;
  my = (y - (magMinY + magMaxY) / 2) * MAG_SCALE;
  mz = (z - (magMinZ + magMaxZ) / 2) * MAG_SCALE;
}