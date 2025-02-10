#include <Wire.h>
#include <math.h>

// MPU6050 I2C address
const int MPU = 0x68;

// Raw sensor readings
float AccX, AccY, AccZ;
float GyroX, GyroY, GyroZ;

// Angles from each sensor
float accAngleX, accAngleY;
float gyroAngleX, gyroAngleY, gyroAngleZ;

// Combined angles
float roll, pitch, yaw;

// Offsets / errors found by calibration
float AccErrorX = 0.0, AccErrorY = 0.0;
float GyroErrorX = 0.0, GyroErrorY = 0.0, GyroErrorZ = 0.0;

// Timing
unsigned long currentTime, previousTime;
float elapsedTime;

// Calibration loop counter
int c = 0;

void setup() {
  Serial.begin(19200);
  Serial.println("Setup started!"); // Debug line
  Wire.begin();

  // Wake the MPU6050
  Wire.beginTransmission(MPU);
  Wire.write(0x6B);  
  Wire.write(0x00);  
  Wire.endTransmission(true);

  // Perform calibration
  calculate_IMU_error();  
  delay(100);

  Serial.println("Setup finished!");
}

void loop() {
  // --- Read accelerometer ---
  Wire.beginTransmission(MPU);
  Wire.write(0x3B); // ACCEL_XOUT_H
  Wire.endTransmission(false);
  Wire.requestFrom(MPU, 6, true);

  AccX = (Wire.read() << 8 | Wire.read()) / 16384.0; 
  AccY = (Wire.read() << 8 | Wire.read()) / 16384.0; 
  AccZ = (Wire.read() << 8 | Wire.read()) / 16384.0; 

  // Convert to angles (in degrees)
  accAngleX = atan(AccY / sqrt(AccX * AccX + AccZ * AccZ)) * 180.0 / PI;
  accAngleY = atan(-AccX / sqrt(AccY * AccY + AccZ * AccZ)) * 180.0 / PI;

  // Subtract measured offsets
  accAngleX -= AccErrorX;
  accAngleY -= AccErrorY;

  // --- Read gyroscope ---
  previousTime = currentTime;
  currentTime = millis();
  elapsedTime = (currentTime - previousTime) / 1000.0; // in seconds

  Wire.beginTransmission(MPU);
  Wire.write(0x43); // GYRO_XOUT_H
  Wire.endTransmission(false);
  Wire.requestFrom(MPU, 6, true);

  GyroX = (Wire.read() << 8 | Wire.read()) / 131.0;
  GyroY = (Wire.read() << 8 | Wire.read()) / 131.0;
  GyroZ = (Wire.read() << 8 | Wire.read()) / 131.0;

  // Subtract gyro offsets
  GyroX -= GyroErrorX;
  GyroY -= GyroErrorY;
  GyroZ -= GyroErrorZ;

  // Integrate gyro rates to get angles
  gyroAngleX += GyroX * elapsedTime;
  gyroAngleY += GyroY * elapsedTime;
  yaw        += GyroZ * elapsedTime;

  // --- Complementary filter ---
  roll  = 0.96 * gyroAngleX + 0.04 * accAngleX;
  pitch = 0.96 * gyroAngleY + 0.04 * accAngleY;

  // Print results
  Serial.print(roll);
  Serial.print("/");
  Serial.print(pitch);
  Serial.print("/");
  Serial.println(yaw);
}

// -----------------------------------------------------
// Calculate offsets for the accelerometer and gyro
// (Place the sensor flat and still during this!)
void calculate_IMU_error() {
  AccErrorX = AccErrorY = 0.0;
  GyroErrorX = GyroErrorY = GyroErrorZ = 0.0;
  c = 0;

  // *** Accelerometer calibration ***
  while (c < 200) {
    Wire.beginTransmission(MPU);
    Wire.write(0x3B);
    Wire.endTransmission(false);
    Wire.requestFrom(MPU, 6, true);

    float tmpAccX = (Wire.read() << 8 | Wire.read()) / 16384.0;
    float tmpAccY = (Wire.read() << 8 | Wire.read()) / 16384.0;
    float tmpAccZ = (Wire.read() << 8 | Wire.read()) / 16384.0;

    AccErrorX += (atan(tmpAccY / sqrt(tmpAccX*tmpAccX + tmpAccZ*tmpAccZ)) * 180.0/PI);
    AccErrorY += (atan(-tmpAccX / sqrt(tmpAccY*tmpAccY + tmpAccZ*tmpAccZ)) * 180.0/PI);

    c++;
    delay(5); // small delay
    Serial.print("."); // Debug marker
  }
  Serial.println();

  AccErrorX /= 200.0;
  AccErrorY /= 200.0;

  // *** Gyroscope calibration ***
  c = 0;
  while (c < 200) {
    Wire.beginTransmission(MPU);
    Wire.write(0x43);
    Wire.endTransmission(false);
    Wire.requestFrom(MPU, 6, true);

    float tmpGyroX = (Wire.read() << 8 | Wire.read()) / 131.0;
    float tmpGyroY = (Wire.read() << 8 | Wire.read()) / 131.0;
    float tmpGyroZ = (Wire.read() << 8 | Wire.read()) / 131.0;

    GyroErrorX += tmpGyroX;
    GyroErrorY += tmpGyroY;
    GyroErrorZ += tmpGyroZ;

    c++;
    delay(5);
    Serial.print("_"); // Debug marker
  }
  Serial.println();

  GyroErrorX /= 200.0;
  GyroErrorY /= 200.0;
  GyroErrorZ /= 200.0;

  Serial.println("Calibration done. Offsets:");
  Serial.print("AccErrorX: "); Serial.println(AccErrorX);
  Serial.print("AccErrorY: "); Serial.println(AccErrorY);
  Serial.print("GyroErrorX: "); Serial.println(GyroErrorX);
  Serial.print("GyroErrorY: "); Serial.println(GyroErrorY);
  Serial.print("GyroErrorZ: "); Serial.println(GyroErrorZ);
}
