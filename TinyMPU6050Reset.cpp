#include "TinyMPU6050Reset.h"

MPU6050::MPU6050(int i2cAddress) {
  if(i2cAddress == MPU6050_ADDRESS_LOW || i2cAddress == MPU6050_ADDRESS_HIGH){
    mpu = i2cAddress;
  } else{
    mpu = MPU6050_ADDRESS_LOW;
  }
}

float MPU6050::readTemperature(void)
{
    int16_t T;
    //T = readRegister16(MPU6050_REG_TEMP_OUT_H);
    TinyWireM.beginTransmission(mpu);
    TinyWireM.write(MPU6050_REG_TEMP_OUT_H);
    TinyWireM.endTransmission();
    TinyWireM.requestFrom(mpu, 2);
    T = TinyWireM.read() << 8 |TinyWireM.read();
    return (float)T/340 + 36.53;
}

void MPU6050::Calibrate() {

  for (int i = 0; i < DISCARDED_MEASURES; i++) {
    UpdateRawAccel();
    UpdateRawGyro();
    delay(2);
  }

  float sumGyroX = 0;
  float sumGyroY = 0;
  float sumGyroZ = 0;
  
  for (int i = 0; i < CALIBRATION_MEASURES; i++) {
    UpdateRawAccel();
    UpdateRawGyro();
    sumGyroX += GetRawGyroX();
    sumGyroY += GetRawGyroY();
    sumGyroZ += GetRawGyroZ();
    delay(2);
  }

  sumGyroX  /= CALIBRATION_MEASURES;
  sumGyroY  /= CALIBRATION_MEASURES;
  sumGyroZ  /= CALIBRATION_MEASURES;

  SetGyroOffsets(sumGyroX, sumGyroY, sumGyroZ);
}

void MPU6050::SetGyroOffsets(float x, float y, float z){

  // Setting offsets
  gyroXOffset = x;
  gyroYOffset = y;
  gyroZOffset = z;
}

void MPU6050::UpdateRawAccel(){

  // Beginning transmission for MPU6050
  TinyWireM.beginTransmission(mpu);

  // Accessing accel data registers
  TinyWireM.write(MPU6050_ACCEL_XOUT_H);
  TinyWireM.endTransmission();

  // Requesting accel data
  TinyWireM.requestFrom(mpu, 6);

  // Storing raw accel data
  rawAccX = TinyWireM.read() << 8 |TinyWireM.read();
  rawAccY = TinyWireM.read() << 8 |TinyWireM.read();
  rawAccZ = TinyWireM.read() << 8 |TinyWireM.read();
}

void MPU6050::UpdateRawGyro() {

  // Beginning transmission for MPU6050
  TinyWireM.beginTransmission(mpu);

  // Accessing gyro data registers
  TinyWireM.write(MPU6050_GYRO_XOUT_H);
  TinyWireM.endTransmission();

  // Requesting gyro data
  TinyWireM.requestFrom(mpu, 6);

  // Storing raw gyro data
  rawGyroX = TinyWireM.read() << 8;
  rawGyroX |= TinyWireM.read();

  rawGyroY = TinyWireM.read() << 8;
  rawGyroY |= TinyWireM.read();

  rawGyroZ = TinyWireM.read() << 8;
  rawGyroZ |= TinyWireM.read();
}

void MPU6050::BaseInititalize() {
  // Setting attributes with default values
  filterAccelCoeff = DEFAULT_ACCEL_COEFF;
  filterGyroCoeff = DEFAULT_GYRO_COEFF;
  TinyWireM.beginTransmission(mpu); 
  TinyWireM.write(MPU6050_PWR_MGMT_1);
  TinyWireM.write(0b00000000); // отключаем спящий режим
  TinyWireM.endTransmission();
  TinyWireM.beginTransmission(mpu); 
  // Setting sample rate divider
  TinyWireM.write(MPU6050_SMPLRT_DIV);
  TinyWireM.endTransmission();
  TinyWireM.beginTransmission(mpu); 
  // Setting frame synchronization and the digital low-pass filter
  TinyWireM.write(MPU6050_CONFIG);
  TinyWireM.endTransmission();
  TinyWireM.beginTransmission(mpu); 
  // Setting gyro self-test and full scale range
  TinyWireM.write(MPU6050_GYRO_CONFIG);
  
  TinyWireM.write(0b00000000); // отключаем спящий режим
  TinyWireM.endTransmission();
  TinyWireM.beginTransmission(mpu); 
  // Setting accelerometer self-test and full scale range
  TinyWireM.write(MPU6050_ACCEL_CONFIG);
  
  TinyWireM.write(0b00000000); // отключаем спящий режим
  
  // Waking up MPU6050
  TinyWireM.endTransmission();
  // Setting angles to zero
  angX = 0;
  angY = 0;
  angZ = 0;

  // Beginning integration timer
  intervalStart = millis();
}

static float wrap(float angle)
{
  while (angle > +180) angle -= 360;
  while (angle < -180) angle += 360;
  return angle;
}

static float angle_average(float waz, float a, float wb, float b)
{
  return wrap(waz * a + wb * (a + sqrt(b-a)));
}

void MPU6050::Execute(){

  // Updating raw data before processing it
  UpdateRawAccel();
  UpdateRawGyro();

  // Computing readable accel/gyro data
  accX = (float)(rawAccX) * ACCEL_TRANSFORMATION_NUMBER;
  accY = (float)(rawAccY) * ACCEL_TRANSFORMATION_NUMBER;
  accZ = (float)(rawAccZ) * ACCEL_TRANSFORMATION_NUMBER;
  gyroX = (float)(rawGyroX - gyroXOffset) * GYRO_TRANSFORMATION_NUMBER;
  gyroY = (float)(rawGyroY - gyroYOffset) * GYRO_TRANSFORMATION_NUMBER;
  gyroZ = (float)(rawGyroZ - gyroZOffset) * GYRO_TRANSFORMATION_NUMBER;

  // Computing accel angles
  angAccX = wrap((atan2(accY, wrap(accZ * accZ + accX * accX))) * RAD_TO_DEG);
  angAccY = wrap((-atan2(accX, wrap(accZ * accZ + accY * accY))) * RAD_TO_DEG);

  // Computing gyro angles
  dt = (millis() - intervalStart) * 0.001;
  angGyroX = wrap(angGyroX + gyroX * dt);
  angGyroY = wrap(angGyroY + gyroY * dt);
  angGyroZ = wrap(angGyroZ + gyroZ * dt);

  // Computing complementary filter angles
  angX = angle_average(filterAccelCoeff, angAccX, filterGyroCoeff, angX + gyroX * dt);
  angY = angle_average(filterAccelCoeff, angAccY, filterGyroCoeff, angY + gyroY * dt);
  angZ = angGyroZ;

  // Reseting the integration timer
  intervalStart = millis();
}

void MPU6050::SetFilterAccCoeff(float coeff) {

	// Setting coefficient
	filterAccelCoeff = coeff;
}

/*
 *	Set function for the gyro coefficient on complementary filter
 */
void MPU6050::SetFilterGyroCoeff(float coeff) {

	// Setting coefficient
	filterGyroCoeff = coeff;
}
