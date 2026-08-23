#include "MPU6500.h"

/************************************************
 * Registers
 ************************************************/

#define WHO_AM_I       0x75

#define PWR_MGMT_1     0x6B
#define PWR_MGMT_2     0x6C

#define ACCEL_CONFIG   0x1C
#define ACCEL_CONFIG2  0x1D
#define GYRO_CONFIG    0x1B

#define CONFIG         0x1A
#define SMPLRT_DIV     0x19

#define ACCEL_XOUT_H   0x3B
#define TEMP_OUT_H     0x41
#define GYRO_XOUT_H    0x43

/************************************************/

MPU6500::MPU6500()
{
    initialized = false;
}

/************************************************/

bool MPU6500::begin()
{
    Wire.begin(IMU_SDA_PIN, IMU_SCL_PIN);

    Wire.setClock(IMU_I2C_SPEED);

    delay(100);

    if(!selfTest())
        return false;

    // Wake device
    writeRegister(PWR_MGMT_1,0x00);

    delay(100);

    // Gyroscope DLPF = 41 Hz.

    writeRegister(CONFIG,0x03);

    // ±2g

    writeRegister(ACCEL_CONFIG,0x00);

    // Accelerometer DLPF = 41 Hz. This is a separate register from the
    // gyroscope filter above, and the accelerometer is the signal used by
    // motion and the current respiration estimator.
    writeRegister(ACCEL_CONFIG2,0x03);

    // ±250°/s

    writeRegister(GYRO_CONFIG,0x00);

    // 1kHz

    writeRegister(SMPLRT_DIV,0x00);

    initialized = true;

    return true;
}

/************************************************/

bool MPU6500::selfTest()
{
    return (readRegister(WHO_AM_I)==0x70);
}

/************************************************/

bool MPU6500::available()
{
    return initialized;
}

/************************************************/

void MPU6500::update()
{
    if(!initialized)
        return;

    readSensor();
}

/************************************************/

bool MPU6500::read(IMUData &data)
{
    if(!initialized)
        return false;

    if(!imuData.newData)
        return false;

    data = imuData;

    imuData.newData = false;

    return true;
}

/************************************************/

bool MPU6500::readSensor()
{
    // Read the complete sensor block in one transaction. The old code made
    // seven separate I2C reads, so axes from different update moments could
    // be combined into one motion sample.
    Wire.beginTransmission(MPU6500_ADDRESS);
    Wire.write(ACCEL_XOUT_H);

    if(Wire.endTransmission(false) != 0)
        return false;

    const uint8_t bytesRequested = 14;
    const uint8_t bytesReceived =
        Wire.requestFrom(MPU6500_ADDRESS, bytesRequested);

    if(bytesReceived != bytesRequested ||
       Wire.available() < bytesRequested)
    {
        while(Wire.available())
            Wire.read();

        return false;
    }

    uint8_t raw[14];

    for(uint8_t i = 0; i < bytesRequested; i++)
        raw[i] = (uint8_t)Wire.read();

    auto readSigned16 = [&raw](uint8_t index) -> int16_t
    {
        return (int16_t)(
            ((uint16_t)raw[index] << 8) |
            (uint16_t)raw[index + 1]
        );
    };

    imuData.ax = readSigned16(0);
    imuData.ay = readSigned16(2);
    imuData.az = readSigned16(4);

    imuData.temperature = readSigned16(6);

    imuData.gx = readSigned16(8);
    imuData.gy = readSigned16(10);
    imuData.gz = readSigned16(12);

    imuData.newData = true;

    return true;
}

/************************************************/

void MPU6500::writeRegister(uint8_t reg,uint8_t value)
{
    Wire.beginTransmission(MPU6500_ADDRESS);

    Wire.write(reg);

    Wire.write(value);

    Wire.endTransmission();
}

/************************************************/

uint8_t MPU6500::readRegister(uint8_t reg)
{
    Wire.beginTransmission(MPU6500_ADDRESS);

    Wire.write(reg);

    Wire.endTransmission(false);

    Wire.requestFrom(MPU6500_ADDRESS,(uint8_t)1);

    if(Wire.available())
        return Wire.read();

    return 0xFF;
}
