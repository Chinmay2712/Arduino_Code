#include "imu_bno055/bno055.hpp"

#include <linux/i2c-dev.h>
#include <sys/ioctl.h>
#include <unistd.h>
#include <fcntl.h>
#include <cstring>
#include <iostream>

// BNO055 Register Map
#define REG_CHIP_ID            0x00
#define REG_OPR_MODE           0x3D
#define REG_PWR_MODE           0x3E
#define REG_SYS_TRIGGER        0x3F
#define REG_UNIT_SEL           0x3B
#define REG_PAGE_ID            0x07
#define REG_CALIB_STAT         0x35

#define REG_ACCEL_DATA_X_LSB   0x08
#define REG_MAG_DATA_X_LSB     0x0E
#define REG_GYRO_DATA_X_LSB    0x14
#define REG_EULER_H_LSB        0x1A
#define REG_QUAT_W_LSB         0x20

#define REG_OFFSET_START       0x55

// Operation modes
#define OPERATION_MODE_CONFIG  0x00
#define OPERATION_MODE_NDOF    0x0C

// Power modes
#define POWER_MODE_NORMAL      0x00

// ---------------------------------------------------------------------------
// Constructor
// ---------------------------------------------------------------------------

BNO055::BNO055(int bus, uint8_t address)
    : bus_(bus), address_(address) {}

// ---------------------------------------------------------------------------
// Begin: open I2C bus and verify chip ID
// ---------------------------------------------------------------------------

bool BNO055::begin() {
    char filename[20];
    sprintf(filename, "/dev/i2c-%d", bus_);

    bus_ = open(filename, O_RDWR);
    if (bus_ < 0) {
        std::cerr << "Failed to open I2C bus\n";
        return false;
    }

    if (ioctl(bus_, I2C_SLAVE, address_) < 0) {
        std::cerr << "Failed to set I2C address\n";
        return false;
    }

    // Check chip ID
    int id = read8(REG_CHIP_ID);
    if (id != 0xA0) {
        std::cerr << "BNO055 not detected. Chip ID: " << id << "\n";
        return false;
    }

    // Switch to config mode
    setMode(OPERATION_MODE_CONFIG);
    usleep(20000);

    // Normal power mode
    write8(REG_PWR_MODE, POWER_MODE_NORMAL);
    usleep(10000);

    // Use default units (deg, m/s^2, uT)
    write8(REG_UNIT_SEL, 0x00);

    // Switch to NDOF fusion mode
    setMode(OPERATION_MODE_NDOF);
    usleep(20000);

    return true;
}

// ---------------------------------------------------------------------------
// Low-level I2C helpers
// ---------------------------------------------------------------------------

bool BNO055::write8(uint8_t reg, uint8_t value) {
    uint8_t buf[2] = {reg, value};
    return write(bus_, buf, 2) == 2;
}

int BNO055::read8(uint8_t reg) {
    if (write(bus_, &reg, 1) != 1)
        return -1;

    uint8_t data;
    if (read(bus_, &data, 1) != 1)
        return -1;

    return data;
}

bool BNO055::readLen(uint8_t reg, uint8_t *buffer, uint8_t len) {
    if (write(bus_, &reg, 1) != 1)
        return false;

    return read(bus_, buffer, len) == len;
}

// ---------------------------------------------------------------------------
// Set operation mode
// ---------------------------------------------------------------------------

bool BNO055::setMode(uint8_t mode) {
    return write8(REG_OPR_MODE, mode);
}

// ---------------------------------------------------------------------------
// Calibration status
// ---------------------------------------------------------------------------

bool BNO055::readCalibration(uint8_t &sys, uint8_t &gyr, uint8_t &acc, uint8_t &mag) {
    int v = read8(REG_CALIB_STAT);
    if (v < 0) return false;

    sys = (v >> 6) & 0x03;
    gyr = (v >> 4) & 0x03;
    acc = (v >> 2) & 0x03;
    mag = v & 0x03;

    return true;
}

// ---------------------------------------------------------------------------
// Read accelerometer (m/s^2)
// ---------------------------------------------------------------------------

bool BNO055::readAccel(float a[3]) {
    uint8_t buf[6];
    if (!readLen(REG_ACCEL_DATA_X_LSB, buf, 6))
        return false;

    int16_t x = (int16_t)((buf[1] << 8) | buf[0]);
    int16_t y = (int16_t)((buf[3] << 8) | buf[2]);
    int16_t z = (int16_t)((buf[5] << 8) | buf[4]);

    a[0] = x / 100.0f;
    a[1] = y / 100.0f;
    a[2] = z / 100.0f;

    return true;
}

// ---------------------------------------------------------------------------
// Read gyroscope (deg/s)
// ---------------------------------------------------------------------------

bool BNO055::readGyro(float g[3]) {
    uint8_t buf[6];
    if (!readLen(REG_GYRO_DATA_X_LSB, buf, 6))
        return false;

    int16_t x = (int16_t)((buf[1] << 8) | buf[0]);
    int16_t y = (int16_t)((buf[3] << 8) | buf[2]);
    int16_t z = (int16_t)((buf[5] << 8) | buf[4]);

    g[0] = x / 16.0f;
    g[1] = y / 16.0f;
    g[2] = z / 16.0f;

    return true;
}

// ---------------------------------------------------------------------------
// Read magnetometer (µT)
// ---------------------------------------------------------------------------

bool BNO055::readMag(float m[3]) {
    uint8_t buf[6];
    if (!readLen(REG_MAG_DATA_X_LSB, buf, 6))
        return false;

    int16_t x = (int16_t)((buf[1] << 8) | buf[0]);
    int16_t y = (int16_t)((buf[3] << 8) | buf[2]);
    int16_t z = (int16_t)((buf[5] << 8) | buf[4]);

    m[0] = x / 16.0f;
    m[1] = y / 16.0f;
    m[2] = z / 16.0f;

    return true;
}

// ---------------------------------------------------------------------------
// Read quaternion (unitless)
// ---------------------------------------------------------------------------

bool BNO055::readQuaternion(float q[4]) {
    uint8_t buf[8];
    if (!readLen(REG_QUAT_W_LSB, buf, 8))
        return false;

    int16_t w = (int16_t)((buf[1] << 8) | buf[0]);
    int16_t x = (int16_t)((buf[3] << 8) | buf[2]);
    int16_t y = (int16_t)((buf[5] << 8) | buf[4]);
    int16_t z = (int16_t)((buf[7] << 8) | buf[6]);

    const float scale = 1.0f / (1 << 14);

    q[0] = w * scale;
    q[1] = x * scale;
    q[2] = y * scale;
    q[3] = z * scale;

    return true;
}

// ---------------------------------------------------------------------------
// Read Euler angles (degrees)
// ---------------------------------------------------------------------------

bool BNO055::readEuler(float &roll, float &pitch, float &yaw) {
    uint8_t buf[6];
    if (!readLen(REG_EULER_H_LSB, buf, 6))
        return false;

    int16_t h = (int16_t)((buf[1] << 8) | buf[0]);
    int16_t r = (int16_t)((buf[3] << 8) | buf[2]);
    int16_t p = (int16_t)((buf[5] << 8) | buf[4]);

    yaw   = h / 16.0f;
    roll  = r / 16.0f;
    pitch = p / 16.0f;

    return true;
}

// ---------------------------------------------------------------------------
// Read calibration offsets (raw register dump)
// ---------------------------------------------------------------------------

bool BNO055::readOffsets(std::vector<int16_t> &offsets) {
    uint8_t buf[22];
    if (!readLen(REG_OFFSET_START, buf, 22))
        return false;

    offsets.clear();
    for (int i = 0; i < 22; i += 2) {
        int16_t v = (int16_t)((buf[i+1] << 8) | buf[i]);
        offsets.push_back(v);
    }

    return true;
}

// ---------------------------------------------------------------------------
// Write calibration offsets
// ---------------------------------------------------------------------------

bool BNO055::writeOffsets(const std::vector<int16_t> &offsets) {
    if (offsets.size() != 11)
        return false;

    // Must be in CONFIG mode
    setMode(OPERATION_MODE_CONFIG);
    usleep(20000);

    for (size_t i = 0; i < offsets.size(); i++) {
        uint8_t reg = REG_OFFSET_START + (i * 2);
        int16_t v = offsets[i];

        write8(reg,     v & 0xFF);
        write8(reg + 1, (v >> 8) & 0xFF);
    }

    // Return to NDOF
    setMode(OPERATION_MODE_NDOF);
    usleep(20000);

    return true;
}
