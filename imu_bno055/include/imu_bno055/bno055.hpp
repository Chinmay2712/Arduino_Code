#pragma once
#include <cstdint>
#include <vector>

class BNO055 {
public:
    BNO055(int bus, uint8_t address = 0x28);
    bool begin();
    bool setMode(uint8_t mode);
    bool readCalibration(uint8_t &sys, uint8_t &gyr, uint8_t &acc, uint8_t &mag);
    bool readEuler(float &roll, float &pitch, float &yaw);
    bool readQuaternion(float q[4]);
    bool readAccel(float a[3]);
    bool readGyro(float g[3]);
    bool readMag(float m[3]);
    bool readOffsets(std::vector<int16_t> &offsets);
    bool writeOffsets(const std::vector<int16_t> &offsets);

private:
    int bus_;
    uint8_t address_;
    int read8(uint8_t reg);
    bool readLen(uint8_t reg, uint8_t *buffer, uint8_t len);
    bool write8(uint8_t reg, uint8_t value);
};
