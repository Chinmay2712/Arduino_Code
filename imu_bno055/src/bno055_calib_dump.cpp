#include "imu_bno055/bno055.hpp"
#include <iostream>
#include <iomanip>
#include <vector>
#include <chrono>
#include <thread>

int main() {
    std::cout << "BNO055 Calibration Offset Dumper\n";
    std::cout << "--------------------------------\n";

    BNO055 imu(1, 0x28);  // I2C bus 1, default address 0x28

    if (!imu.begin()) {
        std::cerr << "ERROR: Could not initialize BNO055\n";
        return 1;
    }

    std::cout << "Waiting for full calibration...\n";

    uint8_t sys = 0, gyr = 0, acc = 0, mag = 0;

    // Wait until all calibration values reach 3
    while (true) {
        imu.readCalibration(sys, gyr, acc, mag);

        std::cout << "SYS:" << int(sys)
                  << " GYR:" << int(gyr)
                  << " ACC:" << int(acc)
                  << " MAG:" << int(mag) << "\r";

        std::cout.flush();

        if (sys == 3 && gyr == 3 && acc == 3 && mag == 3)
            break;

        std::this_thread::sleep_for(std::chrono::milliseconds(200));
    }

    std::cout << "\nCalibration complete!\n";

    // Read offsets
    std::vector<int16_t> offsets;
    if (!imu.readOffsets(offsets)) {
        std::cerr << "ERROR: Could not read offsets\n";
        return 1;
    }

    std::cout << "\nCalibration Offsets (YAML format):\n\n";

    std::cout << "offsets:\n";
    std::cout << "  accel: [" 
              << offsets[0] << ", " << offsets[1] << ", " << offsets[2] << "]\n";
    std::cout << "  mag:   [" 
              << offsets[3] << ", " << offsets[4] << ", " << offsets[5] << "]\n";
    std::cout << "  gyro:  [" 
              << offsets[6] << ", " << offsets[7] << ", " << offsets[8] << "]\n";
    std::cout << "  radius_accel: " << offsets[9] << "\n";
    std::cout << "  radius_mag:   " << offsets[10] << "\n\n";

    std::cout << "You can paste this into config/offsets.yaml\n";

    return 0;
}

