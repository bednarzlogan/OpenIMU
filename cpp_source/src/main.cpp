#include "IMU.hpp"

int main() {
    std::cout << "Loading configs...\n" << std::endl;

    // define path to configs (remember that our working dir in runtime is the build folder)
    std::string config_file_path = "../system_constants.json";

    // create IMU instance
    IMU imu_instance = IMU(config_file_path);
}