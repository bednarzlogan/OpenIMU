#include <iostream>
#include <memory>
#include <string>
#include <fstream>
#include <nlohmann/json.hpp>

#include "UKF.hpp"
#include "ukf_defs.hpp"

using std::string;
using json = nlohmann::json;

int main() {
    std::cout << "Loading configs...\n" << std::endl;

    // define path to configs (remember that our working dir in runtime is the build folder)
    std::string config_file_path = "system_constants.json";

    // load in the system configurations
    std::ifstream inFile(config_file_path);
    if (!inFile.is_open()) {
      std::cerr << "Could not open config file: " << config_file_path << std::endl;
    }

    // find the estimator type
    // access thre required members
    json j;
    inFile >> j;  // streams contents of the config file into 'j'

    // get the type of estimator used
    int filter_type = j["estimator"];

    // make a dummy state
    StateVec initial_state;
    CovMat initial_covariance;

    initial_covariance.setZero();
    initial_covariance.diagonal() << 
    0.5, 0.5, 0.5,   // positions
    0.1, 0.1, 0.1,   // velocities
    0.25, 0.25, 0.25, // attitude angles
    1e-4, 1e-4, 1e-4,   // accelerometer biases
    1e-4, 1e-4, 1e-4;   // gyro biases

    initial_state = 1e-3 * Eigen::Matrix<double, N, 1>::Ones();  

    // create IMU instance
    std::unique_ptr<Estimator> estimator;
    #ifdef SIM_MODE
        switch (filter_type) {
            case 2:
                estimator = std::make_unique<UKF>(config_file_path);
                estimator->initialize(initial_state, initial_covariance);
                estimator->start_filter();
                break;
            default:
                std::cerr << "Unknown filter type provided." << std::endl;
                return -1;
        }
    #else
        std::cerr << "Not finished yet! Please use SIM MODE" << std::endl;
    #endif
}