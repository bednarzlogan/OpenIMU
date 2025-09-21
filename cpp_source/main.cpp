#include <iostream>
#include <memory>
#include <string>
#include <fstream>
#include <nlohmann/json.hpp>
#include <thread>

#include "UKF.hpp"
#include "ukf_defs.hpp"
#include "timed_playback_sim.hpp"

using std::string;
using json = nlohmann::json;


void run_estimator(const std::shared_ptr<Estimator>& estimator, TimedPlaybackSim& sim, std::chrono::milliseconds period) {
    // set into a thread to keep the filter going while the main thread waits for user input
    sim.start_simulation(estimator, period);
}


int main() {
    std::cout << "Loading configs...\n" << std::endl;

    // define path to configs (remember that our working dir in runtime is the build folder)
    std::string config_file_path = "conf/system_constants.json";
    std::string playback_file_path = "conf/rt_playback_conf.json";

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

    // create time timed playback simulator
    TimedPlaybackSim sim(playback_file_path);

    // create IMU instance
    std::shared_ptr<Estimator> estimator;
    std::chrono::milliseconds period(20); // 50 Hz update rate
    #ifdef SIM_MODE
        switch (filter_type) {
            case 2:
                estimator = std::make_shared<UKF>(config_file_path);
                estimator->initialize(initial_state, initial_covariance);
                break;
            default:
                std::cerr << "Unknown filter type provided." << std::endl;
                return -1;
        }

        // keep the main thread alive while the filter runs
        std::cout << "Filter started. Press q to stop." << std::endl;
        std::thread estimator_thread(run_estimator, estimator, std::ref(sim), period);

        // shutdown loop
        while (true) {
            char c = std::cin.get();
            if (c == 'q' || c == 'Q') {
                std::cout << "Stopping filter..." << std::endl;
                sim.stop_simulation();     // stop producers first
                estimator->stop_filter();  // then stop the consumer
                estimator_thread.join();
                break;
            }
        }
    #else
        std::cerr << "Not finished yet! Please use SIM MODE" << std::endl;
    #endif
}