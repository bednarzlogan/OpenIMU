#include "IMU.hpp"
#include "IMU_Matrices.hpp"

int main() {
    std::cout << "Loading configs...\n" << std::endl;

    // define path to configs (remember that our working dir in runtime is the build folder)
    std::string config_file_path = "system_constants.json";

    // define path to pre-recorded IMU data
    std::string data_path = "t2.csv";

    // define dummy measurement queue location
    // make a dummy meausrement
    ImuData dummy_data;
    Eigen::Matrix<double, 6, 1> init_meausrement = Eigen::Matrix<double, 6, 1>::Zero(); 
    dummy_data.updateFromMatrix();
    std::queue<ImuData> measurements_queue;
    measurements_queue.push(dummy_data);

    // make a dummy state
    ImuStateVector initial_state;
    ImuCovariance initial_covariance;

    initial_covariance.covariance_matrix.diagonal() << 
    0.5, 0.5, 0.5,   // positions
    0.1, 0.1, 0.1,   // velocities
    0.25, 0.25, 0.25, // attitude angles
    1e-4, 1e-4, 1e-4,   // accelerometer biases
    1e-4, 1e-4, 1e-4;   // gyro biases

    Eigen::Matrix<double, 15, 1> init_state = Eigen::Matrix<double, 15, 1>::Zero();
    initial_state.updateFromMatrix();    


    // create IMU instance
    #ifdef SIM_MODE
        IMU imu_instance = IMU(config_file_path, data_path);
        imu_instance.set_initialization(initial_state, initial_covariance, dummy_data);
        imu_instance.read_IMU_measurements();
    #else
        IMU imu_instance = IMU(config_file_path, measurements_queue);
    #endif
}