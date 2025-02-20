#ifndef IMU_HPP // prevents multiple inclusions 
#define IMU_HPP

#include <iostream>
#include <fstream>

#include <string>
#include <vector>

#include <Eigen/Dense>
#include <nlohmann/json.hpp>

using Matrix = Eigen::MatrixXd;
using Vector = Eigen::VectorXd;
using json = nlohmann::json;

// the IMU model will be responsible for seamless production of the 
// time updated state

// configuations will be allowed for defining the level of simplification 
// and to specify the expected process noise characteristics 

// this script will produce discrete state estimates in PVA according to 
// its configuration 


// struct for setting up IMU dynamics based on an external file read
struct Config {
    // variance in accelerometer readings
    double sig_ax, sig_ay, sig_az;
    // variance in gyro readings
    double sig_gx, sig_gy, sig_gz;
    // white noise variance for accel. FOGMP
    double sig_tax, sig_tay, sig_taz;
    // white noise variance for gyro FOGMP
    double sig_tgx, sig_tgy, sig_tgz;
    // time constant for gyro bias FOGMP
    double tau_gx, tau_gy, tau_gz;
    // time constant for accelerometer bias FOGMP
    double tau_ax, tau_ay, tau_az;
    // time between system updates
    double Ts;
};

class IMU {
    public:
        // generic constructor definition 
        // we will want to add definitions for alignment procedures soon
        IMU(std::string configs_path);

        // TODO:
        //  we will work to leverage the matrices contained ni IMU_Matrices.hpp to accomplish the 
        //  dynamic updates.
        //  These matrices constitute a full IMU model, so interfacing with them will largely constitute
        //  updating the nominal state that the class holds.

        // Here, we fetch the state transition, control input matrix and process noise matrix
        // for dynamic updates. We provide a state vector to serve as our nominal states
        std::vector<Matrix> set_dynamics(Vector nominal_states);

        // process IMU measurements
        // TODO - this is currently going to read one line from a static file for pre-recorded measurements
        Vector read_IMU_measurements();

    private:
        std::string _configuration_file_path;
        std::string _measurements_file_path;  // the path to the pre-recorded messages
        Vector _nominal_states;
        Matrix _state_covariance;
};

#endif