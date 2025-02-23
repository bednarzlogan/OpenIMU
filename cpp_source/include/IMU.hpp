#ifndef IMU_HPP // prevents multiple inclusions 
#define IMU_HPP

#include "IMU_Matrices.hpp"

#include <string>

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
        void set_dynamics(Vector nominal_states);

        // process IMU measurements
        // TODO - this is currently going to read one line from a static file for pre-recorded measurements
        void read_IMU_measurements();

        // perform time update
        void perform_time_update(ImuData imu_measurements);

    private:
        std::string _configuration_file_path;
        std::string _measurements_file_path;  // the path to the pre-recorded messages
        ImuStateVector _nominal_states;
        Matrix _state_covariance;
        Config _config;  // defined in IMU matrices.hpp
        GeneratedMatrices _state_space_model;  // from IMU matrices.hpp
        bool _solution_initialized;
};

#endif
