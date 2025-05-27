#pragma once

#include "IMU_Matrices.hpp"
#include "measurement_handler.hpp"

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
        #ifdef SIM_MODE
            IMU(const std::string configs_path, std::string path_to_measurements_csv);
        #else
            IMU(const std::string configs_path, std::queue<ImuData>& imu_measurements_queue);
        #endif

        // provide an external solution for the kickoff of the IMU
        void set_initialization(ImuStateVector& initial_solution, 
                                ImuCovariance& initial_covariance, 
                                ImuData& initial_measurement);

        // process IMU measurements
        // TODO - this is currently going to read one line from a static file for pre-recorded measurements
        void read_IMU_measurements();

        // perform time update
        void perform_time_update(ImuData imu_measurements);

        // TODO - make something for a << operator for terminal viewing
        bool get_measurements(ImuData new_data);

    private:
        // basic app configs, state vector size
        std::string _configuration_file_path;
        std::string _measurements_file_path;  // the path to the pre-recorded messages
        Config _config;  // defined in IMU matrices.hpp

        bool _solution_initialized;
        uint _num_states;

        // solutions and points of linearization
        ImuStateVector _nominal_states;  // points of linearization of the state vector
        ImuStateVector _delta_states;  // the perturbation from the linearization points set as the _nominal_state_vector 
        ImuCovariance _state_covariance;
        ImuData _nominal_measurements;  // points of linearization for the IMU measurements
        double _solution_time;  // last solution time

        // main math model for the IMU state/covariance propagation
        GeneratedMatrices _state_space_model;  // from IMU matrices.hpp

        // the handler for incoming IMU measurements
        std::unique_ptr<MeasurementHandler> _measurement_handler;

        // we have a path to sim data if we're running in sim mode
        #ifdef SIM_MODE
            std::string _measurements_file;
        #else
            // for non-sim mode, we'll hold a location for IMU measurements
            std::queue<ImuData>& _imu_measurements_queue;
        #endif
};
