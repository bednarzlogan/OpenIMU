#include "IMU.hpp"
#include "Eigen/src/Core/Matrix.h"
#include "IMU_Matrices.hpp"

// for diag_log
#include "logger.hpp" 

#include <iostream>
#include <fstream>
#include <array>
#include <cmath>
#include <thread>

Logger diag_logger(formatLogName("logs/IMU_diag_log", ".bin")); // create a logger instance for diagnostics

Config read_configs(std::ifstream& inFile) { 
    Config hold_config;

    // access thre required members
    json j;
    inFile >> j;  // streams contents of the config file into 'j'

    // we can now access the features of the configs like a map or dict
    hold_config.sig_ax  = j["sig_ax"];
    hold_config.sig_ay  = j["sig_ay"];
    hold_config.sig_az  = j["sig_az"];
    hold_config.sig_gx  = j["sig_gx"];
    hold_config.sig_gy  = j["sig_gy"];
    hold_config.sig_gz  = j["sig_gz"];
    hold_config.sig_tax = j["sig_tax"];
    hold_config.sig_tay = j["sig_tay"];
    hold_config.sig_taz = j["sig_taz"];
    hold_config.sig_tgx = j["sig_tgx"];
    hold_config.sig_tgy = j["sig_tgy"];
    hold_config.sig_tgz = j["sig_tgz"];
    hold_config.tau_gx  = j["tau_gx"];
    hold_config.tau_gy  = j["tau_gy"];
    hold_config.tau_gz  = j["tau_gz"];
    hold_config.tau_ax  = j["tau_ax"];
    hold_config.tau_ay  = j["tau_ay"];
    hold_config.tau_az  = j["tau_az"];
    hold_config.Ts      = j["Ts"]; 

    // example printouts
    std::cout << "Configuration loaded successfully.\n";
    std::cout << "Accelerometer variance (sig_ax): " << hold_config.sig_ax << "\n";
    std::cout << "System update time (Ts): " << hold_config.Ts << "\n";
    return hold_config;
}


#ifdef SIM_MODE
  IMU::IMU(const std::string configs_path, std::string path_to_measurements_csv):
  _configuration_file_path(configs_path), 
  _solution_initialized(false),
  _measurements_file_path(path_to_measurements_csv),
  _solution_time(0.0) {
    std::cout << "Hello sim mode" << std::endl;
    // load in the system configurations
    std::ifstream inFile(configs_path);
    if (!inFile.is_open()) {
      std::cerr << "Could not open config file: " << configs_path << std::endl;
    }

    // read in configs via json tools
    _config = read_configs(inFile);

    // set configs into state space model manager
    _state_space_model = GeneratedMatrices();
    _state_space_model.accept_configs(_config);

    _num_states = _state_space_model.NUM_STATES_IMU;

    // start the measurement handler
    _measurement_handler = std::make_unique<MeasurementHandler>(configs_path);  
  }
#else
  IMU::IMU(std::string configs_path, std::queue<ImuData>& imu_measurements_queue):
  _configuration_file_path(configs_path), 
  _solution_initialized(false),
  _imu_measurements_queue(imu_measurements_queue),
  _solution_time(0.0) { 
    std::cout << "Hello real mode" << std::endl;
    // load in the system configurations
    std::ifstream inFile(configs_path);
    if (!inFile.is_open()) {
      std::cerr << "Could not open config file: " << configs_path << std::endl;
    }

    // read in configs via json tools
    _config = read_configs(inFile);

    // set configs into state space model manager
    _state_space_model = GeneratedMatrices();
    _state_space_model.accept_configs(_config);

    // we can find our state vector size from the state space model generator
    _num_states = _state_space_model.NUM_STATES_IMU;

    // start the measurement handler
    _measurement_handler = std::make_unique<MeasurementHandler>(configs_path);
  }
#endif


void IMU::set_initialization(ImuStateVector& initial_solution, 
                             ImuCovariance& initial_covariance, 
                             ImuData& initial_measurement) {
  // the state vector components are actually expressed as a deviation from the last known solution
  _nominal_states = initial_solution;
  _state_covariance = initial_covariance;

  // set the initial perturbation state to be zeros (perturbation covariance is the same as the regular state covariance)
  // TODO -- can we make this adjust to the size of hte IMU state vector without requiring dynamic sizing?
  _delta_states.matrix_form_states = Eigen::Matrix<double, 15, 1>::Zero(); 

  // set the initial points of linearization for the IMU measurements to be this inital measurement
  _nominal_measurements = initial_measurement;
  
  _solution_initialized = true;
}


void IMU::read_IMU_measurements() {
  // open the measurements file -- TODO: this serves as the main parsing loop
  // we may maintain this structure as a while true statement where we continuously
  // read from a buffer of SPI-read IMU measurements

  // open the measurement stream via a handler in the background
  std::thread parser_thread(&MeasurementHandler::openMeasurementStream, 
                            _measurement_handler.get(), 
                            _measurements_file_path);

  // set imu measurement for the time update
  ImuData imu_measurements;
  while (_measurement_handler->getSmoothedData(imu_measurements)) {
    perform_time_update(imu_measurements);
  }

  parser_thread.join(); 
}


bool IMU::get_measurements(ImuData new_data) {
    // connect to wherever we're watching for IMU measurements
    #ifdef SIM_MODE
      return false;
    #else
      if (!_imu_measurements_queue.empty()) {
        perform_time_update(_imu_measurements_queue.front());
        return true;
      }
    #endif

    // if we got here, no meausrements were available
    return false;
}

void IMU::perform_time_update(ImuData imu_measurements) {
  // TODO - not using solution times because we're not interfacing with real data yet
  // This function is the core of what the IMU provides to the system runtime
  // Here, we will:
    // read in new IMU measurements (from a file, for now)
    // pass updated IMU measurements and linearization points to the state space manager

  // check that we've initialized
  if (!_solution_initialized)
    return;

  // update the nominal states
  _state_space_model.update_nominal_state(_nominal_states, _nominal_measurements);

  // update the delta for IMU measurements and state
  // TODO -- really should avoid dynamic sizing, where possible
  Eigen::MatrixXd delta_imu_measurements = imu_measurements.matrix_form_measurement - _nominal_measurements.matrix_form_measurement;

  // get state space matrices -- TODO we should statically allocate these when we're sure on what model we want
  std::array<Eigen::MatrixXd, 3> dynamic_system = _state_space_model.eval_generated_system(); 
  Eigen::MatrixXd Phi_k = dynamic_system[0];
  Eigen::MatrixXd Gam_uk = dynamic_system[1];
  Eigen::MatrixXd Gam_wk = dynamic_system[2]; // TODO - actually Q, not the process noise input

  // update nominal state estimate
  // TODO -- this should actually be formed from the deviation state vector
  // TODO -- the IMU measurements are also supposed to be in linearized form (delta states)
  _delta_states.matrix_form_states = Phi_k * _delta_states.matrix_form_states + Gam_uk * delta_imu_measurements;

  // update nominal state covariance
  _state_covariance.covariance_matrix = Phi_k * _state_covariance.covariance_matrix * Phi_k.transpose() + Gam_wk;

  // update nominal state (dx + x_nom) -> x
  _nominal_states.matrix_form_states += _delta_states.matrix_form_states;

  _nominal_states.updateFromMatrix();  // calls a function from the struct to populate the enumerated doubles
  _nominal_measurements = imu_measurements;  // we'll observe perturbations from this state of the body to contextualize future measurements 
  _solution_time = imu_measurements.measurement_time; // update the solution time to the latest measurement time

  // diag log out for state influence from control input
  Eigen::MatrixXd state_perturbation = Gam_uk * delta_imu_measurements;
  std::cout << "TMP debug state perturbation \n" << state_perturbation << "\n" << std::endl;

  // log out relevant states
  _measurement_handler->log_vector_out(diag_logger, _nominal_states.matrix_form_states, LoggedVectorType::NominalState);
  _measurement_handler->log_vector_out(diag_logger, state_perturbation, LoggedVectorType::DeltaStateUpdate);
  std::cout << "Pushed nominal state" << _nominal_states.matrix_form_states << "\n" <<std::endl;
  return;
}