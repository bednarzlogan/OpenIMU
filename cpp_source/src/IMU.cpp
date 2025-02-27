#include "IMU.hpp"
#include "IMU_Matrices.hpp"

#include <iostream>
#include <fstream>
#include <sstream>
#include <vector>


IMU::IMU(std::string configs_path): _configuration_file_path(configs_path), _solution_initialized(false) { 
  // load in the system configurations
  std::ifstream inFile(configs_path);
  if (!inFile.is_open()) {
    std::cerr << "Could not open config file: " << configs_path << std::endl;
  }
  
  // access thre required members
  json j;
  inFile >> j;  // streams contents of the config file into 'j'

  // we can now access the features of the configs like a map or dict
  _config.sig_ax  = j["sig_ax"];
  _config.sig_ay  = j["sig_ay"];
  _config.sig_az  = j["sig_az"];
  _config.sig_gx  = j["sig_gx"];
  _config.sig_gy  = j["sig_gy"];
  _config.sig_gz  = j["sig_gz"];
  _config.sig_tax = j["sig_tax"];
  _config.sig_tay = j["sig_tay"];
  _config.sig_taz = j["sig_taz"];
  _config.sig_tgx = j["sig_tgx"];
  _config.sig_tgy = j["sig_tgy"];
  _config.sig_tgz = j["sig_tgz"];
  _config.tau_gx  = j["tau_gx"];
  _config.tau_gy  = j["tau_gy"];
  _config.tau_gz  = j["tau_gz"];
  _config.tau_ax  = j["tau_ax"];
  _config.tau_ay  = j["tau_ay"];
  _config.tau_az  = j["tau_az"];
  _config.Ts      = j["Ts"]; 
  
  // example printouts
  std::cout << "Configuration loaded successfully.\n";
  std::cout << "Accelerometer variance (sig_ax): " << _config.sig_ax << "\n";
  std::cout << "System update time (Ts): " << _config.Ts << "\n";

  // set configs into state space model manager
  _state_space_model = GeneratedMatrices();
  _state_space_model.accept_configs(_config);
}


void IMU::set_initialization(const ImuStateVector& initial_solution, 
                             const ImuCovariance& initial_covariance, 
                             const ImuData& initial_measurement) {
  // the state vector components are actually expressed as a deviation from the last known solution
  _nominal_states = initial_solution;
  _state_covariance = initial_covariance;

  // sanity check the dimension of the state vector & diagonal of the covariance
  if (initial_solution.len != initial_covariance.len) {
    std::string error_string = "Reveived a state vector with a size that doesn't match the diagonal of the supposed state covariance: (";
    std::cerr << error_string << initial_solution.len << ", " << initial_covariance.len << ")" << std::endl; 
    return;
  }

  // set the initial perturbation state to be zeros (perturbation covariance is the same as the regular state covariance)
  _num_states = initial_solution.len;
  _delta_states.matrix_form_states = Eigen::MatrixXd::Zero(_num_states); 

  // set the initial points of linearization for the IMU measurements to be this inital measurement
  _nominal_measurements = initial_measurement;
  
  _solution_initialized = true;
  return;
}


void IMU::read_IMU_measurements() {
  // open the measurements file -- TODO: this serves as the main parsing loop
  // we may maintain this structure as a while true statement where we continuously
  // read from a buffer of SPI-read IMU measurements
  std::ifstream infile(_measurements_file_path);

  std::string measurement_line;
  std::string line;

  // lines one at a time
  while (std::getline(infile, line, '\n')) {
    // assign a string stream to break up words
    std::stringstream ss(line);
    std::vector<std::string> row;
    std::string value;

    // interact with the stringstream to grab each word between delimiters
    while (std::getline(ss, value, ',')) { // Parse CSV by commas
      row.push_back(value);
    }
    
    // TMP print outputs
    for (const auto& elem : row) {
      std::cout << elem << " ";
    }
    std::cout << std::endl;

    // check for a complete 6 elements and convert to doubles
    if (row.size() != 7) {
      std::cerr << "Less than 6 measurements contained in row. CSV format is invalid!" << std::endl;
      return;
    }

    // allocate the measurement and assign the variables
    ImuData imu_measurements;
    uint count = 0;

    // kind of lengthy, but I'm doing this to be explicit about what is what in assigning members in the 
    // state space model
    for (const auto& elem : row) {
      double measurement = std::stod(value); // convert to double and assign
      switch (count) {
        case 0:
          imu_measurements.accx = measurement;
          break;
        case 1:
          imu_measurements.accy = measurement;
          break;
        case 2:
          imu_measurements.accz = measurement;
          break;
        case 3:
          imu_measurements.dphix = measurement;
          break;
        case 4:
          imu_measurements.dthetay = measurement;
          break;
        case 5:
          imu_measurements.dpsiz = measurement;
          break;
        case 6:
          imu_measurements.measurement_time = measurement;
        default:
          __builtin_unreachable();
      }
      count++;
    }

    // set imu measurement for the time update
    imu_measurements.updateFromMatrix();  // populate enumerated doubles for explicit language in later functions 
    perform_time_update(imu_measurements);
  }

  // close the file
  infile.close();
  return;
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
  _state_space_model.update_nominal_state(_nominal_states, imu_measurements);

  // update the delta for IMU measurements and state
  // TODO -- really should avoid dynamic sizing, where possible
  Eigen::MatrixXd delta_imu_measurements = imu_measurements.matrix_form_measurement - _nominal_measurements.matrix_form_measurement;

  // get state space matrices -- TODO we should statically allocate these when we're sure on what model we want
  Eigen::MatrixXd Phi_k = _state_space_model.eval_phi_k();
  Eigen::MatrixXd Gam_uk = _state_space_model.eval_gamma_uk();
  Eigen::MatrixXd Gam_wk = _state_space_model.eval_gamma_wk(); // TODO - actually Q, not the process noise input

  // update nominal state estimate
  // TODO -- this should actually be formed from the deviation state vector
  // TODO -- the IMU measurements are also supposed to be in linearized form (delta states)
  _delta_states.matrix_form_states = Phi_k * _delta_states.matrix_form_states + Gam_uk * delta_imu_measurements;

  // update nominal state covariance
  _state_covariance.covariance_matrix = Phi_k * _state_covariance.covariance_matrix * Phi_k.transpose() + Gam_wk;

  // update nominal state (x - x_nom) -> x
  _nominal_states.matrix_form_states = _delta_states.matrix_form_states + _nominal_states.matrix_form_states;
  _nominal_states.updateFromMatrix();  // calls a function from the struct to populate the enumerated doubles
  return;
}