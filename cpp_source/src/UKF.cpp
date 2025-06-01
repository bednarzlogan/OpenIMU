#include <iostream>
#include <fstream>
#include <memory>
#include <nlohmann/json.hpp>
#include <thread>

#include "estimator_interface.hpp"
#include "UKF.hpp"
#include "ukf_defs.hpp"

using json = nlohmann::json;


void UKF::read_configs(std::ifstream& inFile) { 
    UKFParams hold_config;

    // access thre required members
    json j;
    inFile >> j;  // streams contents of the config file into 'j'

    // we can now access the features of the configs like a map or dict
    /// NOTE: the UKF only uses a subset of these
    hold_config.gx = j["gx"];
    hold_config.gy = j["gy"];
    hold_config.gz = j["gz"]; 

    // FOGMP process on biases config
    hold_config.tau_a = j["tau_ax"];
    hold_config.tau_g = j["tau_gx"];
    hold_config.sig_a = j["sig_ax"];
    hold_config.sig_g = j["sig_gx"];

    // scaling params for UKF
    _alpha = j["alpha"];
    _beta = j["beta"];
    _kappa = j["kappa"];

    // get measurement file path
    _measurement_file_path = j["path_to_measurement"];

    // example printouts
    std::cout << "Configuration loaded successfully.\n";
    std::cout << "Accelerometer time constant (tau_a): " << hold_config.tau_a << "\n";
    std::cout << "Gyro time constant (tau_g): " << hold_config.tau_g << "\n";
    _params = hold_config;  // assign configurations
}

UKF::UKF(double alpha, double beta, double kappa, UKFParams params):
    _params(params),
    _alpha(alpha),
    _beta(beta),
    _kappa(kappa) {

    // calculate scaling params
    _lambda = pow(_alpha, 2) * (N + _kappa) - N;
    _gamma = pow(N + _lambda, 0.5);

    // weights for sigma point mean and covariance
    _Wm.setConstant(0.5 / (N + _lambda));
    _Wc = _Wm; // copy

    _Wm(0) = _lambda / (N + _lambda); 
    _Wc(0) = _Wm(0) + (1 - pow(_alpha, 2) + _beta);

    // TMP PROCESS NOISE
    _Q.setIdentity();
    _Q *= 1e-3;

    // create a logger instance for diagnostics
    diag_logger = std::make_unique<Logger>(formatLogName("logs/IMU_diag_log", ".bin"));
}

UKF::UKF(const std::string& configs_path) {
    // read in configurables
    // load in the system configurations
    std::ifstream inFile(configs_path);
    if (!inFile.is_open()) {
      std::cerr << "Could not open config file: " << configs_path << std::endl;
    }

    // allocate the measurement file path and read in UKF params
    read_configs(inFile);

    // find ground truth file to compare
    _ground_truth_path = get_ground_truth(_measurement_file_path);

    // setup measurement handler
    _measurement_handler = std::make_unique<MeasurementHandler>(configs_path);

    // ground truth getter
    _ext_measuremment_handler = std::make_unique<TruthHandler>();

    // create a logger instance for diagnostics
    diag_logger = std::make_unique<Logger>(formatLogName("logs/UKF_diag_log", ".bin"));

    // calculate scaling params
    _lambda = pow(_alpha, 2) * (N + _kappa) - N;
    _gamma = pow(N + _lambda, 0.5);

    // weights for sigma point mean and covariance
    _Wm.setConstant(0.5 / (N + _lambda));
    _Wc = _Wm; // copy

    _Wm(0) = _lambda / (N + _lambda); 
    _Wc(0) = _Wm(0) + (1 - pow(_alpha, 2) + _beta);

    // TMP PROCESS NOISE
    _Q.setIdentity();
    _Q *= 1e-3;
}

void UKF::start_filter() {
    // TODO - setup measurement model here?? 
    read_measurements();
}

void UKF::read_measurements() {
    // open the measurements file -- TODO: this serves as the main parsing loop
    // we may maintain this structure as a while true statement where we continuously
    // read from a buffer of SPI-read IMU measurements

    // open the measurement stream via a handler in the background
    std::thread parser_thread(&MeasurementHandler::openMeasurementStream, 
                            _measurement_handler.get(), 
                            _measurement_file_path);

    // external measurements reader
    std::thread sim_measurements_thread(&TruthHandler::startStream,
                                        _ext_measuremment_handler.get(),
                                        _ground_truth_path);

    // set imu measurement for the time update
    ImuData imu_measurements;
    while (_measurement_handler->getSmoothedData(imu_measurements)) {
        // format into generic control input
        ControlInput imu_reading = imu_measurements.matrix_form_measurement;
        double dt = abs(_solution_time - imu_measurements.measurement_time);

        predict(imu_reading, dt);

        // Attempt to fetch a truth measurement close to the IMU time
        Observable external_measurement;
        if (_ext_measuremment_handler->getNextTruth(external_measurement)) {
            if (std::abs(external_measurement.timestamp - imu_measurements.measurement_time) < 1e-3) {
                update(external_measurement.observation, external_measurement.R);
            }
        }

        _solution_time = imu_measurements.measurement_time;
    }

    parser_thread.join(); 
    sim_measurements_thread.join();
}

const StateVec& UKF::get_state() const{
    return _x;
}

const CovMat& UKF::get_covariance() const{
    return _P;
}

void UKF::initialize(const StateVec& initial_state, const CovMat& initial_covariance) {
    _x = initial_state;
    _P = initial_covariance;
    _solution_time = 0;
}

void UKF::generate_sigma_points(
    const StateVec& mu,
    const CovMat& P,
    SigmaPointArray& sigma_points) {
    // compute scaled square root of covariance
    Eigen::LLT<CovMat> llt((N + _lambda) * P);
    CovMat S = llt.matrixL(); // cholesky decomp via llt

    // first point is always the mean
    sigma_points[0] = mu;

    // setup points on mirrored sides of the mean to capture
    // distribution
    for (uint8_t i = 0; i < N; ++i) {
        sigma_points[i + 1]     = mu + _gamma * S.col(i);
        sigma_points[N + i + 1] = mu - _gamma * S.col(i);
    }
}

void UKF::predict(
    const ControlInput& u, double dt) {
    // use the nomlinear dynamics to propagate the sigma points into predicted states
    SigmaPointArray sigma_points;
    generate_sigma_points(_x, _P, sigma_points); // get new sigma points

    // propagate each sigma point through the nonlinear dynamics using rk4
    SigmaPointArray propagated_sigmas;
    for (uint8_t i = 0; i < NumSigma; ++i) {
        propagated_sigmas[i] = rk4_step(sigma_points[i], u, dt, _params);
    }

    // compute predicted mean and covariance
    StateVec mu_pred;
    mu_pred.setZero();
    for (uint8_t i = 0; i < NumSigma; ++i) {
        mu_pred += _Wm[i] * propagated_sigmas[i];
    }

    CovMat P_pred;
    P_pred = _Q; // covariance update is of the form var(x) + Q
    for (uint8_t i = 0; i < NumSigma; ++i) {
        // "measure" the covariance based on deviations in the sigma points from the mean
        StateVec dx = propagated_sigmas[i] - mu_pred;
        P_pred += _Wc[i] * (dx * dx.transpose());
    }

    // update state & covariance
    _x = mu_pred;
    _P = P_pred;
    _sigma_points = propagated_sigmas;

    // log out the new, calculated state
    log_vector_out(*diag_logger, _x, LoggedVectorType::NominalState);
}

void UKF::update(const MeasVec& z, const MeasCov& R) {
    // propagate through measurement model
    std::array<MeasVec, NumSigma> z_sigma;
    for (uint8_t i = 0; i < NumSigma; ++i) {
        z_sigma[i] = _sigma_points[i].head<Z>(); // observing position + velocity
    }

    // predicted measurement mean
    MeasVec z_pred = MeasVec::Zero();
    for (uint8_t i = 0; i < NumSigma; ++i) {
        z_pred += _Wm[i] * z_sigma[i];
    }

    // this was our estimated measurement from the sigma points
    log_vector_out(*diag_logger, z_pred, LoggedVectorType::EstimatedMeasurement);

    // log resudial in this prediction
    Eigen::Matrix<double, Z, 1> residual = z - z_pred;
    log_vector_out(*diag_logger, residual, LoggedVectorType::MeasurementResidual);

    // innovation covariance and cross-covariance
    MeasCov S = R;
    Eigen::Matrix<double, N, Z> P_xz = Eigen::Matrix<double, N, Z>::Zero();
    for (uint8_t i = 0; i < NumSigma; ++i) {
        MeasVec dz = z_sigma[i] - z_pred;
        StateVec dx = _sigma_points[i] - _x;
        S += _Wc[i] * dz * dz.transpose();
        P_xz += _Wc[i] * dx * dz.transpose();
    }

    // Kalman gain
    Eigen::Matrix<double, N, Z> K = P_xz * S.inverse();

    // state update
    _x += K * (z - z_pred);
    _P -= K * S * K.transpose();

    // measurement updated state
    log_vector_out(*diag_logger, _x, LoggedVectorType::NominalState);
}