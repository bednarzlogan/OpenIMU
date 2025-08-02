#pragma once

#include "estimator_interface.hpp"
#include "measurement_handler.hpp"
#include "ukf_defs.hpp"

#include <cstdint>
#include <Eigen/Dense>
#include <memory>
#include <string>
#include "measurement_model.hpp"


// using N and M for adaptive sizing depending on model without heap allocations
class UKF : public Estimator {
public:
    /// NOTE: use of an 8 bit integer limits the number of sigma points to 255 
    static constexpr uint8_t NumSigma = 2 * N + 1; // minimum sample from UKF theory

    UKF(double alpha, double beta, double kappa, UKFParams params);

    // alternate constructor for sim mode
    UKF(const std::string& configs_path);

    // sets params for system and gets measurement file path
    void read_configs(std::ifstream& inFile);

    // kicks off processing loop
    void start_filter();

    // kickoff for sim mode processing
    void read_measurements();

    /** inherited functions from the estimator class **/
    void initialize(const StateVec& initial_state, const CovMat& initial_covariance);

    void predict(const ControlInput& u, double dt);

    void update(const MeasVec& z, const MeasCov& R);

    // getters for state and covariance
    const StateVec& get_state() const;
    const CovMat& get_covariance() const;

private:
    // config 
    UKFParams _params; 

    // UKF scaling parameters
    double _alpha, _beta, _kappa, _lambda, _gamma;

    // weight params
    Eigen::Matrix<double, NumSigma, 1> _Wm;
    Eigen::Matrix<double, NumSigma, 1> _Wc;
    
    // state variables
    CovMat _P;
    StateVec _x;

    // process noise matrix
    CovMat _Q;

    // current sigma points
    SigmaPointArray _sigma_points;

    // the handler for incoming IMU measurements and external measurements
    std::unique_ptr<MeasurementHandler> _measurement_handler;
    std::unique_ptr<TruthHandler> _ext_measuremment_handler;

    std::string _measurement_file_path;
    std::string _ground_truth_path;
    double _solution_time;  // last solution time

    // logging object
    std::shared_ptr<Logger> diag_logger;

    void generate_sigma_points(const StateVec& mu, const CovMat& P, SigmaPointArray& sigma_points);
};