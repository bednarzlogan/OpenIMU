#pragma once

#include "ukf_defs.hpp"

#include <Eigen/Dense>
#include <cstdint>

// using N and M for adaptive sizing depending on model without heap allocations
template<uint8_t N, uint8_t M>
class UKF {
public:
    /// NOTE: use of an 8 bit integer limits the number of sigma points to 255 
    static constexpr uint8_t NumSigma = 2 * N + 1; // minimum sample from UKF theory

    UKF(double alpha, double beta, double kappa, UKFParams params);

    void predict(const StateVec& mu, const CovMat& P, const ControlInput& u, double dt);

    void update(const StateVec& mu_pred, const CovMat& P_pred, SigmaPointArray propagated_sigmas, MeasVec z);

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

    void generate_sigma_points(const StateVec& mu, const CovMat& P, SigmaPointArray& sigma_points);
};