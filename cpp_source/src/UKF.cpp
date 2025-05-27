#include "UKF.hpp"
#include "ukf_defs.hpp"

template<uint8_t N, uint8_t M>
UKF<N, M>::UKF(double alpha, double beta, double kappa, UKFParams params):
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
}

template<uint8_t N, uint8_t M>
void UKF<N, M>::generate_sigma_points(
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

template<uint8_t N, uint8_t M>
void UKF<N, M>::predict(
    const StateVec& mu, 
    const CovMat& P, 
    const ControlInput& u, double dt) {
    // use the nomlinear dynamics to propagate the sigma points into predicted states
    SigmaPointArray sigma_points;
    generate_sigma_points(mu, P, sigma_points); // get new sigma points

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
}