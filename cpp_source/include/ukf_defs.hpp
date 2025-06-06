// Auto-generated by ukf_codegen.py
#pragma once

#include <Eigen/Dense>
#include <array>
#include <iostream>
#include <fstream>
#include <string>
using namespace std;

inline std::ofstream& ukf_log() {
    static std::ofstream file("ukf_trace_log.txt", std::ios::out | std::ios::trunc);
    return file;
}

constexpr int N = 15;
constexpr int M = 6;
constexpr int Z = 6;
constexpr int NumSigma = 2 * N + 1;

using StateVec = Eigen::Matrix<double, N, 1>;
using CovMat   = Eigen::Matrix<double, N, N>;
using MeasVec  = Eigen::Matrix<double, Z, 1>;
using MeasCov  = Eigen::Matrix<double, Z, Z>;
using ControlInput  = Eigen::Matrix<double, M, 1>;
using SigmaPointArray = std::array<StateVec, NumSigma>;

struct Observable {
    double timestamp;
    MeasVec observation;
    MeasCov R;
};

struct UKFParams {
    double tau_a, tau_g;       // time constants for gyro/accel FOGMP
    double sig_a, sig_g;       // variance on AWGN on FOGMP process
    double gx, gy, gz;         // gravity config
};

inline StateVec f_cont(
    const StateVec& x,
    const ControlInput& u,
    double tau_a, double tau_g,
    double gx, double gy, double gz) {

    StateVec dx;
    dx.setZero();

    ukf_log() << "[f_cont] Begin computing with inputs:"<< "\n";
    ukf_log() << "x: " << x.transpose()<< "\n";
    ukf_log() << "u: " << u.transpose()<< "\n";

    // Trig shorthands
    double phi = x(6), theta = x(7), psi = x(8);
    double sin_phi = sin(phi);
    double cos_phi = cos(phi);
    double sin_theta = sin(theta);
    double cos_theta = cos(theta);
    double tan_theta = tan(theta);
    double sec_theta = 1.0 / cos_theta;
    double sin_psi = sin(psi);
    double cos_psi = cos(psi);

    // diag printouts
    string theta_90 = (std::abs(cos_theta) < 1e-6) ? "[X]" : "[ ]";
    string tau_a_zero = (tau_a == 0) ? "[X]" : "[ ]";
    string tau_g_zero = (tau_g == 0) ? "[X]" : "[ ]";
    ukf_log() << "[f_cont] Zero targets => Theta 90: " << theta_90 << " tau_a: " << tau_a_zero << " tau_g: " << tau_g_zero << "\n";

    ukf_log() << "[f_cont] Angles: phi=" << phi << ", theta=" << theta << ", psi=" << psi<< "\n";
    ukf_log() << "[f_cont] Trig: cos_theta=" << cos_theta<< "\n";

    // common subexpressions
    double cse0 = u(3) - x(9);
    double cse1 = cos_theta*cse0;
    double cse2 = u(5) - x(11);
    double cse3 = sin_phi*sin_psi;
    double cse4 = cos_phi*cos_psi;
    double cse5 = u(4) - x(10);
    double cse6 = cos_phi*sin_psi;
    double cse7 = u(2) - x(14);
    double cse8 = cos_phi*cse7;
    double cse9 = u(1) - x(13);
    double cse10 = cse9*sin_phi;
    double cse11 = 1/tau_a;
    double cse12 = 1/tau_g;

    // state derivatives
    dx(0) = x(3);
    dx(1) = x(4);
    dx(2) = x(5);
    dx(3) = cos_psi*cse1 + cse2*(cse3 + cse4*sin_theta) + cse5*(cos_psi*sin_phi*sin_theta - cse6) - gx;
    dx(4) = cse1*sin_psi + cse2*(-cos_psi*sin_phi + cse6*sin_theta) + cse5*(cse3*sin_theta + cse4) - gy;
    dx(5) = cos_phi*cos_theta*cse2 + cos_theta*cse5*sin_phi - cse0*sin_theta - gz;
    dx(6) = cse10*tan_theta + cse8*tan_theta + u(0) - x(12);
    dx(7) = cos_phi*cse9 - cse7*sin_phi;
    dx(8) = cse10*sec_theta + cse8*sec_theta;
    dx(9) = -cse11*x(9);
    dx(10) = -cse11*x(10);
    dx(11) = -cse11*x(11);
    dx(12) = -cse12*x(12);
    dx(13) = -cse12*x(13);
    dx(14) = -cse12*x(14);

    ukf_log() << "[f_cont] dx: " << dx.transpose()<< "\n";
    return dx;
}

inline StateVec f_cont_debug(
    const StateVec& x,
    const ControlInput& u,
    double tau_a, double tau_g,
    double gx, double gy, double gz) {

    StateVec dx;

    // Unpack angles
    double phi = x(6);
    double theta = x(7);
    double psi = x(8);

    // Compute trig
    double sin_phi = sin(phi);
    double cos_phi = cos(phi);
    double sin_theta = sin(theta);
    double cos_theta = cos(theta);
    double tan_theta = tan(theta);
    double sec_theta = 1.0 / cos_theta;
    double sin_psi = sin(psi);
    double cos_psi = cos(psi);

    // Unpack state
    double bx = x(9);
    double by = x(10);
    double bz = x(11);
    double bphi = x(12);
    double btheta = x(13);
    double bpsi = x(14);

    // Unpack control
    double wx = u(0);
    double wy = u(1);
    double wz = u(2);
    double ax = u(3);
    double ay = u(4);
    double az = u(5);

    // Compute derivatives
    dx(0) = x(3);
    dx(1) = x(4);
    dx(2) = x(5);

    dx(3) = cos_psi * cos_theta * (ax - bx)
          + (-cos_phi * sin_psi + cos_psi * sin_phi * sin_theta) * (ay - by)
          + (cos_phi * cos_psi * sin_theta + sin_phi * sin_psi) * (az - bz)
          + gx;

    dx(4) = cos_theta * sin_psi * (ax - bx)
          + (cos_phi * cos_psi + sin_phi * sin_theta * sin_psi) * (ay - by)
          + (cos_phi * sin_theta * sin_psi - cos_psi * sin_phi) * (az - bz)
          + gy;

    dx(5) = -sin_theta * (ax - bx)
           + sin_phi * cos_theta * (ay - by)
           + cos_phi * cos_theta * (az - bz)
           + gz;

    dx(6) = wx - bphi + sin_phi * tan_theta * (wy - btheta) + cos_phi * tan_theta * (wz - bpsi);
    dx(7) = cos_phi * (wy - btheta) - sin_phi * (wz - bpsi);
    dx(8) = sin_phi * sec_theta * (wy - btheta) + cos_phi * sec_theta * (wz - bpsi);

    dx(9)  = -1.0 / tau_a * bx;
    dx(10) = -1.0 / tau_a * by;
    dx(11) = -1.0 / tau_a * bz;
    dx(12) = -1.0 / tau_g * bphi;
    dx(13) = -1.0 / tau_g * btheta;
    dx(14) = -1.0 / tau_g * bpsi;

    return dx;
}

inline StateVec rk4_step(const StateVec& x, const ControlInput& u, double dt, const UKFParams& params) { 
    ukf_log() << "[rk4_step] Starting RK4 step"<< "\n";

    StateVec k1 = f_cont(x, u, params.tau_a, params.tau_g, params.gx, params.gy, params.gz);
    StateVec k2 = f_cont(x + 0.5 * dt * k1, u, params.tau_a, params.tau_g, params.gx, params.gy, params.gz);
    StateVec k3 = f_cont(x + 0.5 * dt * k2, u, params.tau_a, params.tau_g, params.gx, params.gy, params.gz);
    StateVec k4 = f_cont(x + dt * k3, u, params.tau_a, params.tau_g, params.gx, params.gy, params.gz);

    StateVec result = x + (dt / 6.0) * (k1 + 2.0 * k2 + 2.0 * k3 + k4);
    ukf_log() << "[rk4_step] Completed RK4 step. Result: " << result.transpose()<< "\n";
    return result;
}
