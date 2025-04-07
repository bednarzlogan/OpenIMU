// This file is automatically generated by 'write_to_hpp.py'.
#pragma once

#include <Eigen/Dense>
#include <iostream>

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

// measurements and expected state vector definitions to be explicit
// about what we're assigning
struct ImuData {
    double accx, accy, accz;
    double dphix, dthetay, dpsiz;
    double measurement_time;
    Eigen::Matrix<double, 6, 1> matrix_form_measurement;

    // call to update doubles
    void updateFromMatrix() {
        accx    = matrix_form_measurement(0);
        accy    = matrix_form_measurement(1);
        accz    = matrix_form_measurement(2);
        dphix   = matrix_form_measurement(3);
        dthetay = matrix_form_measurement(4);
        dpsiz   = matrix_form_measurement(5);
    }

    void updateFromDoubles() {
        matrix_form_measurement(0) = accx;
        matrix_form_measurement(1) = accy;
        matrix_form_measurement(2) = accz;
        matrix_form_measurement(3) = dphix;
        matrix_form_measurement(4) = dthetay;
        matrix_form_measurement(5) = dpsiz;
    }
};

struct ImuStateVector { 
    double pos_x, pos_y, pos_z;
    double vel_x, vel_y, vel_z;
    double phi, theta, psi;
    double bias_x, bias_y, bias_z;
    double bias_phi, bias_theta, bias_psi;
    double solution_time;
    Eigen::Matrix<double, 15, 1> matrix_form_states;

    // call to update doubles
    void updateFromMatrix() {
        pos_x    = matrix_form_states(0);
        pos_y    = matrix_form_states(1);
        pos_z    = matrix_form_states(2);
        vel_x    = matrix_form_states(3);
        vel_y    = matrix_form_states(4);
        vel_z    = matrix_form_states(5);
        phi      = matrix_form_states(6);
        theta    = matrix_form_states(7);
        psi      = matrix_form_states(8);
        bias_x   = matrix_form_states(9);
        bias_y   = matrix_form_states(10);
        bias_z   = matrix_form_states(11);
        bias_phi = matrix_form_states(12);
        bias_theta = matrix_form_states(13);
        bias_psi = matrix_form_states(14);
    }
};

struct ImuCovariance {
    double solution_time;
    Eigen::Matrix<double, 15, 15> covariance_matrix;
};

class GeneratedMatrices {
public:
    // define the expectation for how many fields will be in an IMU meausrement
    // and how many states are in the state-space model
    static const uint8_t NUM_STATES_IMU = 15;
    static const uint8_t NUM_IMU_MEASUREMENTS = 6;

    GeneratedMatrices() = default;

    // discrete time IMU state transition matrix
    Eigen::Matrix<double, 15, 15> eval_phi_k()  {
        Eigen::Matrix<double, 15, 15> mat;

        double sin_theta = sin(theta);
        double cos_theta = cos(theta);
        double tan_theta = tan(theta);
        double sin_phi = sin(phi);
        double cos_phi = cos(phi);

        double x0 = Ts*(cos_phi*cos_theta*fbz + cos_theta*fby*sin_phi - fbx*sin_theta);  // x0
        double x1 = sin(psi);  // x1
        double x2 = cos_theta*fbx;  // x2
        double x3 = cos(psi);  // x3
        double x4 = cos_phi*x3;  // x4
        double x5 = sin_phi*x1;  // x5
        double x6 = sin_theta*x5 + x4;  // x6
        double x7 = sin_phi*x3;  // x7
        double x8 = cos_phi*x1;  // x8
        double x9 = sin_theta*x8 - x7;  // x9
        double x10 = Ts*(fby*x6 + fbz*x9 + x1*x2);  // x10
        double x11 = Ts*cos_theta;  // x11
        double x12 = -sin_theta*x7 + x8;  // x12
        double x13 = sin_theta*x4 + x5;  // x13
        double x14 = Ts*(-fby*x12 + fbz*x13 + x2*x3);  // x14
        double x15 = Ts*sin_phi;  // x15
        double x16 = Ts*cos_phi;  // x16
        double x17 = Ts*(cos_phi*theta_dot - psi_dot*sin_phi);  // x17
        double x18 = (1.0/cos(theta));  // x18
        double x19 = Ts*(cos_phi*psi_dot + sin_phi*theta_dot);  // x19

        mat(0, 0) = 1;
        mat(0, 1) = 0;
        mat(0, 2) = 0;
        mat(0, 3) = Ts;
        mat(0, 4) = 0;
        mat(0, 5) = 0;
        mat(0, 6) = 0;
        mat(0, 7) = 0;
        mat(0, 8) = 0;
        mat(0, 9) = 0;
        mat(0, 10) = 0;
        mat(0, 11) = 0;
        mat(0, 12) = 0;
        mat(0, 13) = 0;
        mat(0, 14) = 0;
        mat(1, 0) = 0;
        mat(1, 1) = 1;
        mat(1, 2) = 0;
        mat(1, 3) = 0;
        mat(1, 4) = Ts;
        mat(1, 5) = 0;
        mat(1, 6) = 0;
        mat(1, 7) = 0;
        mat(1, 8) = 0;
        mat(1, 9) = 0;
        mat(1, 10) = 0;
        mat(1, 11) = 0;
        mat(1, 12) = 0;
        mat(1, 13) = 0;
        mat(1, 14) = 0;
        mat(2, 0) = 0;
        mat(2, 1) = 0;
        mat(2, 2) = 1;
        mat(2, 3) = 0;
        mat(2, 4) = 0;
        mat(2, 5) = Ts;
        mat(2, 6) = 0;
        mat(2, 7) = 0;
        mat(2, 8) = 0;
        mat(2, 9) = 0;
        mat(2, 10) = 0;
        mat(2, 11) = 0;
        mat(2, 12) = 0;
        mat(2, 13) = 0;
        mat(2, 14) = 0;
        mat(3, 0) = 0;
        mat(3, 1) = 0;
        mat(3, 2) = 0;
        mat(3, 3) = 1;
        mat(3, 4) = 0;
        mat(3, 5) = 0;
        mat(3, 6) = 0;
        mat(3, 7) = -x0;
        mat(3, 8) = x10;
        mat(3, 9) = -x11*x3;
        mat(3, 10) = Ts*x12;
        mat(3, 11) = -Ts*x13;
        mat(3, 12) = 0;
        mat(3, 13) = 0;
        mat(3, 14) = 0;
        mat(4, 0) = 0;
        mat(4, 1) = 0;
        mat(4, 2) = 0;
        mat(4, 3) = 0;
        mat(4, 4) = 1;
        mat(4, 5) = 0;
        mat(4, 6) = x0;
        mat(4, 7) = 0;
        mat(4, 8) = -x14;
        mat(4, 9) = -x1*x11;
        mat(4, 10) = -Ts*x6;
        mat(4, 11) = -Ts*x9;
        mat(4, 12) = 0;
        mat(4, 13) = 0;
        mat(4, 14) = 0;
        mat(5, 0) = 0;
        mat(5, 1) = 0;
        mat(5, 2) = 0;
        mat(5, 3) = 0;
        mat(5, 4) = 0;
        mat(5, 5) = 1;
        mat(5, 6) = -x10;
        mat(5, 7) = x14;
        mat(5, 8) = 0;
        mat(5, 9) = Ts*sin_theta;
        mat(5, 10) = -cos_theta*x15;
        mat(5, 11) = -cos_theta*x16;
        mat(5, 12) = 0;
        mat(5, 13) = 0;
        mat(5, 14) = 0;
        mat(6, 0) = 0;
        mat(6, 1) = 0;
        mat(6, 2) = 0;
        mat(6, 3) = 0;
        mat(6, 4) = 0;
        mat(6, 5) = 0;
        mat(6, 6) = tan_theta*x17 + 1;
        mat(6, 7) = pow(x18, 2)*x19;
        mat(6, 8) = 0;
        mat(6, 9) = 0;
        mat(6, 10) = 0;
        mat(6, 11) = 0;
        mat(6, 12) = -Ts;
        mat(6, 13) = -tan_theta*x15;
        mat(6, 14) = -tan_theta*x16;
        mat(7, 0) = 0;
        mat(7, 1) = 0;
        mat(7, 2) = 0;
        mat(7, 3) = 0;
        mat(7, 4) = 0;
        mat(7, 5) = 0;
        mat(7, 6) = -x19;
        mat(7, 7) = 1;
        mat(7, 8) = 0;
        mat(7, 9) = 0;
        mat(7, 10) = 0;
        mat(7, 11) = 0;
        mat(7, 12) = 0;
        mat(7, 13) = -x16;
        mat(7, 14) = x15;
        mat(8, 0) = 0;
        mat(8, 1) = 0;
        mat(8, 2) = 0;
        mat(8, 3) = 0;
        mat(8, 4) = 0;
        mat(8, 5) = 0;
        mat(8, 6) = x17*x18;
        mat(8, 7) = tan_theta*x18*x19;
        mat(8, 8) = 1;
        mat(8, 9) = 0;
        mat(8, 10) = 0;
        mat(8, 11) = 0;
        mat(8, 12) = 0;
        mat(8, 13) = -x15*x18;
        mat(8, 14) = -x16*x18;
        mat(9, 0) = 0;
        mat(9, 1) = 0;
        mat(9, 2) = 0;
        mat(9, 3) = Ts;
        mat(9, 4) = 0;
        mat(9, 5) = 0;
        mat(9, 6) = 0;
        mat(9, 7) = 0;
        mat(9, 8) = 0;
        mat(9, 9) = -Ts/tau_ax + 1;
        mat(9, 10) = 0;
        mat(9, 11) = 0;
        mat(9, 12) = 0;
        mat(9, 13) = 0;
        mat(9, 14) = 0;
        mat(10, 0) = 0;
        mat(10, 1) = 0;
        mat(10, 2) = 0;
        mat(10, 3) = 0;
        mat(10, 4) = Ts;
        mat(10, 5) = 0;
        mat(10, 6) = 0;
        mat(10, 7) = 0;
        mat(10, 8) = 0;
        mat(10, 9) = 0;
        mat(10, 10) = -Ts/tau_ay + 1;
        mat(10, 11) = 0;
        mat(10, 12) = 0;
        mat(10, 13) = 0;
        mat(10, 14) = 0;
        mat(11, 0) = 0;
        mat(11, 1) = 0;
        mat(11, 2) = 0;
        mat(11, 3) = 0;
        mat(11, 4) = 0;
        mat(11, 5) = Ts;
        mat(11, 6) = 0;
        mat(11, 7) = 0;
        mat(11, 8) = 0;
        mat(11, 9) = 0;
        mat(11, 10) = 0;
        mat(11, 11) = -Ts/tau_az + 1;
        mat(11, 12) = 0;
        mat(11, 13) = 0;
        mat(11, 14) = 0;
        mat(12, 0) = 0;
        mat(12, 1) = 0;
        mat(12, 2) = 0;
        mat(12, 3) = Ts;
        mat(12, 4) = 0;
        mat(12, 5) = 0;
        mat(12, 6) = 0;
        mat(12, 7) = 0;
        mat(12, 8) = 0;
        mat(12, 9) = 0;
        mat(12, 10) = 0;
        mat(12, 11) = 0;
        mat(12, 12) = -Ts/tau_gx + 1;
        mat(12, 13) = 0;
        mat(12, 14) = 0;
        mat(13, 0) = 0;
        mat(13, 1) = 0;
        mat(13, 2) = 0;
        mat(13, 3) = 0;
        mat(13, 4) = Ts;
        mat(13, 5) = 0;
        mat(13, 6) = 0;
        mat(13, 7) = 0;
        mat(13, 8) = 0;
        mat(13, 9) = 0;
        mat(13, 10) = 0;
        mat(13, 11) = 0;
        mat(13, 12) = 0;
        mat(13, 13) = -Ts/tau_gy + 1;
        mat(13, 14) = 0;
        mat(14, 0) = 0;
        mat(14, 1) = 0;
        mat(14, 2) = 0;
        mat(14, 3) = 0;
        mat(14, 4) = 0;
        mat(14, 5) = Ts;
        mat(14, 6) = 0;
        mat(14, 7) = 0;
        mat(14, 8) = 0;
        mat(14, 9) = 0;
        mat(14, 10) = 0;
        mat(14, 11) = 0;
        mat(14, 12) = 0;
        mat(14, 13) = 0;
        mat(14, 14) = -Ts/tau_gz + 1;
        return mat;
    }

    // matrix which projects IMU accelerometer/gyro readouts onto state
    Eigen::Matrix<double, 15, 6> eval_gamma_uk()  {
        Eigen::Matrix<double, 15, 6> mat;

        double sin_theta = sin(theta);
        double cos_theta = cos(theta);
        double tan_theta = tan(theta);
        double sin_phi = sin(phi);
        double cos_phi = cos(phi);

        double x0 = cos(psi);  // x0
        double x1 = 0.5*pow(Ts, 2);  // x1
        double x2 = cos_theta*x1;  // x2
        double x3 = x0*x2;  // x3
        double x4 = sin(psi);  // x4
        double x5 = cos_phi*x4;  // x5
        double x6 = sin_phi*x0;  // x6
        double x7 = -sin_theta*x6 + x5;  // x7
        double x8 = -x1*x7;  // x8
        double x9 = sin_phi*x4;  // x9
        double x10 = cos_phi*x0;  // x10
        double x11 = sin_theta*x10 + x9;  // x11
        double x12 = x1*x11;  // x12
        double x13 = x2*x4;  // x13
        double x14 = sin_theta*x9 + x10;  // x14
        double x15 = x1*x14;  // x15
        double x16 = sin_theta*x5 - x6;  // x16
        double x17 = x1*x16;  // x17
        double x18 = -sin_theta*x1;  // x18
        double x19 = cos_theta*sin_phi;  // x19
        double x20 = x1*x19;  // x20
        double x21 = cos_phi*cos_theta;  // x21
        double x22 = x1*x21;  // x22
        double x23 = Ts*cos_theta;  // x23
        double x24 = -fbx*sin_theta + fby*x19 + fbz*x21;  // x24
        double x25 = (1.0/cos(theta));  // x25
        double x26 = cos_theta*fbx;  // x26
        double x27 = fby*x14 + fbz*x16 + x26*x4;  // x27
        double x28 = -fby*x7 + fbz*x11 + x0*x26;  // x28
        double x29 = x1*(tan_theta*x24 - x25*x28);  // x29
        double x30 = tan_theta*x27;  // x30
        double x31 = cos_phi*theta_dot - psi_dot*sin_phi;  // x31
        double x32 = 0.5*Ts;  // x32
        double x33 = tan_theta*x32;  // x33
        double x34 = x31*x33;  // x34
        double x35 = x34 + 1;  // x35
        double x36 = cos_phi*psi_dot + sin_phi*theta_dot;  // x36
        double x37 = cos_phi*x36;  // x37
        double x38 = pow(x25, 2)*x32;  // x38
        double x39 = sin_phi*x36;  // x39
        double x40 = x33*x39;  // x40
        double x41 = sin_phi + x33*x37;  // x41
        double x42 = Ts*x25;  // x42

        mat(0, 0) = x3;
        mat(0, 1) = x8;
        mat(0, 2) = x12;
        mat(0, 3) = 0;
        mat(0, 4) = 0;
        mat(0, 5) = 0;
        mat(1, 0) = x13;
        mat(1, 1) = x15;
        mat(1, 2) = x17;
        mat(1, 3) = 0;
        mat(1, 4) = 0;
        mat(1, 5) = 0;
        mat(2, 0) = x18;
        mat(2, 1) = x20;
        mat(2, 2) = x22;
        mat(2, 3) = 0;
        mat(2, 4) = 0;
        mat(2, 5) = 0;
        mat(3, 0) = x0*x23;
        mat(3, 1) = -Ts*x7;
        mat(3, 2) = Ts*x11;
        mat(3, 3) = 0;
        mat(3, 4) = x1*(-cos_phi*x24 + sin_phi*x25*x27);
        mat(3, 5) = x1*(cos_phi*x25*x27 + sin_phi*x24);
        mat(4, 0) = x23*x4;
        mat(4, 1) = Ts*x14;
        mat(4, 2) = Ts*x16;
        mat(4, 3) = x1*x24;
        mat(4, 4) = sin_phi*x29;
        mat(4, 5) = cos_phi*x29;
        mat(5, 0) = -Ts*sin_theta;
        mat(5, 1) = sin_phi*x23;
        mat(5, 2) = cos_phi*x23;
        mat(5, 3) = -x1*x27;
        mat(5, 4) = x1*(cos_phi*x28 - sin_phi*x30);
        mat(5, 5) = -x1*(cos_phi*x30 + sin_phi*x28);
        mat(6, 0) = 0;
        mat(6, 1) = 0;
        mat(6, 2) = 0;
        mat(6, 3) = Ts*x35;
        mat(6, 4) = Ts*(sin_phi*tan_theta*x35 + x37*x38);
        mat(6, 5) = Ts*(cos_phi*tan_theta*x35 - x38*x39);
        mat(7, 0) = 0;
        mat(7, 1) = 0;
        mat(7, 2) = 0;
        mat(7, 3) = -x1*x36;
        mat(7, 4) = Ts*(cos_phi - x40);
        mat(7, 5) = -Ts*x41;
        mat(8, 0) = 0;
        mat(8, 1) = 0;
        mat(8, 2) = 0;
        mat(8, 3) = x1*x25*x31;
        mat(8, 4) = x42*(sin_phi*x34 + x41);
        mat(8, 5) = x42*(cos_phi*x34 + cos_phi - x40);
        mat(9, 0) = x3;
        mat(9, 1) = x8;
        mat(9, 2) = x12;
        mat(9, 3) = 0;
        mat(9, 4) = 0;
        mat(9, 5) = 0;
        mat(10, 0) = x13;
        mat(10, 1) = x15;
        mat(10, 2) = x17;
        mat(10, 3) = 0;
        mat(10, 4) = 0;
        mat(10, 5) = 0;
        mat(11, 0) = x18;
        mat(11, 1) = x20;
        mat(11, 2) = x22;
        mat(11, 3) = 0;
        mat(11, 4) = 0;
        mat(11, 5) = 0;
        mat(12, 0) = x3;
        mat(12, 1) = x8;
        mat(12, 2) = x12;
        mat(12, 3) = 0;
        mat(12, 4) = 0;
        mat(12, 5) = 0;
        mat(13, 0) = x13;
        mat(13, 1) = x15;
        mat(13, 2) = x17;
        mat(13, 3) = 0;
        mat(13, 4) = 0;
        mat(13, 5) = 0;
        mat(14, 0) = x18;
        mat(14, 1) = x20;
        mat(14, 2) = x22;
        mat(14, 3) = 0;
        mat(14, 4) = 0;
        mat(14, 5) = 0;
        return mat;
    }

    // matrix which projects IMU accelerometer/gyro reading noise onto state
    Eigen::Matrix<double, 15, 15> eval_gamma_wk()  {
        Eigen::Matrix<double, 15, 15> mat;

        double sin_theta = sin(theta);
        double cos_theta = cos(theta);
        double tan_theta = tan(theta);
        double sin_phi = sin(phi);
        double cos_phi = cos(phi);

        double x0 = cos(psi);  // x0
        double x1 = pow(cos_theta, 2);  // x1
        double x2 = sig_ax*x1;  // x2
        double x3 = sin(psi);  // x3
        double x4 = sin_phi*x3;  // x4
        double x5 = cos_phi*x0;  // x5
        double x6 = sin_theta*x5 + x4;  // x6
        double x7 = cos_phi*x3;  // x7
        double x8 = sin_phi*x0;  // x8
        double x9 = -sin_theta*x8 + x7;  // x9
        double x10 = sig_ay*pow(x9, 2) + sig_az*pow(x6, 2) + pow(x0, 2)*x2;  // x10
        double x11 = -x10;  // x11
        double x12 = sin_theta*x7 - x8;  // x12
        double x13 = sig_az*x6;  // x13
        double x14 = sin_theta*x4 + x5;  // x14
        double x15 = -sig_ay*x14*x9 + x0*x2*x3 + x12*x13;  // x15
        double x16 = -x15;  // x16
        double x17 = -cos_phi*x13 + sig_ax*sin_theta*x0 + sig_ay*sin_phi*x9;  // x17
        double x18 = cos_theta*x17;  // x18
        double x19 = sig_ay*pow(x14, 2) + sig_az*pow(x12, 2) + x2*pow(x3, 2);  // x19
        double x20 = -x19;  // x20
        double x21 = cos_phi*sig_az*x12 - sig_ax*sin_theta*x3 + sig_ay*sin_phi*x14;  // x21
        double x22 = -cos_theta*x21;  // x22
        double x23 = pow(cos_phi, 2);  // x23
        double x24 = pow(sin_phi, 2);  // x24
        double x25 = sig_ax*pow(sin_theta, 2) + sig_ay*x1*x24 + sig_az*x1*x23;  // x25
        double x26 = -x25;  // x26
        double x27 = -cos_theta*x17;  // x27
        double x28 = cos_phi*cos_theta*fbz + cos_theta*fby*sin_phi - fbx*sin_theta;  // x28
        double x29 = cos_phi*sin_phi;  // x29
        double x30 = x29*(sig_gy - sig_gz);  // x30
        double x31 = cos_theta*fbx;  // x31
        double x32 = fby*x14 + fbz*x12 + x3*x31;  // x32
        double x33 = (1.0/cos(theta));  // x33
        double x34 = sig_gz*x23;  // x34
        double x35 = sig_gy*x24;  // x35
        double x36 = x34 + x35;  // x36
        double x37 = x33*x36;  // x37
        double x38 = x28*x30 - x32*x37;  // x38
        double x39 = sig_gy*x23 + sig_gz*x24;  // x39
        double x40 = x30*x32;  // x40
        double x41 = cos_theta*x21;  // x41
        double x42 = pow(tan_theta, 2);  // x42
        double x43 = sig_gx + x34*x42 + x35*x42;  // x43
        double x44 = fbz*x6 + x0*x31;  // x44
        double x45 = -fby*x9 + x44;  // x45
        double x46 = tan_theta*x36;  // x46
        double x47 = -tan_theta*x28 + x33*x45;  // x47
        double x48 = -fby*x9 + x44;  // x48
        double x49 = x30*x48;  // x49
        double x50 = pow(x33, 2);  // x50
        double x51 = cos_phi*psi_dot + sin_phi*theta_dot;  // x51
        double x52 = tan_theta*x30;  // x52
        double x53 = x51*x52;  // x53
        double x54 = psi_dot*sin_phi;  // x54
        double x55 = cos_phi*theta_dot;  // x55
        double x56 = tan_theta*x54 - tan_theta*x55 + 1;  // x56
        double x57 = x39*x51;  // x57
        double x58 = x30*x51;  // x58
        double x59 = sig_tax + sig_tgx;  // x59
        double x60 = sig_tay + sig_tgy;  // x60
        double x61 = sin_phi*x60;  // x61
        double x62 = sig_taz + sig_tgz;  // x62
        double x63 = cos_phi*x62;  // x63
        double x64 = sig_gy*x29;  // x64
        double x65 = sig_gz*x29;  // x65
        double x66 = -x54 + x55;  // x66

        mat(0, 0) = 0;
        mat(0, 1) = 0;
        mat(0, 2) = 0;
        mat(0, 3) = x11;
        mat(0, 4) = x16;
        mat(0, 5) = x18;
        mat(0, 6) = 0;
        mat(0, 7) = 0;
        mat(0, 8) = 0;
        mat(0, 9) = 0;
        mat(0, 10) = 0;
        mat(0, 11) = 0;
        mat(0, 12) = 0;
        mat(0, 13) = 0;
        mat(0, 14) = 0;
        mat(1, 0) = 0;
        mat(1, 1) = 0;
        mat(1, 2) = 0;
        mat(1, 3) = x16;
        mat(1, 4) = x20;
        mat(1, 5) = x22;
        mat(1, 6) = 0;
        mat(1, 7) = 0;
        mat(1, 8) = 0;
        mat(1, 9) = 0;
        mat(1, 10) = 0;
        mat(1, 11) = 0;
        mat(1, 12) = 0;
        mat(1, 13) = 0;
        mat(1, 14) = 0;
        mat(2, 0) = 0;
        mat(2, 1) = 0;
        mat(2, 2) = 0;
        mat(2, 3) = x18;
        mat(2, 4) = x22;
        mat(2, 5) = x26;
        mat(2, 6) = 0;
        mat(2, 7) = 0;
        mat(2, 8) = 0;
        mat(2, 9) = 0;
        mat(2, 10) = 0;
        mat(2, 11) = 0;
        mat(2, 12) = 0;
        mat(2, 13) = 0;
        mat(2, 14) = 0;
        mat(3, 0) = 0;
        mat(3, 1) = 0;
        mat(3, 2) = 0;
        mat(3, 3) = x10;
        mat(3, 4) = x15;
        mat(3, 5) = x27;
        mat(3, 6) = tan_theta*x38;
        mat(3, 7) = x28*x39 - x33*x40;
        mat(3, 8) = x33*x38;
        mat(3, 9) = 0;
        mat(3, 10) = 0;
        mat(3, 11) = 0;
        mat(3, 12) = 0;
        mat(3, 13) = 0;
        mat(3, 14) = 0;
        mat(4, 0) = 0;
        mat(4, 1) = 0;
        mat(4, 2) = 0;
        mat(4, 3) = x15;
        mat(4, 4) = x19;
        mat(4, 5) = x41;
        mat(4, 6) = -x28*x43 + x33*x45*x46;
        mat(4, 7) = x30*x47;
        mat(4, 8) = x37*x47;
        mat(4, 9) = 0;
        mat(4, 10) = 0;
        mat(4, 11) = 0;
        mat(4, 12) = 0;
        mat(4, 13) = 0;
        mat(4, 14) = 0;
        mat(5, 0) = 0;
        mat(5, 1) = 0;
        mat(5, 2) = 0;
        mat(5, 3) = x27;
        mat(5, 4) = x41;
        mat(5, 5) = x25;
        mat(5, 6) = -tan_theta*x49 + x32*x43;
        mat(5, 7) = tan_theta*x40 - x39*x48;
        mat(5, 8) = x33*(tan_theta*x32*x36 - x49);
        mat(5, 9) = 0;
        mat(5, 10) = 0;
        mat(5, 11) = 0;
        mat(5, 12) = 0;
        mat(5, 13) = 0;
        mat(5, 14) = 0;
        mat(6, 0) = 0;
        mat(6, 1) = 0;
        mat(6, 2) = 0;
        mat(6, 3) = 0;
        mat(6, 4) = 0;
        mat(6, 5) = 0;
        mat(6, 6) = x43*x56 - x50*x53;
        mat(6, 7) = -x50*x57 + x52*x56;
        mat(6, 8) = x33*(tan_theta*x36*x56 - x50*x58);
        mat(6, 9) = 0;
        mat(6, 10) = 0;
        mat(6, 11) = 0;
        mat(6, 12) = x59;
        mat(6, 13) = tan_theta*x61;
        mat(6, 14) = tan_theta*x63;
        mat(7, 0) = 0;
        mat(7, 1) = 0;
        mat(7, 2) = 0;
        mat(7, 3) = 0;
        mat(7, 4) = 0;
        mat(7, 5) = 0;
        mat(7, 6) = tan_theta*x64 - tan_theta*x65 + x43*x51;
        mat(7, 7) = x39 + x53;
        mat(7, 8) = x33*(x46*x51 + x64 - x65);
        mat(7, 9) = 0;
        mat(7, 10) = 0;
        mat(7, 11) = 0;
        mat(7, 12) = 0;
        mat(7, 13) = cos_phi*x60;
        mat(7, 14) = -sin_phi*x62;
        mat(8, 0) = 0;
        mat(8, 1) = 0;
        mat(8, 2) = 0;
        mat(8, 3) = 0;
        mat(8, 4) = 0;
        mat(8, 5) = 0;
        mat(8, 6) = x33*(tan_theta*x34 + tan_theta*x35 - x42*x58 - x43*x66);
        mat(8, 7) = x33*(cos_phi*sig_gy*sin_phi - tan_theta*x57 - x52*x66 - x65);
        mat(8, 8) = x50*(x36 - x46*x66 - x53);
        mat(8, 9) = 0;
        mat(8, 10) = 0;
        mat(8, 11) = 0;
        mat(8, 12) = 0;
        mat(8, 13) = x33*x61;
        mat(8, 14) = x33*x63;
        mat(9, 0) = 0;
        mat(9, 1) = 0;
        mat(9, 2) = 0;
        mat(9, 3) = x11;
        mat(9, 4) = x16;
        mat(9, 5) = x18;
        mat(9, 6) = 0;
        mat(9, 7) = 0;
        mat(9, 8) = 0;
        mat(9, 9) = 0;
        mat(9, 10) = 0;
        mat(9, 11) = 0;
        mat(9, 12) = 0;
        mat(9, 13) = 0;
        mat(9, 14) = 0;
        mat(10, 0) = 0;
        mat(10, 1) = 0;
        mat(10, 2) = 0;
        mat(10, 3) = x16;
        mat(10, 4) = x20;
        mat(10, 5) = x22;
        mat(10, 6) = 0;
        mat(10, 7) = 0;
        mat(10, 8) = 0;
        mat(10, 9) = 0;
        mat(10, 10) = 0;
        mat(10, 11) = 0;
        mat(10, 12) = 0;
        mat(10, 13) = 0;
        mat(10, 14) = 0;
        mat(11, 0) = 0;
        mat(11, 1) = 0;
        mat(11, 2) = 0;
        mat(11, 3) = x18;
        mat(11, 4) = x22;
        mat(11, 5) = x26;
        mat(11, 6) = 0;
        mat(11, 7) = 0;
        mat(11, 8) = 0;
        mat(11, 9) = 0;
        mat(11, 10) = 0;
        mat(11, 11) = 0;
        mat(11, 12) = 0;
        mat(11, 13) = 0;
        mat(11, 14) = 0;
        mat(12, 0) = 0;
        mat(12, 1) = 0;
        mat(12, 2) = 0;
        mat(12, 3) = x11;
        mat(12, 4) = x16;
        mat(12, 5) = x18;
        mat(12, 6) = 0;
        mat(12, 7) = 0;
        mat(12, 8) = 0;
        mat(12, 9) = 0;
        mat(12, 10) = 0;
        mat(12, 11) = 0;
        mat(12, 12) = x59*(1 + 1.0/tau_gx);
        mat(12, 13) = 0;
        mat(12, 14) = 0;
        mat(13, 0) = 0;
        mat(13, 1) = 0;
        mat(13, 2) = 0;
        mat(13, 3) = x16;
        mat(13, 4) = x20;
        mat(13, 5) = x22;
        mat(13, 6) = 0;
        mat(13, 7) = 0;
        mat(13, 8) = 0;
        mat(13, 9) = 0;
        mat(13, 10) = 0;
        mat(13, 11) = 0;
        mat(13, 12) = 0;
        mat(13, 13) = x60*(1 + 1.0/tau_gy);
        mat(13, 14) = 0;
        mat(14, 0) = 0;
        mat(14, 1) = 0;
        mat(14, 2) = 0;
        mat(14, 3) = x18;
        mat(14, 4) = x22;
        mat(14, 5) = x26;
        mat(14, 6) = 0;
        mat(14, 7) = 0;
        mat(14, 8) = 0;
        mat(14, 9) = 0;
        mat(14, 10) = 0;
        mat(14, 11) = 0;
        mat(14, 12) = 0;
        mat(14, 13) = 0;
        mat(14, 14) = x62*(1 + 1.0/tau_gz);
        return mat;
    }

    void update_nominal_state(ImuStateVector& nominal_state, ImuData& imu_measurements) {
        // hard code for now 
        // make this a memory map auto generation from the python hpp generator
        // we'll deine the state vector as:
        // [r, v, E, b_a, b_g]
        // r -> position 
        // v -> velocity
        // E -> attitude
        // b_a -> moving IMU accelerometer bias
        // b_e -> moving IMU gyro bias

        // update attitude
        phi = nominal_state.phi;
        theta = nominal_state.theta;
        psi = nominal_state.psi;

        // the IMU measurements will be ingested as the linearization points for the attitude interpretation
        fbx = imu_measurements.accx;
        fby = imu_measurements.accy;
        fbz = imu_measurements.accz;

        psi_dot = imu_measurements.dpsiz;
        theta_dot = imu_measurements.dthetay;
    }

    void accept_configs(Config system_configs) {
        // open loaded configurations and assign member variables
        // accelerometer white noise
        sig_ax = system_configs.sig_ax;
        sig_ay = system_configs.sig_ay;
        sig_az = system_configs.sig_az;

        // gyro white noise
        sig_gx = system_configs.sig_gx;
        sig_gy = system_configs.sig_gy;
        sig_gz = system_configs.sig_gz;

        // FOGMP accelerometer white noise
        sig_tax = system_configs.sig_tax;
        sig_tay = system_configs.sig_tay;
        sig_taz = system_configs.sig_taz;

        // FOGMP gyro white noise
        sig_tgx = system_configs.sig_tgx;
        sig_tgy = system_configs.sig_tgy;
        sig_tgz = system_configs.sig_tgz;

        // time constants for FOGMPs
        tau_gx = system_configs.tau_gx;
        tau_gy = system_configs.tau_gy;
        tau_gz = system_configs.tau_gz;
    
        tau_ax = system_configs.tau_ax;
        tau_ay = system_configs.tau_ay;
        tau_az = system_configs.tau_az;  
        
        // set system update time
        Ts = system_configs.Ts;

        std::cout << "State space models are configured" << std::endl;
    }

    private:
        ///// IMU measurement variables /////

        // last known accelerometer reading
        double fbx;
        double fby;
        double fbz;

        // last known gyro readings
        // TODO - why is yaw not included? Double check derivation
        double psi_dot;
        double theta_dot;


        ///// from the state vector /////

        // last known attitude
        double phi;
        double theta;
        double psi;


        ///// confgiured system constants /////
        ///// NOTE: we may argue later that the sig_a/sig_g values can change with vibration conditions /////
        /// NOTE: we may want to have python do a subs call on these so that they're just static numbers in the matrices 

        // variance in acceleometer readings
        double sig_ax;
        double sig_ay;
        double sig_az;
    
        // variance in gyro readings
        double sig_gx;
        double sig_gy;
        double sig_gz;
    
        // white noise vairance for accel. FOGMP
        double sig_tax;
        double sig_tay;
        double sig_taz;
    
        // white noise variane for gyro FOGMP
        double sig_tgx;
        double sig_tgy;
        double sig_tgz;
    
        // time constant for gyro bias FOGMP
        double tau_gx;
        double tau_gy;
        double tau_gz;
    
        // time constant for accelerometer bias FOGMP
        double tau_ax;
        double tau_ay;
        double tau_az;

        // time between system updates
        double Ts;
};
