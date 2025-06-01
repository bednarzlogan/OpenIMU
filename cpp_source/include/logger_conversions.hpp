#pragma once
#include <Eigen/Dense>
#include <array>
#include <stdexcept>

#include "logger.hpp"
#include "ukf_defs.hpp"


/**
 * @note To log Eigen::VectorXd or Eigen::MatrixXd data, see `logger_conversions.hpp`.
 * These helpers allow safe down-conversion to float32 payloads.
 */


 // used for logging states of interest
constexpr uint16_t MSG_ID_NOMINAL_STATE = 0x01;
constexpr uint16_t MSG_ID_DELTA_STATE_UPDATE = 0x02;
constexpr uint16_t MSG_ID_RAW_IMU_MEASUREMENT = 0x03;
constexpr uint16_t MSG_ID_SMOOTHED_IMU_MEASUREMENT = 0x04;
constexpr uint16_t MSG_ID_GROUND_TRUTH = 0x05;
constexpr uint16_t MSG_ID_ESTIMATED_MEAS = 0x06;
constexpr uint16_t MSG_ID_MEASUREMENT_RESIDUAL = 0x07;


enum class LoggedVectorType : uint16_t {
    NominalState = MSG_ID_NOMINAL_STATE,
    DeltaStateUpdate = MSG_ID_DELTA_STATE_UPDATE,
    ImuRaw = MSG_ID_RAW_IMU_MEASUREMENT, 
    ImuSmoothed =  MSG_ID_SMOOTHED_IMU_MEASUREMENT,
    GroundTruth = MSG_ID_GROUND_TRUTH,
    EstimatedMeasurement = MSG_ID_ESTIMATED_MEAS,
    MeasurementResidual = MSG_ID_MEASUREMENT_RESIDUAL
};


/**
 * @brief Convert a dynamic Eigen column vector to a fixed-size std::array<float, N>
 *
 * @tparam N Expected size of the Eigen vector.
 * @param vec Dynamic Eigen column vector (Nx1).
 * @return std::array<float, N> Down-converted to float.
 * @throws std::invalid_argument if vec.size() != N or vec is not a column vector.
 */
template <size_t N>
std::array<float, N> convertVectorForLogging(const Eigen::Matrix<double, Eigen::Dynamic, 1>& vec) {
    if (vec.rows() != static_cast<int>(N) || vec.cols() != 1) {
        throw std::invalid_argument("convertVectorForLogging: Expected column vector of size N×1");
    }
    
    std::array<float, N> out;
    for (size_t i = 0; i < N; ++i)
        out[i] = static_cast<float>(vec(i));
    return out;
}


inline void log_vector_out(Logger& diag_logger,
                           const Eigen::Matrix<double, Eigen::Dynamic, 1>& vec,
                           LoggedVectorType type) {
    switch (type) {
        case LoggedVectorType::NominalState:
            diag_logger.logMessage<N>(MSG_ID_NOMINAL_STATE, DataType::Float32, 0x01,
                                       convertVectorForLogging<N>(vec));
            break;
        case LoggedVectorType::DeltaStateUpdate:
            diag_logger.logMessage<N>(MSG_ID_DELTA_STATE_UPDATE, DataType::Float32, 0x01,
                                       convertVectorForLogging<N>(vec));
            break;
        case LoggedVectorType::ImuRaw:
            diag_logger.logMessage<M>(MSG_ID_RAW_IMU_MEASUREMENT, DataType::Float32, 0x01,
                                      convertVectorForLogging<M>(vec));
            break;
        case LoggedVectorType::ImuSmoothed:
            diag_logger.logMessage<M>(MSG_ID_SMOOTHED_IMU_MEASUREMENT, DataType::Float32, 0x01,
                                      convertVectorForLogging<M>(vec));
            break;
        case LoggedVectorType::GroundTruth:
            diag_logger.logMessage<Z>(MSG_ID_GROUND_TRUTH, DataType::Float32, 0x01,
                                      convertVectorForLogging<Z>(vec));
            break;
        case LoggedVectorType::EstimatedMeasurement:
            diag_logger.logMessage<Z>(MSG_ID_ESTIMATED_MEAS, DataType::Float32, 0x01,
                                      convertVectorForLogging<Z>(vec));
            break;
        case LoggedVectorType::MeasurementResidual:
            diag_logger.logMessage<Z>(MSG_ID_MEASUREMENT_RESIDUAL, DataType::Float32, 0x01,
                                      convertVectorForLogging<Z>(vec));
            break;
        default:
            throw std::invalid_argument("log_vector_out: Unrecognized message type");
    }
}
