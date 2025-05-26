#pragma once
#include <Eigen/Dense>
#include <array>
#include <stdexcept>


/**
 * @note To log Eigen::VectorXd or Eigen::MatrixXd data, see `logger_conversions.hpp`.
 * These helpers allow safe down-conversion to float32 payloads.
 */


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
