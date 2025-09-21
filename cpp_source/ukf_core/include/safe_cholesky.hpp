// safe_cholesky.hpp
#pragma once

#include <Eigen/Dense>
#include <iostream>
#include <cmath>

// TODO: make configurable later
constexpr double CHOL_SYM_EPS = 1e-10;
constexpr double CHOL_DIAG_FLOOR = 1e-9;
constexpr double CHOL_OFFDIAG_ZERO_THRESH = 1e-12;

/**
 * Ensure a matrix is symmetric positive-definite before performing Cholesky.
 *
 * @param P     The matrix to factor.
 * @param L     Output lower-triangular matrix from LLT.
 * @return      True if successful, false if the matrix had to be repaired significantly.
 */
template <int N>
bool safe_cholesky(const Eigen::Matrix<double, N, N>& P, Eigen::Matrix<double, N, N>& L) {
    using Mat = Eigen::Matrix<double, N, N>;

    // we enforce symmetry explicitly
    Mat sym_P = 0.5 * (P + P.transpose());

    // zero near-zero off-diagonals (roundoff noise)
    for (int i = 0; i < N; ++i) {
        for (int j = 0; j < N; ++j) {
            if (i != j && std::abs(sym_P(i, j)) < CHOL_OFFDIAG_ZERO_THRESH) {
                sym_P(i, j) = 0.0;
            }
        }
    }

    // add diagonal jitter to ensure positive-definiteness
    for (int i = 0; i < N; ++i) {
        if (sym_P(i, i) < CHOL_DIAG_FLOOR) {
            sym_P(i, i) = CHOL_DIAG_FLOOR;
        }
    }

    // finally, perform LLT
    Eigen::LLT<Mat> llt(sym_P);
    if (llt.info() == Eigen::Success) {
        L = llt.matrixL();
        return true;
    }

    std::cerr << "[safe_cholesky] ERROR: Cholesky failed after stabilization\n";
    std::cerr << sym_P << "\n";
    return false;
}
