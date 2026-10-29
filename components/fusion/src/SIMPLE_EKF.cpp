#include "SIMPLE_EKF.h"

#include <cmath>

#include "ekf.h"

SimpleEKF::SimpleEKF(const SimpleEKFConfig& config)
    : ekf(3, 0), config_(config) {}

void SimpleEKF::Init() {
    // Initial state is null
    X *= 0;

    // Initial filter covariance matrix
    P *= 0;
    P(0, 0) = 1.0f;
    P(1, 1) = 1.0f;
    P(2, 2) = 1.0f;

    // Model noise covariance matrix
    Q *= 0;
    Q(0, 0) =
        std::pow(config_.Dt, 4.0f) / 4.0f * std::pow(config_.sigma_mz, 2.0f);
    Q(1, 0) =
        std::pow(config_.Dt, 3.0f) / 2.0f * std::pow(config_.sigma_mz, 2.0f);
    Q(0, 1) =
        std::pow(config_.Dt, 3.0f) / 2.0f * std::pow(config_.sigma_mz, 2.0f);
    Q(1, 1) = std::pow(config_.Dt, 2.0f) * std::pow(config_.sigma_mz, 2.0f);
    Q(2, 2) = std::pow(config_.sigma_mp, 2.0f);
}

void SimpleEKF::LinearizeFG(dspm::Mat& x, float* u) {
    F *= 0;
    G *= 0;  // This simple Kalman filter doesn't take control inputs.

    F(2, 1) = std::pow(x(2, 0), 1 - config_.beta_coef) / config_.gamma_coef *
              config_.Dt;
    F(2, 2) = 1 + (1 - config_.beta_coef) / config_.gamma_coef * x(2, 0) *
                      std::pow(x(2, 0), -config_.beta_coef) * config_.Dt;
}

dspm::Mat SimpleEKF::StateXdot(dspm::Mat& x, float* u) {
    dspm::Mat Xdot(3, 1);
    Xdot *= 0;

    Xdot(2, 0) =
        x(2, 0) + config_.Dt * (std::pow(x(2, 0), 1 - config_.beta_coef) *
                                x(2, 0) / config_.gamma_coef);

    return Xdot;
}
