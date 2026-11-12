#include "SIMPLE_EKF.h"

#include <cmath>

#include "ekf.h"

constexpr float sqr(float x) { return x * x; }
constexpr float cub(float x) { return x * x * x; }
constexpr float tet(float x) { return x * x * x * x; }

SimpleEKF::SimpleEKF(const SimpleEKFConfig& config)
    : ekf(4, 4), config_(config) {}

void SimpleEKF::Init() {
    // Initial state is null
    X *= 0;
    X(3, 0) = 101325;  // Pa

    // Initial filter covariance matrix
    P = dspm::Mat::eye(NUMX);
    G = dspm::Mat::eye(NUMW);

    // Model noise covariance matrix
    Q *= 0;
    Q(0, 0) = tet(config_.Dt) / 4.0f * sqr(config_.sigma_mz);
    Q(1, 0) = cub(config_.Dt) / 2.0f * sqr(config_.sigma_mz);
    Q(0, 1) = cub(config_.Dt) / 2.0f * sqr(config_.sigma_mz);
    Q(1, 1) = sqr(config_.Dt) * sqr(config_.sigma_mz);
    Q(2, 2) = sqr(config_.sigma_mz);
    Q(3, 3) = sqr(config_.sigma_mp);
}

void SimpleEKF::LinearizeFG(dspm::Mat& x, float* u) {
    // Continuous F
    F *= 0;

    F(0, 1) = 1.0f;
    F(1, 2) = 1.0f;
    F(3, 1) = std::pow(x(3, 0), 1.0f - config_.beta_coef) / config_.gamma_coef;
    F(3, 3) = (1.0f - config_.beta_coef) / config_.gamma_coef * x(1, 0) *
              std::pow(x(3, 0), -config_.beta_coef);
}

dspm::Mat SimpleEKF::StateXdot(dspm::Mat& x, float* u) {
    // Continuous Xdot
    dspm::Mat Xdot(NUMX, 1);
    Xdot *= 0;

    Xdot(0, 0) = x(1, 0);
    Xdot(1, 0) = x(2, 0);
    // Xdot(3, 0) = 0;
    Xdot(3, 0) = x(1, 0) * std::pow(x(3, 0), 1.0f - config_.beta_coef) /
                 config_.gamma_coef;

    return Xdot;
}
