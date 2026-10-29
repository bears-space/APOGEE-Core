#pragma once

#include <cmath>

#include "ekf.h"

/**
 * @brief Configuration parameters for SimpleEKF
 */
struct SimpleEKFConfig {
    float beta_coef = 0.0f;   // Atmospheric model beta coefficient
    float gamma_coef = 1.0f;  // Atmospheric model gamma coefficient
    float sigma_mp = 0.01f;   // Model noise: pressure standard deviation
    float sigma_mz =
        0.01f;          // Model noise: z-axis acceleration standard deviation
    float Dt = 0.01f;   // Expected filter time step in seconds
    float r_p = 0.01f;  // Barometer PSD
};

/**
 * @brief 3-state Extended Kalman Filter implementation
 */
class SimpleEKF : public ekf {
   public:
    /**
     * @brief Construct a new SimpleEKF instance
     * @param config Configuration parameters (defaults provided if omitted)
     */
    explicit SimpleEKF(const SimpleEKFConfig& config = SimpleEKFConfig());

    virtual ~SimpleEKF() override = default;

    /**
     * @brief Initializes state vector X and covariance matrices P and Q
     */
    void Init() override;

    /**
     * @brief Computes system Jacobian matrix F and control matrix G
     * @param x Current state estimate vector
     * @param u Control input vector (unused in this model)
     */
    void LinearizeFG(dspm::Mat& x, float* u) override;

    /**
     * @brief State derivative function for Runge-Kutta integration
     * @param x Current state vector
     * @param u Control input vector
     * @return dspm::Mat Time derivative vector (Xdot)
     */
    dspm::Mat StateXdot(dspm::Mat& x, float* u) override;

   public:
    SimpleEKFConfig config_;
};
