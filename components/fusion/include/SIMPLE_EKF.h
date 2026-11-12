#pragma once

#include "ekf.h"

/**
 * @brief Configuration parameters for SimpleEKF
 */
struct SimpleEKFConfig {
    float beta_coef = 0.19025f;   // Atmospheric model beta coefficient
    float gamma_coef = -941.21f;  // Atmospheric model gamma coefficient
    float sigma_mp = 0.01f;       // Pa: pressure std-dev
    float sigma_mz = 0.01f;       // m/s^2: z-axis acceleration std-dev
    float Dt = 0.1f;              // s: Expected filter time step
    float S_p = 0.01f;            // Pa^2/Hz: barometer PSD
    float S_z = 5.80e-5f;         // (m/s^2)^2/Hz: z-axis accelerometer PSD
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
