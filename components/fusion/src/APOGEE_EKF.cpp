#include "ekf.h"

class ApogeeEKF : public ekf {
public:
    ApogeeEKF(): ekf(3, 3) {} // state count, input count

    void Init() override
    {
        // Initial acceleration estimate
        X *= 0;

        // Initial state uncertainty
        P *= 0;
        P(0, 0) = 1.0f;
        P(1, 1) = 1.0f;
        P(2, 2) = 1.0f;

        // Process noise
        Q *= 0;
        Q(0, 0) = 0.1f;
        Q(1, 1) = 0.1f;
        Q(2, 2) = 0.1f;
    }

    void LinearizeFG(dspm::Mat &x, float *u) override {
        F *= 0;
        G *= 0;

        G(0, 0) = 1.0f;
        G(1, 1) = 1.0f;
        G(2, 2) = 1.0f;
    }
};