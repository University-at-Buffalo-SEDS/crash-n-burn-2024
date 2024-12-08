#include "kalman.hpp"
#include <Arduino.h> // For debugging purposes (e.g., Serial)

// Constructor that calculates the gain matrix based on noise parameters
KalmanFilter::KalmanFilter(kfloat_t timeStep,
                           kfloat_t altSigma,
                           kfloat_t accelSigma,
                           kfloat_t modelSigma)
    : stm_{
          1.0f, timeStep, 0.5f * timeStep * timeStep,
          0.0f, 1.0f, timeStep,
          0.0f, 0.0f, 1.0f
      },
      kgain_{},
      estP_{
          2.0f, 0.0f, 0.0f,
          0.0f, 9.0f, 0.0f,
          0.0f, 0.0f, 9.0f
      },
      est_{},
      isFirstStep_(true)
{
    if (!calculateGain(altSigma, accelSigma, modelSigma)) {
        // Handle gain calculation failure (e.g., set default gains or log an error)
        Serial.println(F("KalmanFilter: Gain calculation failed."));
        // Optionally, set kgain_ to a safe default or halt the system
    }
}

// Constructor that uses a precomputed gain matrix
KalmanFilter::KalmanFilter(kfloat_t timeStep,
                           const Matrix<kfloat_t, 3, 2>& precomputedGain)
    : stm_{
          1.0f, timeStep, 0.5f * timeStep * timeStep,
          0.0f, 1.0f, timeStep,
          0.0f, 0.0f, 1.0f
      },
      kgain_(precomputedGain),
      estP_{
          2.0f, 0.0f, 0.0f,
          0.0f, 9.0f, 0.0f,
          0.0f, 0.0f, 9.0f
      },
      est_{},
      isFirstStep_(true)
{
    // Initialization if needed
}

// Private method to calculate the Kalman Gain matrix
bool KalmanFilter::calculateGain(kfloat_t altSigma,
                                 kfloat_t accelSigma,
                                 kfloat_t modelSigma,
                                 uint16_t maxIterations,
                                 kfloat_t convergenceThreshold)
{
    const kfloat_t altVariance = altSigma * altSigma;
    const kfloat_t accelVariance = accelSigma * accelSigma;
    const kfloat_t modelVariance = modelSigma * modelSigma;

    Matrix<kfloat_t, 3, 3> stmT = stm_.transposed();
    Matrix<kfloat_t, 3, 3> pest = {
        2.0f, 0.0f, 0.0f,
        0.0f, 9.0f, 0.0f,
        0.0f, 0.0f, 9.0f
    };

    Matrix<kfloat_t, 3, 3> pestP;
    Matrix<kfloat_t, 3, 2> lastKgain = kgain_;
    uint16_t iterations = 0;

    while (iterations < maxIterations) {
        // Propagate state covariance
        pestP = stm_ * pest * stmT;
        pestP(2, 2) += modelVariance;

        // Compute the innovation covariance matrix
        Matrix<kfloat_t, 2, 2> innovationCov;
        innovationCov(0, 0) = pestP(0, 0) + altVariance;
        innovationCov(0, 1) = pestP(0, 2);
        innovationCov(1, 0) = pestP(2, 0);
        innovationCov(1, 1) = pestP(2, 2) + accelVariance;

        // Compute the determinant
        kfloat_t det = (innovationCov(0, 0) * innovationCov(1, 1)) - (innovationCov(0, 1) * innovationCov(1, 0));
        if (fabs(det) < std::numeric_limits<kfloat_t>::epsilon()) {
            // Singular matrix, cannot invert
            Serial.println(F("KalmanFilter: Innovation covariance matrix is singular."));
            return false;
        }

        // Compute the inverse of the innovation covariance matrix
        Matrix<kfloat_t, 2, 2> innovationCovInv = {
            innovationCov(1, 1) / det, -innovationCov(0, 1) / det,
            -innovationCov(1, 0) / det, innovationCov(0, 0) / det
        };

        // Compute Kalman Gain: K = P * H^T * (H * P * H^T + R)^-1
        // Here, H is the measurement matrix, assumed to be:
        // H = [1 0 0]
        //     [0 0 1]
        // Thus, K = [P00 P02; P20 P22] * innovationCovInv
        Matrix<kfloat_t, 3, 2> K;
        for (int i = 0; i < 3; ++i) {
            for (int j = 0; j < 2; ++j) {
                K(i, j) = 0.0f;
                for (int k = 0; k < 2; ++k) {
                    K(i, j) += pestP(i, k * 2) * innovationCovInv(k, j);
                }
            }
        }

        // Check for convergence
        kfloat_t diff = 0.0f;
        kfloat_t norm = 0.0f;
        for (size_t i = 0; i < 3; ++i) {
            for (size_t j = 0; j < 2; ++j) {
                kfloat_t delta = K(i, j) - lastKgain(i, j);
                diff += delta * delta;
                norm += lastKgain(i, j) * lastKgain(i, j);
            }
        }

        if (norm > 0.0f && (diff / norm) < convergenceThreshold) {
            kgain_ = K;
            Serial.print(F("KalmanFilter: Gain converged after "));
            Serial.print(iterations + 1);
            Serial.println(F(" iterations."));
            return true;
        }

        // Update K gain and covariance
        lastKgain = K;
        pest = (stm_ * pest * stmT) - (K * innovationCov * K.transposed());
        iterations++;
    }

    // If maximum iterations reached without convergence
    Serial.println(F("KalmanFilter: Gain calculation did not converge."));
    return false;
}

// Public method to update the filter with new measurements
bool KalmanFilter::update(kfloat_t accel, kfloat_t altitude)
{
    if (isFirstStep_) {
        est_(0) = altitude;
        est_(1) = 0.0f; // Assuming initial velocity is zero
        est_(2) = accel;
        isFirstStep_ = false;
        return true;
    }

    // Predict the next state
    estP_ = stm_ * est_;

    // Compute innovations
    kfloat_t altitudeInnovation = altitude - estP_(0);
    kfloat_t accelInnovation = accel - estP_(2);

    // Environmental Checks
    // 1. Ignore transonic pressure effects
    if (abs(altitudeInnovation) > 30.0f && estP_(1) > 300.0f && estP_(1) < 400.0f) {
        // Check if decelerating
        if (estP_(2) < 0.0f) {
            // Assume velocity and acceleration estimates are accurate
            // Adjust current altitude estimate to be the same as the measured altitude
            est_(0) = altitude;
            altitudeInnovation = 0.0f;
        }
    }

    // 2. Simple check for over-range on pressure measurement
    if (altitude > 12000.0f) {
        altitudeInnovation = 0.0f;
    }

    // Update the state estimate with the Kalman Gain and innovations
    // est = estP + kgain * [altitudeInnovation; accelInnovation]
    Vector<kfloat_t, 2> innovations = { altitudeInnovation, accelInnovation };
    est_ = estP_ + (kgain_ * innovations);

    return true;
}

// Accessor for the entire state vector
const KalmanState& KalmanFilter::getState() const
{
    return est_;
}

// Accessor for position estimate
kfloat_t KalmanFilter::getPosition() const
{
    return est_(0);
}

// Accessor for velocity estimate
kfloat_t KalmanFilter::getVelocity() const
{
    return est_(1);
}

// Accessor for acceleration estimate
kfloat_t KalmanFilter::getAcceleration() const
{
    return est_(2);
}
