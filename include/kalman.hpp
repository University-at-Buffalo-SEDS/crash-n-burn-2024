#pragma once

#include "matrix.hpp"
#include <cstdint>
#include <cmath>

// Define a type alias for floating-point precision
using kfloat_t = float;

// Define the state vector as a 3-dimensional vector
using KalmanState = Vector<kfloat_t, 3>;

class KalmanFilter {
public:
    // Constructors
    KalmanFilter(kfloat_t timeStep,
                kfloat_t altSigma,
                kfloat_t accelSigma,
                kfloat_t modelSigma);

    // Constructor with pre-computed gain matrix
    KalmanFilter(kfloat_t timeStep,
                const Matrix<kfloat_t, 3, 2>& precomputedGain);

    // Disable default constructor
    KalmanFilter() = delete;

    // Update the filter with new measurements
    bool update(kfloat_t accel, kfloat_t altitude);

    // Accessors for the state estimates
    const KalmanState& getState() const;
    kfloat_t getPosition() const;
    kfloat_t getVelocity() const;
    kfloat_t getAcceleration() const;

private:
    // Private member variables
    Matrix<kfloat_t, 3, 3> stm_;      // State Transition Matrix
    Matrix<kfloat_t, 3, 2> kgain_;    // Kalman Gain Matrix
    KalmanState estP_;                // Predicted Estimate
    KalmanState est_;                 // Current Estimate
    bool isFirstStep_;                // Initialization flag

    // Private methods
    bool calculateGain(kfloat_t altSigma,
                      kfloat_t accelSigma,
                      kfloat_t modelSigma,
                      uint16_t maxIterations = 1000,
                      kfloat_t convergenceThreshold = 1e-6f);
};
