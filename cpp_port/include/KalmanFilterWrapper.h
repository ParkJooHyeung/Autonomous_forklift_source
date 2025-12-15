#ifndef AUTONOMOUS_FORKLIFT_KALMAN_FILTER_WRAPPER_H
#define AUTONOMOUS_FORKLIFT_KALMAN_FILTER_WRAPPER_H

#include <Eigen/Dense> // For Eigen::MatrixXd, Eigen::VectorXd

namespace AutonomousForklift {
namespace Filters {

class KalmanFilterWrapper {
public:
    /**
     * @brief Initializes the Kalman Filter for a 1D system (position and velocity).
     * @param R_val Measurement noise covariance. How much we trust the measurement.
     *              Lower R means more trust in the raw detected value.
     * @param Q_var Process noise variance. How much we expect the object's acceleration to vary.
     * @param dt Time step (default to 1.0, matching Python implementation's assumed dt).
     */
    KalmanFilterWrapper(double R_val = 5.0, double Q_var = 0.1, double dt = 1.0);

    /**
     * @brief Predict the next state.
     */
    void predict();

    /**
     * @brief Update the filter with a new measurement and return the filtered position.
     * @param measurement The new raw measurement (e.g., detected center_x).
     * @return The filtered position.
     */
    double update(double measurement);

private:
    Eigen::VectorXd x; // State vector [position, velocity]
    Eigen::MatrixXd P; // State covariance matrix
    Eigen::MatrixXd F; // State transition matrix
    Eigen::MatrixXd H; // Measurement function matrix
    Eigen::MatrixXd R; // Measurement noise covariance matrix
    Eigen::MatrixXd Q; // Process noise covariance matrix
    Eigen::MatrixXd I; // Identity matrix (for update step)
};

} // namespace Filters
} // namespace AutonomousForklift

#endif // AUTONOMOUS_FORKLIFT_KALMAN_FILTER_WRAPPER_H
