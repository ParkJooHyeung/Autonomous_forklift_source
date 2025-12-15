#include "KalmanFilterWrapper.h"

namespace AutonomousForklift {
namespace Filters {

KalmanFilterWrapper::KalmanFilterWrapper(double R_val, double Q_var, double dt)
    : x(2), P(2, 2), F(2, 2), H(1, 2), R(1, 1), Q(2, 2), I(2, 2)
{
    // Initial State Vector [position, velocity]
    // Will be initialized with actual measurement later, for now set to 0.
    x << 0.0,
         0.0;

    // State Transition Matrix [F]
    // Assumes constant velocity model with dt
    F << 1.0, dt,
         0.0, 1.0;

    // Measurement Function [H]
    // We only measure position
    H << 1.0, 0.0;

    // Measurement Noise Covariance [R]
    R << R_val;

    // Process Noise Covariance [Q]
    // Corresponds to Q_discrete_white_noise(dim=2, dt=dt, var=Q_var)
    // For a constant velocity model with white noise acceleration, Q is:
    // [[dt^4/4, dt^3/2],
    //  [dt^3/2, dt^2]] * Q_var (simplified, actual filterpy is more complex but this approximation is common)
    // FilterPy's Q_discrete_white_noise for dim_x=2, dt=1, var=Q_var
    // [[0.25 * dt^4, 0.5 * dt^3],
    //  [0.5 * dt^3,      dt^2]] * Q_var
    Q << (0.25 * std::pow(dt, 4)), (0.5 * std::pow(dt, 3)),
         (0.5 * std::pow(dt, 3)), (std::pow(dt, 2));
    Q *= Q_var;


    // Initial State Covariance [P]
    // High initial uncertainty
    P << 1000.0, 0.0,
         0.0, 1000.0;

    // Identity matrix
    I = Eigen::MatrixXd::Identity(2, 2);
}

void KalmanFilterWrapper::predict() {
    x = F * x;
    P = F * P * F.transpose() + Q;
}

double KalmanFilterWrapper::update(double measurement) {
    Eigen::VectorXd z(1);
    z << measurement;

    // Innovation
    Eigen::VectorXd y = z - H * x;

    // Innovation covariance
    Eigen::MatrixXd S = H * P * H.transpose() + R;

    // Kalman gain
    Eigen::MatrixXd K = P * H.transpose() * S.inverse();

    // Update state estimate
    x = x + K * y;

    // Update state covariance
    P = (I - K * H) * P;

    // Return filtered position (first element of state vector)
    return x(0);
}

} // namespace Filters
} // namespace AutonomousForklift
