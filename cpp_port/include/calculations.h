#ifndef AUTONOMOUS_FORKLIFT_CALCULATIONS_H
#define AUTONOMOUS_FORKLIFT_CALCULATIONS_H

#include <vector>
#include <cmath> // For std::log, std::pow

namespace AutonomousForklift {
namespace Calculations {

    /**
     * @brief Removes outliers from a vector of doubles using the Interquartile Range (IQR) method.
     * @param data The input vector of doubles.
     * @param threshold The threshold for outlier detection (default is 1.5).
     * @return A new vector containing data points within the IQR bounds.
     */
    std::vector<double> remove_outliers(const std::vector<double>& data, double threshold = 1.5);

    /**
     * @brief Calculates a time value based on distance using a polynomial function.
     * @param distance The input distance.
     * @return The calculated time.
     */
    double calc_distance2time(double distance);

    /**
     * @brief Calculates a time value based on an angle using a logarithmic function.
     * @param angle The input angle.
     * @return The calculated time.
     */
    double calc_angle2time(double angle);

    /**
     * @brief Calculates a 'pseudo-distance' based on center_x and depth.
     *        This function's constants are specific to the original Python implementation.
     * @param center_x The x-coordinate of the center.
     * @param depth The depth value.
     * @return The calculated pseudo-distance.
     */
    double calc_p_distance(double center_x, double depth);

} // namespace Calculations
} // namespace AutonomousForklift

#endif // AUTONOMOUS_FORKLIFT_CALCULATIONS_H
