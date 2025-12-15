#include "calculations.h"
#include <algorithm> // For std::sort
#include <numeric>   // For std::accumulate (not used directly here, but common for stats)

namespace AutonomousForklift {
namespace Calculations {

    std::vector<double> remove_outliers(const std::vector<double>& data, double threshold) {
        if (data.empty()) {
            return {};
        }

        std::vector<double> sorted_data = data;
        std::sort(sorted_data.begin(), sorted_data.end());

        // Calculate Q1 (25th percentile)
        double q1;
        if (sorted_data.size() % 4 == 0) {
            q1 = (sorted_data[sorted_data.size() / 4 - 1] + sorted_data[sorted_data.size() / 4]) / 2.0;
        } else {
            q1 = sorted_data[static_cast<size_t>(std::floor(sorted_data.size() / 4.0))];
        }

        // Calculate Q3 (75th percentile)
        double q3;
        if (sorted_data.size() % 4 == 0) {
            q3 = (sorted_data[sorted_data.size() * 3 / 4 - 1] + sorted_data[sorted_data.size() * 3 / 4]) / 2.0;
        } else {
            q3 = sorted_data[static_cast<size_t>(std::ceil(sorted_data.size() * 3.0 / 4.0)) - 1];
        }

        double iqr = q3 - q1;
        double lower_bound = q1 - threshold * iqr;
        double upper_bound = q3 + threshold * iqr;

        std::vector<double> filtered_arr;
        for (double val : data) {
            if (val >= lower_bound && val <= upper_bound) {
                filtered_arr.push_back(val);
            }
        }
        return filtered_arr;
    }

    double calc_distance2time(double distance) {
        double d = distance;
        // Original Python: -1.1*10**(-8)*d**4 + 5.22*10**(-6)*d**3 - 8.797*10**(-4)*d**2 + 1.079*10**(-1)*d + 1.9
        return (-1.1e-8 * std::pow(d, 4) + 5.22e-6 * std::pow(d, 3)
                - 8.797e-4 * std::pow(d, 2) + 1.079e-1 * d + 1.9);
    }

    double calc_angle2time(double angle) {
        double a = angle;
        // Original Python: -38.68+9.08*math.log(a+94.12)
        // Note: Python's math.log is natural logarithm (base e)
        return -38.68 + 9.08 * std::log(a + 94.12);
    }

    double calc_p_distance(double center_x, double depth) {
        // Original Python: 1.792/1280 * abs(640-center_x) * depth / 1.88
        return 1.792 / 1280.0 * std::abs(640.0 - center_x) * depth / 1.88;
    }

} // namespace Calculations
} // namespace AutonomousForklift
