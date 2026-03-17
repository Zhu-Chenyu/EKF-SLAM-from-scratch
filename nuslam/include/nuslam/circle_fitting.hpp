#ifndef CIRCLE_FITTING_HPP
#define CIRCLE_FITTING_HPP
/// \file
/// \brief Circle fitting class
#include <vector>
#include <cmath>
#include <armadillo>

/// \brief Circle fitting class
///
/// This class provides methods for fitting a circle to a set of points.
class CircleFitting {
public:
    CircleFitting() {}
    ~CircleFitting() {}

    /// \brief Fit a circle to a set of points
    ///
    /// \param x Vector of x-coordinates
    /// \param y Vector of y-coordinates
    /// \return Vector of circle parameters [x, y, radius]
    static std::vector<double> fit(const std::vector<double>& x, const std::vector<double>& y);

    /// \brief Check if a set of points forms a circle
    ///
    /// \param x Vector of x-coordinates
    /// \param y Vector of y-coordinates
    /// \return True if the points form a circle, false otherwise
    static bool is_circle(const std::vector<double>& x, const std::vector<double>& y);
};

#endif