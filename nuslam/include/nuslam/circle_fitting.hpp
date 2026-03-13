#ifndef CIRCLE_FITTING_HPP
#define CIRCLE_FITTING_HPP

#include <vector>
#include <cmath>
#include <armadillo>

class CircleFitting {
public:
    CircleFitting() {}
    ~CircleFitting() {}
    static std::vector<double> fit(const std::vector<double>& x, const std::vector<double>& y);
    static bool is_circle(const std::vector<double>& x, const std::vector<double>& y);
private:
};

#endif