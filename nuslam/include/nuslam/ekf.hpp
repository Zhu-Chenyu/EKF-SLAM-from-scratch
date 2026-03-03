#ifndef NUSLAM_EKF_HPP
#define NUSLAM_EKF_HPP

#include "armadillo"
#include <vector>

class EKF {
public:
    EKF(int obs_num) : obs_num_(obs_num) {
        state_ = std::vector<double>(3 + 2 * obs_num_, 0.0); // [theta, x, y, obs1_x, obs1_y, obs2_x, obs2_y, obs3_x, obs3_y]
        zegma_ = arma::eye(3 + 2 * obs_num_, 3 + 2 * obs_num_); // covariance matrix
        k_ = arma::zeros(3 + 2 * obs_num_, 2 * obs_num_); // Kalman gain
        seen_ = std::vector<bool>(obs_num_, false);
    }
    void predict(std::vector<double> action);
    void update(int id, double dist_obs, double angle_obs);

    double get_theta() const { return state_[0]; }
    double get_x() const { return state_[1]; }
    double get_y() const { return state_[2]; }

private:
    int obs_num_ = 10;
    std::vector<double> state_; // [theta, x, y, obs1_x, obs1_y, obs2_x, obs2_y, obs3_x, obs3_y]
    std::vector<bool> seen_;    // whether each landmark has been initialized
    arma::mat zegma_; // covariance matrix
    arma::mat k_; // Kalman gain
    double process_noise_variance_ = 0.01;
    double sensor_noise_variance_ = 0.5;

};

#endif