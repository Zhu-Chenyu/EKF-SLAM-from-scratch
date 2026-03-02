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
    }
    void predict(std::vector<double> action);
    void update(int id, double dist_obs, double angle_obs);

private:
    int obs_num_ = 10;
    std::vector<double> state_; // [theta, x, y, obs1_x, obs1_y, obs2_x, obs2_y, obs3_x, obs3_y]
    arma::mat zegma_; // covariance matrix
    arma::mat k_; // Kalman gain
    double process_noise_variance_ = 0.01;
    double sensor_noise_variance_ = 0.01;

};

#endif