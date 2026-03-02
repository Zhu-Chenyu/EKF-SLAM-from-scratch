#include "ekf.hpp"
#include "turtlelib/angle.hpp"
#undef pi  // angle.hpp defines pi as a macro which conflicts with arma::Datum<T>::pi

void EKF::predict(std::vector<double> action) {
    auto theta = state_[0];
    state_[0] = turtlelib::normalize_angle(state_[0] + action[0]);
    arma::mat A_mat(3 + 2 * obs_num_, 3 + 2 * obs_num_, arma::fill::eye);
    if (std::abs(action[0]) < 1e-10) {
        // Pure translation (straight line)
        state_[1] += action[1] * std::cos(theta);
        state_[2] += action[1] * std::sin(theta);
        A_mat(1, 0) = -action[1] * std::sin(theta);
        A_mat(2, 0) =  action[1] * std::cos(theta);
    } else {
        // Arc motion
        state_[1] += action[1] / action[0] * (-std::sin(theta) + std::sin(theta + action[0]));
        state_[2] += action[1] / action[0] * ( std::cos(theta) - std::cos(theta + action[0]));
        A_mat(1, 0) = action[1] / action[0] * (-std::cos(theta) + std::cos(theta + action[0]));
        A_mat(2, 0) = action[1] / action[0] * (-std::sin(theta) + std::sin(theta + action[0]));
    }
    // Process noise only on robot pose (top-left 3x3); landmarks are static
    arma::mat Q(3 + 2 * obs_num_, 3 + 2 * obs_num_, arma::fill::zeros);
    Q(0, 0) = process_noise_variance_;
    Q(1, 1) = process_noise_variance_;
    Q(2, 2) = process_noise_variance_;
    zegma_ = A_mat * zegma_ * A_mat.t() + Q;
}

void EKF::update(int id, double dist_obs, double angle_obs) {
    if (!seen_[id]) {
        // Initialize landmark position from first measurement
        state_[3 + 2*id] = state_[1] + dist_obs * std::cos(angle_obs + state_[0]);
        state_[4 + 2*id] = state_[2] + dist_obs * std::sin(angle_obs + state_[0]);
        seen_[id] = true;
    }
    arma::mat H_mat(2, 3 + 2 * obs_num_, arma::fill::zeros);
    auto del_x = state_[3 + 2*id] - state_[1]; // estimated relative x position
    auto del_y = state_[4 + 2*id] - state_[2]; // estimated relative y position
    auto d = del_x * del_x + del_y * del_y;
    H_mat(0, 1) = -del_x / std::sqrt(d);
    H_mat(0, 2) = -del_y / std::sqrt(d);
    H_mat(0, 3 + 2*id) = del_x / std::sqrt(d);
    H_mat(0, 4 + 2*id) = del_y / std::sqrt(d);
    H_mat(1, 0) = -1;
    H_mat(1, 1) = del_y / d;
    H_mat(1, 2) = -del_x / d;
    H_mat(1, 3 + 2*id) = -del_y / d;
    H_mat(1, 4 + 2*id) = del_x / d;
    arma::mat R_mat(2, 2, arma::fill::eye);
    R_mat *= sensor_noise_variance_;
    arma::mat K = zegma_ * H_mat.t() * (H_mat * zegma_ * H_mat.t() + R_mat).i();
    arma::vec y_vec(2); // error between measurement and prediction
    y_vec[0] = dist_obs - std::sqrt(d);                                    // range innovation
    y_vec[1] = turtlelib::normalize_angle(
        angle_obs - (std::atan2(del_y, del_x) - state_[0])); // bearing innovation

    arma::vec x_vec(3 + 2 * obs_num_);
    x_vec.zeros();
    x_vec[0] = state_[0];
    x_vec[1] = state_[1];
    x_vec[2] = state_[2];
    for (int i = 0; i < obs_num_; i++) {
        x_vec[3 + 2*i] = state_[3 + 2*i];
        x_vec[4 + 2*i] = state_[4 + 2*i];
    }
    x_vec += K * y_vec;
    state_[0] = turtlelib::normalize_angle(x_vec[0]);
    state_[1] = x_vec[1];
    state_[2] = x_vec[2];
    for (int i = 0; i < obs_num_; i++) {
        state_[3 + 2*i] = x_vec[3 + 2*i];
        state_[4 + 2*i] = x_vec[4 + 2*i];
    }
    zegma_ = (arma::eye(3 + 2 * obs_num_, 3 + 2 * obs_num_) - K * H_mat) * zegma_;
}