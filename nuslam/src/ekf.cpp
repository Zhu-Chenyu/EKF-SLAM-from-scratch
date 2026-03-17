#include "ekf.hpp"
#include "turtlelib/angle.hpp"
#undef pi  // angle.hpp defines pi as a macro which conflicts with arma::Datum<T>::pi

void EKF::predict(std::vector<double> action) {
    auto theta = state_.at(0);
    state_.at(0) = turtlelib::normalize_angle(state_.at(0) + action.at(0));
    arma::mat A_mat(3 + 2 * obs_num_, 3 + 2 * obs_num_, arma::fill::eye);
    if (std::abs(action.at(0)) < 1e-10) {
        // Pure translation (straight line)
        state_.at(1) += action.at(1) * std::cos(theta);
        state_.at(2) += action.at(1) * std::sin(theta);
        A_mat(1, 0) = -action.at(1) * std::sin(theta);
        A_mat(2, 0) =  action.at(1) * std::cos(theta);
    } else {
        // Arc motion
        state_.at(1) += action.at(1) / action.at(0) * (-std::sin(theta) + std::sin(theta + action.at(0)));
        state_.at(2) += action.at(1) / action.at(0) * ( std::cos(theta) - std::cos(theta + action.at(0)));
        A_mat(1, 0) = action.at(1) / action.at(0) * (-std::cos(theta) + std::cos(theta + action.at(0)));
        A_mat(2, 0) = action.at(1) / action.at(0) * (-std::sin(theta) + std::sin(theta + action.at(0)));
    }
    // Process noise only on robot pose (top-left 3x3); landmarks are static
    arma::mat Q(3 + 2 * obs_num_, 3 + 2 * obs_num_, arma::fill::zeros);
    Q(0, 0) = process_noise_variance_;
    Q(1, 1) = process_noise_variance_;
    Q(2, 2) = process_noise_variance_;
    zegma_ = A_mat * zegma_ * A_mat.t() + Q;
}

void EKF::update_with_landmark(double dist_obs, double angle_obs) {
    
    // Temporarily initialize landmark N_ (the candidate "new" landmark)
    state_.at(landmark_x(N_)) = state_.at(1) + dist_obs * std::cos(angle_obs + state_.at(0));
    state_.at(landmark_y(N_)) = state_.at(2) + dist_obs * std::sin(angle_obs + state_.at(0));
    
    // Compute Mahalanobis distance for each EXISTING landmark (0..N_-1)
    std::vector<double> distances(N_+1);
    for (int i = 0; i < N_; i++) {
        arma::mat H_i(2, 3 + 2 * obs_num_, arma::fill::zeros);
        auto del_x = state_.at(landmark_x(i)) - state_.at(1);
        auto del_y = state_.at(landmark_y(i)) - state_.at(2);
        auto d = del_x * del_x + del_y * del_y;
        H_i(0, 1) = -del_x / std::sqrt(d);
        H_i(0, 2) = -del_y / std::sqrt(d);
        H_i(0, landmark_x(i)) = del_x / std::sqrt(d);
        H_i(0, landmark_y(i)) = del_y / std::sqrt(d);
        H_i(1, 0) = -1;
        H_i(1, 1) = del_y / d;
        H_i(1, 2) = -del_x / d;
        H_i(1, landmark_x(i)) = -del_y / d;
        H_i(1, landmark_y(i)) = del_x / d;
        arma::mat R_i = arma::eye(2, 2) * sensor_noise_variance_;
        arma::mat psi_i = H_i * zegma_ * H_i.t() + R_i;

        arma::vec innov_i(2);
        innov_i[0] = dist_obs - std::sqrt(d);
        innov_i[1] = turtlelib::normalize_angle(
            angle_obs - (std::atan2(del_y, del_x) - state_.at(0)));

        distances.at(i) = arma::as_scalar(innov_i.t() * psi_i.i() * innov_i);
    }
    
    // Find best association
    int best_id = N_;
    double best_dist = 0.3; // distance threshold
    for (int i = 0; i < N_; i++) {
        if (distances.at(i) < best_dist) {
            best_dist = distances.at(i);
            best_id = i;
        }
    }

    // If it's a new landmark, increment the count
    if (best_id == N_) {
        N_++;
    }

    // Perform EKF update using best_id
    arma::mat H_mat(2, 3 + 2 * obs_num_, arma::fill::zeros);
    auto del_x = state_.at(landmark_x(best_id)) - state_.at(1);
    auto del_y = state_.at(landmark_y(best_id)) - state_.at(2);
    auto d = del_x * del_x + del_y * del_y;
    H_mat(0, 1) = -del_x / std::sqrt(d);
    H_mat(0, 2) = -del_y / std::sqrt(d);
    H_mat(0, landmark_x(best_id)) = del_x / std::sqrt(d);
    H_mat(0, landmark_y(best_id)) = del_y / std::sqrt(d);
    H_mat(1, 0) = -1;
    H_mat(1, 1) = del_y / d;
    H_mat(1, 2) = -del_x / d;
    H_mat(1, landmark_x(best_id)) = -del_y / d;
    H_mat(1, landmark_y(best_id)) = del_x / d;
    arma::mat R_mat = arma::eye(2, 2) * sensor_noise_variance_;
    arma::mat K = zegma_ * H_mat.t() * (H_mat * zegma_ * H_mat.t() + R_mat).i();
    arma::vec y_vec(2);
    y_vec[0] = dist_obs - std::sqrt(d);
    y_vec[1] = turtlelib::normalize_angle(
        angle_obs - (std::atan2(del_y, del_x) - state_.at(0)));

    arma::vec x_vec(3 + 2 * obs_num_);
    x_vec.zeros();
    x_vec[0] = state_.at(0);
    x_vec[1] = state_.at(1);
    x_vec[2] = state_.at(2);
    for (int i = 0; i < obs_num_; i++) {
        x_vec[landmark_x(i)] = state_.at(landmark_x(i));
        x_vec[landmark_y(i)] = state_.at(landmark_y(i));
    }
    x_vec += K * y_vec;
    state_.at(0) = turtlelib::normalize_angle(x_vec.at(0));
    state_.at(1) = x_vec.at(1);
    state_.at(2) = x_vec.at(2);
    for (int i = 0; i < obs_num_; i++) {
        state_.at(landmark_x(i)) = x_vec.at(landmark_x(i));
        state_.at(landmark_y(i)) = x_vec.at(landmark_y(i));
    }
    zegma_ = (arma::eye(3 + 2 * obs_num_, 3 + 2 * obs_num_) - K * H_mat) * zegma_;
}

void EKF::update(int id, double dist_obs, double angle_obs) {
    if (!seen_.at(id)) {
        // Initialize landmark position from first measurement
        state_.at(landmark_x(id)) = state_.at(1) + dist_obs * std::cos(angle_obs + state_.at(0));
        state_.at(landmark_y(id)) = state_.at(2) + dist_obs * std::sin(angle_obs + state_.at(0));
        seen_.at(id) = true;
    }
    arma::mat H_mat(2, 3 + 2 * obs_num_, arma::fill::zeros);
    auto del_x = state_.at(landmark_x(id)) - state_.at(1); // estimated relative x position
    auto del_y = state_.at(landmark_y(id)) - state_.at(2); // estimated relative y position
    auto d = del_x * del_x + del_y * del_y;
    H_mat(0, 1) = -del_x / std::sqrt(d);
    H_mat(0, 2) = -del_y / std::sqrt(d);
    H_mat(0, landmark_x(id)) = del_x / std::sqrt(d);
    H_mat(0, landmark_y(id)) = del_y / std::sqrt(d);
    H_mat(1, 0) = -1;
    H_mat(1, 1) = del_y / d;
    H_mat(1, 2) = -del_x / d;
    H_mat(1, landmark_x(id)) = -del_y / d;
    H_mat(1, landmark_y(id)) = del_x / d;
    arma::mat R_mat(2, 2, arma::fill::eye);
    R_mat *= sensor_noise_variance_;
    arma::mat K = zegma_ * H_mat.t() * (H_mat * zegma_ * H_mat.t() + R_mat).i();
    arma::vec y_vec(2); // error between measurement and prediction
    y_vec[0] = dist_obs - std::sqrt(d);                                    // range innovation
    y_vec[1] = turtlelib::normalize_angle(
        angle_obs - (std::atan2(del_y, del_x) - state_.at(0))); // bearing innovation

    arma::vec x_vec(3 + 2 * obs_num_);
    x_vec.zeros();
    x_vec[0] = state_.at(0);
    x_vec[1] = state_.at(1);
    x_vec[2] = state_.at(2);
    for (int i = 0; i < obs_num_; i++) {
        x_vec[landmark_x(i)] = state_.at(landmark_x(i));
        x_vec[landmark_y(i)] = state_.at(landmark_y(i));
    }
    x_vec += K * y_vec;
    state_.at(0) = turtlelib::normalize_angle(x_vec.at(0));
    state_.at(1) = x_vec.at(1);
    state_.at(2) = x_vec.at(2);
    for (int i = 0; i < obs_num_; i++) {
        state_.at(landmark_x(i)) = x_vec.at(landmark_x(i));
        state_.at(landmark_y(i)) = x_vec.at(landmark_y(i));
    }
    zegma_ = (arma::eye(3 + 2 * obs_num_, 3 + 2 * obs_num_) - K * H_mat) * zegma_;
}