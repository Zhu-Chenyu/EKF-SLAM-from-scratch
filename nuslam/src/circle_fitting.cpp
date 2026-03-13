#include "circle_fitting.hpp"

std::vector<double> CircleFitting::fit(const std::vector<double>& x, const std::vector<double>& y) {
    // calculate the mean of x and y
    auto x_sum = 0.0;
    auto y_sum = 0.0;
    for (int i = 0; i < int(x.size()); i++) {
        x_sum += x.at(i);
        y_sum += y.at(i);
    }
    auto x_bar = x_sum / x.size();
    auto y_bar = y_sum / y.size();

    auto z_sum = 0.0;
    for (int i = 0; i < int(x.size()); i++) {
        auto xi = x.at(i) - x_bar;
        auto yi = y.at(i) - y_bar;
        z_sum += xi * xi + yi * yi;
    }
    auto z_bar = z_sum / x.size();

    // define the matrix Z
    arma::mat Z_mat(x.size(), 4, arma::fill::zeros);
    for (int i = 0; i < int(x.size()); i++) {
        auto xi = x.at(i) - x_bar;
        auto yi = y.at(i) - y_bar;
        auto zi = xi * xi + yi * yi;
        Z_mat(i, 0) = zi;
        Z_mat(i, 1) = xi;
        Z_mat(i, 2) = yi;
        Z_mat(i, 3) = 1;
    }

    //define the matrix H
    arma::mat H_mat(4, 4, arma::fill::zeros);
    H_mat(0,0) = 8*z_bar;
    H_mat(0,3) = 2;
    H_mat(1,1) = 1;
    H_mat(2,2) = 1;
    H_mat(3,0) = 2;

    //compute H inverse
    arma::mat H_inv(4, 4, arma::fill::zeros);
    H_inv(0,3) = 0.5;
    H_inv(1,1) = 1;
    H_inv(2,2) = 1;
    H_inv(3,0) = 0.5;
    H_inv(3,3) = -2*z_bar;

    //compute singular value for Z
    arma::mat U_mat, V_mat;
    arma::vec S_vec;
    arma::svd(U_mat, S_vec, V_mat, Z_mat);

    bool small_singular_value = true;
    for (int i = 0; i < int(S_vec.size()); i++) {
        if (S_vec.at(i) > 1e-12) {
            small_singular_value = false;
            break;
        }
    }

    //compute the solution for a
    arma::mat a_mat;
    if (small_singular_value) {
        a_mat = V_mat.col(3);
    } else {
        arma::mat y_mat = V_mat * arma::diagmat(S_vec) * V_mat.t();
        arma::mat q_mat = y_mat * H_inv * y_mat;

        arma::vec eigvals;
        arma::mat eigvecs;
        arma::eig_sym(eigvals, eigvecs, q_mat);

        // Find the smallest POSITIVE eigenvalue
        int min_idx = -1;
        double min_pos_val = std::numeric_limits<double>::max();
        for (int i = 0; i < int(eigvals.n_elem); i++) {
            if (eigvals(i) > 0 && eigvals(i) < min_pos_val) {
                min_pos_val = eigvals(i);
                min_idx = i;
            }
        }
        arma::mat a_star_mat = eigvecs.col(min_idx);
        a_mat = y_mat.i() * a_star_mat;
    }

    //compute the center and radius of the circle
    auto obs_x = -a_mat(1)/(2*a_mat(0)) + x_bar;
    auto obs_y = -a_mat(2)/(2*a_mat(0)) + y_bar;
    auto obs_radius = std::sqrt((a_mat(1)*a_mat(1) + a_mat(2)*a_mat(2) - 4*a_mat(0)*a_mat(3)) / (4*a_mat(0)*a_mat(0)));
    return {obs_x, obs_y, obs_radius};
}

bool CircleFitting::is_circle(const std::vector<double>& x, const std::vector<double>& y) {
    auto x_first = x.at(0);
    auto y_first = y.at(0);
    auto x_last = x.back();
    auto y_last = y.back();
    auto ang_mean = 0.0;
    std::vector<double> ang_vec;
    for (int i = 1; i < int(x.size())-1; i++) {
        auto v1x = x_first - x.at(i);
        auto v1y = y_first - y.at(i);
        auto v2x = x_last - x.at(i);
        auto v2y = y_last - y.at(i);
        auto ang = std::atan2(std::abs(v1x*v2y - v1y*v2x), v1x*v2x + v1y*v2y);
        ang_vec.push_back(ang);
        ang_mean += ang;
    }
    if (ang_mean > 0.75 * M_PI || ang_mean < 0.5 * M_PI) {
        return false;
    }
    ang_mean /= ang_vec.size();

    auto ang_std = 0.0;
    for (int i = 0; i < int(ang_vec.size()); i++) {
        ang_std += (ang_vec.at(i) - ang_mean) * (ang_vec.at(i) - ang_mean);
    }
    ang_std /= ang_vec.size();
    ang_std = std::sqrt(ang_std);
    if (ang_std > 0.15) {
        return false;
    }
    return true;
}