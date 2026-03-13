#include <catch2/catch_test_macros.hpp>
#include <catch2/matchers/catch_matchers_floating_point.hpp>
#include "nuslam/circle_fitting.hpp"

TEST_CASE("smaller radius"){
    std::vector<double> x = {1, 2, 5, 7, 9, 3};
    std::vector<double> y = {7, 6, 8, 7, 5, 7};
    auto obs = CircleFitting::fit(x, y);
    REQUIRE_THAT(obs.at(0), Catch::Matchers::WithinAbs(4.615482, 1e-4));
    REQUIRE_THAT(obs.at(1), Catch::Matchers::WithinAbs(2.807354, 1e-4));
    REQUIRE_THAT(obs.at(2), Catch::Matchers::WithinAbs(4.8275, 1e-4));
}

TEST_CASE("larger radius"){
    std::vector<double> x = {-1, -0.3, 0.3, 1};
    std::vector<double> y = {0, -0.06, 0.1, 0};
    auto obs = CircleFitting::fit(x, y);
    REQUIRE_THAT(obs.at(0), Catch::Matchers::WithinAbs(0.4908357, 1e-4));
    REQUIRE_THAT(obs.at(1), Catch::Matchers::WithinAbs(-22.15212, 1e-4));
    REQUIRE_THAT(obs.at(2), Catch::Matchers::WithinAbs(22.17979, 1e-4));
}