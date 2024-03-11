#include <catch2/catch_test_macros.hpp>
#include <sstream>
#include <catch2/matchers/catch_matchers_floating_point.hpp>
#include "turtlelib/svg.hpp"
#include "turtlelib/se2d.hpp"
#include "turtlelib/diff_drive.hpp"
#include "turtlelib/circle_fit.hpp"

TEST_CASE("Circle fitting", "[circle_fit]") // Rahul Roy
{ 
    std::vector<turtlelib::Vector2D> points{{1, 7}, {2, 6}, {5, 8}, {7, 7},{9,5},{3,7}};
    auto circle = turtlelib::circle_fit(points);
    REQUIRE_THAT(circle.x_centroid, Catch::Matchers::WithinAbs(4.615482, 1.0e-5));
    REQUIRE_THAT(circle.y_centroid, Catch::Matchers::WithinAbs(2.807354, 1.0e-5));
    REQUIRE_THAT(circle.radius, Catch::Matchers::WithinAbs(4.8275751764, 1.0e-5));
}

TEST_CASE("Circle2", "[circle_fit]") // Rahul Roy
{ 
    std::vector<turtlelib::Vector2D> points{{-1,0},{-0.3,-0.06},{0.3,0.1},{1,0}};
    auto circle = turtlelib::circle_fit(points);
    REQUIRE_THAT(circle.x_centroid, Catch::Matchers::WithinAbs(0.4908357, 1.0e-5));
    REQUIRE_THAT(circle.y_centroid, Catch::Matchers::WithinAbs(-22.15212, 1.0e-5));
    REQUIRE_THAT(circle.radius, Catch::Matchers::WithinAbs(22.17979, 1.0e-5));
}