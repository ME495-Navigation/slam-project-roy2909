#include <catch2/catch_test_macros.hpp>
#include <sstream>
#include <catch2/matchers/catch_matchers_floating_point.hpp>
#include "turtlelib/svg.hpp"
#include "turtlelib/se2d.hpp"
#include "turtlelib/diff_drive.hpp"

TEST_CASE("ForwardKinematics(), Pure translation", "[diff_drive]") // Rahul Roy
{
    turtlelib::DiffDrive d(2.0, 6.0);
    turtlelib::WheelPos p{turtlelib::PI, turtlelib::PI};
    turtlelib::RobotConfig w;
    d.ForwardKinematics(p);
    REQUIRE_THAT(d.get_config().x, Catch::Matchers::WithinAbs(6.2831853072, 1.0e-5));
    REQUIRE_THAT(d.get_config().y, Catch::Matchers::WithinAbs(0.0, 1.0e-5));
    REQUIRE_THAT(d.get_config().theta, Catch::Matchers::WithinAbs(0.0, 1.0e-5));
}

TEST_CASE("ForwardKinematics(), Pure rotation", "[diff_drive]") // Rahul Roy
{
    turtlelib::DiffDrive d(2.0, 6.0);
    turtlelib::WheelPos p{2 * turtlelib::PI, -2 * turtlelib::PI};
    turtlelib::RobotConfig w;
    d.ForwardKinematics(p);
    REQUIRE_THAT(d.get_config().x, Catch::Matchers::WithinAbs(0.0, 1.0e-5));
    REQUIRE_THAT(d.get_config().y, Catch::Matchers::WithinAbs(0.0, 1.0e-5));
    REQUIRE_THAT(d.get_config().theta, Catch::Matchers::WithinAbs(2.0943951024, 1.0e-5));
}

TEST_CASE("ForwardKinematics(), arc of circle", "[diff_drive]") // Rahul Roy
{
    turtlelib::DiffDrive d(1.0, 2.0);
    turtlelib::WheelPos p{2.0 * turtlelib::PI, turtlelib::PI};
    turtlelib::RobotConfig w;
    d.ForwardKinematics(p);
    REQUIRE_THAT(d.get_config().x, Catch::Matchers::WithinAbs(3.0, 1.0e-5));
    REQUIRE_THAT(d.get_config().y, Catch::Matchers::WithinAbs(-3.0, 1.0e-5));
    REQUIRE_THAT(d.get_config().theta, Catch::Matchers::WithinAbs(-1.5707963268, 1.0e-5));
}

TEST_CASE("InverseKinematics(), Pure translation", "[diff_drive]") // Rahul Roy
{
    turtlelib::DiffDrive d(1.0, 6.0);
    turtlelib::Twist2D tw{0, 3, 0};
    turtlelib::RobotConfig w;
    turtlelib::WheelPos r = d.InverseKinematics(tw);
    REQUIRE_THAT(r.left, Catch::Matchers::WithinAbs(3.0, 1.0e-5));
    REQUIRE_THAT(r.right, Catch::Matchers::WithinAbs(3.0, 1.0e-5));
}

TEST_CASE("InverseKinematics(), Pure rotation", "[diff_drive]") // Rahul Roy
{
    turtlelib::DiffDrive d(1.0, 6.0);
    turtlelib::Twist2D tw{3, 0, 0};
    turtlelib::RobotConfig w;
    turtlelib::WheelPos r = d.InverseKinematics(tw);
    REQUIRE_THAT(r.left, Catch::Matchers::WithinAbs(-9.0, 1.0e-5));
    REQUIRE_THAT(r.right, Catch::Matchers::WithinAbs(9.0, 1.0e-5));
}

TEST_CASE("InverseKinematics(), arc of circle", "[diff_drive]") // Rahul Roy
{
    turtlelib::DiffDrive d(1.0, 6.0);
    turtlelib::Twist2D tw{-4, 4, 0};
    turtlelib::RobotConfig w;
    turtlelib::WheelPos r = d.InverseKinematics(tw);
    REQUIRE_THAT(r.left, Catch::Matchers::WithinAbs(16.0, 1.0e-5));
    REQUIRE_THAT(r.right, Catch::Matchers::WithinAbs(-8.0, 1.0e-5));
}

TEST_CASE("InverseKinematics(), Impossible-to-folllow twist", "[diff_drive]") // Rahul Roy
{
    turtlelib::DiffDrive d(1.0, 6.0);
    turtlelib::Twist2D tw{0, 0, 2};
    REQUIRE_THROWS_AS(d.InverseKinematics(tw), std::logic_error);
}

TEST_CASE("get_config(), configuration", "[diff_drive]") // Rahul Roy
{
    turtlelib::DiffDrive d(2.0, 6.0, {0.0, 0.0}, {1.0, 2.0, 3.0});
    REQUIRE_THAT(d.get_config().x, Catch::Matchers::WithinAbs(1.0, 1.0e-5));
    REQUIRE_THAT(d.get_config().y, Catch::Matchers::WithinAbs(2.0, 1.0e-5));
    REQUIRE_THAT(d.get_config().theta, Catch::Matchers::WithinAbs(3.0, 1.0e-5));
}