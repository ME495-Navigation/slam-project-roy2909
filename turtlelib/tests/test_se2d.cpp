#include <catch2/catch_test_macros.hpp>
#include <sstream>
#include <catch2/matchers/catch_matchers_floating_point.hpp>
#include "turtlelib/se2d.hpp"

TEST_CASE("stream extraction operator <<"
          "[se2d]") // Rahul,Roy
{
    turtlelib::Twist2D twist;
    twist.omega = 1.0;
    twist.x = 2.0;
    twist.y = 3.0;
    // Create an std::ostringstream to capture the output
    std::ostringstream output_stream;

    output_stream << twist;

    // Check if the output matches the expected format
    REQUIRE(output_stream.str() == "[1 2 3]");
}

TEST_CASE("stream insertion operator >>"
          "[se2d]") // Rahul,Roy
{
    turtlelib::Twist2D twist;
    turtlelib::Twist2D twist1;

    // Create an input string to simulate user input
    std::string input_string = "5.3 8.7 5.3";
    std::string input_string1 = "[5.3 8.7 5.3]";

    // Create an std::istringstream to read from the input string
    std::istringstream input_stream(input_string);
    std::istringstream input(input_string1);

    // Use the input operator to read the Point2D object from the stream
    input_stream >> twist;
    input >> twist1;

    // Check if the coordinates were correctly read
    REQUIRE(twist.omega == 5.3);
    REQUIRE(twist.x == 8.7);
    REQUIRE(twist.y == 5.3);
    REQUIRE(twist1.omega == 5.3);
    REQUIRE(twist1.x == 8.7);
    REQUIRE(twist1.y == 5.3);
}

TEST_CASE("operator() transformation for Point2D"
          "[se2d]") // Rahul,Roy
{
    double angle = turtlelib::PI / 2.0;
    turtlelib::Vector2D vec = {0.0, 1.0};
    turtlelib::Transform2D trans = {vec, angle};
    turtlelib::Point2D p;
    p.x = 1.0;
    p.y = 1.0;
    turtlelib::Point2D result = trans(p);
    REQUIRE_THAT(result.x, Catch::Matchers::WithinAbs(-1.0, 1e-5));
    REQUIRE_THAT(result.y, Catch::Matchers::WithinAbs(2.0, 1e-5));
}

TEST_CASE("operator() transformation for Vector2D"
          "[se2d]") // Rahul,Roy
{
    double angle = turtlelib::PI / 2.0;
    turtlelib::Vector2D vec = {0.0, 1.0};
    turtlelib::Transform2D trans = {vec, angle};
    turtlelib::Vector2D v;
    v.x = 1.0;
    v.y = 1.0;
    turtlelib::Vector2D result = trans(v);
    REQUIRE_THAT(result.x, Catch::Matchers::WithinAbs(-1.0, 1e-5));
    REQUIRE_THAT(result.y, Catch::Matchers::WithinAbs(2.0, 1e-5));
}

TEST_CASE("operator() transformation for Twist2D"
          "[se2d]") // Rahul,Roy
{
    double angle = turtlelib::PI / 2.0;
    turtlelib::Vector2D vec = {0.0, 1.0};
    turtlelib::Transform2D trans = {vec, angle};
    turtlelib::Twist2D t;
    t.omega = 1.0;
    t.x = 1.0;
    t.y = 1.0;
    turtlelib::Twist2D result = trans(t);
    REQUIRE_THAT(result.omega, Catch::Matchers::WithinAbs(1.0, 1e-5));
    REQUIRE_THAT(result.x, Catch::Matchers::WithinAbs(0.0, 1e-5));
    REQUIRE_THAT(result.y, Catch::Matchers::WithinAbs(1.0, 1e-5));
}

TEST_CASE("inv() Transform"
          "[se2d]") // Rahul,Roy
{
    double a = turtlelib::PI / 2.0;
    turtlelib::Vector2D vec = {0.0, 1.0};
    turtlelib::Transform2D trans = {{vec}, a};
    REQUIRE_THAT(trans.inv().rotation(), Catch::Matchers::WithinAbs(-a, 1e-5));
    REQUIRE_THAT(trans.inv().translation().x, Catch::Matchers::WithinAbs(-1.0, 1e-5));
    REQUIRE_THAT(trans.inv().translation().y, Catch::Matchers::WithinAbs(0.0, 1e-5));
}

TEST_CASE("stream extraction operator << Transform"
          "[se2d]") // Rahul,Roy
{
    turtlelib::Vector2D v;
    double angle = 2.0;
    v.x = 2.1;
    v.y = 3.0;
    turtlelib::Transform2D t(v, angle);
    std::string str = "deg: 114.592 x: 2.1 y: 3";
    // Create an std::ostringstream to capture the output
    std::ostringstream output_stream;

    output_stream << t;

    // Check if the output matches the expected format
    REQUIRE(output_stream.str() == str);
}

TEST_CASE("stream insertion operator >> transform"
          "[se2d]") // Rahul,Roy
{

    // Create an std::istringstream to read from the input string
    std::istringstream input_stream("deg: 45.0 x: 2.0 y: 3.0");

    turtlelib::Transform2D tf;
    input_stream >> tf;

    REQUIRE_THAT(tf.rotation(), Catch::Matchers::WithinAbs(turtlelib::deg2rad(45.0), 1e-5));
    REQUIRE_THAT(tf.translation().x, Catch::Matchers::WithinAbs(2.0, 1e-5));
    REQUIRE_THAT(tf.translation().y, Catch::Matchers::WithinAbs(3.0, 1e-5));
}

TEST_CASE("rotation()"
          "[se2d]") // Rahul,Roy
{
    double angle_test = 5.0;
    turtlelib::Transform2D test{angle_test};
    REQUIRE_THAT(test.rotation(), Catch::Matchers::WithinAbs(angle_test, 1e-5));
}

TEST_CASE("translation()"
          "[se2d]") // Rahul,Roy
{
    double x_test = 5.0;
    double y_test = 3.0;
    turtlelib::Transform2D test{{x_test, y_test}};
    REQUIRE_THAT(test.translation().x, Catch::Matchers::WithinAbs(x_test, 1e-5));
    REQUIRE_THAT(test.translation().y, Catch::Matchers::WithinAbs(y_test, 1e-5));
}

TEST_CASE("testing * operator for two transform2D objects", "[multiplication*]") // Shail,Dalal
{
    turtlelib::Transform2D lhs{turtlelib::Vector2D{3.0, 5.2}, turtlelib::PI / 2};
    turtlelib::Transform2D rhs{turtlelib::Vector2D{3.7, 6.3}, turtlelib::PI / 4};
    lhs = operator*(lhs, rhs);

    REQUIRE(lhs.rotation() == 3 * (turtlelib::PI / 4));
    REQUIRE_THAT(lhs.translation().x,
                 Catch::Matchers::WithinAbs(-3.3, 0.1));
    REQUIRE_THAT(lhs.translation().y,
                 Catch::Matchers::WithinAbs(8.9, 0.1));
}

TEST_CASE("testing the *= operator", "[multiplication*=]") // Shail,Dalal
{
    turtlelib::Transform2D trans{turtlelib::Vector2D{2.4, 4.4}, 0.0};
    turtlelib::Transform2D rhs{turtlelib::Vector2D{2.1, 1.1}, 0.0};
    trans.operator*=(rhs);
    REQUIRE_THAT(trans.rotation(),
                 Catch::Matchers::WithinAbs(0.0, 0.1));
    REQUIRE_THAT(trans.translation().x,
                 Catch::Matchers::WithinAbs(4.5, 0.1));
    REQUIRE_THAT(trans.translation().y,
                 Catch::Matchers::WithinAbs(5.5, 0.1));
}

TEST_CASE("integrate_twist()", "[se2d]") // Rahul,Roy
{
    // Pure traslation
    turtlelib::Twist2D t1 = {0.0, 1.0,2.0};
    turtlelib::Transform2D T1 = turtlelib::integrate_twist(t1);
    REQUIRE_THAT(T1.translation().x, Catch::Matchers::WithinAbs(1.0, 1e-5));
    REQUIRE_THAT(T1.translation().y, Catch::Matchers::WithinAbs(2.0, 1e-5));
    // Pure rotation
    turtlelib::Twist2D t2 = {-6.0, 0.0, 0.0};
    turtlelib::Transform2D T2 = turtlelib::integrate_twist(t2);
    REQUIRE_THAT(T2.rotation(), Catch::Matchers::WithinAbs(-6.0, 1e-5));
    // Rotation and Traslation
    turtlelib::Twist2D t3 = {2.5,3.2,-1.5};
    turtlelib::Transform2D T3 = turtlelib::integrate_twist(t3);
    REQUIRE_THAT(T3.translation().x, Catch::Matchers::WithinAbs(1.8467305138, 1e-5));
    REQUIRE_THAT(T3.translation().y, Catch::Matchers::WithinAbs(1.9463805414, 1e-5));
    REQUIRE_THAT(T3.rotation(), Catch::Matchers::WithinAbs(2.5, 1e-5));
}

