#include <catch2/catch_test_macros.hpp>
#include <sstream>
#include <catch2/matchers/catch_matchers_floating_point.hpp>
#include "turtlelib/geometry2d.hpp"

TEST_CASE("stream extraction operator <<"
          "[geometry2d]") // Rahul,Roy
{
    turtlelib::Point2D point;
    point.x = 3.5;
    point.y = 7.2;
    // Create an std::ostringstream to capture the output
    std::ostringstream output_stream;

    output_stream << point;

    // Check if the output matches the expected format
    REQUIRE(output_stream.str() == "[3.5 7.2]");
}

TEST_CASE("stream insertion operator >>"
          "[geometry2d]") // Rahul,Roy
{
    turtlelib::Point2D point;

    // Create an input string to simulate user input
    std::string input_string = "5.3 8.7";

    // Create an std::istringstream to read from the input string
    std::istringstream input_stream(input_string);

    // Use the input operator to read the Point2D object from the stream
    input_stream >> point;

    // Check if the coordinates were correctly read
    REQUIRE(point.x == 5.3);
    REQUIRE(point.y == 8.7);
}

TEST_CASE("operator+ >>"
          "[geometry2d]") // Rahul,Roy
{
    turtlelib::Point2D result;
    turtlelib::Vector2D disp;
    disp.x = 3.0;
    disp.y = 4.0;
    turtlelib::Point2D tail;
    tail.x = 1.0;
    tail.y = 2.0;
    result = tail + disp;

    REQUIRE(result.x == 4.0);
    REQUIRE(result.y == 6.0);
}

TEST_CASE("operator- >>"
          "[geometry2d]") // Rahul,Roy
{
    turtlelib::Vector2D result;
    turtlelib::Point2D head;
    head.x = 6.0;
    head.y = 8.0;
    turtlelib::Point2D tail;
    tail.x = 2.0;
    tail.y = 5.0;
    result = head - tail;

    REQUIRE(result.x == 4.0);
    REQUIRE(result.y == 3.0);
}

TEST_CASE("normalize_vector"
          "[geometry2d]") // Rahul,Roy
{
    turtlelib::Vector2D vec;
    turtlelib::Vector2D result;
    vec.x = 6.0;
    vec.y = 6.0;
    result = turtlelib::normalize_vector(vec);

    REQUIRE_THAT(result.x, Catch::Matchers::WithinAbs(0.707107, 1e-5));
    REQUIRE_THAT(result.y, Catch::Matchers::WithinAbs(0.707107, 1e-5));
}

TEST_CASE("normalize_angle() >>"
          "[geometry2d]") // Rahul,Roy
{
    REQUIRE_THAT(turtlelib::normalize_angle(turtlelib::PI), Catch::Matchers::WithinAbs(turtlelib::PI, 1e-5));
    REQUIRE_THAT(turtlelib::normalize_angle(-(turtlelib::PI)), Catch::Matchers::WithinAbs((turtlelib::PI), 1e-5));
    REQUIRE_THAT(turtlelib::normalize_angle(0.0), Catch::Matchers::WithinAbs(0.0, 1e-5));
    REQUIRE_THAT(turtlelib::normalize_angle(-(turtlelib::PI / 4.0)), Catch::Matchers::WithinAbs(-(turtlelib::PI / 4.0), 1e-6));
    REQUIRE_THAT(turtlelib::normalize_angle(3.0 * turtlelib::PI / 2.0), Catch::Matchers::WithinAbs(-(turtlelib::PI / 2.0), 1e-6));
    REQUIRE_THAT(turtlelib::normalize_angle(-(5.0 * turtlelib::PI / 2.0)), Catch::Matchers::WithinAbs(-(turtlelib::PI / 2.0), 1e-6));
}

TEST_CASE(" * operator for two vector objects", "[se2d*]") // Rahul,Roy
{
    turtlelib::Vector2D vec;
    vec.x=5.0;
    vec.y=4.0;
    double lhs = 5.0;
    vec = turtlelib::operator*(lhs, vec);

    ;
    REQUIRE_THAT(vec.x,
                 Catch::Matchers::WithinAbs(25.0, 1e-5));
    REQUIRE_THAT(vec.y,
                 Catch::Matchers::WithinAbs(20.0, 1e-5));
}

TEST_CASE(" *= operator", "[se2d*=]") // Rahul,Roy
{
   
    turtlelib::Vector2D vec;
    vec.x=5.0;
    vec.y=4.0;
    double rhs = 5.0;
   vec.operator*=(rhs);

    ;
    REQUIRE_THAT(vec.x,
                 Catch::Matchers::WithinAbs(25.0, 1e-5));
    REQUIRE_THAT(vec.y,
                 Catch::Matchers::WithinAbs(20.0, 1e-5));
}

TEST_CASE("operator +", "[se2d]") // Rahul,Roy
{
    turtlelib::Vector2D v1 = {1.5, 2.0};
    turtlelib::Vector2D v2 = {2.0, -3.5};
    REQUIRE_THAT((v1+v2).x, Catch::Matchers::WithinAbs(3.5, 1e-5));
    REQUIRE_THAT((v1+v2).y, Catch::Matchers::WithinAbs(-1.5, 1e-5));
}

TEST_CASE("operator +=", "[se2d]") // Rahul,Roy
{
    turtlelib::Vector2D v1 = {1.5, 2.0};
    turtlelib::Vector2D v2 = {2.0, -3.5};
    v1.operator+=(v2);
    REQUIRE_THAT(v1.x, Catch::Matchers::WithinAbs(3.5, 1e-5));
    REQUIRE_THAT(v1.y, Catch::Matchers::WithinAbs(-1.5, 1e-5));
}

TEST_CASE("operator -", "[se2d]") // Rahul,Roy
{
    turtlelib::Vector2D v1 = {3.0, 2.0};
    turtlelib::Vector2D v2 = {2.0, 5.0};
    turtlelib::Vector2D result = v1-v2;
    REQUIRE_THAT(result.x, Catch::Matchers::WithinAbs(1.0, 1e-5));
    REQUIRE_THAT(result.y, Catch::Matchers::WithinAbs(-3.0, 1e-5));
}

TEST_CASE("operator -=", "[se2d]") // Rahul,Roy
{
    turtlelib::Vector2D v1 = {3.0, 2.0};
    turtlelib::Vector2D v2 = {2.0, 5.0};
     v1.operator-=(v2);
    REQUIRE_THAT(v1.x, Catch::Matchers::WithinAbs(1.0, 1e-5));
    REQUIRE_THAT(v1.y, Catch::Matchers::WithinAbs(-3.0, 1e-5));
}

TEST_CASE("dot()", "[se2d]") // Rahul,Roy
{
    turtlelib::Vector2D v1 = {1.0, 2.0};
    turtlelib::Vector2D v2 = {2.0, -1.0};
    double dp = turtlelib::dot(v1, v2);
    REQUIRE_THAT(dp, Catch::Matchers::WithinAbs(0.0, 1e-5));
}

TEST_CASE("magnitude()", "[se2d]") // Rahul,Roy
{
    turtlelib::Vector2D v1 = {2.0, 4.5};
    double mag = turtlelib::magnitude(v1);
    REQUIRE_THAT(mag, Catch::Matchers::WithinAbs(4.9244289009, 1e-7));
    
}

TEST_CASE("angle()", "[se2d]") // Rahul,Roy
{
    turtlelib::Vector2D v1 = {1.0, 2.0};
    turtlelib::Vector2D v2 = {2.0, -1.0};
    double ang = turtlelib::angle(v1, v2);
    REQUIRE_THAT(ang, Catch::Matchers::WithinAbs(1.57079633, 1e-5));

}
