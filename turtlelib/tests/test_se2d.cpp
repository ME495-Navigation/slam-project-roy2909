#include <catch2/catch_test_macros.hpp>
#include <sstream> 
#include <catch2/matchers/catch_matchers_floating_point.hpp>
#include "turtlelib/se2d.hpp"

TEST_CASE("stream extraction operator <<" "[se2d]") // Rahul,Roy
{
    turtlelib::Twist2D twist;
    twist.omega = 1.0;
    twist.x = 2.0;
    twist.y = 3.0;
    // Create an std::ostringstream to capture the output
    std::ostringstream output_stream;
    
    
    output_stream << twist;

    // Check if the output matches the expected format
    REQUIRE(output_stream.str() == "[1.0 2.0 3.0]");
}

TEST_CASE("stream insertion operator >>" "[se2d]") // Rahul,Roy
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


TEST_CASE("operator() transformation for Point2D" "[se2d]") // Rahul,Roy
{
    double angle = turtlelib::PI/4;
    turtlelib::Vector2D vec ={1.0,2.0};
    turtlelib::Transform2D trans = {vec,angle};
    turtlelib::Point2D p;
    p.x=3.0;
    p.y=4.0;
    turtlelib::Point2D result = trans(p);
    REQUIRE_THAT(result.x, Catch::Matchers::WithinAbs(-0.7071, 1e-5));
    REQUIRE_THAT(result.y, Catch::Matchers::WithinAbs(5.9497, 1e-5));


}

TEST_CASE("operator() transformation for Vector2D" "[se2d]") // Rahul,Roy
{
    double angle = turtlelib::PI/4;
    turtlelib::Vector2D vec ={1.0,2.0};
    turtlelib::Transform2D trans = {vec,angle};
    turtlelib::Vector2D v;
    v.x=3.0;
    v.y=4.0;
    turtlelib::Vector2D result = trans(v);
    REQUIRE_THAT(result.x, Catch::Matchers::WithinAbs(-0.7071, 1e-5));
    REQUIRE_THAT(result.y, Catch::Matchers::WithinAbs(5.9497, 1e-5));


}

TEST_CASE("operator() transformation for Twist2D" "[se2d]") // Rahul,Roy
{
    double angle = turtlelib::PI/2.0;
    turtlelib::Vector2D vec ={0.0,1.0};
    turtlelib::Transform2D trans = {vec,angle};
    turtlelib::Twist2D t;
    t.omega=1.0;
    t.x=1.0;
    t.y=1.0;
    turtlelib::Twist2D result = trans(t);
    REQUIRE_THAT(result.omega, Catch::Matchers::WithinAbs(1.0, 1e-5));
    REQUIRE_THAT(result.x, Catch::Matchers::WithinAbs(0.0, 1e-5));
    REQUIRE_THAT(result.y, Catch::Matchers::WithinAbs(1.0, 1e-5));


}

TEST_CASE("inv() Transform" "[se2d]") // Rahul,Roy
{
    double angle = turtlelib::PI/2.0;
    turtlelib::Vector2D vec ={0.0,1.0};
    turtlelib::Transform2D trans = {vec,angle};
    turtlelib::Transform2D test_inv;
    REQUIRE_THAT(test_inv.inv().rotation(), Catch::Matchers::WithinAbs(-angle, 1e-5));
    REQUIRE_THAT(test_inv.inv().translation().x, Catch::Matchers::WithinAbs(-1.0, 1e-5));
    REQUIRE_THAT(test_inv.inv().translation().y, Catch::Matchers::WithinAbs(0.0, 1e-5));

}

TEST_CASE("stream extraction operator << Transform" "[se2d]") // Rahul,Roy
{
    turtlelib::Vector2D v;
    double angle = 2.0;
    v.x = 2.0;
    v.y = 3.0;
    turtlelib::Transform2D t(v,angle);
    std::string str = "deg: 2.0 x: 2.0 y: 3.0";
    // Create an std::ostringstream to capture the output
    std::ostringstream output_stream;
    
    
    output_stream << t;

    // Check if the output matches the expected format
    REQUIRE(output_stream.str() == str);
}


TEST_CASE("stream insertion operator >> transform" "[se2d]") // Rahul,Roy
{

    // Create an std::istringstream to read from the input string
    std::istringstream input_stream("deg: 45.0 x: 2.0 y: 3.0");

    turtlelib:: Transform2D tf;
    input_stream >> tf;

    REQUIRE_THAT(tf.rotation(), Catch::Matchers::WithinAbs(turtlelib::deg2rad(45.0), 1e-5));
    REQUIRE_THAT(tf.translation().x, Catch::Matchers::WithinAbs(2.0, 1e-5));
    REQUIRE_THAT(tf.translation().y, Catch::Matchers::WithinAbs(3.0, 1e-5));
}


TEST_CASE("rotation()" "[se2d]") // Rahul,Roy
{
    double angle_test= 5.0;
    turtlelib::Transform2D test{angle_test};
    REQUIRE_THAT(test.rotation(), Catch::Matchers::WithinAbs(angle_test, 1e-5));

}

TEST_CASE("translation()" "[se2d]") // Rahul,Roy
{
    double x_test= 5.0;
    double y_test= 3.0;
    turtlelib::Transform2D test{{x_test,y_test}};
    REQUIRE_THAT(test.translation().x, Catch::Matchers::WithinAbs(x_test, 1e-5));
    REQUIRE_THAT(test.translation().y, Catch::Matchers::WithinAbs(y_test, 1e-5));

}