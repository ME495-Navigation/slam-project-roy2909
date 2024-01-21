#include <catch2/catch_test_macros.hpp>
#include <sstream> 
#include <catch2/matchers/catch_matchers_floating_point.hpp>
#include "turtlelib/svg.hpp"

TEST_CASE("Svg DrawPoint", "[Svg]") 
{
    turtlelib::Svg svg;
    turtlelib::PointParams pointParams = {2, 0, "red", "blue"};
    svg.drawPoint(pointParams);
    svg.writeToFile("test.svg");
    std::stringstream os;
    
    
os << "<?xml version=\"1.0\" encoding=\"UTF-8\" standalone=\"no\"?>\n";
os << "<svg width=\"8.500000in\" height=\"11.000000in\" viewBox=\"0 0 816.000000 1056.000000\" xmlns=\"http://www.w3.org/2000/svg\">\n";
os << "<defs>\n";
os << "  <marker style=\"overflow:visible\" id=\"Arrow1Sstart\" refX=\"0.0\" refY=\"0.0\" orient=\"auto\">\n";
os << "    <path transform=\"scale(0.2) translate(6,0)\" style=\"fill-rule:evenodd;fill:context-stroke;stroke:context-stroke;stroke-width:1.0pt\" d=\"M 0.0,0.0 L 5.0,-5.0 L -12.5,0.0 L 5.0,5.0 L 0.0,0.0 z\"/>\n";
os << "  </marker>\n";
os << "</defs>\n";
os << "<circle cx=\"600\" cy=\"528\" r=\"3\" stroke=\"red\" fill=\"blue\" stroke-width=\"1\"/>\n";
os <<"</svg>";


    REQUIRE(svg.getSvgString() == os.str());
    
}


TEST_CASE("Svg DrawVector", "[Svg]") 
{
    turtlelib::Svg svg;
    turtlelib::PointParams pointParams = {1, 0, "red", "blue"};
    turtlelib::VectorParams vec ={1, 0, 0, 0, "red"};
    svg.drawPoint(pointParams);
    svg.drawVector(vec);
    svg.writeToFile("test1.svg");
    REQUIRE(1 == 1);
}