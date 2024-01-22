#include <catch2/catch_test_macros.hpp>
#include <sstream>
#include <catch2/matchers/catch_matchers_floating_point.hpp>
#include "turtlelib/svg.hpp"
#include "turtlelib/se2d.hpp"

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
    os << "</svg>";

    REQUIRE(svg.getSvgString() == os.str());
}

TEST_CASE("Svg DrawVector", "[Svg]")
{
    turtlelib::Svg svg;
    turtlelib::PointParams pointParams = {1, 0, "red", "blue"};
    turtlelib::VectorParams vec = {1, 0, 0, 0, "red","a"};
    turtlelib::VectorParams vec3;
    turtlelib::Transform2D T{turtlelib::PI/2};
    turtlelib::Vector2D vec2;
    svg.drawPoint(pointParams);
    svg.drawVector(vec);
    svg.writeToFile("test2.svg");
    std::stringstream os;

    os << "<?xml version=\"1.0\" encoding=\"UTF-8\" standalone=\"no\"?>\n";
    os << "<svg width=\"8.500000in\" height=\"11.000000in\" viewBox=\"0 0 816.000000 1056.000000\" xmlns=\"http://www.w3.org/2000/svg\">\n";
    os << "<defs>\n";
    os << "  <marker style=\"overflow:visible\" id=\"Arrow1Sstart\" refX=\"0.0\" refY=\"0.0\" orient=\"auto\">\n";
    os << "    <path transform=\"scale(0.2) translate(6,0)\" style=\"fill-rule:evenodd;fill:context-stroke;stroke:context-stroke;stroke-width:1.0pt\" d=\"M 0.0,0.0 L 5.0,-5.0 L -12.5,0.0 L 5.0,5.0 L 0.0,0.0 z\"/>\n";
    os << "  </marker>\n";
    os << "</defs>\n";
    os << "<circle cx=\"504\" cy=\"528\" r=\"3\" stroke=\"red\" fill=\"blue\" stroke-width=\"1\"/>\n";
    os << "<line x1=\"504\" y1=\"528\" x2=\"408\" y2=\"528\" stroke=\"red\" stroke-width=\"5\" marker-start=\"url(#Arrow1Sstart)\"/>\n";
    os << "</svg>";
    REQUIRE(svg.getSvgString() == os.str());
    
}

TEST_CASE("Svg DrawCoordinateFrames", "[Svg]")
{
    turtlelib::Svg svg;
    turtlelib::PointParams pointParams = {0, 0, "red", "blue"};
    turtlelib::Point2D origin{0.0,0.0};
    turtlelib::Vector2D X{1,0};
    svg.drawCoordinateFrame(origin,X,"a");
    svg.writeToFile("test1.svg");
    std::stringstream os;
    os << "<?xml version=\"1.0\" encoding=\"UTF-8\" standalone=\"no\"?>\n";
    os << "<svg width=\"8.500000in\" height=\"11.000000in\" viewBox=\"0 0 816.000000 1056.000000\" xmlns=\"http://www.w3.org/2000/svg\">\n";
    os << "<defs>\n";
    os << "  <marker style=\"overflow:visible\" id=\"Arrow1Sstart\" refX=\"0.0\" refY=\"0.0\" orient=\"auto\">\n";
    os << "    <path transform=\"scale(0.2) translate(6,0)\" style=\"fill-rule:evenodd;fill:context-stroke;stroke:context-stroke;stroke-width:1.0pt\" d=\"M 0.0,0.0 L 5.0,-5.0 L -12.5,0.0 L 5.0,5.0 L 0.0,0.0 z\"/>\n";
    os << "  </marker>\n";
    os << "</defs>\n";
    os << "<g>\n";
    os << "<line x1=\"504\" y1=\"528\" x2=\"408\" y2=\"528\" stroke=\"red\" stroke-width=\"5\" marker-start=\"url(#Arrow1Sstart)\"/>\n";
    os << "<line x1=\"408\" y1=\"432\" x2=\"408\" y2=\"528\" stroke=\"green\" stroke-width=\"5\" marker-start=\"url(#Arrow1Sstart)\"/>\n";
    os << "<text x=\"408\" y=\"528\">{a}</text>\n";
    os << "</g>\n";
    os << "</svg>";
    REQUIRE(svg.getSvgString() == os.str());
    
}