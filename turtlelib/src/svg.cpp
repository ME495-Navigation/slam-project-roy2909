// svg.cpp
#include "turtlelib/svg.hpp"
#include "turtlelib/se2d.hpp"
#include "turtlelib/geometry2d.hpp"
#include <sstream>
#include <fstream>
#include <iostream>

namespace turtlelib
{

    Svg::Svg()
    {
        // Initialize SVG with XML prolog and opening svg tag
        svgElements.push_back("<?xml version=\"1.0\" encoding=\"UTF-8\" standalone=\"no\"?>");
        svgElements.push_back("<svg width=\"8.500000in\" height=\"11.000000in\" viewBox=\"0 0 816.000000 1056.000000\" xmlns=\"http://www.w3.org/2000/svg\">");

        // arrowhead definition to the <defs> section
        svgElements.push_back("<defs>");
        svgElements.push_back("  <marker style=\"overflow:visible\" id=\"Arrow1Sstart\" refX=\"0.0\" refY=\"0.0\" orient=\"auto\">");
        svgElements.push_back("    <path transform=\"scale(0.2) translate(6,0)\" style=\"fill-rule:evenodd;fill:context-stroke;stroke:context-stroke;stroke-width:1.0pt\" d=\"M 0.0,0.0 L 5.0,-5.0 L -12.5,0.0 L 5.0,5.0 L 0.0,0.0 z\"/>");
        svgElements.push_back("  </marker>");
        svgElements.push_back("</defs>");
    }

    void Svg::drawPoint(const PointParams &pparams)
    {
        std::stringstream ss;
        turtlelib::Transform2D T_ab{{midpointX, midpointY}, turtlelib::PI};
        turtlelib::Point2D p_a, p_t;
        p_t = T_ab(p_a);
        p_t.x = p_t.x + pparams.x * conversion_factor;
        p_t.y = p_t.y - pparams.y * conversion_factor;
        ss << "<circle cx=\"" << p_t.x << "\" cy=\"" << p_t.y << "\" r=\"3\" stroke=\"" << pparams.strokeColor << "\" fill=\"" << pparams.fillColor << "\" stroke-width=\"1\"/>";
        svgElements.push_back(ss.str());
    }

    void Svg::drawVector(const VectorParams &vparams)
    {
        std::stringstream ss;
        turtlelib::Transform2D T_ab{{midpointX, midpointY}, turtlelib::PI};
        turtlelib::Vector2D v_h, v_t;
        v_h = T_ab(v_h);
        v_t = T_ab(v_t);
        v_h.x = v_h.x + vparams.x1 * conversion_factor;
        v_h.y = v_h.y - vparams.y1 * conversion_factor;
        v_t.x = v_t.x + vparams.x2 * conversion_factor;
        v_t.y = v_t.y - vparams.y2 * conversion_factor;
        ss << "<line x1=\"" << v_h.x << "\" y1=\"" << v_h.y << "\" x2=\"" << v_t.x << "\" y2=\"" << v_t.y << "\" stroke=\"" << vparams.strokeColor << "\" stroke-width=\"5\" marker-start=\"url(#Arrow1Sstart)\"/>";
        svgElements.push_back(ss.str());
    }

    void Svg::drawCoordinateFrame(const VectorParams &vparams)
    {  
        
         std::stringstream ss;
    svgElements.push_back("<g>");
    // Draw the original vector
    drawVector(vparams);
    // Calculate and draw the green "y" vector
    turtlelib::Transform2D T{turtlelib::PI / 2};
    turtlelib::Vector2D v_yh, v_yt,v_headt,v_tailt;
    v_headt.x=vparams.x1;
    v_headt.y=vparams.y1;
    v_tailt.x=vparams.x2;
    v_tailt.y=vparams.y2;
    v_yh = T(v_headt);
    v_yt = T(v_tailt);

    turtlelib::VectorParams greenVectorParams = {
        v_yh.x, v_yh.y,
        v_yt.x, v_yt.y,
        "green", vparams.text
    };

    drawVector(greenVectorParams);

    // Draw the text
    ss << "<text x=\""<< midpointX<<"\" y=\""<< midpointY <<"\">"<< "{"<<vparams.text<<"}" <<"</text>";
    svgElements.push_back(ss.str());

    // Close the group tag
    svgElements.push_back("</g>");
    }


    std::string Svg::getSvgString() const
    {
        std::stringstream ss;
        for (const auto &element : svgElements)
        {
            ss << element << "\n";
        }
        ss << "</svg>";
        return ss.str();
    }

    void Svg::writeToFile(const std::string &filename) const
    {
        std::ofstream file(filename);
        file << getSvgString();
        file.close();
    }

} // namespace turtlelib