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
        turtlelib::Transform2D T_ab{{408, 528}, turtlelib::PI};
        turtlelib::Point2D p_a, p_t;
        p_t = T_ab(p_a);
        p_t.x = p_t.x + pparams.x * 96;
        p_t.y = p_t.y + pparams.y * 96;
        ss << "<circle cx=\"" << p_t.x << "\" cy=\"" << p_t.y << "\" r=\"3\" stroke=\"" << pparams.strokeColor << "\" fill=\"" << pparams.fillColor << "\" stroke-width=\"1\"/>";
        svgElements.push_back(ss.str());
    }

    void Svg::drawVector(const VectorParams &vparams)
    {
        std::stringstream ss;
        turtlelib::Transform2D T_ab{{408, 528}, turtlelib::PI};
        turtlelib::Vector2D v_h, v_t;
        v_h = T_ab(v_h);
        v_t = T_ab(v_t);
        v_h.x = v_h.x + vparams.x1 * 96;
        v_h.y = v_h.y + vparams.y1 * 96;
        v_t.x = v_t.x + vparams.x2 * 96;
        v_t.y = v_t.y + vparams.y2 * 96;
        ss << "<line x1=\"" << v_h.x << "\" y1=\"" << v_h.y << "\" x2=\"" << v_t.x << "\" y2=\"" << v_t.y << "\" stroke=\"" << vparams.strokeColor << "\" stroke-width=\"5\" marker-start=\"url(#Arrow1Sstart)\"/>";
        svgElements.push_back(ss.str());
    }

    void Svg::drawCoordinateFrame(const VectorParams &vparams)
    {
        turtlelib::Transform2D T_ab{{408, 528}, turtlelib::PI};
        turtlelib::Vector2D v_h, v_t;
        v_h = T_ab(v_h);
        v_t = T_ab(v_t);
        v_h.x = v_h.x + vparams.x1 * 96;
        v_h.y = v_h.y + vparams.y1 * 96;
        v_t.x = v_t.x + vparams.x2 * 96;
        v_t.y = v_t.y + vparams.y2 * 96;
        drawGroup({"<line x1=\"" + std::to_string(v_h.x) + "\" x2=\"" + std::to_string(v_t.x) + "\" y1=\"" + std::to_string(v_h.y) + "\" y2=\"" + std::to_string(v_t.y) + "\" stroke=\"" + std::string(vparams.strokeColor) + "\" stroke-width=\"5\" marker-start=\"url(#Arrow1Sstart)\"/>",
                   "<text x=\"" + std::to_string(v_t.x) + "\" y=\"" + std::to_string(v_t.y) + "\">" + vparams.text + "</text>"});
    }

    void Svg::drawGroup(const std::vector<std::string> &groupElements)
    {
        svgElements.push_back("<g>");
        for (const auto &element : groupElements)
        {
            svgElements.push_back(element);
        }
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