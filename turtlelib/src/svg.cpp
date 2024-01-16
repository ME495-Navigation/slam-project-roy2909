// svg.cpp
#include "svg.hpp"
#include <sstream>
#include <fstream>

namespace turtlelib {

    Svg::Svg() {
        // Initialize SVG with XML prolog and opening svg tag
        svgElements.push_back("<?xml version=\"1.0\" encoding=\"UTF-8\" standalone=\"no\"?>");
        svgElements.push_back("<svg width=\"8.500000in\" height=\"11.000000in\" viewBox=\"0 0 816.000000 1056.000000\" xmlns=\"http://www.w3.org/2000/svg\">");

        // arrowhead definition to the <defs> section
        svgElements.push_back("<defs>");
        svgElements.push_back("  <marker style=\"overflow:visible\" id=\"Arrow1Sstart\" refX=\"0.0\" refY=\"0.0\" orient=\"auto\">");
        svgElements.push_back("    <path transform=\"scale(0.2) translate(6,0)\" style=\"fill-rule:evenodd;fill:context-stroke;stroke:context-stroke;stroke-width:1.0pt\" d=\"M 0.0,0.0 L 5.0,-5.0 L -12.5,0.0 L 5.0,5.0 L 0.0,0.0 z\" />");
        svgElements.push_back("  </marker>");
        svgElements.push_back("</defs>");
    }

    void Svg::drawPoint(const PointParams& pparams) {
        std::stringstream ss;
        ss << "<circle cx=\"" << pparams.x << "\" cy=\"" << pparams.y << "\" r=\"3\" stroke=\"" << pparams.strokeColor << "\" fill=\"" << pparams.fillColor << "\" stroke-width=\"1\" />";
        svgElements.push_back(ss.str());
    }

     void Svg::drawVector(const VectorParams& vparams) {
        std::stringstream ss;
        ss << "<line x1=\"" << vparams.x1 << "\" y1=\"" << vparams.y1 << "\" x2=\"" << vparams.x2 << "\" y2=\"" << vparams.y2 << "\" stroke=\"" << vparams.strokeColor << "\" stroke-width=\"5\" marker-start=\"url(#Arrow1Sstart)\" />";
        svgElements.push_back(ss.str());
    }


    void Svg::drawCoordinateFrame(const CoordinateFrameParams& cparams) {
        drawGroup({
            "<line x1=\"" + std::to_string(cparams.originX) + "\" x2=\"" + std::to_string(cparams.xAxisX) + "\" y1=\"" + std::to_string(cparams.originY) + "\" y2=\"" + std::to_string(cparams.xAxisY) + "\" stroke=\"red\" stroke-width=\"5\" marker-start=\"url(#Arrow1Sstart)\" />",
            "<line x1=\"" + std::to_string(cparams.originX) + "\" x2=\"" + std::to_string(cparams.yAxisX) + "\" y1=\"" + std::to_string(cparams.originY) + "\" y2=\"" + std::to_string(cparams.yAxisY) + "\" stroke=\"green\" stroke-width=\"5\" marker-start=\"url(#Arrow1Sstart)\" />",
            "<text x=\"" + std::to_string(cparams.originX) + "\" y=\"" + std::to_string(cparams.originY + 0.25) + "\">" + cparams.text + "</text>"
        });
    }

    void Svg::drawGroup(const std::vector<std::string>& groupElements) {
        svgElements.push_back("<g>");
        for (const auto& element : groupElements) {
            svgElements.push_back(element);
        }
        svgElements.push_back("</g>");
    }

    std::string Svg::getSvgString() const {
        std::stringstream ss;
        for (const auto& element : svgElements) {
            ss << element << "\n";
        }
        ss << "</svg>";
        return ss.str();
    }

    void Svg::writeToFile(const std::string& filename) const {
        std::ofstream file(filename);
        file << getSvgString();
        file.close();
    }

} // namespace turtlelib