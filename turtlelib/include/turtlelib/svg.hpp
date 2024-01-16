// svg.hpp
#ifndef SVG_HPP
#define SVG_HPP

#include <string>
#include <vector>

namespace turtlelib {
    struct PointParams {
        double x;
        double y;
        std::string strokeColor;
        std::string fillColor;
    };

    struct VectorParams {
        double x1;
        double y1;
        double x2;
        double y2;
        std::string strokeColor;
    };

    // Structure to hold coordinate frame parameters
    struct CoordinateFrameParams {
        double originX;
        double originY;
        double xAxisX;
        double xAxisY;
        double yAxisX;
        double yAxisY;
        std::string text;
    };


    class Svg {
    public:
        Svg();
        
        void drawPoint(const PointParams& pparams);
        void drawVector(const VectorParams& vparams);
        void drawCoordinateFrame(const CoordinateFrameParams& cparams);
        void drawGroup(const std::vector<std::string>& groupElements);

        std::string getSvgString() const;
        void writeToFile(const std::string& filename) const;

    private:
        std::vector<std::string> svgElements;
    };

} // namespace turtlelib

#endif // SVG_HPP
