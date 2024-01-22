// svg.hpp
#ifndef SVG_HPP
#define SVG_HPP

#include <string>
#include <vector>
#include "geometry2d.hpp"

namespace turtlelib
{   
    ///  \brief represent a 2-Dimensional Point
    struct PointParams
    {   
        /// \brief point x coordinate
        double x;
        /// \brief point y coordinate
        double y;
        /// \brief color of outline
        std::string strokeColor;
        /// \brief filled color
        std::string fillColor;
    };
    /// \brief represent a 2-Dimensional Vector
    struct VectorParams
    {   
        /// \brief vector x coordinate head
        double x1;
        /// \brief vector y coordinate head
        double y1;
        /// \brief vector x coordinate tail
        double x2;
        /// \brief vector y coordinate tail
        double y2;
        /// \brief vector color outline
        std::string strokeColor;
        /// \brief vector xcolor filled
        std::string text;
    };
     /// \brief a class to display SVG elements
    class Svg
    {
    public:
    /// \brief constructor
        Svg();
        /// \brief draw a point
        /// \param pparams - point parameters
        void drawPoint(const PointParams &pparams);
        /// \brief draw a vector
        /// \param vparams - vector parameters
        void drawVector(const VectorParams &vparams);
        /// \brief draw Coordinate frames
        /// \param origin - point 
        /// \param x_vector - Vector
        void drawCoordinateFrame(Point2D origin, Vector2D x_vector, const std::string);
        /// \brief get Svg as string
        std::string getSvgString() const;
        /// \brief output to file
        /// \param filename - name of output file
        void writeToFile(const std::string &filename) const;

    private:
        std::vector<std::string> svgElements;
        double midpointX = 408;
        double midpointY = 528;
        double conversion_factor = 96;
    };

} // namespace turtlelib

#endif // SVG_HPP