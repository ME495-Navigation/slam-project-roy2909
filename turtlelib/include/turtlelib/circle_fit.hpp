#ifndef CIRCLE_FIT_INCLUDE_GUARD_HPP
#define CIRCLE_FIT_INCLUDE_GUARD_HPP
/// \file
/// \brief A library for fitting circles 
#include <vector>
#include "turtlelib/geometry2d.hpp"
#include "turtlelib/se2d.hpp"
#include <cmath>
#include <algorithm>
#include <armadillo>
namespace turtlelib
{
    struct Circle_chords
    {
        double x_centroid=0.0;
        double y_centroid=0.0;
        double radius=0.0;
    };
    /// \brief Circle fitting
    ///param points - the points to fit the circle to
    ///\returns the circle parameters
    Circle_chords circle_fit(const std::vector<Vector2D> &points);
}



#endif