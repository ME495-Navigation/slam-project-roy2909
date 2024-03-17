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
    /// \brief Circle parameters
    struct Circle_chords
    {   /// x coordinate of the centroid
        double x_centroid=0.0;
        /// y coordinate of the centroid
        double y_centroid=0.0;
        /// radius of the circle
        double radius=0.0;
    };
    /// \brief Circle fitting
    ///param points - the points to fit the circle to
    ///\returns the circle parameters
    Circle_chords circle_fit(const std::vector<turtlelib::Vector2D> &points);

    /// \brief Circle classification
    ///param cluster - the cluster to classify
    ///\returns true if the cluster is a circle
    bool isCircle(const std::vector<turtlelib::Point2D>& cluster);

}



#endif