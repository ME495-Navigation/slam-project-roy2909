#ifndef LANDMARK_EST_INCLUDE_GUARD_HPP
#define LANDMARK_EST_INCLUDE_GUARD_HPP

#include "turtlelib/geometry2d.hpp"
#include "turtlelib/se2d.hpp"
#include <vector>
#include <cmath>
#include <algorithm>
#include <armadillo>

namespace turtlelib
{
    /// \brief Landmark Estimation
    class Landmark
    {
    public:
        Landmark();
        ///\brief get sensor data
        ///\param ranges - the ranges of the laser scan
        ///\param angle_increment - the angle increment of the laser scan
        ///\param angle_min - the minimum angle of the laser scan
        Landmark(const std::vector<float> &ranges, double angle_increment, double angle_min);

        /// \brief polar to cartesian
        /// \param range - distace of landmark
        /// \param angle - angle of landmark
        /// \returns the cartesian coordinates of the landmark
        Vector2D polar_to_cartesian(double range, double angle);
        /// \brief cluster the laser scan data
        /// \param distance_threshold - the distance threshold for clustering
        /// \returns the clusters
        std::vector<std::vector<Vector2D>> cluster_points(double distance_threshold);
        /// \brief get the clusters
        /// \returns the clusters
        std::vector<double> get_clusters() const;

    private:
        std::vector<double> clusters_;
        std::vector<float> ranges_;
        double angle_increment_;
        double angle_min_;
    };
}
#endif