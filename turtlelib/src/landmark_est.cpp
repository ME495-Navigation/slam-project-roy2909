#include <cmath>
#include <vector>
#include <algorithm>
#include <armadillo>
#include "turtlelib/geometry2d.hpp"
#include "turtlelib/se2d.hpp"
#include "turtlelib/landmark_est.hpp"

namespace turtlelib
{
    Landmark::Landmark()
    {
    }

    Landmark::Landmark(const std::vector<float> &ranges, double angle_increment, double angle_min)
    {
        ranges_ = ranges;
        angle_increment_ = angle_increment;
        angle_min_ = angle_min;
    }

    Vector2D Landmark::polar_to_cartesian(double range, double angle) 
    {
        double x = range * std::cos(angle);
        double y = range * std::sin(angle);
        return Vector2D{x, y};
    }

  std::vector<std::vector<Vector2D>> Landmark::cluster_points(double distance_threshold)
{
    std::vector<std::vector<Vector2D>> clusters;
    std::vector<Vector2D> current_cluster;

    for (size_t i = 0; i < ranges_.size(); i++)
    {
        Vector2D current_point = polar_to_cartesian(ranges_[i], angle_min_ + i * angle_increment_);

        // Check if the current cluster is empty
        if (current_cluster.empty())
        {
            current_cluster.push_back(current_point);
            continue; // Move to the next iteration to avoid duplicate entry
        }

        Vector2D previous_point = polar_to_cartesian(ranges_[i - 1], angle_min_ + (i - 1) * angle_increment_);

        double dx = current_point.x - previous_point.x;
        double dy = current_point.y - previous_point.y;
        double distance = std::sqrt(dx * dx + dy * dy);

        if (distance <= distance_threshold)
        {
            // Add the point to the current cluster
            current_cluster.push_back(current_point);
        }
        else if (!current_cluster.empty())
        {
            // Check if the current cluster has enough points
            if (current_cluster.size() >= 3)
            {
                // Add the current cluster to the list of clusters
                clusters.push_back(current_cluster);
            }

            // Start a new cluster with the current point
            current_cluster.clear();
        }
    }

    // Check if the last cluster has enough points
    if (!current_cluster.empty() && current_cluster.size() >= 3)
    {
        // Add the last cluster to the list of clusters
        clusters.push_back(current_cluster);
    }

    return clusters;
}

}

   