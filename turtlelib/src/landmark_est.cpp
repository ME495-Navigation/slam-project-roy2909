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

    Vector2D first_point = polar_to_cartesian(ranges_[0], angle_min_);
    Vector2D prev_point = first_point;

    for (size_t i = 0; i < ranges_.size(); i++)
    {
        Vector2D current_point = polar_to_cartesian(ranges_[i], angle_min_ + i * angle_increment_);

        // Skip the initial (0, 0) ...temp solution
        if (current_point.x == 0 && current_point.y == 0)
        {
            continue;
        }

        // Check the distance threshold
        Vector2D diff = current_point - prev_point;
        double distance = magnitude(diff);

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
            current_cluster.push_back(current_point);
        }

        // Add the last point to the cluster if it is the last iteration
        if (i == ranges_.size() - 1 && !current_cluster.empty())
        {
            if (current_cluster.size() >= 3)
            {
                // Add the last cluster to the list of clusters
                clusters.push_back(current_cluster);
            }
        }

        prev_point = current_point;
    }

    // Check if the last and first points are close enough to be in the same cluster
    Vector2D diff = first_point - prev_point;
    double distance = magnitude(diff);

    if (distance <= distance_threshold && clusters.size() > 0)
    {
        // Merge the last cluster with the first cluster
        clusters[0].insert(clusters[0].end(), current_cluster.begin(), current_cluster.end());
    }
    else if (!current_cluster.empty())
    {
        // Check if the last cluster has enough points
        if (current_cluster.size() >= 3)
        {
            // Add the last cluster to the list of clusters
            clusters.push_back(current_cluster);
        }
    }

    return clusters;
}

}

   