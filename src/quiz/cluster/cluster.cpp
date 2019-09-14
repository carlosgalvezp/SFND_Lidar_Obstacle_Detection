#include "cluster.h"

namespace
{

void proximity(const int point_idx, const std::vector<std::vector<float>>& points,
               const KdTree& tree, const float distanceTol, std::vector<bool>& processed_points,
               std::vector<int>& cluster)
{
    processed_points[point_idx] = true;
    cluster.push_back(point_idx);

    const std::vector<int> nearby_points = tree.search(points[point_idx], distanceTol);

    for (const int nearby_point : nearby_points)
    {
        if (!processed_points[nearby_point])
        {
            proximity(nearby_point, points, tree, distanceTol, processed_points, cluster);
        }
    }
}

}  // namespace

std::vector<std::vector<int>> euclideanCluster(const std::vector<std::vector<float>>& points,
                                               const KdTree& tree,
                                               const float distanceTol)
{
    // Return the list of indices for each cluster
    std::vector<std::vector<int>> clusters;

    std::vector<bool> processed_points(points.size());

    for (int i = 0; i < points.size(); ++i)
    {
        if (!processed_points[i])
        {
            std::vector<int> cluster;
            proximity(i, points, tree, distanceTol, processed_points, cluster);
            clusters.push_back(cluster);
        }
    }

    return clusters;
}

template <typename PointT>
std::vector<pcl::PointIndices> euclideanCluster(const typename pcl::PointCloud<PointT>::Ptr& points,
                                                const KdTree& tree,
                                                const float distanceTol)
{}

// Explicit instantiations
template
std::vector<pcl::PointIndices> euclideanCluster<pcl::PointXYZI>(const pcl::PointCloud<pcl::PointXYZI>::Ptr&,
                                                                const KdTree&, const float);
