#include "cluster.h"

namespace
{

template <typename PointT>
void proximity(const int point_idx, const std::vector<std::vector<float>>& points,
               const KdTree<PointT>& tree, const float distanceTol, std::vector<bool>& processed_points,
               pcl::PointIndices& cluster)
{
    processed_points[point_idx] = true;
    cluster.indices.push_back(point_idx);

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

template <typename PointT>
std::vector<pcl::PointIndices> EuclideanClustering<PointT>::run(const typename pcl::PointCloud<PointT>::Ptr& cloud,
                                                                const KdTree<PointT>& tree,
                                                                const float distanceTol,
                                                                const int min_size,
                                                                const int max_size) const
{
    // Convert point cloud into vector<vector<float>> for easier processing in kd-tree
    std::vector<std::vector<float>> points;
    for (const PointT& point : cloud->points)
    {
        points.push_back({point.x, point.y, point.z});
    }

    // Run euclidean clustering
    std::vector<pcl::PointIndices> clusters;
    std::vector<bool> processed_points(points.size());

    for (int i = 0; i < points.size(); ++i)
    {
        if (!processed_points[i])
        {
            pcl::PointIndices cluster;
            proximity(i, points, tree, distanceTol, processed_points, cluster);

            if ((cluster.indices.size() >= min_size) &&
                (cluster.indices.size() <= max_size))
            {
                clusters.push_back(cluster);
            }
        }
    }

    return clusters;
}

// Explicit instantiations
template class EuclideanClustering<pcl::PointXYZ>;
template class EuclideanClustering<pcl::PointXYZI>;
