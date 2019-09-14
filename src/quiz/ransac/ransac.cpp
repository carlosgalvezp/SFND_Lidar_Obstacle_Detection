
#include "ransac.h"

#include <cmath>
#include <limits>
#include <random>

namespace
{
    template <typename PointT>
    std::vector<float> fitModel(const PointT& point1, const PointT& point2)
    {
        const float x1 = point1.x;
        const float x2 = point2.x;
        const float y1 = point1.y;
        const float y2 = point2.y;

        return {y1 - y2, x1 - x2, x1*y2 - x2*y1};
    }

    template <typename PointT>
    std::vector<float> fitModel(const PointT& point1,
                                const PointT& point2,
                                const PointT& point3)
    {
        const float x1 = point1.x;
        const float x2 = point2.x;
        const float x3 = point3.x;

        const float y1 = point1.y;
        const float y2 = point2.y;
        const float y3 = point3.y;

        const float z1 = point1.z;
        const float z2 = point2.z;
        const float z3 = point3.z;

        const float i = (y2 - y1)*(z3 - z1) - (z2 - z1)*(y3 - y1);
        const float j = (z2 - z1)*(x3 - x1) - (x2 - x1)*(z3 - z1);
        const float k = (x2 - x1)*(y3 - y1) - (y2 - y1)*(x3 - x1);
        const float d = -(i*x1 + j*y1 + k*z1);

        return {i, j, k, d};
    }

    template <typename PointT>
    float distanceToLine(const PointT& point, const std::vector<float>& line)
    {
        const float a = line[0];
        const float b = line[1];
        const float c = line[2];

        const float x = point.x;
        const float y = point.y;

        return std::fabs(a*x + b*y + c) / std::sqrt(a*a + b*b);
    }

    template <typename PointT>
    float distanceToPlane(const PointT& point, const std::vector<float>& plane)
    {
        const float a = plane[0];
        const float b = plane[1];
        const float c = plane[2];
        const float d = plane[3];

        const float x = point.x;
        const float y = point.y;
        const float z = point.z;

        return std::fabs(a*x + b*y + c*z + d) / std::sqrt(a*a + b*b + c*c);
    }

    template <typename PointT>
    float distance(const PointT& point, const std::vector<float>&model)
    {
        if (model.size() == 3U)
        {
            return distanceToLine(point, model);
        }
        else if (model.size() == 4U)
        {
            return distanceToPlane(point, model);
        }
        else
        {
            std::cerr << "Unknown model with " << model.size() << " parameters" << std::endl;
            return 0.0F;
        }
    }
}

template <typename PointT>
std::unordered_set<int> Ransac<PointT>::run(const typename pcl::PointCloud<PointT>::Ptr& cloud,
                                            const int maxIterations,
                                            const float distanceTol,
                                            const RansacModel model_type) const
{
    std::unordered_set<int> inliersResult;

    // Setup random number generator
    std::seed_seq seed{12345};
    std::mt19937 gen(seed);
    std::uniform_int_distribution<> dis(0, cloud->size() - 1);

    std::vector<float> best_model;
    int max_number_inliers = 0;

    // For max iterations
    for (int i = 0; i < maxIterations; ++i)
    {
        std::vector<float> model;

        // Randomly sample subset and fit model
        if (model_type == RansacModel::LINE)
        {
            const PointT& point1 = cloud->points[dis(gen)];
            const PointT& point2 = cloud->points[dis(gen)];
            model = fitModel(point1, point2);
        }
        else if (model_type == RansacModel::PLANE)
        {
            const PointT& point1 = cloud->points[dis(gen)];
            const PointT& point2 = cloud->points[dis(gen)];
            const PointT& point3 = cloud->points[dis(gen)];
            model = fitModel(point1, point2, point3);
        }
        else
        {
            std::cerr << "Unknown model" << std::endl;
            break;
        }

        // Measure distance between every point and fitted line
        int n_inliers = 0;
        for (const PointT& point : cloud->points)
        {
            // If distance is smaller than threshold count it as inlier
            if (distance(point, model) < distanceTol)
            {
                ++n_inliers;
            }
        }

        if (n_inliers > max_number_inliers)
        {
            max_number_inliers = n_inliers;
            best_model = model;
        }
    }

    // Return indices of inliers according to best model
    for (int i = 0; i < cloud->points.size(); ++i)
    {
        if (distance(cloud->points[i], best_model) < distanceTol)
        {
            inliersResult.insert(i);
        }
    }

    return inliersResult;
}

// Explicit instantiations
template class Ransac<pcl::PointXYZ>;
template class Ransac<pcl::PointXYZI>;
