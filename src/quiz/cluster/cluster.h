#ifndef CLUSTER_H_
#define CLUSTER_H_

#include <pcl/common/common.h>

#include <limits>
#include <vector>

#include "kdtree.h"

template <typename PointT>
class EuclideanClustering
{
 public:
    std::vector<pcl::PointIndices> run(const typename pcl::PointCloud<PointT>::Ptr& points,
                                       const KdTree<PointT>& tree, float distanceTol,
                                       const int min_size = 1,
                                       const int max_size = std::numeric_limits<int>::max()) const;
};

#endif  // CLUSTER_H_
