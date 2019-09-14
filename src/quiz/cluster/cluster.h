#ifndef CLUSTER_H_
#define CLUSTER_H_

#include <pcl/common/common.h>

#include <vector>

#include "kdtree.h"

std::vector<std::vector<int>> euclideanCluster(const std::vector<std::vector<float>>& points,
                                               const KdTree& tree,
                                               float distanceTol);

template <typename PointT>
std::vector<pcl::PointIndices> euclideanCluster(const typename pcl::PointCloud<PointT>::Ptr& points,
                                                const KdTree& tree,
                                                float distanceTol);


#endif  // CLUSTER_H_
