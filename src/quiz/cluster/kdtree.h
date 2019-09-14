#ifndef KD_TREE_H_
#define KD_TREE_H_

#include <pcl/common/common.h>

#include <memory>
#include <vector>

/// \brief Struct defining a Node in the Kd-tree
struct Node
{
    std::vector<float> point;
    int id;
    int depth;
    std::shared_ptr<Node> left = nullptr;
    std::shared_ptr<Node> right = nullptr;

    Node(const std::vector<float>& setPoint, const int setId, const int setDepth);
};

/// \brief Class implementing a KD-tree.
template <typename PointT>
class KdTree
{
 public:
    /// \brief Insert a point in the tree.
    ///
    /// \param point Point to insert.
    /// \param id ID of the point.
    void insert(const std::vector<float>& point, const int id);

    /// \brief Get a list of point ids in the tree that are within distanceTol to target.
    ///
    /// \param target Point to search in the tree.
    /// \param distanceTol Distance threshold to define proximity to target.
    /// \return List of point ids that are close to target.
    std::vector<int> search(const std::vector<float>& target, const float distanceTol) const;

    /// \brief Get the root of the tree.
    ///
    /// \return The root of the tree.
    std::shared_ptr<Node> getRoot() const;

    /// \brief Set a PCL point cloud as input to build the tree.
    ///
    /// \param cloud Input cloud.
    void setInputCloud(const typename pcl::PointCloud<PointT>::Ptr& cloud);

 private:
    std::shared_ptr<Node> root_ = nullptr;
};

#endif  // KD_TREE_H
