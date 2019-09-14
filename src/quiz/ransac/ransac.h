#ifndef RANSAC_H_
#define RANSAC_H

#include <pcl/common/common.h>

#include <unordered_set>

/// \brief Type of RANSAC model to use.
enum class RansacModel
{
    LINE,  ///< Fit a linear model
    PLANE  ///< Fit a planar model
};

/// \brief Class to perform RANSAC on a point cloud.
///
/// \tparam PointT Type of the point cloud to process.
template <typename PointT>
class Ransac
{
 public:
    /// \brief Run the RANSAC algorithm on a point cloud.
    ///
    /// \param cloud Point cloud to process.
    /// \param maxIterations Number of iterations to run RANSAC for.
    /// \param distanceTol Distance (in meters) from a point to the model to be considered
    ///        inlier or not.
    /// \param model_type Type of the model to fit.
    /// \return List of indices corresponding to the points belonging to the fitted model.
    std::unordered_set<int> run(const typename pcl::PointCloud<PointT>::Ptr& cloud,
                                const int maxIterations,
                                const float distanceTol,
                                const RansacModel model_type) const;
};

#endif  // RANSAC_H_
