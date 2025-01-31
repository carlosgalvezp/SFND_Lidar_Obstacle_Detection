// PCL lib Functions for processing point clouds
#include "processPointClouds.h"

#include <pcl/io/pcd_io.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/crop_box.h>
#include <pcl/common/transforms.h>

#include <iostream>
#include <string>
#include <vector>
#include <ctime>
#include <chrono>

#include "render/box.h"

#include "ransac.h"
#include "cluster.h"

//constructor:
template<typename PointT>
ProcessPointClouds<PointT>::ProcessPointClouds() {}


//de-constructor:
template<typename PointT>
ProcessPointClouds<PointT>::~ProcessPointClouds() {}


template<typename PointT>
void ProcessPointClouds<PointT>::numPoints(typename pcl::PointCloud<PointT>::Ptr cloud)
{
    std::cout << cloud->points.size() << std::endl;
}


template<typename PointT>
typename pcl::PointCloud<PointT>::Ptr ProcessPointClouds<PointT>::FilterCloud(
    typename pcl::PointCloud<PointT>::Ptr cloud, float filterRes,
    Eigen::Vector4f minPoint, Eigen::Vector4f maxPoint)
{

    // Time segmentation process
    auto startTime = std::chrono::steady_clock::now();

    // Voxel grid point reduction
    typename pcl::PointCloud<PointT>::Ptr cloud_filtered(new pcl::PointCloud<PointT>());
    pcl::VoxelGrid<PointT> voxel_grid;
    voxel_grid.setInputCloud(cloud);
    voxel_grid.setLeafSize(filterRes, filterRes, filterRes);
    voxel_grid.filter(*cloud_filtered);

    // Region based filtering
    typename pcl::PointCloud<PointT>::Ptr cropped_cloud(new pcl::PointCloud<PointT>());
    pcl::CropBox<PointT> box_cropper;
    box_cropper.setMin(minPoint);
    box_cropper.setMax(maxPoint);
    box_cropper.setInputCloud(cloud_filtered);
    box_cropper.filter(*cropped_cloud);

    // Remove roof of the car
    pcl::IndicesPtr roof_indices(new std::vector<int>());
    box_cropper.setMin(Eigen::Vector4f(-3.0F, -2.0F, -2.0F, 1.0F));
    box_cropper.setMax(Eigen::Vector4f( 3.0F,  2.0F, 0.0F, 1.0F));
    box_cropper.setInputCloud(cropped_cloud);
    box_cropper.filter(*roof_indices);

    typename pcl::PointCloud<PointT>::Ptr output_cloud(new pcl::PointCloud<PointT>());
    pcl::ExtractIndices<PointT> extractor;
    extractor.setInputCloud(cropped_cloud);
    extractor.setIndices(roof_indices);
    extractor.setNegative(true);
    extractor.filter(*output_cloud);

    auto endTime = std::chrono::steady_clock::now();
    auto elapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime);
    std::cout << "filtering took " << elapsedTime.count() << " milliseconds" << std::endl;

    return output_cloud;
}


template<typename PointT>
std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> ProcessPointClouds<PointT>::SeparateClouds(pcl::PointIndices::Ptr inliers, typename pcl::PointCloud<PointT>::Ptr cloud)
{
    // Create two new point clouds, one cloud with obstacles and other with segmented plane
    typename pcl::PointCloud<PointT>::Ptr cloud_plane(new typename pcl::PointCloud<PointT>());
    typename pcl::PointCloud<PointT>::Ptr cloud_obstacles(new typename pcl::PointCloud<PointT>());

    // Create extractor
    pcl::ExtractIndices<PointT> extract;
    extract.setInputCloud(cloud);
    extract.setIndices(inliers);

    // Extract plane
    extract.setNegative(false);
    extract.filter(*cloud_plane);

    // Extract obstacles
    extract.setNegative(true);
    extract.filter(*cloud_obstacles);

    std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> segResult(cloud_obstacles, cloud_plane);
    return segResult;
}


template<typename PointT>
std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> ProcessPointClouds<PointT>::SegmentPlane(typename pcl::PointCloud<PointT>::Ptr cloud, int maxIterations, float distanceThreshold)
{
    // Time segmentation process
    auto startTime = std::chrono::steady_clock::now();

    // Find inliers for the cloud.
    pcl::ModelCoefficients::Ptr plane_coefficients(new pcl::ModelCoefficients());
    pcl::PointIndices::Ptr plane_inliers(new pcl::PointIndices());

    // Segment point cloud into plane + obstacles
    const Ransac<PointT> ransac;
    const std::unordered_set<int> inliers = ransac.run(cloud, maxIterations, distanceThreshold, RansacModel::PLANE);

    // Copy inliners into required data structure for next step
    for (const int inlier : inliers)
    {
        plane_inliers->indices.push_back(inlier);
    }

    auto endTime = std::chrono::steady_clock::now();
    auto elapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime);
    std::cout << "plane segmentation took " << elapsedTime.count() << " milliseconds" << std::endl;

    // Separate segmented point clouds
    std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> segResult = SeparateClouds(plane_inliers,cloud);
    return segResult;
}


template<typename PointT>
std::vector<typename pcl::PointCloud<PointT>::Ptr> ProcessPointClouds<PointT>::Clustering(typename pcl::PointCloud<PointT>::Ptr cloud, float clusterTolerance, int minSize, int maxSize)
{

    // Time clustering process
    auto startTime = std::chrono::steady_clock::now();

    std::vector<typename pcl::PointCloud<PointT>::Ptr> clusters;

    // Create the KdTree object for the search method of the extraction
    KdTree<PointT> tree;
    tree.setInputCloud(cloud);

    // Run euclidean clustering
    const EuclideanClustering<PointT> euclidean_clustering;
    std::vector<pcl::PointIndices> cluster_indices = euclidean_clustering.run(
        cloud, tree, clusterTolerance, minSize, maxSize);

    // Create output
    typename pcl::ExtractIndices<PointT> index_extractor;
    for (const pcl::PointIndices& cluster_indices_i : cluster_indices)
    {
        typename pcl::PointCloud<PointT>::Ptr cluster_i(new typename pcl::PointCloud<PointT>());

        for (const int point_idx : cluster_indices_i.indices)
        {
            cluster_i->push_back(cloud->points[point_idx]);
        }

        cluster_i->width = cluster_i->points.size();
        cluster_i->height = 1;
        cluster_i->is_dense = true;

        clusters.push_back(cluster_i);
    }

    auto endTime = std::chrono::steady_clock::now();
    auto elapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime);
    std::cout << "clustering took " << elapsedTime.count() << " milliseconds and found " << clusters.size() << " clusters" << std::endl;

    return clusters;
}


template<typename PointT>
Box ProcessPointClouds<PointT>::BoundingBox(typename pcl::PointCloud<PointT>::Ptr cluster)
{

    // Find bounding box for one of the clusters
    PointT minPoint, maxPoint;
    pcl::getMinMax3D(*cluster, minPoint, maxPoint);

    Box box;
    box.x_min = minPoint.x;
    box.y_min = minPoint.y;
    box.z_min = minPoint.z;
    box.x_max = maxPoint.x;
    box.y_max = maxPoint.y;
    box.z_max = maxPoint.z;

    return box;
}


template<typename PointT>
void ProcessPointClouds<PointT>::savePcd(typename pcl::PointCloud<PointT>::Ptr cloud, std::string file)
{
    pcl::io::savePCDFileASCII (file, *cloud);
    std::cerr << "Saved " << cloud->points.size () << " data points to "+file << std::endl;
}


template<typename PointT>
typename pcl::PointCloud<PointT>::Ptr ProcessPointClouds<PointT>::loadPcd(std::string file)
{

    typename pcl::PointCloud<PointT>::Ptr cloud (new pcl::PointCloud<PointT>);

    if (pcl::io::loadPCDFile<PointT> (file, *cloud) == -1) //* load the file
    {
        PCL_ERROR ("Couldn't read file \n");
    }
    std::cerr << "Loaded " << cloud->points.size () << " data points from "+file << std::endl;

    return cloud;
}


template<typename PointT>
std::vector<boost::filesystem::path> ProcessPointClouds<PointT>::streamPcd(std::string dataPath)
{

    std::vector<boost::filesystem::path> paths(boost::filesystem::directory_iterator{dataPath}, boost::filesystem::directory_iterator{});

    // sort files in accending order so playback is chronological
    sort(paths.begin(), paths.end());

    return paths;

}

// Template specializations
template class ProcessPointClouds<pcl::PointXYZ>;
template class ProcessPointClouds<pcl::PointXYZI>;
