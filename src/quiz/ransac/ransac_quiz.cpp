/* \author Aaron Brown */
// Quiz on implementing simple RANSAC line fitting
#include <unordered_set>

#include "../../render/render.h"
#include "../../processPointClouds.h"

#include "ransac.h"

pcl::PointCloud<pcl::PointXYZ>::Ptr CreateData()
{
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>());
    // Add inliers
    float scatter = 0.6;
    for(int i = -5; i < 5; i++)
    {
        double rx = 2*(((double) rand() / (RAND_MAX))-0.5);
        double ry = 2*(((double) rand() / (RAND_MAX))-0.5);
        pcl::PointXYZ point;
        point.x = i+scatter*rx;
        point.y = i+scatter*ry;
        point.z = 0;
        cloud->points.push_back(point);
    }
    // Add outliers
    int numOutliers = 10;
    while(numOutliers--)
    {
        double rx = 2*(((double) rand() / (RAND_MAX))-0.5);
        double ry = 2*(((double) rand() / (RAND_MAX))-0.5);
        pcl::PointXYZ point;
        point.x = 5*rx;
        point.y = 5*ry;
        point.z = 0;
        cloud->points.push_back(point);
    }
    cloud->width = cloud->points.size();
    cloud->height = 1;
    return cloud;
}

pcl::PointCloud<pcl::PointXYZ>::Ptr CreateData3D()
{
    ProcessPointClouds<pcl::PointXYZ> pointProcessor;
    return pointProcessor.loadPcd("src/sensors/data/pcd/simpleHighway.pcd");
}


pcl::visualization::PCLVisualizer::Ptr initScene()
{
    pcl::visualization::PCLVisualizer::Ptr viewer(new pcl::visualization::PCLVisualizer ("2D Viewer"));
    viewer->setBackgroundColor (0, 0, 0);
      viewer->initCameraParameters();
      viewer->setCameraPosition(0, 0, 15, 0, 1, 0);
      viewer->addCoordinateSystem (1.0);
      return viewer;
}

int main ()
{

    // Create viewer
    pcl::visualization::PCLVisualizer::Ptr viewer = initScene();

    // Create data
    // pcl::PointCloud<pcl::PointXYZ>::Ptr cloud = CreateData();
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud = CreateData3D();

    // Change the max iteration and distance tolerance arguments for Ransac function
    Ransac<pcl::PointXYZ> ransac;
    // std::unordered_set<int> inliers = ransac.run(cloud, 50, 0.5F, RansacModel::LINE);
    std::unordered_set<int> inliers = ransac.run(cloud, 50, 0.5F, RansacModel::PLANE);

    pcl::PointCloud<pcl::PointXYZ>::Ptr cloudInliers(new pcl::PointCloud<pcl::PointXYZ>());
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloudOutliers(new pcl::PointCloud<pcl::PointXYZ>());

    for(int index = 0; index < cloud->points.size(); index++)
    {
        pcl::PointXYZ point = cloud->points[index];
        if(inliers.count(index))
        {
            cloudInliers->points.push_back(point);
        }
        else
        {
            cloudOutliers->points.push_back(point);
        }
    }


    // Render 2D point cloud with inliers and outliers
    if(inliers.size())
    {
        renderPointCloud(viewer,cloudInliers,"inliers",Color(0,1,0));
        renderPointCloud(viewer,cloudOutliers,"outliers",Color(1,0,0));
    }
    else
    {
        renderPointCloud(viewer,cloud,"data");
    }

    while (!viewer->wasStopped ())
    {
        viewer->spinOnce ();
    }
}
