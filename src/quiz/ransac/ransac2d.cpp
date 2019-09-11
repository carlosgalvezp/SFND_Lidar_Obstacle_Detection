/* \author Aaron Brown */
// Quiz on implementing simple RANSAC line fitting

#include "../../render/render.h"
#include <cmath>
#include <limits>
#include <random>
#include <unordered_set>
#include "../../processPointClouds.h"
// using templates for processPointClouds so also include .cpp to help linker
#include "../../processPointClouds.cpp"

namespace
{
    enum class RansacModel
    {
        LINE,
        PLANE
    };

    std::vector<float> fitModel(const pcl::PointXYZ& point1, const pcl::PointXYZ& point2)
    {
        const float x1 = point1.x;
        const float x2 = point2.x;
        const float y1 = point1.y;
        const float y2 = point2.y;

        return {y1 - y2, x1 - x2, x1*y2 - x2*y1};
    }

    std::vector<float> fitModel(const pcl::PointXYZ& point1,
                                const pcl::PointXYZ& point2,
                                const pcl::PointXYZ& point3)
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

    float distanceToLine(const pcl::PointXYZ& point, const std::vector<float>& line)
    {
        const float a = line[0];
        const float b = line[1];
        const float c = line[2];

        const float x = point.x;
        const float y = point.y;

        return std::fabs(a*x + b*y + c) / std::sqrt(a*a + b*b);
    }

    float distanceToPlane(const pcl::PointXYZ& point, const std::vector<float>& plane)
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

    float distance(const pcl::PointXYZ& point, const std::vector<float>&model)
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

std::unordered_set<int> Ransac(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, int maxIterations, float distanceTol,
                               const RansacModel model_type)
{
    std::unordered_set<int> inliersResult;
    srand(time(NULL));

    std::seed_seq seed{12345};  //Will be used to obtain a seed for the random number engine
    std::mt19937 gen(seed); //Standard mersenne_twister_engine seeded with rd()
    std::uniform_int_distribution<> dis(0, cloud->size() - 1);

    std::vector<float> best_line_model(3);
    int max_number_inliers = 0;

    // For max iterations
    for (int i = 0; i < maxIterations; ++i)
    {
        std::vector<float> model;

        // Randomly sample subset and fit model
        if (model_type == RansacModel::LINE)
        {
            const pcl::PointXYZ point1 = cloud->points[dis(gen)];
            const pcl::PointXYZ point2 = cloud->points[dis(gen)];
            model = fitModel(point1, point2);
        }
        else if (model_type == RansacModel::PLANE)
        {
            const pcl::PointXYZ point1 = cloud->points[dis(gen)];
            const pcl::PointXYZ point2 = cloud->points[dis(gen)];
            const pcl::PointXYZ point3 = cloud->points[dis(gen)];
            model = fitModel(point1, point2, point3);
        }
        else
        {
            std::cerr << "Unknown model" << std::endl;
            break;
        }

        // Measure distance between every point and fitted line
        int n_inliers = 0;
        for (const pcl::PointXYZ point : cloud->points)
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
            best_line_model = model;
        }
    }

    // Return indicies of inliers from fitted line with most inliers
    for (int i = 0; i < cloud->points.size(); ++i)
    {
        if (distance(cloud->points[i], best_line_model) < distanceTol)
        {
            inliersResult.insert(i);
        }
    }

    return inliersResult;

}

int main ()
{

    // Create viewer
    pcl::visualization::PCLVisualizer::Ptr viewer = initScene();

    // Create data
    // pcl::PointCloud<pcl::PointXYZ>::Ptr cloud = CreateData();
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud = CreateData3D();


    // Change the max iteration and distance tolerance arguments for Ransac function
    // std::unordered_set<int> inliers = Ransac(cloud, 50, 0.5F, RansacModel::LINE);
    std::unordered_set<int> inliers = Ransac(cloud, 50, 0.5F, RansacModel::PLANE);

    pcl::PointCloud<pcl::PointXYZ>::Ptr cloudInliers(new pcl::PointCloud<pcl::PointXYZ>());
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloudOutliers(new pcl::PointCloud<pcl::PointXYZ>());

    for(int index = 0; index < cloud->points.size(); index++)
    {
        pcl::PointXYZ point = cloud->points[index];
        if(inliers.count(index))
            cloudInliers->points.push_back(point);
        else
            cloudOutliers->points.push_back(point);
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
