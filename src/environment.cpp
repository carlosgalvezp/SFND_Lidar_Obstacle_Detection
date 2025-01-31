/* \author Aaron Brown */
// Create simple 3d highway enviroment using PCL
// for exploring self-driving car sensors
#include "sensors/lidar.h"
#include "render/render.h"
#include "processPointClouds.h"

#include <memory>

std::vector<Car> initHighway(bool renderScene, pcl::visualization::PCLVisualizer::Ptr& viewer)
{

    Car egoCar( Vect3(0,0,0), Vect3(4,2,2), Color(0,1,0), "egoCar");
    Car car1( Vect3(15,0,0), Vect3(4,2,2), Color(0,0,1), "car1");
    Car car2( Vect3(8,-4,0), Vect3(4,2,2), Color(0,0,1), "car2");
    Car car3( Vect3(-12,4,0), Vect3(4,2,2), Color(0,0,1), "car3");

    std::vector<Car> cars;
    cars.push_back(egoCar);
    cars.push_back(car1);
    cars.push_back(car2);
    cars.push_back(car3);

    if(renderScene)
    {
        renderHighway(viewer);
        egoCar.render(viewer);
        car1.render(viewer);
        car2.render(viewer);
        car3.render(viewer);
    }

    return cars;
}


void simpleHighway(pcl::visualization::PCLVisualizer::Ptr& viewer)
{
    // ----------------------------------------------------
    // -----Open 3D viewer and display simple highway -----
    // ----------------------------------------------------

    // RENDER OPTIONS
    bool renderScene = false;
    double ground_slope = 0.0;
    std::vector<Car> cars = initHighway(renderScene, viewer);

    // Create lidar sensor
    std::unique_ptr<Lidar> lidar(new Lidar(cars, ground_slope));

    // Take a scan and render the rays and the point cloud
    const pcl::PointCloud<pcl::PointXYZ>::Ptr cloud = lidar->scan();
    const Vect3 origin = lidar->position;
    // renderRays(viewer, origin, cloud);
    // renderPointCloud(viewer, cloud, "pointcloud");

    // Create point processor
    ProcessPointClouds<pcl::PointXYZ> processor{};

    auto segmentCloud = processor.SegmentPlane(cloud, 100, 0.2);
    // renderPointCloud(viewer,segmentCloud.first, "obstacleCloud", Color(1,0,0));
    // renderPointCloud(viewer,segmentCloud.second, "planeCloud", Color(0,1,0));

    // Cluster obstacles and render them
    std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> cloudClusters =
        processor.Clustering(segmentCloud.first, 1.0, 3, 30);

    int clusterId = 0;
    std::vector<Color> colors = {Color(1,0,0), Color(1,1,0), Color(0,0,1)};

    for(pcl::PointCloud<pcl::PointXYZ>::Ptr cluster : cloudClusters)
    {
        std::cout << "cluster size ";
        processor.numPoints(cluster);
        renderPointCloud(viewer,cluster,"obstCloud"+std::to_string(clusterId),colors[clusterId]);
        Box box = processor.BoundingBox(cluster);
        renderBox(viewer,box,clusterId);
        ++clusterId;
    }
}

void cityBlock(pcl::visualization::PCLVisualizer::Ptr& viewer,
               ProcessPointClouds<pcl::PointXYZI>* processor,
               const pcl::PointCloud<pcl::PointXYZI>::Ptr& inputCloud)
{
    // ----------------------------------------------------
    // -----Open 3D viewer and display City Block     -----
    // ----------------------------------------------------
    // Filter point cloud
    auto filterCloud = processor->FilterCloud(inputCloud, 0.25F,
                                              Eigen::Vector4f(-10.0F, -6.0F, -3.0F, 1.0F),
                                              Eigen::Vector4f(30.0F, 7.0F, 1.0F, 1.0F));
    // renderPointCloud(viewer, filterCloud, "filterCloud");

    // Segment cloud between road and obstacles
    const int maxIterations = 100;
    const float distanceThreshold = 0.2F;
    auto segmentCloud = processor->SegmentPlane(filterCloud, maxIterations, distanceThreshold);
    renderPointCloud(viewer,segmentCloud.first, "obstacleCloud", Color(1,0,0));
    renderPointCloud(viewer,segmentCloud.second, "planeCloud", Color(0,1,0));

    // Cluster obstacles and render them
    const float clusterTolerance = 0.5F;
    const int minSize = 10;
    const int maxSize = 1000;
    std::vector<pcl::PointCloud<pcl::PointXYZI>::Ptr> cloudClusters =
        processor->Clustering(segmentCloud.first, clusterTolerance, minSize, maxSize);

    int clusterId = 0;
    std::vector<Color> colors = {Color(1,0,0), Color(1,1,0), Color(0,0,1)};

    for(pcl::PointCloud<pcl::PointXYZI>::Ptr cluster : cloudClusters)
    {
        std::cout << "cluster size ";
        processor->numPoints(cluster);
        renderPointCloud(viewer,cluster,"obstCloud" + std::to_string(clusterId), colors[clusterId % 3]);
        Box box = processor->BoundingBox(cluster);
        renderBox(viewer,box,clusterId);
        ++clusterId;
    }
}

//setAngle: SWITCH CAMERA ANGLE {XY, TopDown, Side, FPS}
void initCamera(CameraAngle setAngle, pcl::visualization::PCLVisualizer::Ptr& viewer)
{

    viewer->setBackgroundColor (0, 0, 0);

    // set camera position and angle
    viewer->initCameraParameters();
    // distance away in meters
    int distance = 16;

    switch(setAngle)
    {
        case XY : viewer->setCameraPosition(-distance, -distance, distance, 1, 1, 0); break;
        case TopDown : viewer->setCameraPosition(0, 0, distance, 1, 0, 1); break;
        case Side : viewer->setCameraPosition(0, -distance, 0, 0, 0, 1); break;
        case FPS : viewer->setCameraPosition(-10, 0, 0, 0, 0, 1);
    }

    if(setAngle!=FPS)
        viewer->addCoordinateSystem (1.0);
}


int main (int argc, char** argv)
{
    std::cout << "starting enviroment" << std::endl;

    pcl::visualization::PCLVisualizer::Ptr viewer (new pcl::visualization::PCLVisualizer ("3D Viewer"));
    CameraAngle setAngle = XY;
    initCamera(setAngle, viewer);
    // simpleHighway(viewer);

    ProcessPointClouds<pcl::PointXYZI>* pointProcessorI = new ProcessPointClouds<pcl::PointXYZI>();
    std::vector<boost::filesystem::path> stream = pointProcessorI->streamPcd("../src/sensors/data/pcd/data_1");
    auto streamIterator = stream.begin();
    pcl::PointCloud<pcl::PointXYZI>::Ptr inputCloudI;

    while (!viewer->wasStopped ())
    {
        // Clear viewer
        viewer->removeAllPointClouds();
        viewer->removeAllShapes();

        // Load pcd and run obstacle detection process
        inputCloudI = pointProcessorI->loadPcd((*streamIterator).string());
        cityBlock(viewer, pointProcessorI, inputCloudI);

        streamIterator++;
        if(streamIterator == stream.end())
            streamIterator = stream.begin();

        viewer->spinOnce();
    }
}
