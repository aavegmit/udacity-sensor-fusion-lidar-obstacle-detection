/* \author Aaron Brown */
// Create simple 3d highway enviroment using PCL
// for exploring self-driving car sensors

#include "sensors/lidar.h"
#include "render/render.h"
#include "processPointClouds.h"
// using templates for processPointClouds so also include .cpp to help linker
#include "processPointClouds.cpp"

std::vector<Car> initHighway(bool renderScene, pcl::visualization::PCLVisualizer::Ptr& viewer)
{

    Car egoCar( Vect3(0,0,0), Vect3(4,2,2), Color(0,1,0), "egoCar");
    Car car1( Vect3(15,0,0), Vect3(4,2,6), Color(0,0,1), "car1");
    Car car2( Vect3(8,-4,0), Vect3(4,2,6), Color(0,0,1), "car2");	
    Car car3( Vect3(-12,4,0), Vect3(4,2,6), Color(0,0,1), "car3");
  
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

void cityBlock(pcl::visualization::PCLVisualizer::Ptr& viewer, ProcessPointClouds<pcl::PointXYZI>* processorI, const pcl::PointCloud<pcl::PointXYZI>::Ptr& inputCloud) {
    auto filterCloud = processorI->FilterCloud(inputCloud, .2, Eigen::Vector4f(-15,-5,-20,1), Eigen::Vector4f(20,5,3,1));

    pcl::PointXYZI minPoint, maxPoint;
    pcl::getMinMax3D(*inputCloud, minPoint, maxPoint);
    Box box;
    box.x_min = -1.5;
    box.y_min = -1.7;
    box.z_min = -1;
    box.x_max = 2.6;
    box.y_max = 1.7;
    box.z_max = -.4;

    //renderBox(viewer, box, 12);

    std::pair<pcl::PointCloud<pcl::PointXYZI>::Ptr, pcl::PointCloud<pcl::PointXYZI>::Ptr> segmentCloud = processorI->SegmentPlane(filterCloud, 10, .3);
    renderPointCloud(viewer, segmentCloud.first, "planeCloud", Color(0,1,0));
    //renderPointCloud(viewer, segmentCloud.second, "obstacleCloud", Color(1,0,0));

    std::vector<pcl::PointCloud<pcl::PointXYZI>::Ptr> cloudClusters = processorI->ClusteringCustomImpl(segmentCloud.second, 0.48, 10, 500);

    int clusterId = 0;
    std::vector<Color> colors = {Color(1,0,0), Color(0,1,0), Color(0,0,1) };

    for (auto cluster : cloudClusters) {
        std::cout << "cluster size ";
        processorI->numPoints(cluster);
        renderPointCloud(viewer, cluster, "obstCloud"+std::to_string(clusterId), colors[clusterId%3]);
        Box box = processorI->BoundingBox(cluster);
        renderBox(viewer, box, clusterId);
        ++clusterId; 

    }
    
}

void simpleHighway(pcl::visualization::PCLVisualizer::Ptr& viewer)
{
    // ----------------------------------------------------
    // -----Open 3D viewer and display simple highway -----
    // ----------------------------------------------------
    
    // RENDER OPTIONS
    bool renderScene = false;
    std::vector<Car> cars = initHighway(renderScene, viewer);
    
    // TODO:: Create lidar sensor 
    Lidar *lidar = new Lidar(cars, 0);

    // TODO:: Create point processor
    pcl::PointCloud<pcl::PointXYZ>::Ptr inputCloud  = lidar->scan();
    //renderRays(viewer, lidar->position , pointPro"cessor);
    renderPointCloud(viewer, inputCloud,"inputCloud");

    ProcessPointClouds<pcl::PointXYZ> pointProcessor;
    std::pair<pcl::PointCloud<pcl::PointXYZ>::Ptr, pcl::PointCloud<pcl::PointXYZ>::Ptr> segmentCloud = pointProcessor.SegmentPlane(inputCloud, 30, 0.2);
    //renderPointCloud(viewer, segmentCloud.first, "planeCloud", Color(1,0,0) );
    //renderPointCloud(viewer, segmentCloud.second, "obstacleCloud", Color(0,1,0));

    std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> cloudClusters = pointProcessor.Clustering(segmentCloud.second, 1.3, 3, 30);

    int clusterId = 0;
    std::vector<Color> colors = {Color(1,0,0), Color(0,1,0), Color(0,0,1) };

    for (auto cluster : cloudClusters) {
        std::cout << "cluster size ";
        pointProcessor.numPoints(cluster);
        renderPointCloud(viewer, cluster, "obstCloud"+std::to_string(clusterId), colors[clusterId]);
        Box box = pointProcessor.BoundingBox(cluster);
        renderBox(viewer, box, clusterId);
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
    CameraAngle setAngle = FPS;
    initCamera(setAngle, viewer);
    ProcessPointClouds<pcl::PointXYZI>* processorI = new ProcessPointClouds<pcl::PointXYZI>();
    std::vector<boost::filesystem::path> stream = processorI->streamPcd("../src/sensors/data/pcd/data_1");
    auto streamIterator = stream.begin();
    pcl::PointCloud<pcl::PointXYZI>::Ptr inputCloudI;
    //simpleHighway(viewer);
    //cityBlock(viewer);
    // inputCloudI = processorI->loadPcd((*streamIterator).string());
    // cityBlock(viewer, processorI, inputCloudI);

    while (!viewer->wasStopped ())
    {
        viewer->removeAllPointClouds();
        viewer->removeAllShapes();

        inputCloudI = processorI->loadPcd((*streamIterator).string());
        cityBlock(viewer, processorI, inputCloudI);
        streamIterator++;
        if (streamIterator == stream.end()) {
            streamIterator = stream.begin();
        }

        viewer->spinOnce ();
    } 
}