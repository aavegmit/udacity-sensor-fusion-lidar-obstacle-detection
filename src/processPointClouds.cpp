// PCL lib Functions for processing point clouds 

#include "processPointClouds.h"
#include <unordered_set>


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
typename pcl::PointCloud<PointT>::Ptr ProcessPointClouds<PointT>::FilterCloud(typename pcl::PointCloud<PointT>::Ptr cloud, float filterRes, Eigen::Vector4f minPoint, Eigen::Vector4f maxPoint)
{

    // Time segmentation process
    auto startTime = std::chrono::steady_clock::now();

    // TODO:: Fill in the function to do voxel grid point reduction and region based filtering
    pcl::VoxelGrid<PointT> sor;
    typename pcl::PointCloud<PointT>::Ptr cloudFiltered (new pcl::PointCloud<PointT>);
    sor.setInputCloud(cloud);
    sor.setLeafSize(filterRes, filterRes, filterRes);
    sor.filter(*cloudFiltered);

    typename pcl::PointCloud<PointT>::Ptr cloudRegion (new pcl::PointCloud<PointT>);

    pcl::CropBox<PointT> region(true);
    region.setMin(minPoint);
    region.setMax(maxPoint);
    region.setInputCloud(cloudFiltered);
    region.filter(*cloudRegion);

    std::vector<int> indices;

    pcl::CropBox<PointT> roof(true);
    roof.setMin(Eigen::Vector4f (-1.5, -1.7, -1, 1));
    roof.setMax(Eigen::Vector4f (2.6, 1.7, -.4, 1));
    roof.setInputCloud(cloudRegion);
    roof.filter(indices);

    pcl::PointIndices::Ptr inliers {new pcl::PointIndices};
    for (int point : indices) {
        inliers->indices.push_back(point);
    }

    pcl::ExtractIndices<PointT> extract;
    extract.setInputCloud (cloudRegion);
    extract.setIndices (inliers);
    extract.setNegative(true);
    extract.filter(*cloudRegion);

    
    

    auto endTime = std::chrono::steady_clock::now();
    auto elapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime);
    std::cout << "filtering took " << elapsedTime.count() << " milliseconds" << std::endl;

    return cloudRegion;

}


template<typename PointT>
std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> ProcessPointClouds<PointT>::SeparateClouds(pcl::PointIndices::Ptr inliers, typename pcl::PointCloud<PointT>::Ptr cloud) 
{
  // TODO: Create two new point clouds, one cloud with obstacles and other with segmented plane
  typename pcl::PointCloud<PointT>::Ptr road (new pcl::PointCloud<PointT>());
  typename pcl::PointCloud<PointT>::Ptr obstacles (new pcl::PointCloud<PointT>());

  for (int index : inliers->indices) {
      road->points.push_back(cloud->points[index]);

  }

  pcl::ExtractIndices<pcl::PointXYZ> extract;
  extract.setInputCloud(cloud);
  extract.setIndices(inliers);
  extract.setNegative(true);
  extract.filter(*obstacles);


    std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> segResult(road, obstacles);
    return segResult;
}

template<typename PointT>
std::unordered_set<int> ProcessPointClouds<PointT>::segmentUsingRansac(typename pcl::PointCloud<PointT>::Ptr cloud, int maxIterations, float distanceThreshold) {

    std::unordered_set<int> inliersResult;
	srand(time(NULL));
	
	

	while(maxIterations--) {
        std::unordered_set<int> inlines;

		while (inlines.size() < 3) {
			inlines.insert(rand() % cloud->points.size());
		}

		float x1, x2, x3, y1, y2, y3, z1, z2, z3;

		auto itr = inlines.begin();
		x1 = cloud->points[*itr].x;
		y1 = cloud->points[*itr].y;
		z1 = cloud->points[*itr].z;
		itr++;
		x2 = cloud->points[*itr].x;
		y2 = cloud->points[*itr].y;
		z2 = cloud->points[*itr].z;
		itr++;
		x3 = cloud->points[*itr].x;
		y3 = cloud->points[*itr].y;
		z3 = cloud->points[*itr].z;

		float i = (y2 - y1)*(z3 - z1) - (z2 - z1)*(y3 - y1);
		float j = (z2 - z1)*(x3 - x1) - (x2 - x1)*(z3 - z1);
		float k = (x2 - x1)*(y3 - y1) - (y2 - y1)*(x3 - x1);



		float a = i;
		float b = j;
		float c = k;
		float d = -1 * (i*x1 + j*y1 + k*z1);

		for (int ind = 0; ind < cloud->points.size(); ind++) {
			if (inlines.count(ind) > 0) {
				continue;
			}

			auto point = cloud->points[ind];
			float x4 = point.x;
			float y4 = point.y;
			float z4 = point.z;
			float dist = fabs(a*x4 + b*y4 + c*z4 + d)/sqrt(a*a + b*b + c*c);

			if (dist <= distanceThreshold) {
				inlines.insert(ind);
			}
           

		}

		if (inlines.size() > inliersResult.size()) {
			inliersResult = inlines;
		}

        



	}
    return inliersResult;
    // for (auto local_it = inliersResult.begin(); local_it != inliersResult.end(); ++local_it) {
    //         inliners->indices.push_back(*local_it);
    //     }
}

template<typename PointT>
std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> ProcessPointClouds<PointT>::SegmentPlane(typename pcl::PointCloud<PointT>::Ptr cloud, int maxIterations, float distanceThreshold)
{
    // Time segmentation process
    auto startTime = std::chrono::steady_clock::now();
	//pcl::PointIndices::Ptr inliners (new pcl::PointIndices);
   
/*    pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients());
    // TODO:: Fill in this function to find inliers for the cloud.
    pcl::SACSegmentation<PointT> seg;
    seg.setModelType(pcl::SACMODEL_PLANE);
    seg.setMethodType(pcl::SAC_RANSAC);
    seg.setMaxIterations(maxIterations);
    seg.setDistanceThreshold(distanceThreshold);


    seg.setInputCloud(cloud);
    seg.segment(*inliners, *coefficients);
*/
    std::unordered_set<int> inliers = segmentUsingRansac(cloud, maxIterations, distanceThreshold);

    // if (inliers->indices.size() == 3 ) {
    //     std::cout << "could not estimate a planar model" << std::endl;
    // }

    auto endTime = std::chrono::steady_clock::now();
    auto elapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime);
    std::cout << "plane segmentation took " << elapsedTime.count() << " milliseconds" << std::endl;

    std::cout << "Total points: " << cloud->points.size() << " inliers has " << inliers.size() << std::endl;

    typename pcl::PointCloud<PointT>::Ptr  cloudInliers(new pcl::PointCloud<PointT>());
	typename pcl::PointCloud<PointT>::Ptr cloudOutliers(new pcl::PointCloud<PointT>());

	for(int index = 0; index < cloud->points.size(); index++)
	{
		auto point = cloud->points[index];
		if(inliers.count(index))
			cloudInliers->points.push_back(point);
		else
			cloudOutliers->points.push_back(point);
	}

    //std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> segResult = SeparateClouds(inliners,cloud);
    std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> segResult(cloudInliers, cloudOutliers);
    return segResult;
}
template<typename PointT>
std::vector<std::vector<float>> ProcessPointClouds<PointT>::getPointsV(typename pcl::PointCloud<PointT>::Ptr cloud) {
    std::vector<std::vector<float>> pointsV;
    for (auto point : cloud->points) {
        std::vector<float> pointV;
        pointV.push_back(point.x);
        pointV.push_back(point.y);
        pointV.push_back(point.z);
        pointsV.push_back(pointV);
    }
    return pointsV;
}

template<typename PointT>
void ProcessPointClouds<PointT>::proximityImpl(const std::vector<std::vector<float>>& points, int id, std::vector<int>& cluster, KdTree *tree, std::map<int, bool>& pointsProcessed, float distanceTol) {
	pointsProcessed[id] = true;
	cluster.push_back(id);
	std::vector<int> nearby_points = tree->search(points[id], distanceTol);
	for (auto nid : nearby_points) {
		if (pointsProcessed.find(nid) == pointsProcessed.end() ) {
			proximityImpl(points, nid, cluster, tree, pointsProcessed, distanceTol );
		}
	}
}

template<typename PointT>
std::vector<std::vector<int>> ProcessPointClouds<PointT>::euclideanClusterImpl(const std::vector<std::vector<float>>& points, KdTree* tree, float distanceTol)
{

	std::vector<std::vector<int>> clusters;
	std::map<int, bool> pointsProcessed;

	for (int id = 0; id < points.size(); id++) {
		// Not found in the map
		if (pointsProcessed.find(id) == pointsProcessed.end()) {
			std::vector<int> cluster;
			proximityImpl(points, id, cluster, tree, pointsProcessed, distanceTol);
			clusters.push_back(cluster);

		}
	}
 
	return clusters;

}

template<typename PointT>
std::vector<typename pcl::PointCloud<PointT>::Ptr> ProcessPointClouds<PointT>::ClusteringCustomImpl(typename pcl::PointCloud<PointT>::Ptr cloud, float clusterTolerance, int minSize, int maxSize)
{
     // Time clustering process
    auto startTime = std::chrono::steady_clock::now();

    std::vector<typename pcl::PointCloud<PointT>::Ptr> clusters;

    KdTree* tree = new KdTree;
    auto pointsV = getPointsV(cloud);
  
    for (int i=0; i<pointsV.size(); i++) {
    	tree->insert(pointsV[i],i); 
    }

    // Time segmentation process
  	//
  	std::vector<std::vector<int>> cluster_indices = euclideanClusterImpl(pointsV, tree, clusterTolerance);
  	//
  	auto endTime = std::chrono::steady_clock::now();
  	auto elapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime);
  	std::cout << "clustering found " << cluster_indices.size() << " and took " << elapsedTime.count() << " milliseconds" << std::endl;

    for (auto it = cluster_indices.begin(); it != cluster_indices.end(); ++it) {
        if ((*it).size() < minSize || (*it).size() > maxSize) {
            continue;
        }
        typename pcl::PointCloud<PointT>::Ptr cloud_cluster (new typename pcl::PointCloud<PointT>);
        for (auto pit = (*it).begin(); pit != (*it).end(); ++pit) {
            cloud_cluster->points.push_back (cloud->points[*pit]);
            cloud_cluster->width = cloud_cluster->points.size();
            cloud_cluster->height = 1;
            cloud_cluster->is_dense = true;
        }
        clusters.push_back(cloud_cluster);
    }
    return clusters;


}

template<typename PointT>
std::vector<typename pcl::PointCloud<PointT>::Ptr> ProcessPointClouds<PointT>::Clustering(typename pcl::PointCloud<PointT>::Ptr cloud, float clusterTolerance, int minSize, int maxSize)
{

    // Time clustering process
    auto startTime = std::chrono::steady_clock::now();

    std::vector<typename pcl::PointCloud<PointT>::Ptr> clusters;

    // TODO:: Fill in the function to perform euclidean clustering to group detected obstacles
    typename pcl::search::KdTree<PointT>::Ptr tree (new typename pcl::search::KdTree<PointT>);
    tree->setInputCloud(cloud);

    std::vector<pcl::PointIndices> cluster_indices;
    typename pcl::EuclideanClusterExtraction<PointT> ec;
    ec.setClusterTolerance(clusterTolerance);
    ec.setMinClusterSize(minSize);
    ec.setMaxClusterSize(maxSize);
    ec.setSearchMethod(tree);
    ec.setInputCloud(cloud);
    ec.extract(cluster_indices);

    for (auto it = cluster_indices.begin(); it != cluster_indices.end(); ++it) {
        typename pcl::PointCloud<PointT>::Ptr cloud_cluster (new typename pcl::PointCloud<PointT>);
        for (auto pit = it->indices.begin(); pit != it->indices.end(); ++pit) {
            cloud_cluster->points.push_back (cloud->points[*pit]);
            cloud_cluster->width = cloud_cluster->points.size();
            cloud_cluster->height = 1;
            cloud_cluster->is_dense = true;
        }
        clusters.push_back(cloud_cluster);
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