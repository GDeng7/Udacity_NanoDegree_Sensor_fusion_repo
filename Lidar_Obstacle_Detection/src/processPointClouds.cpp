// PCL lib Functions for processing point clouds 

#include "processPointClouds.h"


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
    typename pcl::PointCloud<PointT>::Ptr cloud_filtered(new pcl::PointCloud<PointT>);
    typename pcl::PointCloud<PointT>::Ptr cloud_region(new pcl::PointCloud<PointT>);

    //Voxel grid point reduction
    pcl::VoxelGrid<PointT> sor;
    sor.setInputCloud (cloud);
    sor.setLeafSize(filterRes, filterRes, filterRes);
    sor.filter(*cloud_filtered);

    //region based filtering
    pcl::CropBox<PointT> region(true);
    region.setMin(minPoint);
    region.setMax(maxPoint);
    region.setInputCloud(cloud_filtered);
    region.filter(*cloud_region);

    //remove roof points
    std::vector<int> indices;
    pcl::CropBox<PointT> roof(true);
    roof.setMin(Eigen::Vector4f (-1.5, -1.7, -1, 1));
    roof.setMax(Eigen::Vector4f (2.6, 1.7, -0.4, 1));
    roof.setInputCloud(cloud_region);
    roof.filter(indices);

    pcl::PointIndices::Ptr inliers{new pcl::PointIndices()};
    for(int i: indices){
        inliers->indices.push_back(i);
    }

    pcl::ExtractIndices<PointT> extract;
    extract.setInputCloud (cloud_region);
    extract.setIndices (inliers);
    extract.setNegative(true);
    extract.filter(*cloud_region);


    auto endTime = std::chrono::steady_clock::now();
    auto elapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime);
    std::cout << "filtering took " << elapsedTime.count() << " milliseconds" << std::endl;

    return cloud_region;
    //return cloud;

}


template<typename PointT>
std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> ProcessPointClouds<PointT>::SeparateClouds(pcl::PointIndices::Ptr inliers, typename pcl::PointCloud<PointT>::Ptr cloud) 
{
  // TODO: Create two new point clouds, one cloud with obstacles and other with segmented plane
    typename pcl::PointCloud<PointT>::Ptr obstCloud (new pcl::PointCloud<PointT> ());
    typename pcl::PointCloud<PointT>::Ptr planeCloud (new pcl::PointCloud<PointT> ());

    for(int index : inliers -> indices)
        planeCloud->points.push_back(cloud->points[index]);

    pcl::ExtractIndices<PointT> extract;
    extract.setInputCloud (cloud);
    extract.setIndices (inliers);
    extract.setNegative(true);
    extract.filter (*obstCloud);

    std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> segResult(obstCloud, planeCloud);

    //std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> segResult(cloud, cloud);
    return segResult;
}


template<typename PointT>
std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> ProcessPointClouds<PointT>::SegmentPlane(typename pcl::PointCloud<PointT>::Ptr cloud, int maxIterations, float distanceThreshold)
{
    // Time segmentation process
    auto startTime = std::chrono::steady_clock::now();
	//pcl::PointIndices::Ptr inliers;
    // TODO:: Fill in this function to find inliers for the cloud.
    pcl::SACSegmentation<PointT> seg;
    pcl::PointIndices::Ptr inliers {new pcl::PointIndices};
    pcl::ModelCoefficients::Ptr coefficients {new pcl::ModelCoefficients};

    seg.setOptimizeCoefficients(true);
    seg.setModelType(pcl::SACMODEL_PLANE);
    seg.setMethodType(pcl::SAC_RANSAC);
    seg.setMaxIterations(maxIterations);
    seg.setDistanceThreshold(distanceThreshold);

    // Segment the largest planar component from the input cloud
    seg.setInputCloud(cloud);
    seg.segment(*inliers, *coefficients);
    if(inliers->indices.size() == 0){
        std::cout<<"Could not estimate a planar model for the given dataset"<< std::endl;
    }


    auto endTime = std::chrono::steady_clock::now();
    auto elapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime);
    std::cout << "plane segmentation took " << elapsedTime.count() << " milliseconds" << std::endl;

    std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> segResult = SeparateClouds(inliers,cloud);
    return segResult;
}


template<typename PointT>
std::vector<typename pcl::PointCloud<PointT>::Ptr> ProcessPointClouds<PointT>::Clustering(typename pcl::PointCloud<PointT>::Ptr cloud, float clusterTolerance, int minSize, int maxSize)
{

    // Time clustering process
    auto startTime = std::chrono::steady_clock::now();

    std::vector<typename pcl::PointCloud<PointT>::Ptr> clusters;

    // TODO:: Fill in the function to perform euclidean clustering to group detected obstacles
    
    //create kd tree object
    typename pcl::search::KdTree<PointT>::Ptr tree(new pcl::search::KdTree<PointT>);
    tree->setInputCloud(cloud);

    std::vector<pcl::PointIndices> clusterIndices;
    pcl::EuclideanClusterExtraction<PointT> ec;
    ec.setClusterTolerance(clusterTolerance);
    ec.setMinClusterSize(minSize);
    ec.setMaxClusterSize(maxSize);
    ec.setSearchMethod(tree);
    ec.setInputCloud(cloud);
    ec.extract(clusterIndices);

    for(pcl::PointIndices getIndices: clusterIndices){
        typename pcl::PointCloud<PointT>::Ptr cloudCluster (new pcl::PointCloud<PointT>);

        for(int index : getIndices.indices){
            cloudCluster->points.push_back(cloud->points[index]);
        }

        cloudCluster->width = cloudCluster->points.size();
        cloudCluster->height = 1;
        cloudCluster->is_dense = true;

        clusters.push_back(cloudCluster);
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

// my algorithm implementation
template <typename PointT>
std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> ProcessPointClouds<PointT>::RansacPlane(typename pcl::PointCloud<PointT>::Ptr cloud, int maxIterations, float distanceTol){
  std::unordered_set<int> inliersResult;
  srand(time(NULL));

  auto startTime = std::chrono::steady_clock::now();

    for (int m=0 ; m < maxIterations; m++) {
        // create set
        std::unordered_set<int> inliers;

        // take three random indicies this time
        inliers.insert(rand()%cloud->points.size());
        inliers.insert(rand()%cloud->points.size());
        inliers.insert(rand()%cloud->points.size());

        float x1,y1,z1,x2,y2,z2,x3,y3,z3;

        auto itr = inliers.begin();
        // get the x and y of those indices
        x1 = cloud->points[*itr].x;
        y1 = cloud->points[*itr].y;
        z1 = cloud->points[*itr].z;
        ++itr; // move to the next index of the map
        x2 = cloud->points[*itr].x;
        y2 = cloud->points[*itr].y;
        z2 = cloud->points[*itr].z;
        ++itr;
        x3 = cloud->points[*itr].x;
        y3 = cloud->points[*itr].y;
        z3 = cloud->points[*itr].z;

        float i = (y2 - y1)*(z3-z1) - (z2-z1)*(y3-y1);
        float j = (z2-z1)*(x3-x1) - (x2-x1)*(z3-z1);
        float k = (x2-x1)*(y3-y1) - (y2-y1)*(x3-x1);

        for(int index = 0; index< cloud->points.size(); index++){

          if(inliers.count(index)>0){
            continue;
          }

          float a = i;
          float b = j;
          float c = k;
          float d = -(i*x1 + j*y1 + k*z1);

          PointT point = cloud->points[index];
          float x4 = point.x;
          float y4 = point.y;
          float z4 = point.z;

          // calculate distance
          float dist = std::fabs(a*x4+b*y4+c*z4+d)/std::sqrt(a*a+b*b+c*c);

          if(dist <= distanceTol){
            inliers.insert(index);
          }
        }

        if(inliers.size()>inliersResult.size()){
          inliersResult = inliers;
        }
    }

    typename pcl::PointCloud<PointT>::Ptr  cloudInliers(new pcl::PointCloud<PointT>());
    typename pcl::PointCloud<PointT>::Ptr cloudOutliers(new pcl::PointCloud<PointT>());

    if(!inliersResult.empty()){
        for(int index = 0; index < cloud->points.size(); index++)
        {
            const PointT point = cloud->points[index];
            if(inliersResult.count(index))
                cloudInliers->points.push_back(point);
            else
                cloudOutliers->points.push_back(point);
        }
    }

  
    auto endTime = std::chrono::steady_clock::now();
    auto elapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime);
    std::cout << "Ransac took " << elapsedTime.count() << " milliseconds" << std::endl;
      
    return std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr>(cloudOutliers, cloudInliers);
}

template <typename PointT>
void ProcessPointClouds<PointT>::Cluster_helper(const std::vector<std::vector<float>>& points, KdTree* tree, float distanceTol, int curr_idx, std::vector<bool>& processed, std::vector<int>& cluster){

  
  //mark as processed
  processed[curr_idx] = true;
  cluster.push_back(curr_idx);

  //search
  std::vector<int> nearby = tree->search(points[curr_idx],distanceTol);

  // iterate other vector
  for(int idx: nearby){
    if(!processed[idx]){
      Cluster_helper(points,tree,distanceTol, idx, processed, cluster);
    }
  }
  
  /*
    processed.at(curr_idx) = true;
    cluster.push_back(curr_idx);

    const std::vector<int> nearbyPoints{ tree->search(points.at(curr_idx), distanceTol) };

    for (int nearbyIndex : nearbyPoints) {
        if (!processed.at(nearbyIndex)) {
            Cluster_helper(points,tree,distanceTol, nearbyIndex, processed, cluster);
        }
    }
    */
}

template <typename PointT>
std::vector<std::vector<int>> ProcessPointClouds<PointT>::Euclidean_cluster(const std::vector<std::vector<float>>& points, KdTree* tree, float distanceTol)
{
    
    // TODO: Fill out this function to return list of indices for each cluster

    std::vector<std::vector<int>> clusters;
  
  std::vector<bool> processed(points.size(),false);

   std::cout << "cluster function point size: "<< points.size() <<endl;

  for (int i = 0; i<points.size();i++){
    if (!processed[i]){
        //main cluster
    std::vector<int> cluster;
    //std::cout << "Current index: "<< i <<endl;
    Cluster_helper(points,tree, distanceTol, i, processed, cluster);
    //push
    clusters.push_back(cluster);
    }
  }
 
    return clusters;
    /*

    std::vector<std::vector<int>> clusters;
    std::vector<bool> processed(points.size(), false);

    std::cout << "cluster function point size: "<< points.size() <<endl;

    for (int i{ 0 }; i < points.size(); i++) {
        if (!processed.at(i)) {
            std::vector<int> cluster;
            //std::cout << "Current index: "<< i <<endl;
            Cluster_helper(points,tree,distanceTol, i, processed, cluster);
            clusters.push_back(cluster);
        }
    }

    return clusters;
    */
}

template<typename PointT>
std::vector<typename pcl::PointCloud<PointT>::Ptr> ProcessPointClouds<PointT>::Clustering_UD(typename pcl::PointCloud<PointT>::Ptr cloud, float clusterTolerance, int minSize, int maxSize)
{
  
    // Time clustering process
    auto startTime = std::chrono::steady_clock::now();

    std::vector<typename pcl::PointCloud<PointT>::Ptr> clusters;
    
    //create kd tree object
    KdTree* tree = new KdTree;
    std::vector<std::vector<float>> points;


    //for (auto i: cloud->points){
    for (int i=0; i<cloud->points.size(); i++){

        PointT point = cloud->points[i];

        std::vector<float> point_vector;
        point_vector.push_back(point.x);
        point_vector.push_back(point.y);
        point_vector.push_back(point.z);

        tree->insert(point_vector,i);
        points.push_back(point_vector);
    } 

    std::cout << "point size: "<< points.size() <<endl;

    std::cout << "Before cluster called " <<endl;
    std::vector<std::vector<int>> clusterIndices = Euclidean_cluster(points, tree, clusterTolerance);
    std::cout << "After cluster called " <<endl;

    for(std::vector<int> getIndices: clusterIndices){
        //if(getIndices.size() < minSize || getIndices.size() > maxSize){
         //   continue;
        //}

        typename pcl::PointCloud<PointT>::Ptr cloudCluster (new pcl::PointCloud<PointT>);

        for(int index : getIndices){
            cloudCluster->points.push_back(cloud->points[index]);
        }

        cloudCluster->width = cloudCluster->points.size();
        cloudCluster->height = 1;
        cloudCluster->is_dense = true;

        clusters.push_back(cloudCluster);
    }
    
    auto endTime = std::chrono::steady_clock::now();
    auto elapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime);
    std::cout << "clustering took " << elapsedTime.count() << " milliseconds and found " << clusters.size() << " clusters" << std::endl;

    return clusters;
    

    /*
    const auto startTime{ std::chrono::steady_clock::now() };

    int i{ 0 };
    KdTree *kdTree{ new KdTree() };

    std::vector<std::vector<float>> points;

    for (auto point : cloud->points) {
        const std::vector<float> p{ point.x, point.y, point.z };
        kdTree->insert(p, i++);
        points.push_back(p);
    }

    const std::vector<std::vector<int>> listOfIndices{ Euclidean_cluster(points, kdTree, clusterTolerance) };

    std::vector<typename pcl::PointCloud<PointT>::Ptr> clusters;

    for (auto indices : listOfIndices) {

        if (indices.size() < minSize || indices.size() > maxSize) { continue; }

        typename pcl::PointCloud<PointT>::Ptr cluster{ new pcl::PointCloud<PointT> };

        for (auto index : indices) { cluster->points.push_back(cloud->points[index]); }

        cluster->width = cluster->points.size();
        cluster->height = 1;
        cluster->is_dense = true;

        clusters.push_back(cluster);
    }

    const auto endTime{ std::chrono::steady_clock::now() };
    const auto elapsedTime{ std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime) };

    std::cout << "clustering took " << elapsedTime.count();
    std::cout << " milliseconds and found " << clusters.size() << " clusters" << std::endl;

    return clusters;*/
}
