#include <pcl/point_cloud.h>
#include <pcl/kdtree/kdtree_flann.h>

#include <iostream>
#include <vector>
#include <ctime>


#include <stdio.h>

template<typename T>
void nearest_k_search(pcl::KdTreeFLANN<T> &kdtree, T &searchPoint,
                        pcl::PointCloud<T> cloud, int K = 10){

    // K nearest neighbor search 

    std::vector<int> pointIdxNKNSearch(K);
    std::vector<float> pointNKNSquaredDistance(K);
    if ( kdtree.nearestKSearch (searchPoint, K, pointIdxNKNSearch, pointNKNSquaredDistance) > 0 )
    {
        for (size_t i = 0; i < pointIdxNKNSearch.size (); ++i)
            std::cout << "    "  <<   cloud.points[ pointIdxNKNSearch[i] ].x 
                << " " << cloud.points[ pointIdxNKNSearch[i] ].y 
                << " " << cloud.points[ pointIdxNKNSearch[i] ].z 
                << " (squared distance: " << pointNKNSquaredDistance[i] << ")" << std::endl;
    }

}

template<typename T>
void radius_search(pcl::KdTreeFLANN<T> &kdtree, T &searchPoint,
                        pcl::PointCloud<T> cloud, float radius = 10){
    // Neighbors within radius search

    std::vector<int> pointIdxRadiusSearch;
    std::vector<float> pointRadiusSquaredDistance;


    std::cout << "Neighbors within radius search at (" << searchPoint.x 
        << " " << searchPoint.y 
        << " " << searchPoint.z
        << ") with radius=" << radius << std::endl;


    if ( kdtree.radiusSearch (searchPoint, radius, pointIdxRadiusSearch, pointRadiusSquaredDistance) > 0 )
    {
        for (size_t i = 0; i < pointIdxRadiusSearch.size (); ++i)
            std::cout << "    "  <<   cloud->points[ pointIdxRadiusSearch[i] ].x 
                << " " << cloud->points[ pointIdxRadiusSearch[i] ].y 
                << " " << cloud->points[ pointIdxRadiusSearch[i] ].z 
                << " (squared distance: " << pointRadiusSquaredDistance[i] << ")" << std::endl;
    }

}
