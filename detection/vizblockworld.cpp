#include "vizblockworld.h"
#include <pcl/filters/voxel_grid.h>
#include <pcl/common/transforms.h>
#include <boost/algorithm/minmax_element.hpp>
#include <algorithm>
#include <boost/algorithm/string.hpp>

// Eigen::Matrix4f t;
//t<<1, 2, 3, 4,
//   5, 6, 7, 8,
//   5, 6, 7, 8,
//   5, 6, 7, 8,

void transform_pointcloud(const pcl::PointCloud<PointT> &cloud_in,
        pcl::PointCloud<PointT> &cloud_out,
        const Eigen::Matrix4f &transform){
    pcl::transformPointCloud(cloud_in, cloud_out, transform);
}

void save_cloud(const std::string &filename, PointCloudPtr cloud){
    if (pcl::io::savePCDFile(filename, *cloud, true) == 0) {
        cout << "Saved " << filename << "." << endl;
    } else PCL_ERROR("Problem saving %s.\n", filename.c_str());
}
void read_cloud(const std::string &filename, PointCloudPtr cloud){
    if (pcl::io::loadPCDFile(filename, *cloud) == 0) {
        cout << "read: " << filename << "." << endl;
    } else PCL_ERROR("Problem saving %s.\n", filename.c_str());
}

void read_off(const std::string &filename, PointCloudPtr cloud){
    std::cout<<filename<<std::endl;
    std::ifstream in(filename);
    std::string line;
    int num = 0, count = 0;
    while(std::getline(in, line)){
        num++;
        vector<string> xyz;
        boost::trim_right_if(line, boost::is_any_of(" \n"));
        boost::split(xyz, line, boost::is_space());
        if(xyz.size() == 3){
            if(num == 2)
                // ignore the second line
                continue;
            PointT point;
            point.x = boost::lexical_cast<float>(xyz[0]);
            point.y = boost::lexical_cast<float>(xyz[1]);
            point.z = boost::lexical_cast<float>(xyz[2]);
            cloud->points.push_back(point);
            count++;
        }else if(xyz.size() == 4)
            // ignore mesh information
            break;
    };
    std::cout<<"loaded "<<count<<"points!"<<std::endl;
}

void camera_info(pcl::visualization::Camera &camera)
{

    std::cout<<"focal: "<<camera.focal[0]<<" "<<camera.focal[1]<<" "<<camera.focal[2]<<std::endl;
    std::cout<<"pos: "<<camera.pos[0]<<" "<<camera.pos[1]<<" "<<camera.pos[2]<<std::endl;
    std::cout<<"view: "<<camera.view[0]<<" "<<camera.view[1]<<" "<<camera.view[2]<<std::endl;
    std::cout<<"window size: "<<camera.window_size[0]<<" "<<camera.window_size[1]<<std::endl;
    std::cout<<"window pos: "<<camera.window_pos[0]<<" "<<camera.window_pos[1]<<std::endl;
}
void VizBlockWorld::generte_mesh(const pcl::PointCloud<pcl::PointXYZ>::Ptr cloud,
                                 pcl::PolygonMesh *triangles, float radius)
{
    // Normal estimation*
    pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> n;
    pcl::PointCloud<pcl::Normal>::Ptr normals (new pcl::PointCloud<pcl::Normal>);
    pcl::search::KdTree<pcl::PointXYZ>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZ>);
    tree->setInputCloud (cloud);
    n.setInputCloud (cloud);
    n.setSearchMethod (tree);
    n.setKSearch (20);
    n.compute (*normals);
    //* normals should not contain the point normals + surface curvatures

    // Concatenate the XYZRGB and normal fields*
    pcl::PointCloud<pcl::PointNormal>::Ptr cloud_with_normals (new pcl::PointCloud<pcl::PointNormal>);
    pcl::concatenateFields (*cloud, *normals, *cloud_with_normals);
    //* cloud_with_normals = cloud + normals

    // Create search tree*
    pcl::search::KdTree<pcl::PointNormal>::Ptr tree2 (new pcl::search::KdTree<pcl::PointNormal>);
    tree2->setInputCloud (cloud_with_normals);

    // Initialize objects
    pcl::GreedyProjectionTriangulation<pcl::PointNormal> gp3;

    // Set the maximum distance between connected points (maximum edge length)
    gp3.setSearchRadius (radius);

    // Set typical values for the parameters
    gp3.setMu (3.0);
    gp3.setMaximumNearestNeighbors (100);
    gp3.setMaximumSurfaceAngle(M_PI/4); // 45 degrees
    gp3.setMinimumAngle(M_PI/18); // 10 degrees
    gp3.setMaximumAngle(2*M_PI/3); // 120 degrees
    gp3.setNormalConsistency(false);

    // Get result
    gp3.setInputCloud (cloud_with_normals);
    gp3.setSearchMethod (tree2);
    gp3.reconstruct (*triangles);

    // Additional vertex information
    std::vector<int> parts = gp3.getPartIDs();
    std::vector<int> states = gp3.getPointStates();
    std::cout<<"size of parts:"<<std::endl
        <<parts.size()<<std::endl;

    std::cout<<"size of states:"<<std::endl
        <<states.size()<<std::endl;

    // output mesh
//    std::ofstream out("origina.off");
//    out<<"OFF"<<std::endl;
//    out<<cloud->points.size()<<" "<<triangles->polygons.size()<<" "<<0<<std::endl;
//    for(auto &p : cloud->points){
//        out<<p.x<<" "<<p.y<<" "<<p.z<<std::endl;
//    }
//    for(pcl::Vertices &mesh : triangles->polygons){
//        out<<3<<" "<<mesh.vertices[0]<<" "<<mesh.vertices[1]<<" "<<mesh.vertices[2]<<std::endl;
//    };

}

void VizBlockWorld::display(std::vector<pcl::visualization::PCLVisualizer*> viewers){
    while (!viewers[0]->wasStopped ())
    {
        viewers[0]->spinOnce (100);
        boost::this_thread::sleep (boost::posix_time::microseconds (100000));
    }

}
void VizBlockWorld::display(){
    while (!_viewer->wasStopped ())
    {
        _viewer->spinOnce (100);
        boost::this_thread::sleep (boost::posix_time::microseconds (100000));
    }

}

#include <pcl/ModelCoefficients.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/features/normal_3d.h>
#include <pcl/kdtree/kdtree.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/segmentation/extract_clusters.h>
#include <cassert>
typedef pcl::PointXYZRGB PointT;
void segment_plane (const pcl::PointCloud<PointT>::Ptr cloud,
        pcl::PointCloud<PointT>::Ptr objects, CloudVector* planes){

    // Create the segmentation object for the planar model and set all the parameters
    pcl::SACSegmentation<PointT> seg;
    pcl::PointIndices::Ptr inliers (new pcl::PointIndices);
    pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients);
    pcl::PointCloud<PointT>::Ptr cloud_plane (new pcl::PointCloud<PointT> ());
    seg.setOptimizeCoefficients (true);
    seg.setModelType (pcl::SACMODEL_PLANE);
    seg.setMethodType (pcl::SAC_RANSAC);
    seg.setMaxIterations (100);
    seg.setDistanceThreshold (0.01);

    int i=0;
    //    while (cloud_filtered->points.size () > 0.3 * nr_points)
    //    {
    // Segment the largest planar component from the remaining cloud
    seg.setInputCloud (cloud);
    seg.segment (*inliers, *coefficients);
    if (inliers->indices.size () == 0)
    {
        std::cout << "Could not estimate a planar model for the given dataset." << std::endl;
        return;
    }

    // Extract the planar inliers from the input cloud
    pcl::ExtractIndices<PointT> extract;
    extract.setInputCloud (cloud);
    extract.setIndices (inliers);
    extract.setNegative (false);

    // Write the planar inliers to disk
    extract.filter (*cloud_plane);
    planes->push_back(cloud_plane);
    std::cout << "PointCloud representing the planar component: " << cloud_plane->points.size () << " data points." << std::endl;
    planes->push_back(cloud_plane);

    // Remove the planar inliers, extract the rest
    extract.setNegative (true);
    extract.filter (*objects);

}
void cluster_points(const pcl::PointCloud<PointT>::Ptr cloud, CloudVector *clusters){

    // Creating the KdTree object for the search method of the extraction
    pcl::search::KdTree<PointT>::Ptr tree (new pcl::search::KdTree<PointT>);
    tree->setInputCloud (cloud);
    std::vector<pcl::PointIndices> cluster_indices;
    pcl::EuclideanClusterExtraction<PointT> ec;
    ec.setClusterTolerance (0.03); // 2cm
    ec.setMinClusterSize (100);
    ec.setMaxClusterSize (7500009);
    ec.setSearchMethod (tree);
    ec.setInputCloud (cloud);
    ec.extract (cluster_indices);
    // for each cluster
    for (std::vector<pcl::PointIndices>::const_iterator it = cluster_indices.begin ();
            it != cluster_indices.end (); ++it)
    {
        pcl::PointCloud<PointT>::Ptr cloud_cluster (new pcl::PointCloud<PointT>);
        for (std::vector<int>::const_iterator pit = it->indices.begin (); pit != it->indices.end (); pit++){

            // push points
            cloud_cluster->points.push_back (cloud->points[*pit]); //*
            //            std::cout<<(unsigned int)cloud->points[*pit].rgb<<std::endl;
        }
        cloud_cluster->width = cloud_cluster->points.size ();
        cloud_cluster->height = 1;
        cloud_cluster->is_dense = true;
        clusters->push_back(cloud_cluster);
        std::cout << "PointCloud representing the Cluster: " 
            << cloud_cluster->points.size () 
            << " data points." << std::endl;
    }

}
PointCloudPtr down_samples(PointCloudPtr input, float leafSize){
    // Create the filtering object: downsample the dataset using a leaf size of 1cm
    pcl::VoxelGrid<PointT> vg;
    pcl::PointCloud<PointT>::Ptr output(new pcl::PointCloud<PointT>);
    vg.setInputCloud (input);
    vg.setLeafSize (leafSize, leafSize, leafSize);
    vg.filter (*output);
    std::cout << "PointCloud after filtering has: " <<output->points.size ()  << " data points." << std::endl; //*
    return output;
}
