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

inline PointT create_point(float x, float y, float z, int r, int g, int b){
    PointT point;
    //point.x = x * step;
    point.x = x ;
    point.y = y ;
    point.z = z ;

    uint8_t _r(r), _g(g), _b(b);
    uint32_t rgb = (static_cast<uint32_t>(_r) << 16 |
            static_cast<uint32_t>(_g) << 8 | static_cast<uint32_t>(_b));
    point.rgb = *reinterpret_cast<float*>(&rgb);
    return point;
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
void grid_color(VizBlockWorld *viz){
    PointCloudPtr temp(new VizBlockWorld::PointCloud);
    viz->set_def_cloud(temp);
    vector<vector<float>> rgbMatrix;
    vector<vector<float>> emdMatrix;
    vector<float> human;
    human.push_back(4 + 0.2);
    human.push_back(3 + 0.7);
    human.push_back(3 + 0.2);
    human.push_back(1 + 0.6);
    human.push_back(2 + 0.5);
    rgbMatrix.push_back(human);
    vector<float> lion(1);
    lion.push_back(4 + 0.4);
    lion.push_back(4 + 0.1);
    lion.push_back(1 + 0.1);
    lion.push_back(3 + 0.2);
    rgbMatrix.push_back(lion);
    vector<float> tiger(2);
    tiger.push_back(4 +  0.3);
    tiger.push_back(1 + 0.3);
    tiger.push_back(3 + 0.4);
    rgbMatrix.push_back(tiger);
    vector<float> bottle(3);
    bottle.push_back(4 + 0.2);
    bottle.push_back(1 + 0.3);
    rgbMatrix.push_back(bottle);
    vector<float> deer(4);
    deer.push_back(4 + 0.4);
    rgbMatrix.push_back(deer);
    //
    std::vector<float> weights;
    for(auto &row : rgbMatrix)
        for(auto v : row)
            weights.push_back(v);
    // @@snipet gradient display
    auto wMinMax = boost::minmax_element(weights.begin(), weights.end());
    int unitGray = 250 / ( *wMinMax.second- *wMinMax.first);
    int size = rgbMatrix.size();
    for (int row = 0; row < size; row++) 
        for (int col = 0; col < size; col++) 
            if (col >= row) {
                int grayColor = 255 - (rgbMatrix[row][col] - *wMinMax.first) * unitGray;
                std::cout<<grayColor<<std::endl;
                viz->add_point(col, size - row, 0, grayColor, grayColor, grayColor);
//                viz->add_text3D(m_util::sth2string<float>(rgbMatrix[row][col]),
//                        col-0.25, size - row, 1, 255-grayColor, 255-grayColor, 255-grayColor, 0.2);
            }
    viz->push_def_cloud(0,80);
    viz->set_backgroundcolor(255, 255, 255);

}

void grid_color2(VizBlockWorld *viz){
    PointCloudPtr temp(new VizBlockWorld::PointCloud);
    viz->set_def_cloud(temp);
    vector<vector<float>> rgbMatrix;
    vector<vector<float>> emdMatrix;
    vector<float> human0;
    human0.push_back(4 + 0.2);
    human0.push_back(3 + 0.5);
    human0.push_back(2 + 0.5);
    human0.push_back(2 + 0.2);
    human0.push_back(1 + 0.3);
    human0.push_back(1 + 0.7);
    rgbMatrix.push_back(human0);
    human0.clear();
    human0.push_back(0);
    human0.push_back(16.1);
    human0.push_back(25.3);
    human0.push_back(28.4);
    human0.push_back(37.1);
    human0.push_back(33.7);
    emdMatrix.push_back(human0);

    vector<float> human1(1);
    human1.push_back(4 + 0.7);
    human1.push_back(3 + 0.2);
    human1.push_back(2 + 0.3);
    human1.push_back(1 + 0.8);
    human1.push_back(2 + 0.1);
    rgbMatrix.push_back(human1);
    human1.clear();
    human1.push_back(0);
    human1.push_back(0);
    human1.push_back(18.4);
    human1.push_back(27.5);
    human1.push_back(32.8);
    human1.push_back(29.3);
    emdMatrix.push_back(human1);

    vector<float> human2(2);
    human2.push_back(4 + 0.2);
    human2.push_back(1 + 0.9);
    human2.push_back(2 + 0.2);
    human2.push_back(1 + 0.7);
    rgbMatrix.push_back(human2);
    human2.clear();
    human2.push_back(0);
    human2.push_back(0);
    human2.push_back(0);
    human2.push_back(31.1);
    human2.push_back(28.5);
    human2.push_back(33.4);
    emdMatrix.push_back(human2);

    vector<float> lion0(3);
    lion0.push_back(4 + 0.2);
    lion0.push_back(3 + 0.5);
    lion0.push_back(2 + 0.3);
    rgbMatrix.push_back(lion0);
    lion0.clear();
    lion0.push_back(0);
    lion0.push_back(0);
    lion0.push_back(0);
    lion0.push_back(0);
    lion0.push_back(15.7);
    lion0.push_back(27.3);
    emdMatrix.push_back(lion0);

    vector<float> lion1(4);
    lion1.push_back(4 + 0.2);
    lion1.push_back(2 + 0.8);
    rgbMatrix.push_back(lion1);
    lion1.clear();
    lion1.push_back(0);
    lion1.push_back(0);
    lion1.push_back(0);
    lion1.push_back(0);
    lion1.push_back(0);
    lion1.push_back(22.6);
    emdMatrix.push_back(lion1);

    vector<float> lion2(5);
    lion2.push_back(4 + 0.2);
    rgbMatrix.push_back(lion2);
    lion2.clear();
    lion2.push_back(0);
    lion2.push_back(0);
    lion2.push_back(0);
    lion2.push_back(0);
    lion2.push_back(0);
    lion2.push_back(0);
    emdMatrix.push_back(lion2);
    //
    std::vector<float> weights;
    for(auto &row : rgbMatrix)
        for(auto v : row)
            weights.push_back(v);
    // @@snipet gradient display
    auto wMinMax = boost::minmax_element(weights.begin(), weights.end());
    int unitGray = 250 / ( *wMinMax.second- *wMinMax.first);
    int size = rgbMatrix.size();
    for (int row = 0; row < size; row++) 
        for (int col = 0; col < size; col++) 
            if (col >= row) {
                int grayColor = 255 - (rgbMatrix[row][col] - *wMinMax.first) * unitGray;
                std::cout<<grayColor<<std::endl;
                viz->add_point(col, size - row, 0, grayColor, grayColor, grayColor);
                viz->add_text3D(m_util::sth2string<float>(emdMatrix[row][col]),
                        col-0.2, size - row -0.1, 1, 255-grayColor, 255-grayColor, 255-grayColor, 0.15);
            }
    viz->push_def_cloud(0,80);
    viz->set_backgroundcolor(255, 255, 255);

}
