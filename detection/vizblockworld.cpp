#include "vizblockworld.h"
void VizBlockWorld::generte_mesh(const pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, pcl::PolygonMesh *triangles){
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
    gp3.setSearchRadius (0.025);

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
    std::ofstream out("original.off");
    out<<"OFF"<<std::endl;
    out<<cloud->points.size()<<" "<<triangles->polygons.size()<<" "<<0<<std::endl;
    for(auto &p : cloud->points){
        out<<p.x<<" "<<p.y<<" "<<p.z<<std::endl;
    }
    for(pcl::Vertices &mesh : triangles->polygons){
        out<<3<<" "<<mesh.vertices[0]<<" "<<mesh.vertices[1]<<" "<<mesh.vertices[2]<<std::endl;
    };

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
