// Original code by Geoffrey Biggs, taken from the PCL tutorial in
// http://pointclouds.org/documentation/tutorials/pcl_visualizer.php

// Simple Kinect viewer that also allows to write the current scene to a .pcd
// when pressing SPACE.

#include <iostream>
#include <fstream>
#include <pcl/io/openni_grabber.h>
#include <pcl/io/pcd_io.h>
#include <pcl/filters/filter.h>
#include <pcl/io/ply_io.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/console/parse.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/point_types.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/features/normal_3d.h>
#include <cassert>
#include <pcl/surface/gp3.h>
#include <string>
#include <boost/algorithm/string.hpp>
using namespace std;
using namespace pcl;

PointCloud<PointXYZRGBA>::Ptr cloudptr(new PointCloud<PointXYZRGBA>);
PointCloud<PointXYZ>::Ptr fallbackCloud(new PointCloud<PointXYZ>);
PointCloud<PointXYZRGB>::Ptr rgbPoints(new PointCloud<PointXYZRGB>);
boost::shared_ptr<visualization::CloudViewer> viewer;
pcl::visualization::PCLVisualizer *pcl_viewer;
Grabber* kinectGrabber;
unsigned int filesSaved = 0;
bool saveCloud(false), noColour(false);
void readOFF(std::string filename, pcl::PointCloud<PointXYZ>::Ptr cloud){
    std::ifstream in(filename);
    std::string line;
    int num = 0;
    while(std::getline(in, line)){
        num++;
        vector<string> xyz;
        boost::trim_right_if(line, boost::is_any_of(" \n"));
        boost::split(xyz, line, boost::is_space());
        if(xyz.size() == 3){
            if(num == 2)
                // ignore the second line
                continue;
            PointXYZ point;
            point.x = boost::lexical_cast<float>(xyz[0]);
            point.y = boost::lexical_cast<float>(xyz[1]);
            point.z = boost::lexical_cast<float>(xyz[2]);
            cloud->points.push_back(point);
        }
    };
}
void generte_mesh(const pcl::PointCloud<pcl::PointXYZRGB> &inCloud, pcl::PolygonMesh *triangles){
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);
    pcl::copyPointCloud(inCloud, *cloud);
    // to make sure the index found would be useful
    assert(inCloud.points[1].x == cloud->points[1].x);
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

    int nn = 0;
    size_t temp = triangles->polygons[10000].vertices[0];
    size_t count = 0;
    std::ofstream out("test.off");
    out<<"OFF"<<std::endl;
    out<<inCloud.points.size()<<" "<<triangles->polygons.size()<<" "<<0<<std::endl;
    for(auto &p : inCloud.points){
        out<<p.x<<" "<<p.y<<" "<<p.z<<std::endl;
    }
    for(pcl::Vertices &mesh : triangles->polygons){
        int size = mesh.vertices.size();
        nn+= size;
        if(temp == mesh.vertices[0])
            count++;

        if(temp == mesh.vertices[1])
            count++;

        if(temp == mesh.vertices[2])
            count++;
        out<<3<<" "<<mesh.vertices[0]<<" "<<mesh.vertices[1]<<" "<<mesh.vertices[2]<<std::endl;
    };
    std::cout<<"the number of mesh:"<<endl
        <<triangles->polygons.size()<<std::endl;

    std::cout<<"the number of points:"<<endl
        <<rgbPoints->points.size()<<std::endl;

    std::cout<<"the number of points in all mesh:"<<endl
        <<nn<<endl;
    std::cout<<"dddddd"<<endl
        <<count<<std::endl;

}

    void
printUsage(const char* programName)
{
    cout << "Usage: " << programName << " [options]"
        << endl
        << endl
        << "Options:\n"
        << endl
        << "\t<none>     start capturing from a Kinect device.\n"
        << "\t-v NAME    visualize the given .pcd file.\n"
        << "\t-h         shows this help.\n";
}

    void
grabberCallback(const PointCloud<PointXYZRGBA>::ConstPtr& cloud)
{
    if (! viewer->wasStopped())
        viewer->showCloud(cloud);

    if (saveCloud)
    {
        stringstream stream;
        stream << "inputCloud" << filesSaved << ".pcd";
        string filename = stream.str();

        std::vector<int> tt;
        PointCloud<PointXYZRGBA>::Ptr temp(new PointCloud<PointXYZRGBA>);
        pcl::removeNaNFromPointCloud(*cloud, *temp, tt);
        if (io::savePCDFile(filename, *temp, true) == 0)
        {
            filesSaved++;
            pcl::PLYWriter ply;
            ply.write("test.ply",*temp);
            cout << "Saved " << filename << "." << endl;
        }
        else PCL_ERROR("Problem saving %s.\n", filename.c_str());
        saveCloud = false;
    }
}

    void
keyboardEventOccurred(const visualization::KeyboardEvent& event,
        void* nothing)
{
    if (event.getKeySym() == "space" && event.keyDown())
        saveCloud = true;
}

    boost::shared_ptr<visualization::CloudViewer>
createViewer()
{
    boost::shared_ptr<visualization::CloudViewer> v
        (new visualization::CloudViewer("3D Viewer"));
    v->registerKeyboardCallback(keyboardEventOccurred);

    return(v);
}

    int
main(int argc, char** argv)
{
    if (console::find_argument(argc, argv, "-h") >= 0)
    {
        printUsage(argv[0]);
        return 0;
    }

    bool justVisualize(false);
    string filename;
    if (console::find_argument(argc, argv, "-v") >= 0)
    {
        if (argc < 3)
        {
            printUsage(argv[0]);
            return 0;
        }

        filename = argv[2];
        justVisualize = true;

    }
    else if (argc != 1)
    {
        printUsage(argv[0]);
        return 0;
    }

    if (justVisualize)
    { try
        {
            if (console::find_argument(argc, argv, "-big") >= 0){
                io::loadPCDFile<PointXYZRGB>(filename.c_str(), *rgbPoints);

                pcl_viewer = new pcl::visualization::PCLVisualizer ("3D _viewer");
                pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGB> rgbs(rgbPoints);
                pcl_viewer->addPointCloud<pcl::PointXYZRGB> (rgbPoints, rgbs);
                pcl_viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 10);
                pcl_viewer->initCameraParameters ();
                while (!pcl_viewer->wasStopped ())
                {
                    pcl_viewer->spinOnce (100);
                    boost::this_thread::sleep (boost::posix_time::microseconds (100000));
                }
                delete pcl_viewer;
                return 0;
            }else if(console::find_argument(argc, argv, "-mesh") >= 0){
                io::loadPCDFile<PointXYZRGB>(filename.c_str(), *rgbPoints);
                pcl::PolygonMesh triangles;
                generte_mesh(*rgbPoints, &triangles);
                pcl_viewer = new pcl::visualization::PCLVisualizer ("3D _viewer");
                pcl_viewer->addPolygonMesh(triangles);
                pcl_viewer->initCameraParameters ();
                while (!pcl_viewer->wasStopped ())
                {
                    pcl_viewer->spinOnce (100);
                    boost::this_thread::sleep (boost::posix_time::microseconds (100000));
                }
                delete pcl_viewer;
                return 0;

            } else if(console::find_argument(argc, argv, "-off") >= 0){
                PointCloud<PointXYZ>::Ptr offcloud(new PointCloud<PointXYZ>);
                readOFF(string(argv[2]), offcloud);
                pcl_viewer = new pcl::visualization::PCLVisualizer ("3D _viewer");
                pcl_viewer->addPointCloud<pcl::PointXYZ> (offcloud);
//                pcl_viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 10);
                pcl_viewer->initCameraParameters ();
                io::savePCDFile("off_to_pcd.pcd", *offcloud, true);
                while (!pcl_viewer->wasStopped ())
                {
                    pcl_viewer->spinOnce (100);
                    boost::this_thread::sleep (boost::posix_time::microseconds (100000));
                }
                delete pcl_viewer;
                return 0;
            } else{
                // -v
                io::loadPCDFile<PointXYZRGBA>(filename.c_str(), *cloudptr);

            }
        }
        catch (PCLException e1)
        {
            try
            {
                io::loadPCDFile<PointXYZ>(filename.c_str(), *fallbackCloud);

            }
            catch (PCLException e2)
            {
                return -1;
            }

            noColour = true;
        }

        cout << "Loaded " << filename << "." << endl;
        if (noColour)
            cout << "This file has no RGBA colour information present." << endl;
    }
    else
    {
        kinectGrabber = new OpenNIGrabber();
        if (kinectGrabber == 0)
            return false;
        boost::function<void (const PointCloud<PointXYZRGBA>::ConstPtr&)> f =
            boost::bind(&grabberCallback, _1);
        kinectGrabber->registerCallback(f);
    }

    viewer = createViewer();
    if (justVisualize)
    {
        if (noColour)
            viewer->showCloud(fallbackCloud);
        else viewer->showCloud(cloudptr);
    }
    else kinectGrabber->start();

    while (! viewer->wasStopped()){
        boost::this_thread::sleep(boost::posix_time::seconds(1));
    }

    if (! justVisualize)
        kinectGrabber->stop();
}
