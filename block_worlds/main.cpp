/* \author Geoffrey Biggs */


#include <iostream>

#include <boost/thread/thread.hpp>
#include <pcl/common/common_headers.h>
#include <pcl/features/normal_3d.h>
#include <pcl/io/pcd_io.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/console/parse.h>
#include <sstream>
#include "vizblockworld.h" 
#include "m_util.h"
#include <xmlrpc-c/base.hpp>
#include <cassert>
#include <data.h>
using std::vector;
VizBlockWorld viz;
int v1(0);
int v2(0);
void keyboardEventOccurred (const pcl::visualization::KeyboardEvent &event,
        void* viewer_void)
{
    static bool existSight = false;
    pcl::visualization::PCLVisualizer *viewer = static_cast<pcl::visualization::PCLVisualizer*> (viewer_void);
    if (event.getKeySym () == "f" && event.keyDown ())
    {
        //viewer->removeShape ("o1");
        //viewer->removePointCloud("p1");
        //viewer->resetCameraViewpoint("o2");
        //viewer->setCameraPosition(1,1,1,3,2,1);
        //viewer->setCameraPosition(_x--,_y,_z,x,y,z);

    }

    if (event.getKeySym () == "j" && event.keyDown ()){
        // get camera
        std::vector<pcl::visualization::Camera> cameras;
        viewer->getCameras(cameras);
        pcl::visualization::Camera &c = *cameras.begin();
        camera_info(*cameras.begin());
        // remove camera point and line of sight
        if(existSight){
            viewer->removePointCloud("camera");
            viewer->removeShape("sight");
        }
        existSight = true;
        // add camera point
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZRGB>);
        cloud->points.push_back(create_point(c.pos[0], c.pos[1], c.pos[2], 0, 0, 255));
        pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGB> rgbs(cloud);
        viewer->addPointCloud<pcl::PointXYZRGB> (cloud, rgbs,"camera");
        viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 30, "camera");
        // add line of sight
        pcl::PointXYZ cameraPos(c.pos[0], c.pos[1], c.pos[2]);
        pcl::PointXYZ view(c.view[0], c.view[1], c.view[2]);
        viewer->addLine(cameraPos, view, "sight");

        /*pcl::PointCloud<pcl::PointXYZ>::Ptr scene (new pcl::PointCloud<pcl::PointXYZ>());*/
        /*viewer->renderView(300, 600, scene);*/
    }
}

void mouseEventOccurred (const pcl::visualization::MouseEvent &event,
        void* viewer_void)
{
    pcl::visualization::PCLVisualizer *viewer = static_cast<pcl::visualization::PCLVisualizer*> (viewer_void);
    if (event.getButton () == pcl::visualization::MouseEvent::LeftButton &&
            event.getType () == pcl::visualization::MouseEvent::MouseButtonRelease)
    {
        std::cout << "Left mouse button released at position (" << event.getX () << ", " << event.getY () << ")" << std::endl;
        //char str[512];
        //viewer->addText ("clicked here", event.getX (), event.getY (), str);
    }
}
int main (int argc, char** argv)
{

    std::vector<float> xList;
    std::vector<float> yList;
    std::vector<float> zList;
    std::vector<int> rList;
    std::vector<int> gList;
    std::vector<int> bList;
    std::vector<std::string> materialList;
    std::vector<int> sizeList;
    // get block attributes  from server
    get_block_attrs(xList, yList, zList, rList, gList, bList, materialList, sizeList, "get_block_attrs", DETECTION_SERVER_PORT);

    // find the coordinate of the center of block worlds
    // set the (0, 0, 0) of visualization coordinate system to the center
    float c_x = (*min_element(xList.begin(), xList.end()) + *max_element(xList.begin(), xList.end()));
    float c_y = (*min_element(yList.begin(), yList.end()) + *max_element(yList.begin(), yList.end()));
    float c_z = (*min_element(zList.begin(), zList.end()) + *max_element(zList.begin(), zList.end()));
    viz.set_offset(c_x/2, c_y/2, c_z/2);
    // register event
    viz.register_keyboard_event(keyboardEventOccurred);
    viz.register_mouse_event(mouseEventOccurred);
    viz.create_v_viewport(v1, v2);
    // add origin blocks to visualizer
    for (int i = 0; i < xList.size(); i++) {
        viz.add_point(xList[i], yList[i], zList[i], rList[i], gList[i], bList[i], 0.1);
        //viz.add_cube(xList[i], yList[i], zList[i], 0, 255, 0, 0.1, 2);
        //viz.add_cube(xList[i], yList[i], zList[i], 255, 255, 255, 0.1, 1);
    }
    viz.push_pointCloud(v1, 30);

    // add segmentation result to visualizer
    xList.clear();
    yList.clear();
    zList.clear();
    rList.clear();
    gList.clear();
    bList.clear();
    materialList.clear();
    sizeList.clear();
    get_block_attrs(xList, yList, zList, rList, gList, bList, materialList, sizeList, "get_segmentation_result", DETECTION_SERVER_PORT);
    for (int i = 0; i < xList.size(); i++) {
        viz.add_point(xList[i], yList[i], zList[i], rList[i], gList[i], bList[i], 0.1);
    }
    viz.push_pointCloud(v2, 30);
    // display 
    viz.draw();
    viz.display();
}

//#include <iostream>
//#include <pcl/point_cloud.h>
//#include <pcl/io/pcd_io.h>
//#include <pcl/point_types.h>
///*
// * Sample code for reading point clouds in the RGB-D Object Dataset using the Point Cloud Library
// *
// * Author: Kevin Lai
// */
//struct PointXYZRGBIM
//{
//    union
//    {
//        struct
//        {
//            float x;
//            float y;
//            float z;
//            float rgb;
//            float imX;
//            float imY;
//        };
//        float data[6];
//    };
//    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
//} EIGEN_ALIGN16;
//
//POINT_CLOUD_REGISTER_POINT_STRUCT (PointXYZRGBIM,
//        (float, x, x)
//        (float, y, y)
//        (float, z, z)
//        (float, rgb, rgb)
//        (float, imX, imX)
//        (float, imY, imY)
//        )
//
//    int
//main (int argc, char** argv)
//{
//    pcl::PointCloud<PointXYZRGBIM>::Ptr cloud (new pcl::PointCloud<PointXYZRGBIM>);
//
//    if (pcl::io::loadPCDFile<PointXYZRGBIM> (argv[1], *cloud) == -1) //* load the file
//    {
//        printf ("Couldn't read file test_pcd.pcd \n");
//        return (-1);
//    }
//    std::cout << "Loaded "
//        << cloud->width * cloud->height
//        << " data points from test_pcd.pcd with the following fields: "
//        << std::endl;
//
//    for (size_t i = 0; i < cloud->points.size (); ++i){
//        uint32_t rgb = *reinterpret_cast<int*>(&cloud->points[i].rgb);
//        uint8_t r = (rgb >> 16) & 0x0000ff;
//        uint8_t g = (rgb >> 8)  & 0x0000ff;
//        uint8_t b = (rgb)       & 0x0000ff;
//        viz.add_point(cloud->points[i].x, cloud->points[i].y, cloud->points[i].z, (int)r, (int)g, (int)b, 1);
//    }
//    std::cout<<cloud->points.size()<<std::endl;        
//    viz.push_pointCloud(3);
//    viz.set_backgroundcolor(255, 255, 255);
//    viz.draw();
//    viz.display();
//
//
//    return (0);
//}
