#include <iostream>
#include <algorithm>
#include <iterator>
#include <cv.h>
#include <stdio.h>
#include "m_opencv.h"
#include "m_algorithm.h"
#include <vector>
#include <map>
#include <string>
#include <pcl/io/pcd_io.h>
#include <xmlrpc-c/base.hpp>
#include "segment3d.h" 
#include "segment2d.h" 
#include "m_util.h"

using namespace std;
using namespace m_opencv;
using namespace m_lib;


int main(int argc, char** argv)
{ 

using std::vector;
VizBlockWorld viz;
int v1(0);
int v2(0);

//        Segment3D<RgbPixel3D> image;
//        printf("Usage: %s <image>\n", argv[0]);
//        return 1;
//        image.segment(0);
//        image.save("3d_world, 15");

//    std::vector<float> xList;
//    std::vector<float> yList;
//    std::vector<float> zList;
//    std::vector<int> rList;
//    std::vector<int> gList;
//    std::vector<int> bList;
//    std::vector<std::string> materialList;
//    std::vector<int> sizeList;
//    // get block attributes  from server
//    get_block_attrs(xList, yList, zList, rList, gList, bList, materialList, sizeList, "get_block_attrs", DETECTION_SERVER_PORT);

    // find the coordinate of the center of block worlds
    // set the (0, 0, 0) of visualization coordinate system to the center
    // register event
    viz.register_keyboard_event(keyboardEventOccurred);
    viz.register_mouse_event(mouseEventOccurred);
    pcl::PCDReader reader;
    pcl::PointCloud<PointT>::Ptr cloud (new pcl::PointCloud<PointT>);
    pcl::PointCloud<PointT>::Ptr cloud2 (new pcl::PointCloud<PointT>);
//    for(auto &p : cloud->points){
//    
//    }
//    float c_x = (*min_element(xList.begin(), xList.end()) + *max_element(xList.begin(), xList.end()));
//    float c_y = (*min_element(yList.begin(), yList.end()) + *max_element(yList.begin(), yList.end()));
//    float c_z = (*min_element(zList.begin(), zList.end()) + *max_element(zList.begin(), zList.end()));
//    viz.set_offset(c_x/2, c_y/2, c_z/2);
    reader.read (argv[1], *cloud);
    reader.read (argv[1], *cloud2);
    viz.create_v_viewport(v1, v2);
    // add origin blocks to visualizer
    viz.update_pointCloud(cloud, v1);
    viz.update_pointCloud(cloud2, v2);
    // display 
    viz.draw();
    viz.display();

//        Segment2D<RgbPixel2D> image(argv[1]);
//        image.segment(70);
//        image.save("Rgbsegmentation", 15);


    return 0;
}
