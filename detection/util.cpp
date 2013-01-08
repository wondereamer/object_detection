//#include "util.h"
#include <m_opencv.h>
#include <vector>

#include <boost/thread/thread.hpp>
#include <sstream>
#include "vizblockworld.h" 
//#include "m_util.h"
#include <cassert>
using namespace m_opencv;
std::vector<RgbColor>& components_color( ){
    static std::vector<RgbColor> marks;
    if(marks.size() == 0){

        RgbColor color;
        color.r = 234;
        color.g = 16;
        color.b = 7;
        marks.push_back(color);

        color.r = 162;
        color.g = 210;
        color.b = 101;
        marks.push_back(color);

        color.r = 28;
        color.g = 166;
        color.b = 205;
        marks.push_back(color);

        color.r = 69;
        color.g = 183;
        color.b = 17;
        marks.push_back(color);

        color.r = 255;
        color.g = 112;
        color.b = 117;
        marks.push_back(color);

        color.r = 184;
        color.g = 255;
        color.b = 92;
        marks.push_back(color);

        color.r = 162;
        color.g = 92;
        color.b = 255;
        marks.push_back(color);

        color.r = 137;
        color.g = 242;
        color.b = 218;
        marks.push_back(color);
    }
    return marks;
}

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
