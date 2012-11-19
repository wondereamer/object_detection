#ifndef VIZBLOCKWORLD_H

#define VIZBLOCKWORLD_H
#include <iostream>

#include <boost/thread/thread.hpp>
#include <pcl/common/common_headers.h>
#include <pcl/features/normal_3d.h>
#include <pcl/io/pcd_io.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/console/parse.h>
#include "m_util.h"
// helper function  to create point 
inline pcl::PointXYZRGB create_point(float x, float y, float z, int r, int g, int b, float step = 1){
    pcl::PointXYZRGB point;
    //point.x = x * step;
    point.x = x * step;
    point.y = y * step;
    point.z = z * step;

    uint8_t _r(r), _g(g), _b(b);
    uint32_t rgb = (static_cast<uint32_t>(_r) << 16 |
            static_cast<uint32_t>(_g) << 8 | static_cast<uint32_t>(_b));
    point.rgb = *reinterpret_cast<float*>(&rgb);
    return point;
}

inline void camera_info(pcl::visualization::Camera &camera)
{

    std::cout<<"focal: "<<camera.focal[0]<<" "<<camera.focal[1]<<" "<<camera.focal[2]<<std::endl;
    std::cout<<"pos: "<<camera.pos[0]<<" "<<camera.pos[1]<<" "<<camera.pos[2]<<std::endl;
    std::cout<<"view: "<<camera.view[0]<<" "<<camera.view[1]<<" "<<camera.view[2]<<std::endl;
    std::cout<<"window size: "<<camera.window_size[0]<<" "<<camera.window_size[1]<<std::endl;
    std::cout<<"window pos: "<<camera.window_pos[0]<<" "<<camera.window_pos[1]<<std::endl;
}


//enum RenderingProperties
//69     {

//70       PCL_VISUALIZER_POINT_SIZE,
//71       PCL_VISUALIZER_OPACITY,
//72       PCL_VISUALIZER_LINE_WIDTH,
//73       PCL_VISUALIZER_FONT_SIZE,
//74       PCL_VISUALIZER_COLOR,
//75       PCL_VISUALIZER_REPRESENTATION,
//76       PCL_VISUALIZER_IMMEDIATE_RENDERING
//77     };
//78 
//79     enum RenderingRepresentationProperties
//80     {

//81       PCL_VISUALIZER_REPRESENTATION_POINTS,
//82       PCL_VISUALIZER_REPRESENTATION_WIREFRAME,
//83       PCL_VISUALIZER_REPRESENTATION_SURFACE
//}
//}
//***************************************************************
class VizBlockWorld {
    public:
        VizBlockWorld ():_cloud(new pcl::PointCloud<pcl::PointXYZRGB>){
            _viewer = new pcl::visualization::PCLVisualizer ("3D _viewer");
            // will increase object id automatically, when add new visual element
            _objId = 0;
            set_offset(0, 0, 0);
        };
        VizBlockWorld (pcl::PointCloud<pcl::PointXYZRGB>::Ptr  cloud){
            _cloud = cloud;
            _viewer = new pcl::visualization::PCLVisualizer ("3D _viewer");
            _objId = 0;

            std::string pointId = m_util::sth2string<int>(_objId);
            pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGB> rgbs(_cloud);
            _viewer->addPointCloud<pcl::PointXYZRGB> (_cloud, rgbs, pointId);
            /*_viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, pointSize, pointId);*/
            _objId++;
            set_offset(0, 0, 0);
        }

        void draw(){
            _viewer->initCameraParameters ();
        }
        // display multiwindow viewer
        static void display(std::vector<pcl::visualization::PCLVisualizer*> viewers){
            while (!viewers[0]->wasStopped ())
            {
                viewers[0]->spinOnce (100);
                boost::this_thread::sleep (boost::posix_time::microseconds (100000));
            }

        }
        void display(){
            while (!_viewer->wasStopped ())
            {
                _viewer->spinOnce (100);
                boost::this_thread::sleep (boost::posix_time::microseconds (100000));
            }

        }
        void set_view(std::string object){
            // reset this point to the center of screen.
            _viewer->resetCameraViewpoint(object);
        }
        void set_offset(float x, float y, float z){
            _x_offset = x;
            _y_offset = y;
            _z_offset = z;
        }
        void set_render(){
            _viewer->setRepresentationToSurfaceForAllActors ();
            //_viewer->setRepresentationToPointsForAllActors();
            //_viewer->setRepresentationToWireframeForAllActors();
        }

        /**
         * @brief 
         *
         * @param xmin 0 <= 1.0
         * @param ymin 0 <= 1.0
         * @param xmax 0 <= 1.0
         * @param ymax 0 <= 1.0
         * @param viewport
         */
        void create_viewport(float xmin, float ymin, float xmax, float ymax, int &viewport){
            _viewer->createViewPort(xmin, ymin, xmax, ymax, viewport);
            set_backgroundcolor(0, 0, 0, viewport);
        }
        void create_h_viewport(int &v1, int &v2){
            create_viewport(0.0, 0.0, 1.0, 0.5, v1);
            create_viewport(0.0, 0.5, 1.0, 1.0, v2);
        }
        void create_v_viewport(int &v1, int &v2){
            create_viewport(0.0, 0.0, 0.5, 1.0, v1);
            create_viewport(0.5, 0.0, 1.0, 1.0, v2);
        }
        void set_backgroundcolor(int r, int g, int b, int viewport = 0){
            _viewer->setBackgroundColor (r, g, b, viewport);
        }
        // register event
        void register_keyboard_event(void (*callback) (const pcl::visualization::KeyboardEvent &event,
                                                                                    void *viewer_void)){
            _viewer->registerKeyboardCallback (callback, (void*)_viewer);
        }
        void register_mouse_event(void (*callback) (const pcl::visualization::MouseEvent &event,
                                                                                    void *viewer_void)){
            _viewer->registerMouseCallback (callback, (void*)_viewer);
        }
        //! add point to point cloud
        void  add_point(float x, float y, float z, int r, int g, int b, float unit){ 
            x = (x  - _x_offset) * unit;
            y = (y  - _y_offset) * unit;
            z = (z  - _z_offset) * unit;
            _cloud->points.push_back(create_point(x, y, z, r, g, b));
        }
        //! add point cloud to viewport
        std::string push_pointCloud(int &viewport, int pointSize = 1){
            std::string pointId = m_util::sth2string<int>(_objId);
            pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGB> rgbs(_cloud);
            _viewer->addPointCloud<pcl::PointXYZRGB> (_cloud, rgbs, pointId, viewport);
            _viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, pointSize, pointId);
            _objId++;
            _cloud->clear();
            return pointId;

        }
        //! add point cloud to default viewport(the only one)
        std::string push_pointCloud(int pointSize = 1){
            std::string pointId = m_util::sth2string<int>(_objId);
            pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGB> rgbs(_cloud);
            _viewer->addPointCloud<pcl::PointXYZRGB> (_cloud, rgbs, pointId);
            _viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, pointSize, pointId);
            _objId++;
            return pointId;

        }
        pcl::visualization::PCLVisualizer *get_viewer(){
            return _viewer;
        }
        /** @param SURFACE 0: point, 1: outline, 2: surface */
        void add_cube(float x, float y, float z, int r, int g, int b, float cubeSize = 1, int STYLE = 1){
            x = (x  - _x_offset) * cubeSize;
            y = (y  - _y_offset) * cubeSize;
            z = (z  - _z_offset) * cubeSize;
            float xMin = x;
            float xMax = x + cubeSize;
            float yMin = y;
            float yMax = y + cubeSize;
            float zMin = z;
            float zMax = z + cubeSize;
            std::string cubeId = m_util::sth2string<int>(_objId++);
            _viewer->addCube(xMin, xMax, yMin, yMax, zMin, zMax, r, g, b, cubeId);
            _viewer->setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_REPRESENTATION, STYLE, cubeId);
        }
        // add cube to specific viewport
        void add_cube(float x, float y, float z, int r, int g, int b, int &viewport, float cubeSize = 1, int STYLE = 1){
            x = (x  - _x_offset) * cubeSize;
            y = (y  - _y_offset) * cubeSize;
            z = (z  - _z_offset) * cubeSize;
            float xMin = x;
            float xMax = x + cubeSize;
            float yMin = y;
            float yMax = y + cubeSize;
            float zMin = z;
            float zMax = z + cubeSize;
            std::string cubeId = m_util::sth2string<int>(_objId++);
            _viewer->addCube(xMin, xMax, yMin, yMax, zMin, zMax, r, g, b, cubeId, viewport);
            _viewer->setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_REPRESENTATION, STYLE, cubeId);
        }
    private:
        pcl::visualization::PCLVisualizer *_viewer;
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr _cloud;
        // cube_id
        int _objId;
        // offset to the center of the world
        float _x_offset;
        float _y_offset;
        float _z_offset;
};

#endif /* end of include guard: VIZBLOCKWORLD_H */
