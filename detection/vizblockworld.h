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
#include <pcl/surface/gp3.h>
#include <boost/algorithm/minmax_element.hpp>
#include <vector>
// helper function  to create point 
typedef pcl::PointXYZRGB PointT;
typedef std::vector<pcl::PointCloud<PointT>::Ptr> CloudVector;
typedef pcl::PointCloud<PointT>::Ptr PointCloudPtr;
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

//transformPointCloud (output, output, transformation_);
void transform_pointcloud(const pcl::PointCloud<PointT> &cloud_in,
                            pcl::PointCloud<PointT> &cloud_out,
                            const Eigen::Matrix4f &transform);

void save_cloud(const std::string &fname, PointCloudPtr cloud);
void read_cloud(const std::string &fname, PointCloudPtr cloud);

inline void camera_info(pcl::visualization::Camera &camera)
{

    std::cout<<"focal: "<<camera.focal[0]<<" "<<camera.focal[1]<<" "<<camera.focal[2]<<std::endl;
    std::cout<<"pos: "<<camera.pos[0]<<" "<<camera.pos[1]<<" "<<camera.pos[2]<<std::endl;
    std::cout<<"view: "<<camera.view[0]<<" "<<camera.view[1]<<" "<<camera.view[2]<<std::endl;
    std::cout<<"window size: "<<camera.window_size[0]<<" "<<camera.window_size[1]<<std::endl;
    std::cout<<"window pos: "<<camera.window_pos[0]<<" "<<camera.window_pos[1]<<std::endl;
}
//pcl::transformPointCloud
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

/**
 * @brief an wrapper class to display rgb points
 */
class VizBlockWorld {
    public:
        typedef pcl::PointCloud<PointT> PointCloud;

        VizBlockWorld (){
            _viewer = new pcl::visualization::PCLVisualizer ("3D _viewer");
            // will increase object id automatically, when add new visual element
            _objId = 0;
            set_offset(0, 0, 0);
        };

        void set_def_cloud(pcl::PointCloud<PointT>::Ptr cloud){ _cloud = cloud; }
        void remove_def_cloud( ){
            PointCloudPtr temp(new PointCloud);
            _cloud = temp; }
        void generte_mesh(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, pcl::PolygonMesh *triangles);
        //            _viewer->addCoordinateSystem(1.0); 
        void draw(){ _viewer->initCameraParameters (); }
        // display multiwindow viewer
        static void display(std::vector<pcl::visualization::PCLVisualizer*> viewers);
        void display();
        //! reset this point to the center of screen.
        void set_view(std::string object){ _viewer->resetCameraViewpoint(object); }

        template < typename T >
            void move2offset(T cloud, float unit = 1){
                std::vector<float> xList, yList, zList;
                for(auto &p : cloud->points ){
                    xList.push_back(p.x);
                    yList.push_back(p.y);
                    zList.push_back(p.z);
                }
                assert(cloud->points.size());
                auto xMinMax = boost::minmax_element(xList.begin(), xList.end());
                auto yMinMax = boost::minmax_element(yList.begin(), yList.end());
                auto zMinMax = boost::minmax_element(zList.begin(), zList.end());
                float centerX = (*xMinMax.first + *xMinMax.second) / 2;
                float centerY = (*yMinMax.first + *yMinMax.second) / 2;
                float centerZ = (*zMinMax.first + *zMinMax.second) / 2;
                for(auto &p : cloud->points ){
                    p.x = (p.x - centerX) * unit;
                    p.y = (p.y - centerY) * unit;
                    p.z = (p.z - centerZ) * unit;
                }
            }
        void set_offset(float x, float y, float z){ _x_offset = x; _y_offset = y; _z_offset = z; }
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
        inline void  add_point(float x, float y, float z, int r, int g, int b, float unit = 1){ 
            x = (x  - _x_offset) * unit;
            y = (y  - _y_offset) * unit;
            z = (z  - _z_offset) * unit;
            assert(_cloud);
            _cloud->points.push_back(create_point(x, y, z, r, g, b));
        }
        inline void add_point(const PointT &p, int r, int g, int b, float unit = 1){
            assert(_cloud);
            add_point(p.x, p.y, p.z, r, g, b, unit);
        }
        //! update default point cloud to viewport
        std::string push_def_cloud(int viewport = 0, int pointSize = 1){
            assert(_cloud);
            std::string pointId = m_util::sth2string<int>(_objId++);
            pcl::visualization::PointCloudColorHandlerRGBField<PointT> rgbs(_cloud);
            _viewer->addPointCloud<PointT> (_cloud, rgbs, pointId, viewport);
            _viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, pointSize, pointId);
            return pointId;

        }
        //! add point pointcloud to viewport
        std::string add_cloud(const pcl::PointCloud<PointT>::Ptr cloud, int viewport = 0, int pointSize = 1){
            std::string pointId = m_util::sth2string<int>(_objId++);
            pcl::visualization::PointCloudColorHandlerRGBField<PointT> rgbs(cloud);
            _viewer->addPointCloud<PointT> (cloud, rgbs, pointId, viewport);
//            std::cout<<cloud->points.size()<<std::endl;
//            for(auto &p : cloud->points){
//                 std::cout<<(unsigned int)p.rgb<<std::endl;
//            }
            _viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, pointSize, pointId);
            return pointId;

        }

        pcl::visualization::PCLVisualizer *get_viewer(){
            return _viewer;
        }
        /** @param SURFACE 0: point, 1: outline, 2: surface */
        std::string add_cube(float x, float y, float z, int r, int g, int b,
                           int viewport = 0, float cubeSize = 1, int STYLE = 1){
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
            return cubeId;
        }
        //! add an mesh
        std::string  add_mesh(const pcl::PolygonMesh *mesh, int viewport = 0){
            std::string id = m_util::sth2string<int>(_objId++);
            _viewer->addPolygonMesh(*mesh, id, viewport);
            return id;
        }
        void clear(int viewport = 0){
            _objId = 0;
            _viewer->removeAllPointClouds(viewport);
            _viewer->removeAllShapes(viewport);
            if(_cloud)
                _cloud->points.clear();
        }
        void remove_shape(const std::string &id){
            _viewer->removeShape(id);
        }
        void remove_cloud(const std::string &id){
            _viewer->removePointCloud(id);
        }
        std::string add_cylinder(double centerX, double centerY, double centerZ, double normalX,
                double normalY, double normalZ, double radius, int viewport = 0){
            pcl::ModelCoefficients cylinder_coeff;
            cylinder_coeff.values.resize (7); // We need 7 values
            cylinder_coeff.values[0] = centerX;
            cylinder_coeff.values[1] = centerY;
            cylinder_coeff.values[2] = centerZ;
            cylinder_coeff.values[3] = normalX;
            cylinder_coeff.values[4] = normalY;
            cylinder_coeff.values[5] = normalZ;
            cylinder_coeff.values[6] = radius;
            std::string id = m_util::sth2string<int>(_objId++);
            _viewer->addCylinder (cylinder_coeff, id, viewport);
            return id;
        }
        std::string add_plane(double a, double b, double c, double d, int viewport = 0){
            pcl::ModelCoefficients plane_coeff;
            plane_coeff.values.resize (4); // We need 4 values
            plane_coeff.values[0] =  a;
            plane_coeff.values[1] =  b;
            plane_coeff.values[2] =  c;
            plane_coeff.values[3] =  d;
            std::string id = m_util::sth2string<int>(_objId++);
            _viewer->addPlane (plane_coeff, id, viewport);
            return id;
        }
        std::string add_plane(double px, double py, double pz, double normalX,
                               double normalY, double normalZ, int viewport = 0){
            double d = 0-px*normalX - py*normalY - pz*normalZ;
            return add_plane(px, py, pz, d, viewport);

        }
        std::string add_sphere(double centerX, double centerY, double centerZ,
                                                double radius, int viewport = 0){
            auto point = create_point(centerX, centerY, centerZ, 0, 0, 0);
            std::string id = m_util::sth2string<int>(_objId++);
            _viewer->addSphere(point, radius, id, viewport);
            return id;
        }

        std::string add_sphere(double centerX, double centerY, double centerZ, 
                            double radius, int r, int g, int b, int viewport = 0){
            auto point = create_point(centerX, centerY, centerZ, 0, 0, 0);
            std::string id = m_util::sth2string<int>(_objId++);
            _viewer->addSphere(point, radius, r, g, b, id, viewport);
            return id;
        }

        std::string add_line(double aX, double aY, double aZ, 
                             double bX, double bY, double bZ,
                               int viewport = 0){
            auto pa = create_point(aX, aY, aZ, 0, 0, 0);
            auto pb = create_point(bX, bY, bZ, 0, 0, 0);
            std::string id = m_util::sth2string<int>(_objId++);
            _viewer->addLine(pa, pb, id, viewport);
            return id;
        }
        std::string add_line(double aX, double aY, double aZ, 
                             double bX, double bY, double bZ,
                               int r, int g, int b, int viewport = 0){
            auto pa = create_point(aX, aY, aZ, 0, 0, 0);
            auto pb = create_point(bX, bY, bZ, 0, 0, 0);
            std::string id = m_util::sth2string<int>(_objId++);
            _viewer->addLine(pa, pb, r, g, b, id, viewport);
            return id;
        }
        std::string add_text(const std::string &text, int x, int y, int viewport = 0){
            std::string id = m_util::sth2string<int>(_objId++);
            _viewer->addText(text, x, y, id, viewport);
            return id;
        }
        std::string add_text3D(const std::string &text, float x, float y, float z, 
                            int r, int g, int b, double scale = 1.0, int viewport = 0){
            auto pos = create_point(x, y, z, 0, 0, 0);
            std::string id = m_util::sth2string<int>(_objId++);
            _viewer->addText3D(text, pos, scale, (double)r, (double)g, (double)b,
                              id, viewport);
            return id;
        }
        void create_viewport_camera(int view){
            // only work for pcl 1.7
//            _viewer->createViewPortCamera(view);
        }
        void reset_camera(){
            _viewer->resetCamera();
        }
        void save_screenshot(const std::string &fname){
            _viewer->saveScreenshot(fname);
        }



    private:
        pcl::visualization::PCLVisualizer *_viewer;
        pcl::PointCloud<PointT>::Ptr _cloud;
        // cube_id
        int _objId;
        // offset to the center of the world
        float _x_offset;
        float _y_offset;
        float _z_offset;
};

//! return planes and objects
void segment_plane (const pcl::PointCloud<PointT>::Ptr cloud, pcl::PointCloud<PointT>::Ptr objects, CloudVector* planes);
void cluster_points(const pcl::PointCloud<PointT>::Ptr cloud, CloudVector *clusters);
PointCloudPtr down_samples(PointCloudPtr input);
void grid_color(VizBlockWorld *viz);
void grid_color2(VizBlockWorld *viz);
#endif /* end of include guard: VIZBLOCKWORLD_H */
