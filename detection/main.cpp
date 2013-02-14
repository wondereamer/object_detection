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
#include "m_util.h"
#include <boost/filesystem.hpp>
#include <cassert>
#include <vector>
#include <algorithm>
#include "vizblockworld.h"
#include <pcl/console/parse.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/filters/filter.h>
#include "util.h"
#include "triangulate.h"
#include "shape_segment.h" 
#include "graph.h" 
#include <boost/graph/breadth_first_search.hpp>
#include <boost/algorithm/minmax_element.hpp>
using namespace std;
using namespace m_opencv;
using namespace m_lib;
using namespace boost::filesystem;
typedef pcl::PointXYZRGB PointT;
vector<string> pcdFiles;
VizBlockWorld viz;
string target;
int v1(0);
int v2(1);
int id(0);
void pcd_in_path(const std::string &fname){
    path p (fname);   // p reads clearer than argv[1] in the following code
    try {
        if (exists(p))    // does p actually exist?
        {
            if (is_regular_file(p)){
                assert(p.extension() == ".pcd");
                std::cout<<"load file:"<<p.string()<<std::endl;
                pcdFiles.push_back(p.string());
            }
            else if (is_directory(p))      // is p a directory?
            {
                vector<path> files;   
                copy(directory_iterator(p), directory_iterator(), back_inserter(files));
                for(path &file : files){
                    if (file.extension() == ".pcd" ) {
                        std::cout<<"load file:"<<file.string()<<std::endl;
                        pcdFiles.push_back(file.string());
                    }
                }
            }
            else
                cout << p << " exists, but is neither a regular file nor a directory\n";
        }
        else{
            cout << p << " does not exist\n";
            assert(false);
        }
    }catch (const filesystem_error& ex)
    {
        cout << ex.what() << '\n';
    }

}
//! find triangles of specific meaningful component
class TrianglesOfComponent : public boost::default_bfs_visitor
{
    public:
        static std::vector<Triangle*> triangles;
        template < typename Vertex, typename Graph >
            void discover_vertex(Vertex u, const Graph & g) const
            {
                auto &tree = ClusterNode::hieracTree;
                auto &node = tree.get_node(u);
                Triangle *t;
                if (node.id2 == -1) {
                    for (int i = 0; i < node.size; i++) {
                        triangles.push_back(static_cast<Triangle*>(node.triangles[i]));
                    }
                }
            }
};
std::vector<Triangle*> TrianglesOfComponent::triangles;

//! find meaningful components
class FindComponents : public boost::default_bfs_visitor
{
    public:
        template < typename Vertex, typename Graph >
            void discover_vertex(Vertex u, const Graph & g) const
            {
                std::cout << u << std::endl;
            }
};
void get_and_display_triangles(int root){
    // get triangles of a specific node in the tree
    TrianglesOfComponent get_triangles;
    boost::breadth_first_search(ClusterNode::hieracTree.get_container(), 
            boost::vertex(root, ClusterNode::hieracTree.get_container()), 
                 boost::visitor(get_triangles));
    // display the centers of the triangles
    pcl::PointCloud<PointT>::Ptr points(new pcl::PointCloud<PointT>);
    viz.set_def_cloud(points);
    for(auto *t : TrianglesOfComponent::triangles){
        auto p = t->getCenter();
        viz.add_point((float)p.x, (float)p.y, (float)p.z, 255, 255, 255, 1);
    }
    viz.push_def_cloud();
    // 
    TrianglesOfComponent::triangles.clear();
}
void fn(){
    // get meaningful components

    // display meaningful components with different color
}
void slip2next_cloud(int i){
    viz.clear();
    pcl::PCDReader reader;
    pcl::PointCloud<PointT>::Ptr cloud (new pcl::PointCloud<PointT>);
    pcl::PointCloud<PointT>::Ptr cloud2 (new pcl::PointCloud<PointT>);
    reader.read (pcdFiles[i], *cloud);
    viz.set_def_cloud(cloud2);
    // set the center of mass
    std::vector<float> xList, yList, zList;
    for(auto &p : cloud->points ){
        xList.push_back(p.x);
        yList.push_back(p.y);
        zList.push_back(p.z);
    }
    auto xMinMax = boost::minmax_element(xList.begin(), xList.end());
    auto yMinMax = boost::minmax_element(yList.begin(), yList.end());
    auto zMinMax = boost::minmax_element(zList.begin(), zList.end());
    float centerX = (*xMinMax.first + *xMinMax.second) / 2;
    float centerY = (*yMinMax.first + *yMinMax.second) / 2;
    float centerZ = (*zMinMax.first + *zMinMax.second) / 2;
    viz.set_offset(centerX/2, centerY/2, centerZ/2);
    // add points
    for(auto &p : cloud->points){
        viz.add_point((float)p.x, (float)p.y, (float)p.z, 255, 255, 255, 1); 
    }
    viz.push_def_cloud();
}

void slip2next_mesh(int i){
    viz.clear();
    // read pcd file and generate mesh
    pcl::PCDReader reader;
    pcl::PointCloud<pcl::PointXYZ>::Ptr mesh_points (new pcl::PointCloud<pcl::PointXYZ>);
    reader.read (pcdFiles[i], *mesh_points);
    pcl::PolygonMesh mesh;
    viz.generte_mesh(mesh_points, &mesh);
    // load mesh and cluster
    MyTriangulation tr;
    tr.load_mesh(mesh_points, mesh);
    auto &tree = ClusterNode::cluster(&tr);
    m_graph::GraphUtil<ClusterNode::HieraTree> gu(&tree);
    gu.write2dot("tree1.dot");
    // 
    fn();
    //
    std::cout<<"triangles in hieracTree:"<<TrianglesOfComponent::triangles.size()<<std::endl;
    // reset the node
    ClusterNode::reset();

}
void keyboardEventOccurred (const pcl::visualization::KeyboardEvent &event,
        void* viewer_void) {
    pcl::visualization::PCLVisualizer *viewer = static_cast<pcl::visualization::PCLVisualizer*> (viewer_void);
    if (event.getKeySym () == "n" && event.keyDown () && id < pcdFiles.size() - 1)
    {
        if ( ::target == "point" )
            slip2next_cloud(++id);
        else if ( ::target == "mesh" )
            slip2next_mesh(++id);
    }else if( event.getKeySym() == "s" && event.keyDown()){

    }

}
int main(int argc, char** argv)
{ 

    pcd_in_path(argv[2]);
    if (argc < 3) {
        std::cout<<"Usage: program -option filename";
    }
    // register event
    viz.register_keyboard_event(keyboardEventOccurred);
    viz.register_mouse_event(mouseEventOccurred);
    //    viz.create_h_viewport(v1, v2);

    if (pcl::console::find_argument(argc, argv, "-p") >= 0){
        ::target = "point";
        slip2next_cloud(0);
    } else if (pcl::console::find_argument(argc, argv, "-m") >= 0){
        ::target = "mesh";
        slip2next_mesh(0);
    }
    // display 
    viz.draw();
    viz.display();

    //        Segment3D<RgbPixel3D> image;
    //        printf("Usage: %s <image>\n", argv[0]);
    //        image.segment(0);
    //        image.save("3d_world, 15");



    return 0;
}
