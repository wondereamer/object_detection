#include <iostream>
#include <algorithm>
#include <iterator>
#include <cv.h>
#include <stdio.h>
#include "m_algorithm.h"
#include <vector>
#include <map>
#include <string>
#include <pcl/io/pcd_io.h>
#include <xmlrpc-c/base.hpp>
#include <pcl/io/pcd_io.h>
#include "m_util.h"
#include <boost/filesystem.hpp>
#include <cassert>
#include <vector>
#include <algorithm>
#include <pcl/console/parse.h>
#include <pcl/filters/filter.h>
#include "util.h"
#include "triangulate.h"
#include "cluster_node.h" 
#include "graph.h" 
#include <boost/algorithm/minmax_element.hpp>
#include "vizblockworld.h" 
#include <pcl/sample_consensus/sac_model_sphere.h>
#include <pcl/sample_consensus/ransac.h>
#include <algorithm>
#include <iterator>
#include <pcl/features/normal_3d.h>
#include <pcl/sample_consensus/ransac.h>

using namespace std;
using namespace m_opencv;
using namespace m_lib;
using namespace boost::filesystem;
std::string fname;
vector<string> pcdFiles;
VizBlockWorld viz;
MyTriangulation vizMesh(&viz);
MyTriangulation *vizMesh2 = NULL;
string target;
// 
std::vector<pcl::PointCloud<PointT>::Ptr> components;
CloudVector planes;
CloudVector objects;
//
int v1(0);
int v2(1);
// file compId
int id(0);
int objId(-1);
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
//

void slip2next_mesh(const std::string &fname){

    // read pcd file and generate mesh
    viz.clear();
    pcl::PCDReader reader;
    pcl::PointCloud<pcl::PointXYZ>::Ptr points (new pcl::PointCloud<pcl::PointXYZ>);
    reader.read (fname, *points);
    pcl::PolygonMesh meshes;
    //    viz.generte_mesh(points, &meshes);
    //    viz.add_mesh(&meshes);
    vizMesh.segment_points(points);
    vizMesh.viz_next_level();
    //    vizMesh.compute_components();

}

void segment_mesh(PointCloudPtr points){

    viz.clear();
    pcl::PolygonMesh meshes;
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud( new pcl::PointCloud<pcl::PointXYZ>);
    pcl::copyPointCloud(*points, *cloud);
    //    viz.generte_mesh(cloud, &meshes);
    //    viz.add_mesh(&meshes);
    vizMesh2 = new MyTriangulation(&viz);
    vizMesh2->segment_points(cloud);
    vizMesh2->viz_next_level();
    viz.reset_camera();

}

void slip2next_cloud(PointCloudPtr cloud, const RgbColor &c){
    viz.clear();

    PointCloudPtr temp(new VizBlockWorld::PointCloud);
    viz.set_def_cloud(temp);
    for(auto &p : cloud->points){
        auto rgb = p.getRGBVector3i();
        viz.add_point(p.x, p.y, p.z, rgb[0], rgb[1], rgb[2]);
    }
    viz.push_def_cloud(0, 3);
    viz.reset_camera();

}
void slip2next_cloud(const std::string &fname){
    viz.clear();
    pcl::PCDReader reader;
    pcl::PointCloud<PointT>::Ptr cloud (new pcl::PointCloud<PointT>);
    pcl::PointCloud<PointT>::Ptr cloud2 (new pcl::PointCloud<PointT>);
    reader.read (fname, *cloud);
    viz.set_def_cloud(cloud2);
    // add points
    for(auto &p : cloud->points){
        viz.add_point((float)p.x, (float)p.y, (float)p.z, 255, 0, 0, 1); 
    }
    //    std::vector<int> inliers;
    //
    //    pcl::SampleConsensusModelCylinder<PointT, pcl::Normal>::Ptr
    //        model(new pcl::SampleConsensusModelCylinder<PointT,pcl::Normal> (cloud));
    //    pcl::NormalEstimation<PointT, pcl::Normal> ne;
    //    pcl::search::KdTree<PointT>::Ptr tree (new pcl::search::KdTree<PointT> ());
    //    pcl::PointCloud<pcl::Normal>::Ptr cloud_normals (new pcl::PointCloud<pcl::Normal>);
    //    ne.setSearchMethod (tree);
    //    ne.setInputCloud (cloud);
    //    ne.setKSearch (50);
    //    ne.compute (*cloud_normals);
    //    model->setInputNormals(cloud_normals); 
    //    std::cout<<"***************cylinder 1********"<<std::endl;    
    //    //     
    //    pcl::RandomSampleConsensus<PointT> ransac (model);
    //    std::cout<<"***************cylinder 1********"<<std::endl;    
    //    ransac.setDistanceThreshold (.01);
    //    ransac.computeModel();
    //    ransac.getInliers(inliers);
    //
    //    std::cout<<"***************cylinder 1********"<<std::endl;    
    //    typedef Eigen::VectorXf Coefficient;
    //    Coefficient modelCoefficient;
    //    ransac.getModelCoefficients(modelCoefficient);
    //
    //    auto &p = cloud->points;
    //    for(int i : inliers){
    //        viz.add_point((float)p[i].x, (float)p[i].y, (float)p[i].z, 255, 0, 0, 1); 
    //    }
    //
    viz.push_def_cloud();
    //    viz.add_sphere(modelCoefficient[0], modelCoefficient[1], modelCoefficient[2], modelCoefficient[3]);

}
void scene_cluster(const std::string &fname){
    // initial
    viz.clear();
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZRGB>);
    viz.set_def_cloud(cloud);
    // read file
    PointCloudPtr rgbCloud (new VizBlockWorld::PointCloud);
    pcl::io::loadPCDFile(fname, *rgbCloud);
    // down sample points
    //    pcl::PointCloud<PointT>::Ptr cloud_filtered = down_samples(points);
    cluster_points(rgbCloud, &components);

    std::cout<<"*********components size:********"<<std::endl;    
    std::cout<<components.size()<<std::endl;
    for(auto cluster : components){
        std::cout<<cluster->points.size()<<std::endl;
        pcl::PointCloud<PointT>::Ptr rest(new pcl::PointCloud<PointT>);
        // segment each component to planes and rest
        segment_plane(cluster, rest, &planes);

        /// @todo if some plane is not supporter, put it back to #rest

        // cluster the rest points to objects
        if(rest->points.size()){
            cluster_points(rest, &objects);

        }
    }

    // mark planes green
    for(auto plane : planes){
        for(auto &p: plane->points){
            viz.add_point(p, 0, 255, 0);
        }
    }

    // mark  objects red
    for(auto obj : objects){
        for(auto &p: obj->points){
            viz.add_point(p, 255, 0, 0);
        }
    }


    std::cout<<"*********objects size:********"<<std::endl;    
    std::cout<<objects.size()<<std::endl;
    viz.push_def_cloud();
}

void keyboardEventOccurred (const pcl::visualization::KeyboardEvent &event,
        void* viewer_void) {
    static bool is_points = false;
    static string primitiveId = "";
    static bool isComponent = false;
    pcl::visualization::PCLVisualizer *viewer = 
        static_cast<pcl::visualization::PCLVisualizer*> (viewer_void);
    if (event.getKeySym () == "n" && event.keyDown () )
    {
        if ( ::target == "point" )
            slip2next_cloud(pcdFiles[++id]);
        else if ( ::target == "mesh" ){
            viz.clear();
            slip2next_mesh(pcdFiles[++id]);
        }else if( ::target == "browse components" && id < components.size()){
            std::cout<<"components:"<<std::endl;
            RgbColor c(0, 255, 0);
            slip2next_cloud(components[id++], c);
            isComponent = true;

        }else if( ::target == "browse objects" && objId + 1 < objects.size()){
            // -> size - 1
            // @@snipet
            ++objId;
            RgbColor c(255, 0, 0);
            // find the first element satisfy the requirement
            while(objId + 1< objects.size() &&
                    objects[objId]->points.size() > 90000) 
                objId++;
            if(objects[objId]->points.size() <= 90000)
                slip2next_cloud(objects[objId], c);
            std::cout<<"object: "<<objId<<std::endl;
            viz.set_backgroundcolor(255, 255, 255);
        }

    }else if( event.getKeySym() == "p" && event.keyDown()){
        // 0 <-
        if( ::target == "browse objects" && objId > 0){
            RgbColor c(255, 0, 0);
            std::cout<<objId<<std::endl;
            while(objId > 0 &&
                    objects[--objId]->points.size() > 90000){
            }; 
            if(objects[objId]->points.size() <= 90000)
                slip2next_cloud(objects[objId], c);
        }
    }else if( event.getKeySym() == "i" && event.keyDown()){
        vizMesh2->viz_next_level();
    }else if( event.getKeySym() == "d" && event.keyDown()){
        vizMesh2->viz_previous_level();
    }else if( event.getKeySym() == "c" && event.keyDown()){
        vizMesh2->change_color();
        // add primitive
        if (primitiveId == "" ) {
            auto &cov =ClusterNode::hierarchyTree.get_node(ClusterNode::rootId).coefficient;
            switch(BinaryTree::_currentNode.type) {
                case HFP_FIT_PLANES:
                    primitiveId = viz.add_plane(cov.point.x, cov.point.y, cov.point.z, 
                            cov.direction.x, cov.direction.y,cov.direction.z);
                    break;
                case HFP_FIT_SPHERES:
                    primitiveId = viz.add_sphere(cov.point.x, cov.point.y, cov.point.z, cov.radius);
                    break;
                case HFP_FIT_CYLINDERS:
                    primitiveId = viz.add_cylinder(cov.point.x, cov.point.y, cov.point.z, 
                            cov.direction.x, cov.direction.y,cov.direction.z, cov.radius);
                    break;
                default:
                    assert(false);
            }
            std::cout<<"*********************************"<<std::endl;    
            std::cout<<(double)BinaryTree::_currentNode.type<<std::endl;
            std::cout<<(double)BinaryTree::_currentNode.cost<<std::endl;
            std::cout<<(double)BinaryTree::_currentNode.cost0<<std::endl;
            std::cout<<(double)BinaryTree::_currentNode.cost1<<std::endl;
            std::cout<<(double)BinaryTree::_currentNode.cost2<<std::endl;

        }
        else{
            viz.remove_shape(primitiveId);
            primitiveId = "";
        }
    }else if( event.getKeySym() == "space" && event.keyDown()){
        std::string fname = m_util::string_format("object_%d", objId);
        std::vector<float> weights;
        segment_mesh(objects[objId]);
        PointCloudPtr pos(new pcl::PointCloud<PointT>);
        PointCloudPtr source(new pcl::PointCloud<PointT>);
        vizMesh2->pos_and_weight(fname, pos, &weights);
        pcl::copyPointCloud(*pos, *source);
        vizMesh2->dynamic_EMD(pos, weights, source, weights);
        viz.set_backgroundcolor(255, 255, 255);

    }else if( event.getKeySym() == "s" && event.keyDown()){
        std::string fname = m_util::string_format("object_%d", objId);
        std::vector<float> weights;
        segment_mesh(objects[objId]);
        PointCloudPtr pos(new pcl::PointCloud<PointT>);
        PointCloudPtr source(new pcl::PointCloud<PointT>);
        vizMesh2->pos_and_weight(fname, pos, &weights);

    }else if( event.getKeySym() == "d" && event.keyDown()){

        std::string fname = m_util::string_format("object_%d", objId);
        pcl::PCDWriter writer; 
        writer.write(fname + ".pcd" , *objects[objId]);
        std::cout<<"save points to "<<fname<<std::endl;

    }else if( event.getKeySym() == "l" && event.keyDown()){
        int  t = 4;
        // s
        std::vector<float> wT;
        PointCloudPtr target(new pcl::PointCloud<PointT>);
        segment_mesh(objects[t]);
        vizMesh2->pos_and_weight("temp", target, &wT);
        //
        vector<int> s;

        s.push_back(6);   // pp
        s.push_back(16); // 
        s.push_back(2); //tiger
        s.push_back(19);// chang 
        s.push_back(22); // ping zi
        for(int id : s){
            std::vector<float> wS;
            PointCloudPtr source(new pcl::PointCloud<PointT>);
            segment_mesh(objects[id]);
            vizMesh2->pos_and_weight(fname, source, &wS);
            std::cerr<<"*********************************"<<std::endl;    
            std::cerr<<"emd to id: "<<id<<endl
                     <<vizMesh2->dynamic_EMD(target, wT, source, wS)<<std::endl;
            viz.set_backgroundcolor(255,255,255);
            viz.save_screenshot(m_util::sth2string<int>(id)+".PNG");
        }

    }


}
int main(int argc, char** argv)
{ 
    fname = std::string(argv[2]);
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
        slip2next_cloud(pcdFiles[0]);
    } else if (pcl::console::find_argument(argc, argv, "-m") >= 0){
        ::target = "mesh";
        slip2next_mesh(pcdFiles[0]);
    }else if (pcl::console::find_argument(argc, argv, "-c") >= 0){
        ::target = "browse components";
        scene_cluster(pcdFiles[0]);
    } else if (pcl::console::find_argument(argc, argv, "-o") >= 0){
        ::target = "browse objects";
        // seperate components and objects
        scene_cluster(pcdFiles[0]);
        // save objects
        //        for (int i = 0; i < objects.size(); i++) {
        //            pcl::PCDWriter writer; 
        //            fname = m_util::string_format("object_%d", i);
        //            writer.write(fname + ".pcd" , *objects[i]);
        //            std::cout<<"save points to "<<fname<<std::endl;
        //        }
    } else if (pcl::console::find_argument(argc, argv, "-of") >= 0){
        ::target = "browse objects";
        // load objects from files
        m_util::print_list(pcdFiles);
        for(auto &fname : pcdFiles){
            pcl::PCDReader reader;
            pcl::PointCloud<PointT>::Ptr cloud (new pcl::PointCloud<PointT>);
            reader.read (fname, *cloud);
            objects.push_back(cloud);
            std::cout<<"*******************rrrr**********"<<std::endl;    
        }
    }

//    grid_color2(&viz);
    // display 
    viz.draw();
    viz.display();

    //        Segment3D<RgbPixel3D> image;
    //        printf("Usage: %s <image>\n", argv[0]);
    //        image.segment(0);
    //        image.save("3d_world, 15");



    return 0;
}
