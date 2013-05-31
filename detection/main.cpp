#include <iostream>
#include <algorithm>
#include <iterator>
#include <cv.h>
#include <numeric>
#include <stdio.h>
#include <library/m_algorithm.h>
#include <vector>
#include <map>
#include <string>
#include <pcl/io/pcd_io.h>
#include <xmlrpc-c/base.hpp>
#include <pcl/io/pcd_io.h>
#include <library/m_util.h>
#include <boost/filesystem.hpp>
#include <cassert>
#include <vector>
#include <algorithm>
#include <pcl/console/parse.h>
#include <pcl/filters/filter.h>
#include "util.h"
#include "eye3d.h"
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
#include <fstream>

using namespace std;
using namespace m_opencv;
using namespace m_lib;
using namespace boost::filesystem;
std::string fname;
vector<string> pcdFiles;
VizBlockWorld viz;
Eye3D *vizMesh2 = NULL;
string target;
// 
std::vector<pcl::PointCloud<PointT>::Ptr> components;
CloudVector planes;
CloudVector objects;
//
int v1(0);
int v2(1);
const int MAX_SIZE_OF_OBJECT = 20000;
//const float MESH_RADIUS = 30;
//const float MESH_RADIUS = 0.002;
const float MESH_RADIUS = 0.025;
// display zoom
float ZOOM = 10;
// file compId
int id(0);
int objId(-1);

struct ObjWeights{
    std::vector<float> pWeights;                  // proportion weight
    std::vector<float> dWeights;                  // degree weight 
    std::vector<float> tWeights;                // type weight 
    std::vector<float> aWeights;                 // angle weight 
    PointCloudPtr pos;
};

vector<string>
list_files(const std::string &fname){
    path p (fname);   // p reads clearer than argv[1] in the following code
    vector<string> str_files;
    try {
        if (exists(p))    // does p actually exist?
        {
            if (is_regular_file(p)){
                //                assert(p.extension() == ".pcd");
                std::cout<<"load file:"<<p.string()<<std::endl;
                str_files.push_back(p.string());
            }
            else if (is_directory(p))      // is p a directory?
            {
                vector<path> files;   
                copy(directory_iterator(p), directory_iterator(), back_inserter(files));
                for(path &file : files){
                    if (file.extension() == ".pcd" || file.extension() == ".off" ) {
                        std::cout<<"load file:"<<file.string()<<std::endl;
                        str_files.push_back(file.string());
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
    return str_files;

}
//

void slip2next_mesh(const std::string &fname){

    // read pcd file and generate mesh
    viz.clear();
    pcl::PCDReader reader;
    pcl::PointCloud<pcl::PointXYZ>::Ptr points (new pcl::PointCloud<pcl::PointXYZ>);
    reader.read (fname, *points);
    pcl::PolygonMesh meshes;
    viz.generte_mesh(points, &meshes, MESH_RADIUS);
    viz.add_mesh(&meshes);
    //    vizMesh.segment_points(points, MESH_RADIUS);
    //    vizMesh.viz_next_level();
    //    vizMesh.compute_components();

}

void segment_mesh(PointCloudPtr points){

    viz.clear();
    pcl::PolygonMesh meshes;
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud( new pcl::PointCloud<pcl::PointXYZ>);
    pcl::copyPointCloud(*points, *cloud);
    //    viz.add_mesh(&meshes);
    vizMesh2 = new Eye3D(&viz);
    //    pcl::PolygonMesh mesh;
    //    viz.generte_mesh(cloud, &meshes, 3);
    //    viz.add_mesh(&mesh);
    //    std::cout<<mesh.polygons.size()<<std::endl;
    vizMesh2->segment_points(cloud,MESH_RADIUS);

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
    viz.set_backgroundcolor(255, 255, 255);

}
void slip2next_cloud(const std::string &fname){
    std::cout<<"****************slip*************"<<std::endl;    
    viz.clear();
    PointCloudPtr  cloud(new pcl::PointCloud<PointT>);
    // open an "off" file
    auto range = boost::find_first(fname, ".off");
    if (range.begin()!= range.end()) {
        read_off(fname, cloud);
        std::cout<<"**********off********************"<<std::endl;    
    }
    else
        pcl::io::loadPCDFile(fname, *cloud);

    pcl::PointCloud<PointT>::Ptr cloud2(new pcl::PointCloud<PointT>);
    viz.set_def_cloud(cloud2);
    // add points
    for(auto &p : cloud->points){
        viz.add_point((float)p.x, (float)p.y, (float)p.z, 255, 0, 0, 1); 
    }
    viz.push_def_cloud();
    std::cout<<"*********************************"<<std::endl;    
    std::cout<<cloud->points.size()<<std::endl;
    //    viz.set_backgroundcolor(255, 255, 255);
    //    viz.add_sphere(modelCoefficient[0], modelCoefficient[1], modelCoefficient[2], modelCoefficient[3]);

}
//! cluster points to components, objects, planes
void scene_cluster(const std::string &fname){
    // initial
    viz.clear();
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZRGB>);
    viz.set_def_cloud(cloud);
    PointCloudPtr rgbCloud (new pcl::PointCloud<PointT>);
    // open an "off" file
    auto range = boost::find_first(fname, ".off");
    if (range.begin()!= range.end()) {
        read_off(fname, rgbCloud);
        std::cout<<"**********off********************"<<std::endl;    
    }
    else
        pcl::io::loadPCDFile(fname, *rgbCloud);
    // down sample points
    pcl::PointCloud<PointT>::Ptr cloud_filtered = down_samples(rgbCloud, MESH_RADIUS);
    //    slip2next_cloud(cloud_filtered, RgbColor(255, 0, 0));
    cluster_points(cloud_filtered, &components);

    std::cout<<"*********components num:********"<<std::endl;    
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

    std::cout<<"*********planes num:********"<<std::endl;    
    std::cout<<planes.size()<<std::endl;
    // mark planes green
    for(auto plane : planes){
        for(auto &p: plane->points){
            viz.add_point(p, 0, 255, 0);
        }
    }

    std::cout<<"*********objects num:********"<<std::endl;    
    std::cout<<objects.size()<<std::endl;
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
typedef map<string, ObjWeights> WeightMap;

    WeightMap::value_type
cloud_model(PointCloudPtr cloud, string fname)
{
    // down sample points, if necessary
    pcl::PointCloud<PointT>::Ptr cloud_filtered;
    if (cloud->points.size() > MAX_SIZE_OF_OBJECT) 
        cloud_filtered = down_samples(cloud, 0.002);
    else
        cloud_filtered = cloud;
    ObjWeights obj;
    // build cloud model
    std::cout<<"segment file:"<<fname<<std::endl;
    segment_mesh(cloud_filtered);
    Eye3D::TopoGraph graphicModel;
    PointCloudPtr source(new pcl::PointCloud<PointT>);
    vizMesh2->graphic_model(&graphicModel);
    // get position
    vizMesh2->embedding_graph(source, graphicModel);
    // get weight
    auto nodes = graphicModel.get_all_nodes();
    while (nodes.first != nodes.second) {
        auto &temp = graphicModel.get_node(*nodes.first);
        obj.pWeights.push_back(vizMesh2->weight_propotion(temp));
        obj.dWeights.push_back(vizMesh2->weight_degree(temp));
        obj.tWeights.push_back(vizMesh2->weight_type(temp));
        obj.aWeights.push_back(vizMesh2->weight_angle(temp));
        nodes.first++;
    }
    obj.pos = source;
    return make_pair(fname, obj);
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
        RefineSegManual::_numComp = 11;
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
            std::cout<<"*********************************"<<std::endl;    
            std::cout<<objects.size()<<std::endl;
            // -> size - 1
            // @@snipet
            ++objId;
            RgbColor c(255, 0, 0);
            // find the first element satisfy the requirement
            //            while(objId + 1< objects.size() &&
            //                    objects[objId]->points.size() > 90000) 
            //                objId++;
            //            if(objects[objId]->points.size() <= 90000)
            slip2next_cloud(objects[objId], c);
            std::cout<<"object: "<<objId<<std::endl;
            //            viz.set_zackgroundcolor(255, 255, 255);
        }

    }else if( event.getKeySym() == "p" && event.keyDown()){
        // 0 <-
        if( ::target == "browse objects"){
            RgbColor c(255, 0, 0);
            // filter bigger objects
            while(objId > 0 &&
                    objects[--objId]->points.size() > 900000){
            }; 
            std::cout<<objId<<std::endl;
            if(objects[objId]->points.size() <= 900000)
                slip2next_cloud(objects[objId], c);
        }
    }else if( event.getKeySym() == "i" && event.keyDown()){
        vizMesh2->viz_next_level();
    }else if( event.getKeySym() == "s" && event.keyDown()){
        vizMesh2->viz_previous_level();
    }else if( event.getKeySym() == "c" && event.keyDown()){
        vizMesh2->change_color();
        // add primitive
        if (primitiveId == "" ) {
            auto &cov = ClusterNode::hierarchyTree.get_node(
                    ClusterNode::hierarchyTree.rootId).coefficient;
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
                            cov.direction.x, cov.direction.y,cov.direction.z, cov.radius, 0.2);
                    break;
                default:
                    assert(false);
            }
            std::cout<<"*********************************"<<std::endl;    
            std::cout<<(double)BinaryTree::_currentNode.type<<std::endl;
            std::cout<<(double)BinaryTree::_currentNode.cost<<std::endl;

        }
        else{
            viz.remove_shape(primitiveId);
            primitiveId = "";
        }
    }else if( event.getKeySym() == "space" && event.keyDown()){
        std::string fname = m_util::string_format("object_%d", objId);
        std::vector<float> weights;
        segment_mesh(objects[objId]);
        vizMesh2->viz_next_level();
        viz.reset_camera();
        //        viz.set_backgroundcolor(0,0,0);
        //        PointCloudPtr pos(new pcl::PointCloud<PointT>);
        //        PointCloudPtr source(new pcl::PointCloud<PointT>);
        //        Eye3D::TopoGraph graphicModel;
        //        vizMesh2->graphic_model(&graphicModel);
        //        vizMesh2->embedding_graph(pos, graphicModel);
        //        std::vector<float> wS;
        //        // get weight
        //        auto nodes = graphicModel.get_all_nodes();
        //        while (nodes.first != nodes.second) {
        //            auto &temp = graphicModel.get_node(*nodes.first);
        //            wS.push_back(temp.proportion);
        //            nodes.first++;
        //        }
        //        pcl::copyPointCloud(*pos, *source);
        //        vizMesh2->dynamic_EMD(pos,wS, source, wS);
        viz.set_backgroundcolor(255, 255, 255);

    }else if( event.getKeySym() == "z" && event.keyDown()){
        std::string fname = m_util::string_format("object_%d", objId);
        std::vector<float> weights;
        segment_mesh(objects[objId]);
        PointCloudPtr pos(new pcl::PointCloud<PointT>);
        PointCloudPtr source(new pcl::PointCloud<PointT>);
        std::vector<int> leafs;
        Eye3D::TopoGraph graphicModel;
        vizMesh2->graphic_model(&graphicModel);
        vizMesh2->viz_components(graphicModel);
        // get weight
        auto nodes = graphicModel.get_all_nodes();
        while (nodes.first != nodes.second) {
            auto &temp = graphicModel.get_node(*nodes.first);
            std::cout<<"angle: "<<vizMesh2->weight_angle(temp)<<" "
                <<"degree: "<<vizMesh2->weight_degree(temp)<<" " 
                <<"type: "<<vizMesh2->weight_type(temp)<<" " 
                <<"proportion: "<<vizMesh2->weight_propotion(temp)<<" " 
                <<"dist: "<<vizMesh2->weight_dist(temp)<<std::endl;
            nodes.first++;
        }
        //        vizMesh2->viz_skeleton(graphicModel);
        viz.reset_camera();
        viz.set_backgroundcolor(255, 255, 255);
        // save 
        LDotty<typename Eye3D::TopoGraph> dot2(&graphicModel);
        dot2.write(fname + "leafs.dot", true);
        std::cout<<"write to "<< fname + "leafs.dot"<<std::endl;
        //        RefineSegManual::_numComp += 2;

    }else if( event.getKeySym() == "d" && event.keyDown()){

        std::string fname = m_util::string_format("object_%d", objId);
        pcl::PCDWriter writer; 
        writer.write(fname + ".pcd" , *objects[objId]);
        std::cout<<"save points to "<<fname<<std::endl;

    }else if( event.getKeySym() == "l" && event.keyDown()){
        int  t = 1;
        string sDir;
        string tDir;
        vector<string> dirs;
        std::cout<<"We are going to files:"<<std::endl;
        while(true){
            string temp;
            std::cin >> temp;
            if (temp == "0" ) {
                break;
            }
            dirs.push_back(temp);
        }
        CloudVector tClouds;
        CloudVector sClouds;
        WeightMap fname2weights;
        // load files
        for(auto dir : dirs){
            vector<string> files = list_files(dir);
            for(auto &fname : files){
                pcl::PointCloud<PointT>::Ptr cloud (new pcl::PointCloud<PointT>);
                auto range = boost::find_first(fname, ".off");
                if (range.begin()!= range.end()) 
                    read_off(fname, cloud);
                else
                    pcl::io::loadPCDFile(fname, *cloud);

                // compute the cloud model of object
                fname2weights.insert(cloud_model(cloud, fname));
            }

        }

        std::ofstream log("similarity.log");
        std::ofstream pylog("to_py.log");
        for (int i = 0; i < dirs.size(); i++) {
            string &dirT = dirs[i] ;
            vector<string> filesT = list_files(dirT);
            for (int j = 0; j < dirs.size(); j++) {
                if (i > j)
                    continue;
                string &dirS = dirs[j] ;
                vector<string> filesS = list_files(dirS);
                int all = 0;
                float total = 0;
                float totala = 0;
                float totalp = 0;
                float totald = 0;
                for(auto &fileT : filesT)
                {
                    log << "-------------------------------------------------------------"<<std::endl;
                    pylog << "-------------------------------------------------------------"<<std::endl;
                    auto &wT = fname2weights[fileT];
                    log << fileT << std::endl;
                    pylog << fileT << std::endl;
                    int count = 0;
                    float sum = 0;
                    float suma = 0;
                    float sump = 0;
                    float sumd = 0;
                    /// @todo remove smallest and biggest
                    for(auto &fileS : filesS){
                        auto &wS = fname2weights[fileS];
                        // calculating EMD
                        float p = vizMesh2->dynamic_EMD(wT.pos, wT.pWeights,
                                wS.pos, wS.pWeights);

                        float a = vizMesh2->dynamic_EMD(wT.pos, wT.aWeights,
                                wS.pos, wS.aWeights);

                        float d = vizMesh2->dynamic_EMD(wT.pos, wT.dWeights,
                                wS.pos, wS.dWeights);
                        if (a<0 || p<0 || d<0) {
                            std::cout<<m_util::string_format(
                                    "warning: failed to compare object %s to object %s\n",fileT.c_str(), fileS.c_str());
                            continue;
                        }
                        log<<"emd to file: "<<fileS<<endl
                            <<"propotion:"
                            <<p<<" "
                            <<"degree:"
                            <<d<<" " 
                            <<"angle:"
                            <<a<<" " 
                            <<"total:"
                            <<a+p+d<<std::endl;
                        count++;
                        sum += (a+p+d);
                        suma += a;
                        sump += p;
                        sumd += d;
                        all++;
                    } // end of target file vs. dir

                    assert(count != 0);
                    log << "average: "<< sum/count << std::endl;
                    pylog << sum/count << std::endl;
                    pylog << suma/count << std::endl;
                    pylog << sump/count << std::endl;
                    pylog << sumd/count << std::endl;
                    total += sum;
                    totala += suma;
                    totalp += sump;
                    totald += sumd;
                } //end of target dir vs. source dir
                pylog<<"---------------------------------"<<std::endl;    
                pylog<<total / all<<std::endl
                    <<totala / all<<std::endl
                    <<totalp / all<<std::endl
                    <<totald / all<<std::endl;
                pylog<<dirT<<std::endl
                    <<dirS<<std::endl;
                pylog<<"#################################"<<std::endl;    

                log<<"#################################"<<std::endl;    
                log<<dirT<<std::endl
                    <<dirS<<std::endl;
                log<<"average: "<<total / all<<std::endl;
                log<<"#################################"<<std::endl;    
            } // end of target dir vs. all source dir
            break;
        }

        std::cout<<"Done!"<<std::endl;

    }//end of key trigger



}
int main(int argc, char** argv)
{ 
    fname = std::string(argv[2]);
    pcdFiles = list_files(argv[2]);
    m_util::print_list(pcdFiles);
    std::cout<<argv[2]<<std::endl;
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
    } else if (pcl::console::find_argument(argc, argv, "-o") >= 0){
        ::target = "browse objects";
        // seperate components and objects
        scene_cluster(pcdFiles[0]);
        //        // save objects
        //        for (int i = 0; i < objects.size(); i++) {
        //            pcl::PCDWriter writer; 
        //            fname = m_util::string_format("object_%d", i);
        //            writer.write(fname + ".pcd" , *objects[i]);
        //            std::cout<<"save points to "<<fname<<std::endl;
        //        }
        //        viz.set_backgroundcolor(255, 255, 255);
        viz.set_backgroundcolor(0,0,0);

    } else if (pcl::console::find_argument(argc, argv, "-of") >= 0){
        ::target = "browse objects";
        std::cout<<"*******browsing object:**********"<<std::endl;    
        // load objects from files
        for(auto &fname : pcdFiles){
            pcl::PointCloud<PointT>::Ptr cloud (new pcl::PointCloud<PointT>);
            auto range = boost::find_first(fname, ".off");
            if (range.begin()!= range.end()) 
                read_off(fname, cloud);
            else
                pcl::io::loadPCDFile(fname, *cloud);
            if (cloud->points.size() > MAX_SIZE_OF_OBJECT) {
                // down sample points
                pcl::PointCloud<PointT>::Ptr cloud_filtered = down_samples(cloud, 0.002);
                objects.push_back(cloud_filtered);
            }else{

                objects.push_back(cloud);
            }
        }
        std::cout<<objects.size()<<std::endl;
        viz.set_backgroundcolor(255, 255, 255);
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
