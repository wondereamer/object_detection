#ifndef UTIL_H
#define UTIL_H
#include <getopt.h>
#include <langinfo.h>
#include <locale.h>

#include <boost/filesystem/operations.hpp>

#include <boost/algorithm/string.hpp>
#include <boost/foreach.hpp>

#include <utility>
#include <string>

#include <pcl/common/common_headers.h>
#include <pcl/features/normal_3d.h>
#include <pcl/io/pcd_io.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/console/parse.h>
#include <xmlrpc-c/base.hpp>
#include <xmlrpc-c/registry.hpp>
#include <xmlrpc-c/server_abyss.hpp>
#include <vector>
#include <cassert>
#include <stdlib.h>
#include "m_util.h" 
#include "component.h" 
#include <list>
#include <m_opencv.h>
#include <m_graph.h>
#include <map>
#include <iostream>
#include <flann/flann.hpp>
#include <flann/io/hdf5.h>

using m_util::sth2string;

std::vector<m_opencv::RgbColor>& components_color( );

template < typename T >
bool vertex_coloring(m_graph::VizGraph<T, int> &imgModel){
    typedef m_graph::VizGraph<T, int> ImageModel;
    std::map<int, std::string> colormap;
    std::map<int, std::string> vertex2strColor;
    std::map<std::string, m_opencv::RgbColor> strColor2color;
    colormap[0] = "#66BBAE";
    colormap[1] = "#8FBC8F";
    colormap[2] = "#9DD4FF";
    colormap[3] = "#D53533";
    colormap[4] = "#509467";
    colormap[5] = "#A6CD1B";
    colormap[6] = "#ED9F9F";
    colormap[7] = "#373A7F";
    strColor2color["#66BBAE"] = components_color()[0];
    strColor2color["#8FBC8F"] = components_color()[1];
    strColor2color["#9DD4FF"] = components_color()[2];
    strColor2color["#D53533"] = components_color()[3];
    strColor2color["#509467"] = components_color()[4];
    strColor2color["#A6CD1B"] = components_color()[5];
    strColor2color["#ED9F9F"] = components_color()[6];
    strColor2color["#373A7F"] = components_color()[7];
    // mark regions with different colors
    std::cout<<"mark regions with different colors..."<<std::endl;
    if(!imgModel.vertex_coloring(colormap, vertex2strColor))
        return false;

    std::vector<typename ImageModel::NodeH> nodeHandles;
    imgModel.all_nodes(nodeHandles);
    int index = 0;
    for(auto node: nodeHandles){
        T &region = imgModel.get_node_attrs(node);
        region._regionColor = strColor2color[vertex2strColor[index++]];
        for(auto *pixel : region._comp->get_members()){
            pixel->_color = region._regionColor;
        }

    }
    return true;
}


/**
 * @brief return block datas from data server
 *
 * @tparam T
 */
template < typename T >
class OriginMethod: public xmlrpc_c::method {
    public:
        OriginMethod(const std::set<T> &pixels):_pixels(pixels){ 

            std::vector<xmlrpc_c::value> position_array;
            std::vector<xmlrpc_c::value> size_array;
            std::vector<xmlrpc_c::value> color_array;
            std::vector<xmlrpc_c::value> material_array;
            for(const T &block : _pixels){
                position_array.push_back(xmlrpc_c::value_string(sth2string<int>(block._x) + "," + sth2string<int>(block._y) + "," + sth2string<int>(block._z)));
                size_array.push_back(xmlrpc_c::value_int(block._size));
                color_array.push_back(xmlrpc_c::value_string(sth2string<int>(block.get_color().r) + "," + sth2string<int>(block.get_color().g) 
                            + "," + sth2string<int>(block.get_color().b)));
                material_array.push_back(xmlrpc_c::value_string(""));
            }

            std::vector<xmlrpc_c::value> ttt;
            ttt.push_back(xmlrpc_c::value_array(position_array));
            ttt.push_back(xmlrpc_c::value_array(size_array));
            ttt.push_back(xmlrpc_c::value_array(material_array));
            ttt.push_back(xmlrpc_c::value_array(color_array));
            _rst = xmlrpc_c::value_array(ttt);

        }
        void execute(xmlrpc_c::paramList const& paramList,
                xmlrpc_c::value *   const  retvalP) {
            *retvalP = _rst;
            std::cout<<"Having sent origin data."<<std::endl;
        }
    private:
        xmlrpc_c::value _rst;
        const std::set<T> &_pixels;
};

template < class T >
void nearest_k_search2(std::vector<T> &points, const T& searchPoint, int nn = 10, int dim = 3){


    flann::Matrix<float> dataset;
    flann::Matrix<float> query;
    dataset =  flann::Matrix<float>(new float[points.size() * dim], points.size(), dim);
    //
    flann::Matrix<int> indices(new int[query.rows*nn], query.rows, nn);
    flann::Matrix<float> dists(new float[query.rows*nn], query.rows, nn);

    // construct an randomized kd-tree index using 4 kd-trees
    flann::Index<flann::L2<float> > index(dataset, flann::KDTreeIndexParams(4));
    index.buildIndex();                                                                                               
//
//    // do a knn search, using 128 checks
//    index.knnSearch(query, indices, dists, nn, flann::SearchParams(128));
//
//    flann::save_to_file(indices,"result.hdf5","result");
//
//    delete[] dataset.ptr();
//    delete[] query.ptr();
//    delete[] indices.ptr();
//    delete[] dists.ptr();
    
}

void mouseEventOccurred (const pcl::visualization::MouseEvent &event,
        void* viewer_void);

void keyboardEventOccurred (const pcl::visualization::KeyboardEvent &event,
        void* viewer_void);
#endif /* end of include guard: UTIL_H */
