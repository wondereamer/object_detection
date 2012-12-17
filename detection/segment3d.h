/**
 * @file segment3d.h
 * @brief 
 * @author Dingjie.Wang(dingjie.wang@gmail.com)
 * @version 0.1
 * @date 2012-11-22
 */
#ifndef SEGMENT3D_H

#define SEGMENT3D_H
#include "pixelworld3d.h"
#include "pixel.h" 
#include <string>
#include <set>
#include <iterator>
#include "m_opencv.h" 
#include "m_algorithm.h"
#include <iostream>
#include <iterator>
#include "m_opencv.h"
#include "component.h"
#include "data.h"
#include "util.h"
#include "m_util.h"
#include <m_graph.h>
/**
 * @brief 
 */
template < typename T >
class Segment3D:public PixelWorld3D<T> {


    /*---------------------------  lifecycle  ------------------------------------------------ */
    public:

        typedef typename PixelWorld3D<T>::PixelSet PixelSet;
        typedef Component<T*> Component3D;
        typedef WeightEdge<T*> WeightEdge3D;

        struct Region3D{
            PointF3D  _centroid;
            std::vector<T*> _boundary;
            //! color calculated by average color of pixels
            typename T::MeasureColorType _averageColor;
            //! color calculated by vertex coloring algorithm
            m_opencv::RgbColor  _regionColor;
            Component3D *_comp;
        };

        typedef m_graph::VizGraph<Region3D, int> ImageModel;
        explicit Segment3D(bool isGrid = true);
        ~Segment3D(){ };

        /*------------------------------------------------------------------------------------ */
        void segment(int arg = 0);
        /**
         * @brief 
         *
         * @param filename
         * @param optScale: the scale of optimization(defualt: none optimization)
         * @param filterSize: filter component size less than filterSize
         */
        void save(std::string filename, int optScale = 0, int filterSize = 0);
        void _extract_model(int noise);

    protected:
        //
        double _edge_weight(const T &a, const T &b){ return T::density_distance(a,b) ;}
        //
        void _gaussian(){ }

        /*typename PixelSet::iterator get_pixel(int x, int y, int z) const;*/

        /*--------------------  accessor methods  -------------------------------------------- */
    public:

    private:
        std::list<Component3D> _components;
        std::set<Component3D> _noises;
        ImageModel _imgModel;


}; 



    template < typename T >
Segment3D<T>::Segment3D(bool isGrid):PixelWorld3D<T>(isGrid)
{
    /// @todo float to int
    std::vector<float> xList;
    std::vector<float> yList;
    std::vector<float> zList;
    std::vector<int> rList;
    std::vector<int> gList;
    std::vector<int> bList;
    std::vector<int> sizeList;
    std::vector<std::string> materialList;
    std::set<std::string> material_types;
    // set the segmentation scale;
    // get blocks from data server
    get_block_attrs(xList, yList, zList, rList, gList, bList, materialList, sizeList, "get_block_attrs", DATA_SERVER_PORT);
    // create 3dpixel pool
    for (int i = 0; i < xList.size(); i++) {
        material_types.insert(materialList[i]);
        RgbColor color;
        color.r = rList[i];
        color.g = gList[i];
        color.b = bList[i];
        this->_pixels.insert(T(color, xList[i], yList[i], zList[i], sizeList[i]));

    }
    // tracing code
    std::copy(material_types.begin(), material_types.end(),    
            std::ostream_iterator<std::string>(std::cout, "\n"));
    std::cout<<material_types.size()<<std::endl;
    std::cout<<"size of blockworld: "<<xList.size()<<std::endl;

}


template < typename T >
void Segment3D<T>::segment(int arg ){

    Component3D::K = arg;
    std::set<WeightEdge3D> graph;
    // construt the graph, sort edges in nondecreasing order
    for(typename PixelSet::iterator p = this->_pixels.begin(); p != this->_pixels.end(); p++){
        T *t = const_cast<T*>(&(*p));
        // get neighbor pixels, it could be at most 8 nearest grid pixel,
        // or nearest neighbors in the feature space
        std::vector<T*> neighbors = get_neighbors(*t);
        for(T *nb : neighbors){
            if( graph.find(WeightEdge3D(_edge_weight(*nb, *t), nb, t)) != graph.end() )
                // exist symmetrical edge
                continue;
            // create weight edges
            graph.insert(WeightEdge3D(_edge_weight(*t, *nb), t, nb));

        }
    }


    std::cout<<"size of graph:"<<graph.size()<<std::endl;
    // statics
    float com_com = 0;
    float pixel_pixel = 0;
    float com_pixel = 0;
    float a_com_com = 0;
    float a_pixel_pixel = 0;
    float a_com_pixel = 0;
    double count = 0;
    int display = 0;
    double lengthComponents = 0;
    // create and merge components from edge list
    // as the number of components is always not small, means the convertion from
    // Pixel to Component won't cost that much, the total number of covertions
    // will be num(pixels) + max length of components;
    for(const WeightEdge3D &edge : graph){
        count++;
        display++;
        if(display > 1000){
            // output the number of edges dealed with every 1000
            std::cout<<count<<std::endl;
            display = 0;
        }
        auto i_b = _components.end();    
        auto i_e = _components.end();    
        // for every pixel
        for (auto i = _components.begin(); i != _components.end() ; i++) {
            if( i->contains(edge._b)){
                i_b = i;
                continue;
            }
            if( i->contains(edge._e)){
                i_e = i;
                continue;
            }
            if (i_b != _components.end() && i_e != _components.end() )
                break;
        }
        if (i_b != _components.end() && i_e != _components.end()) {
            // both component exist
            int rst = Component3D::merge(edge._weight, *i_b, *i_e);
            if (rst == 0){
                // have merged i_e to i_b, so remove component i_e
                // from #_components list.
                _components.erase(i_e);
                com_com++;
            }
            else if(rst == 1){
                _components.erase(i_b);
                com_com++;
            }
            a_com_com++;
        }
        else if( i_b == _components.end() && i_e == _components.end()) {
            // merge two pixels
            Component3D tempB(edge._b);
            Component3D tempE(edge._e);
            int rst = Component3D::merge(edge._weight, tempB, tempE);
            // the only place insert elements to #_components
            if(rst == 0){
                _components.push_back(tempB);
                lengthComponents++;
            }
            else if(rst == 1){
                _components.push_back(tempE);
                lengthComponents++;
            }else{
                // failed to merge two pixel
                // add two single pixel components
                // single pixel component is actually treated as
                // noise, and would be merged to other components during later 
                // optimizing
                /*_noises.insert(tempB);*/
                /*_noises.insert(tempE);*/

            }
        }
        else if( i_b == _components.end()){
            // merge pixel edge._b to component i_e
            Component3D temp(edge._b);
            if(Component3D::merge(edge._weight, temp, *i_e) == -1);
            /*_noises.insert(temp);*/
        }
        else{
            // merge pixel edge._e to component i_b
            Component3D temp(edge._e);
            if(Component3D::merge(edge._weight, *i_b, temp) == -1);
            /*_noises.insert(temp);*/
        }

    }

    std::cout<<"**************size of _components: "<<_components.size()<<std::endl;
    for(Component3D &comp: _components){
        std::cout<<"[ "<< comp.get_members().size()<<" ]";
    }
    std::cout<<std::endl;
    std::cout<<"length of components:"<<lengthComponents<<std::endl;
    std::cout<<std::endl;



}

/**
 * @brief extract graph model from image
 *
 * @tparam T
 * @param noise: filter out small components whose size less than #noise
 */
template < typename T >
void Segment3D<T>::_extract_model(int noise){
    // construt region graph, vertexs are segmentation components
    // edge between two vertexs, means two components are adjacent
    std::cout<<"extracting graph model...."<<std::endl;
    std::multiset<double> weightSet;
    typedef std::map<Component3D*, typename ImageModel::NodeH> CompNodeMap;
    CompNodeMap comp2node;
    // data related to current region
    Region3D region;
    double centr_x = 0;
    double centr_y = 0;
    double centr_z = 0;
    for(auto &comp : _components){
        // fill out small component whose size less than noise 
        // get an smaller graph model
        if(comp.size() < noise)
            continue;
        // get or create an handle of current region in the  model
        typename CompNodeMap::iterator i = comp2node.find(&comp);
        if(i == comp2node.end()){
            // for visualisation
            int size = comp.get_members().size() * 0.1 ;
            size = size == 0? 5: size;
            //
            comp2node[&comp] = _imgModel.add_node(size);
        }
        typename ImageModel::NodeH regH = comp2node[&comp];
        for(T *pixel : comp.get_members()){
            // calculating centroid
            centr_x += pixel->_x;
            centr_y += pixel->_y;
            centr_z += pixel->_z;
            // calculating average color 
            T::MeasureColorType::add2colorpool(pixel->get_measure_color());
            //
            auto neighbors = get_neighbors(*pixel);
            // this pixel is on the boundary
            bool pushed2bounary = false;
            if( neighbors.size() < 8){
                region._boundary.push_back(pixel);
                pushed2bounary = true;
            }
            // find adjacent regions
            for(T *nb : neighbors){
                if(!comp.contains(nb)){
                    for(auto &nbComp : _components){
                        if(nbComp.contains(nb)){
                            // filter out small componets
                            if(nbComp.size() < noise)
                                break;
                            // get or create neighbor handle
                            typename CompNodeMap::iterator i = comp2node.find(&nbComp);
                            if(i == comp2node.end()){
                                // for visualisation
                                int size = nbComp.get_members().size() * 0.1;
                                size = size == 0? 5: size;
                                //
                                comp2node[&nbComp] = _imgModel.add_node(size);
                            }
                            // add adjacent relation to image model
                            _imgModel.add_edge(regH, comp2node[&nbComp]);
                            break;
                        }
                    }
                    // some neighbor is in different component, 
                    // so this pixel is on the boundary
                    if(!pushed2bounary){
                        region._boundary.push_back(pixel);
                        pushed2bounary = true;
                    }
                }
            }
        } //end of pixel iterate
        // set current region property
        centr_x /= comp.size();
        centr_y /= comp.size();
        centr_z /= comp.size();
        region._centroid = PointF3D(centr_x, centr_y, centr_z);
        region._comp = &comp;
        region._averageColor = T::MeasureColorType::calcu_average_color(comp.size());
        _imgModel.set_node_attrs(regH, region);
    } //end of component iterate
    // set position of nodes for visualisation 
    // and weight of edges
    std::vector<typename ImageModel::EdgeH> edges;
    _imgModel.all_edges(edges);
    for(auto edge : edges){
        // set postion of nodes
        Region3D &s = _imgModel.get_node_attrs(edge->source());
        Region3D &t = _imgModel.get_node_attrs(edge->target());
        /*_imgModel.set_node_pos(edge->source(), s._centroid.x, s._centroid.y);*/
        /*_imgModel.set_node_pos(edge->target(), t._centroid.x, t._centroid.y);*/
        // set weight of edges
        double weight = T::MeasureColorType::color_distance(s._averageColor, t._averageColor);
        weightSet.insert(weight);
        _imgModel.set_edge_weight(edge, weight);
    }
    std::cout<<"******** wight of edges ********" <<std::endl;
    for(auto w : weightSet){
        std::cout<<w<<" ";
    }
    std::cout<<std::endl;
}
template < typename T >
void Segment3D<T>::save(std::string filename, int optScale, int filterSize){

    // extract graph model from components of segmentation
    _extract_model(filterSize);

//    // optimizing model
//    _optimze_model(optScale);

    // coloring components of segmentation
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
//    strColor2color["#66BBAE"] = components_color()[0];
//    strColor2color["#8FBC8F"] = components_color()[1];
//    strColor2color["#9DD4FF"] = components_color()[2];
//    strColor2color["#D53533"] = components_color()[3];
//    strColor2color["#509467"] = components_color()[4];
//    strColor2color["#A6CD1B"] = components_color()[5];
//    strColor2color["#ED9F9F"] = components_color()[6];
//    strColor2color["#373A7F"] = components_color()[7];
//    if(!_imgModel.vertex_coloring(colormap, vertex2strColor))
//        std::cout<<"Failed to color vertex!!!"<<std::endl;
//
//    // mark regions with different colors
//    std::cout<<"mark regions with different colors..."<<std::endl;
//    std::vector<typename ImageModel::NodeH> nodeHandles;
//    _imgModel.all_nodes(nodeHandles);
//    int index = 0;
//    for(auto node: nodeHandles){
//        Region2D &region = _imgModel.get_node_attrs(node);
//        region._regionColor = strColor2color[vertex2strColor[index++]];
//        for(auto *pixel : region._comp->get_members()){
//            img[pixel->_y][pixel->_x] = region._regionColor;
//        }
//
//    }
 
//    m_util::RpcServer server(DETECTION_SERVER_PORT);
//    xmlrpc_c::methodPtr const originMethod(new OriginMethod<T>(this->_pixels));
//    xmlrpc_c::methodPtr const segMethod(new SegMethod<T>(_components));
//    server.register_method(originMethod, "get_block_attrs");
//    server.register_method(segMethod, "get_segmentation_result");
//    server.run();

    _imgModel.write("3d_graph_model");
}

/*template < typename T >*/
/*std::vector<T*> Segment3D<T>::_feature_neighbors(const T &t) const{*/
/*return std::vector<T*>();*/
/*}*/

#endif /* end of include guard: SEGMENT3D_H */
