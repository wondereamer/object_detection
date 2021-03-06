/**
 * @file segment2d.h
 * @brief 
 * @author Dingjie.Wang(dingjie.wang@gmail.com)
 * @version 0.1
 * @date 2012-11-22
 */

#ifndef SEGMENT2D_H

#define SEGMENT2D_H

#include "pixelworld2d.h" 
#include "component.h"
#include <algorithm>
#include <list>
#include <iterator>
#include <set>
#include <iostream>
#include <numeric>
#include <map>
#include <library/m_algorithm.h>
#include <library/m_graph.h>
#include <algorithm>
#include "util.h" 
#define  PointF m_geometry::PointF
/**
 * @brief 
 */
template < typename T >
class Segment2D: public PixelWorld2D<T> {

    /*---------------------------  lifecycle  ------------------------------------------------ */
    public:
        typedef Component<T*> Component2D;
        typedef WeightEdge<T*> WeightEdge2D;

        struct Filter : public unary_function<Component2D, bool> {
            Filter(int n = 1):_scale(n){ }
            bool  operator()(const Component2D& arg1) const{
                return arg1.size() < _scale;
            };
            int _scale;
        };

        struct Region2D{
            PointF  _centroid;
            std::vector<T*> _boundary;
            //! color calculated by average color of pixels
            typename T::MeasureColorType _averageColor;
            //! color calculated by vertex coloring algorithm
            m_opencv::RgbColor  _regionColor;
            Component2D *_comp;
        };
        typedef m_graph::VizGraph<Region2D, int> ImageModel;
        explicit Segment2D(std::string filename, bool isGrid = true);
        virtual ~Segment2D(){ };

        /*------------------------------------------------------------------------------------ */
        std::vector<T*> get_neighbors(const T &t) const;
        void segment(int arg = 70);

        /**
         * @brief 
         *
         * @param filename
         * @param optScale: the scale of optimization(defualt: none optimization)
         * @param filterSize: filter component size less than filterSize
         */
        void save(std::string filename, int optScale = 0, int filterSize = 0);
        void filter(int noise = 1);
        void model_info( );

    protected:
        //
        double _edge_weight(const T &a, const T &b){ return T::density_distance(a,b) ;}
        //
        void _gaussian(){ }

        /**
         * @brief 
         *
         * @param noise filter out components whose size less than #noise
         */
        void _extract_model(int noise = 1);
        void _optimze_model(int scale );

        bool _is_inside(int x, int y) const;
        std::vector<T*> _grid_neighbors(const T &t) const;
        std::vector<T*> _feature_neighbors(const T &t) const;

        /*-----------------------  attributes  ------------------------------------------------ */

    private:
        bool            _isGrid;
        std::vector<Component2D> _components;
        std::set<Component2D> _noises;
        ImageModel _imgModel;

}; 

    template < typename T >
Segment2D<T>::Segment2D(std::string filename, bool isGrid):PixelWorld2D<T>(filename),
    _imgModel(false)
{
    _isGrid = isGrid;

}
//
template < typename T >
inline std::vector<T*> Segment2D<T>::get_neighbors(const T &t) const{
    return _isGrid ? _grid_neighbors(t) :  _feature_neighbors(t);
}
template < typename T >
inline bool Segment2D<T>::_is_inside(int x, int y) const{
    // when the neighbor is out of image, return force with strength zero
    if (x >= 0 && x < this->_width && y >= 0 && y < this->_height) 
        return true;
    return false;
}
template < typename T >
inline std::vector<T*> Segment2D<T>::_grid_neighbors(const T &t) const{
    // push 8 nearest neighbors
    std::vector<T*> neighbors;
    if(_is_inside(t._x - 1, t._y - 1))
        neighbors.push_back(&this->_pixels[t._y - 1][t._x - 1]);
    if(_is_inside(t._x, t._y - 1))
        neighbors.push_back(&this->_pixels[t._y - 1][t._x]);
    if(_is_inside(t._x + 1, t._y - 1))
        neighbors.push_back(&this->_pixels[t._y - 1][t._x + 1]);

    if(_is_inside(t._x - 1, t._y ))
        neighbors.push_back(&this->_pixels[t._y][t._x - 1]);
    if(_is_inside(t._x + 1, t._y))
        neighbors.push_back(&this->_pixels[t._y][t._x + 1]);

    if(_is_inside(t._x + 1, t._y + 1))
        neighbors.push_back(&this->_pixels[t._y + 1][t._x + 1]);
    if(_is_inside(t._x , t._y + 1))
        neighbors.push_back(&this->_pixels[t._y + 1][t._x]);
    if(_is_inside(t._x - 1, t._y + 1))
        neighbors.push_back(&this->_pixels[t._y + 1][t._x - 1]);
    return neighbors;

}




template < typename T >
void Segment2D<T>::segment(int arg){
    Component2D::K = arg;
    // map pixel to it's component 
    std::set<WeightEdge2D> graph;
    // construt the graph, sort edges in nondecreasing order
    for (int row = 0; row < this->_height; row++) 
        for (int col = 0; col < this->_width; col++){
            T *t = &this->_pixels[row][col];
            // get neighbor pixels, it could be at most 8 nearest grid pixel,
            // or nearest neighbors in the feature space
            std::vector<T*> neighbors = get_neighbors(*t);
            for(T *nb : neighbors){
                if( graph.find(WeightEdge2D(_edge_weight(*nb, *t), nb, t)) != graph.end() )
                    // exist symmetrical edge
                    continue;
                // create weight edges
                graph.insert(WeightEdge2D(_edge_weight(*t, *nb), t, nb));
            }
        }
    std::cout<<"size of graph:"<<graph.size()<<std::endl;
    m_util::EveryDisplay display(1000);
    // create and merge components from edge list
    // as the number of components is always small, means the convertion from
    // Pixel to Component won't cost that much, the total number of covertions
    // will be num(pixels) + max length of components;
    for(const WeightEdge2D &edge : graph){
        // for every pixel
        display.every_display();
        auto *compOfb = edge._b->_parent;
        auto *compOfe = edge._e->_parent;
        // both component exist
        assert(edge._b != edge._e);
        if (compOfb && compOfe ) {
//            assert(compOfb->size() >=2 && compOfe->size() >= 2);
            if(compOfb == compOfe)
                continue;
            Component2D::merge(edge._weight, *compOfb, *compOfe);
//            assert(compOfb->size() >=2 || compOfe->size() >= 2);
        } else if( !compOfb  && !compOfe) {
            // merge two pixels
            Component2D tempB(edge._b);
            Component2D tempE(edge._e);
            int rst = Component2D::merge(edge._weight, tempB, tempE);
            // the only place insert elements to #_components
            if(rst == 0){
                _components.push_back(tempB);
//                assert(edge._b->_parent == &_components.back());
//                assert(edge._e->_parent == &_components.back());
//                assert(_components.back().size() == 2);
            }
            else if(rst == 1){
                _components.push_back(tempE);
//                assert(edge._b->_parent == &_components.back());
//                assert(edge._e->_parent == &_components.back());
//                assert(_components.back().size() == 2);
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
        else if( !compOfb){
            // merge pixel edge._b to component compOfe
            Component2D temp(edge._b);
            int t = compOfe->size();
//            if(Component2D::merge(edge._weight, *compOfe, temp) != -1)
                /*_noises.insert(temp);*/
//                assert(compOfe->size() == t + 1 && compOfe->size() > 2);
                Component2D::merge(edge._weight, *compOfe, temp);
        }
        else{
            // merge pixel edge._e to component compOfb
            Component2D temp(edge._e);
//            if(Component2D::merge(edge._weight, *compOfb, temp) == -1);
            /*_noises.insert(temp);*/
            Component2D::merge(edge._weight, *compOfb, temp);
        }

    }


}
template < typename T >
void Segment2D<T>::model_info(){
    int size = 0;
    for(Component2D &comp: _components){
        std::cout<<"[ "<< comp.get_members().size()<<" ]";
        size += comp.get_members().size();
    }
    std::cout<<std::endl<<"************size of _components: "<<_components.size()<<std::endl;
    std::cout<<"missing pixels: "<<this->_width * this->_height - size<<std::endl;
    std::cout<<"move num: "<<Component2D::MOVE_NUM<<std::endl;
    std::cout<<std::endl;
}
template < typename T >
std::vector<T*> Segment2D<T>::_feature_neighbors(const T &t) const{
    return std::vector<T*>();
}
template < typename T >
void Segment2D<T>::filter(int noise){
    // fill out small component whose size less than noise 
    // get an smaller graph model
    std::cout<<"Filtering regions whose size less than "<<noise<<std::endl;
    Filter filter(noise);
    auto end = std::remove_if(_components.begin(), _components.end(), filter);
    _components.resize(end - _components.begin());
}

/**
 * @brief extract graph model from image
 *
 * @tparam T
 * @param noise: filter out small components whose size less than #noise
 */
template < typename T >
void Segment2D<T>::_extract_model(int noise){
    // construt region graph, vertexs are segmentation components
    // edge between two vertexs, means two components are adjacent
    filter(noise);
    model_info( );
    std::cout<<"extracting graph model...."<<std::endl;
    std::multiset<double> weightSet;
    typedef std::map<Component2D*, typename ImageModel::NodeH> CompNodeMap;
    CompNodeMap comp2node;
    // data related to current region
    Region2D region;
    double centr_x = 0;
    double centr_y = 0;
    m_util::EveryDisplay display(50);
    for(auto &comp : _components){
        display.every_display(); 
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
        /*std::set<*/
        for(T *pixel : comp.get_members()){
            // calculating centroid
            centr_x += pixel->_x;
            centr_y += pixel->_y;
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
        region._centroid = PointF(centr_x, centr_y);
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
        Region2D &s = _imgModel.get_node_attrs(edge->source());
        Region2D &t = _imgModel.get_node_attrs(edge->target());
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
void Segment2D<T>::_optimze_model(int scale){
    std::cout<<"optimizing graph model"<<std::endl;
    std::vector<typename ImageModel::EdgeH> edges;
    // if merge happens, end up with an new model
    bool new_model = true;
    // to trace to influence of optimizing
    std::multiset<double> averColorChange;
    std::multiset<double> mergedWeight;
    while(new_model){
        new_model = false;
        edges.clear();
        _imgModel.sorted_edges(edges);
        /*std::sort(edges.begin(), edges.end());*/
        // begin one edge mergeing
        for(auto edge : edges){
            if(_imgModel.get_edge_weight(edge) < scale){
                // merge nodes of the edge
                Region2D &s = _imgModel.get_node_attrs(edge->source());
                Region2D &t = _imgModel.get_node_attrs(edge->target());
                // merge smaller to components to bigger component;
                if(t._comp->size() > s._comp->size()){
                    t._comp->merge_from(*s._comp);
                    double centr_x = 0;
                    double centr_y = 0;
                    for(auto p : t._comp->get_members()){
                        centr_x += p->_x;
                        centr_y += p->_y;
                        T::MeasureColorType::add2colorpool(p->get_measure_color());
                    }
                    centr_x /= t._comp->size();
                    centr_y /= t._comp->size();
                    t._centroid = PointF(centr_x, centr_y);
                    auto temp = T::MeasureColorType::calcu_average_color(t._comp->size());
                    double change = T::MeasureColorType::color_distance(temp, t._averageColor);
                    averColorChange.insert(change);
                    t._averageColor = temp;

                    //
                    typename ImageModel::NodeH nodeToDelete = edge->source();
                    std::vector<typename ImageModel::NodeH> neighborNodes;
                    _imgModel.adjacency_nodes(nodeToDelete, neighborNodes);
                    // remove one component node in graph model
                    _imgModel.remove_node(nodeToDelete);
                    // add edges between neighbors of deleted node and bigger component
                    for(auto nh : neighborNodes){
                        if(nh != edge->target()){
                            typename ImageModel::EdgeH eh = _imgModel.add_edge(edge->target(), nh);
                            Region2D& nb = _imgModel.get_node_attrs(nh);
                            double weight = T::MeasureColorType::color_distance(nb._averageColor, t._averageColor);
                            _imgModel.set_edge_weight(eh, weight);
                            mergedWeight.insert(weight);
                        }
                    }

                }else{
                    s._comp->merge_from(*t._comp);
                    double centr_x = 0;
                    double centr_y = 0;
                    for(auto p : s._comp->get_members()){
                        centr_x += p->_x;
                        centr_y += p->_y;
                        T::MeasureColorType::add2colorpool(p->get_measure_color());
                    }
                    centr_x /= s._comp->size();
                    centr_y /= s._comp->size();
                    s._centroid = PointF(centr_x, centr_y);
                    auto temp = T::MeasureColorType::calcu_average_color(s._comp->size());
                    double change = T::MeasureColorType::color_distance(temp, s._averageColor);
                    averColorChange.insert(change);
                    s._averageColor = temp;

                    //
                    typename ImageModel::NodeH nodeToDelete = edge->target();
                    std::vector<typename ImageModel::NodeH> neighborNodes;
                    _imgModel.adjacency_nodes(nodeToDelete, neighborNodes);
                    // remove one component node in graph model
                    _imgModel.remove_node(nodeToDelete);
                    // add edges between neighbors of deleted node and bigger component
                    for(auto nh : neighborNodes){
                        if(nh != edge->source()){
                            typename ImageModel::EdgeH eh = _imgModel.add_edge(edge->source(), nh);
                            Region2D& nb = _imgModel.get_node_attrs(nh);
                            double weight = T::MeasureColorType::color_distance(nb._averageColor, s._averageColor);
                            _imgModel.set_edge_weight(eh, weight);
                            mergedWeight.insert(weight);
                        }
                    }

                }
                new_model = true;
                break;

            } //end of merge (if)

        } //end of edge iterate
    }
    std::cout<<std::endl<<"**********color change****************"<<std::endl;
    for(auto t : averColorChange){
        std::cout<<t<<" ";
    }
    std::cout<<std::endl<<"***********new weight***************"<<std::endl;
    for(auto t : mergedWeight){
        std::cout<<t<<" ";
    }
    std::cout<<std::endl<<"**************************"<<std::endl;

}

template < typename T >
void Segment2D<T>::save(std::string filename, int optScale, int filterSize){
    // create an white color image
    CvSize size;
    size.width = this->_width;
    size.height = this->_height;
    IplImage* temp  = cvCreateImage(size, IPL_DEPTH_8U, 3);
    m_opencv::RgbImage img(temp);
    m_opencv::RgbColor white = m_opencv::RgbColor::white_color();
    img.set_color(white);
    ///// @todo ... have the ability to save segmentation result here
    // extract graph model from components of segmentation
    _extract_model(filterSize);

    // optimizing model
    _optimze_model(optScale);
    /// @todo model info is in _imgModel here, not _components
    
    model_info();

    // coloring components of segmentation
    if(!vertex_coloring(_imgModel)){
        std::cout<<"Failed to color vertex!!!"<<std::endl;
        return;
    }

    // copy pixels from pixelWorld to image
    std::vector<typename ImageModel::NodeH> nodeHandles;
    _imgModel.all_nodes(nodeHandles);
    for(auto node: nodeHandles){
        Region2D &region = _imgModel.get_node_attrs(node);
        for(auto *pixel : region._comp->get_members()){
            img[pixel->_y][pixel->_x] = region._regionColor;
        }

    }

    _imgModel.write("hello");
    /*this->show();*/
    img.show();
    img.save(filename + ".jpg");
}

#endif /* end of include guard: SEGMENT2D_H */
