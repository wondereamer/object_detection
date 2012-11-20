/**
 * @file pixelworld.h
 * @brief compute attraction force
 * @author Dignjie.Wang (dingjie.wang@gmail.com)
 * @version 0.1
 * @date 2012-10-10
 */
#ifndef PIXElWORLD3D_H
#define PIXELWORLD3D_H
#include <string>
#include <set>
#include "pixel.h" 
#include <algorithm>
#include <list>
#include <iterator>
#include "m_opencv.h" 
#include "m_algorithm.h"
#include <numeric>
#include <iostream>
#include <iterator>
#include <cassert>
#include <map>
#include "m_opencv.h"
#include "component.h"
#include <data.h>
#include "segmentation_server.h"
#include "m_util.h"
#include <string>


using std::string;
/**
 * @brief 
 */
template < typename T >
class PixelWorld3D {


    /*---------------------------  lifecycle  ------------------------------------------------ */
    public:
        typedef std::set<T> PixelSet;
        typedef Component<T*> Component3D;
        typedef WeightEdge<T*> WeightEdge3D;
        explicit PixelWorld3D(bool isGrid = true);
        ~PixelWorld3D(){ };

        /*------------------------------------------------------------------------------------ */
        T* operator[](int rowIndx) { return _pixels[rowIndx]; }
        const T* operator[](int rowIndx)const { return _pixels[rowIndx]; }
        std::vector<T*> get_neighbors(const T &t) const;
        void construct_graph();
        void save_segmentation( );

    protected:
        //
        double _edge_weight(const T &a, const T &b){ return T::density_distance(a,b) ;}
        //
        void _gaussian(){ }

        /*typename PixelSet::iterator get_pixel(int x, int y, int z) const;*/
        std::vector<T*> _grid_neighbors(const T &t) const;
        std::vector<T*> _feature_neighbors(const T &t) const;

        /*--------------------  accessor methods  -------------------------------------------- */
    public:

    private:
        bool            _isGrid;
        PixelSet     _pixels;    
        std::list<Component3D> _components;


}; 

//
template < typename T >
inline std::vector<T*> PixelWorld3D<T>::get_neighbors(const T &t) const{
    return _isGrid ? _grid_neighbors(t) :  _feature_neighbors(t);
}
/*template < typename T >*/
/*inline typename PixelSet::iterator PixelWorld3D<T>::get_pixel(int x, int y, int z) const{*/
/*return _pixels.find(T(x, y, z));*/
/*}*/
template < typename T >
inline std::vector<T*> PixelWorld3D<T>::_grid_neighbors(const T &t) const{
    // push 8 nearest neighbors
    std::vector<T*> neighbors;
    typename PixelSet::iterator i;
    // the middle plane
    i = _pixels.find(T(t._x - 1, t._y - 1, t._z));
    if( i != _pixels.end())
        neighbors.push_back(const_cast<T*>(&(*i)));

    i = _pixels.find(T(t._x, t._y - 1, t._z));
    if( i != _pixels.end())
        neighbors.push_back(const_cast<T*>(&(*i)));

    i = _pixels.find(T(t._x + 1, t._y - 1, t._z));
    if( i != _pixels.end())
        neighbors.push_back(const_cast<T*>(&(*i)));

    i = _pixels.find(T(t._x - 1, t._y, t._z));
    if( i != _pixels.end())
        neighbors.push_back(const_cast<T*>(&(*i)));

    i = _pixels.find(T(t._x + 1, t._y, t._z));
    if( i != _pixels.end())
        neighbors.push_back(const_cast<T*>(&(*i)));

    i = _pixels.find(T(t._x + 1, t._y + 1, t._z));
    if( i != _pixels.end())
        neighbors.push_back(const_cast<T*>(&(*i)));

    i = _pixels.find(T(t._x, t._y + 1, t._z));
    if( i != _pixels.end())
        neighbors.push_back(const_cast<T*>(&(*i)));

    i = _pixels.find(T(t._x - 1, t._y + 1, t._z));
    if( i != _pixels.end())
        neighbors.push_back(const_cast<T*>(&(*i)));

    // the top plane

    i = _pixels.find(T(t._x - 1, t._y - 1, t._z + 1));
    if( i != _pixels.end())
        neighbors.push_back(const_cast<T*>(&(*i)));

    i = _pixels.find(T(t._x, t._y - 1, t._z + 1));
    if( i != _pixels.end())
        neighbors.push_back(const_cast<T*>(&(*i)));

    i = _pixels.find(T(t._x + 1, t._y - 1, t._z + 1));
    if( i != _pixels.end())
        neighbors.push_back(const_cast<T*>(&(*i)));

    i = _pixels.find(T(t._x - 1, t._y, t._z + 1));
    if( i != _pixels.end())
        neighbors.push_back(const_cast<T*>(&(*i)));

    i = _pixels.find(T(t._x , t._y, t._z + 1));
    if( i != _pixels.end())
        neighbors.push_back(const_cast<T*>(&(*i)));

    i = _pixels.find(T(t._x + 1, t._y, t._z + 1));
    if( i != _pixels.end())
        neighbors.push_back(const_cast<T*>(&(*i)));

    i = _pixels.find(T(t._x + 1, t._y + 1, t._z + 1));
    if( i != _pixels.end())
        neighbors.push_back(const_cast<T*>(&(*i)));

    i = _pixels.find(T(t._x, t._y + 1, t._z + 1));
    if( i != _pixels.end())
        neighbors.push_back(const_cast<T*>(&(*i)));

    i = _pixels.find(T(t._x - 1, t._y + 1, t._z + 1));
    if( i != _pixels.end())
        neighbors.push_back(const_cast<T*>(&(*i)));

    // the bottom plane

    i = _pixels.find(T(t._x - 1, t._y - 1, t._z - 1));
    if( i != _pixels.end())
        neighbors.push_back(const_cast<T*>(&(*i)));

    i = _pixels.find(T(t._x, t._y - 1, t._z - 1));
    if( i != _pixels.end())
        neighbors.push_back(const_cast<T*>(&(*i)));

    i = _pixels.find(T(t._x + 1, t._y - 1, t._z - 1));
    if( i != _pixels.end())
        neighbors.push_back(const_cast<T*>(&(*i)));

    i = _pixels.find(T(t._x - 1, t._y, t._z - 1));
    if( i != _pixels.end())
        neighbors.push_back(const_cast<T*>(&(*i)));

    i = _pixels.find(T(t._x , t._y, t._z - 1));
    if( i != _pixels.end())
        neighbors.push_back(const_cast<T*>(&(*i)));

    i = _pixels.find(T(t._x + 1, t._y, t._z - 1));
    if( i != _pixels.end())
        neighbors.push_back(const_cast<T*>(&(*i)));

    i = _pixels.find(T(t._x + 1, t._y + 1, t._z - 1));
    if( i != _pixels.end())
        neighbors.push_back(const_cast<T*>(&(*i)));

    i = _pixels.find(T(t._x, t._y + 1, t._z - 1));
    if( i != _pixels.end())
        neighbors.push_back(const_cast<T*>(&(*i)));

    i = _pixels.find(T(t._x - 1, t._y + 1, t._z - 1));
    if( i != _pixels.end())
        neighbors.push_back(const_cast<T*>(&(*i)));

    return neighbors;

}


    template < typename T >
PixelWorld3D<T>::PixelWorld3D(bool isGrid)
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
    Component3D::K = 0;
    // get blocks from data server
    get_block_attrs(xList, yList, zList, rList, gList, bList, materialList, sizeList, "get_block_attrs", DATA_SERVER_PORT);
    // create 3dpixel pool
    for (int i = 0; i < xList.size(); i++) {
        material_types.insert(materialList[i]);
        RgbColor color;
        color.r = rList[i];
        color.g = gList[i];
        color.b = bList[i];
        _pixels.insert(T(color, xList[i], yList[i], zList[i], sizeList[i]));
    }
    // tracing code
    std::copy(material_types.begin(), material_types.end(),    
            std::ostream_iterator<std::string>(std::cout, "\n"));
    std::cout<<material_types.size()<<std::endl;
    std::cout<<"size of blockworld: "<<xList.size()<<std::endl;

}


template < typename T >
void PixelWorld3D<T>::construct_graph(){
    std::multiset<WeightEdge3D> graph;
    // construt the graph, sort edges in nondecreasing order
    for(typename PixelSet::iterator p = _pixels.begin(); p != _pixels.end(); p++){
        T *t = const_cast<T*>(&(*p));
        // get neighbor pixels, it could be at most 8 nearest grid pixel,
        // or nearest neighbors in the feature space
        std::vector<T*> neighbors = get_neighbors(*t);
        for(T *nb : neighbors){
            // create weight edges
            graph.insert(WeightEdge3D(_edge_weight(*t, *nb), t, nb));

        }
    }

    // create and merge components from edge list
    // as the number of components is always not small, means the convertion from
    // Pixel to Component won't cost that much, the total number of covertions
    // will be num(pixels) + max length of components;
    for(const WeightEdge3D &edge : graph){
        auto i_b = _components.end();    
        auto i_e = _components.end();    
        // for every pixel
        for (auto i = _components.begin(); i != _components.end() ; i++) {
            // find components contain node of edge, if exist
            if( i->contains(edge._b)){
                i_b = i;
                continue;
            }
            if( i->contains(edge._e)){
                i_e = i;
                continue;
            }
            // found both!
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
            }
            else if(rst == 1){
                _components.erase(i_b);
            }
        }
        else if( i_b == _components.end() && i_e == _components.end()) {
            // merge two pixels
            Component3D tempB(edge._b);
            Component3D tempE(edge._e);
            int rst = Component3D::merge(edge._weight, tempB, tempE);
            // the only place insert elements to #_components
            if(rst == 0){
                _components.push_back(tempB);
            }
            else if(rst == 1){
                _components.push_back(tempE);
            }
        }
        else if( i_b == _components.end()){
            // merge pixel edge._b to component i_e
            Component3D temp(edge._b);
            Component3D::merge(edge._weight, temp, *i_e);
        }
        else{
            // merge pixel edge._e to component i_b
            Component3D temp(edge._e);
           Component3D::merge(edge._weight, *i_b, temp);
        }

    }
    std::cout<<"************size of graph"<<graph.size()<<std::endl;
    std::cout<<"**************size of _components: "<<_components.size()<<std::endl;
    // output the size of components
    for(Component3D &comp: _components){
        std::cout<<"[ "<< comp.get_members().size()<<" ]";
    }
    std::cout<<std::endl;
    /*save_segmentation();*/
}

template < typename T >
std::vector<T*> PixelWorld3D<T>::_feature_neighbors(const T &t) const{
    return std::vector<T*>();
}
template < typename T >
void PixelWorld3D<T>::save_segmentation( ){
    // 
    m_util::RpcServer server(DETECTION_SERVER_PORT);
    xmlrpc_c::methodPtr const originMethod(new OriginMethod<T>(_pixels));
    xmlrpc_c::methodPtr const segMethod(new SegMethod<T>(_components));
    server.register_method(originMethod, "get_block_attrs");
    server.register_method(segMethod, "get_segmentation_result");
    server.run();

    /*typename T::ColorType segmentColor = comp.compute_average_color();*/
    /*for(GrayPixel3D *pixel : comp.get_members()){*/
    /*gray_img[pixel->_y][pixel->_x] = segmentColor;*/
    /*}*/
    /*}*/
    /*// restore some component composed of one single pixel*/
    /*for (int row = 0; row < _height; row++) */
    /*for (int col = 0; col < _width; col++){*/
    /*if(gray_img[row][col] == white)*/
    /*gray_img[row][col] = _pixels[row][col].get_color();*/
    /*}*/
    /*gray_img.show();*/
    /*gray_img.save("segmentation.jpg");*/
}

#endif /* end of include guard: PIXELWORLD3D.H */
