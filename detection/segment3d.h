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
#include "segmentation_server.h"
#include "m_util.h"
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
        explicit Segment3D(bool isGrid = true);
        ~Segment3D(){ };

        /*------------------------------------------------------------------------------------ */
        void segment( );
        void save_segmentation( );

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
        this->_pixels.insert(T(color, xList[i], yList[i], zList[i], sizeList[i]));
    }
    // tracing code
    std::copy(material_types.begin(), material_types.end(),    
            std::ostream_iterator<std::string>(std::cout, "\n"));
    std::cout<<material_types.size()<<std::endl;
    std::cout<<"size of blockworld: "<<xList.size()<<std::endl;

}


template < typename T >
void Segment3D<T>::segment( ){
    std::multiset<WeightEdge3D> graph;
    // construt the graph, sort edges in nondecreasing order
    for(typename PixelSet::iterator p = this->_pixels.begin(); p != this->_pixels.end(); p++){
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
            }else{
                // failed to merge two pixel
                // add two single pixel components
                // single pixel component is actually treated as
                // noise, and would be merged to other components during later 
                // optimizing
                /*_components.push_back(tempB);*/
                /*_components.push_back(tempE);*/
            }
        }
        else if( i_b == _components.end()){
            // merge pixel edge._b to component i_e
            Component3D temp(edge._b);
            if(Component3D::merge(edge._weight, temp, *i_e) == -1);
                // add signle pixel comonent
                /*_components.push_back(temp);*/
        }
        else{
            // merge pixel edge._e to component i_b
            Component3D temp(edge._e);
            if(Component3D::merge(edge._weight, *i_b, temp) == -1);
                /*_components.push_back(temp);*/
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
void Segment3D<T>::save_segmentation( ){
    // 
    m_util::RpcServer server(DETECTION_SERVER_PORT);
    xmlrpc_c::methodPtr const originMethod(new OriginMethod<T>(this->_pixels));
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

/*template < typename T >*/
/*std::vector<T*> Segment3D<T>::_feature_neighbors(const T &t) const{*/
/*return std::vector<T*>();*/
/*}*/

#endif /* end of include guard: SEGMENT3D_H */
