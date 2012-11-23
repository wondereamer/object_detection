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
#include <m_algorithm.h>

/**
 * @brief 
 */
template < typename T >
class Segment2D: public PixelWorld2D<T> {

    /*---------------------------  lifecycle  ------------------------------------------------ */
    public:
        typedef Component<T*> Component2D;
        typedef WeightEdge<T*> WeightEdge2D;
        explicit Segment2D(std::string filename, bool isGrid = true);
        virtual ~Segment2D(){ };

        /*------------------------------------------------------------------------------------ */
        std::vector<T*> get_neighbors(const T &t) const;
        void segment();
        void save(std::string filename);

    protected:
        //
        double _edge_weight(const T &a, const T &b){ return T::density_distance(a,b) ;}
        //
        void _gaussian(){ }

        bool is_inside(int x, int y) const;
        std::vector<T*> _grid_neighbors(const T &t) const;
        std::vector<T*> _feature_neighbors(const T &t) const;

        /*-----------------------  attributes  ------------------------------------------------ */

    private:
        bool            _isGrid;
        std::list<Component2D> _components;

}; 

template < typename T >
Segment2D<T>::Segment2D(std::string filename, bool isGrid):PixelWorld2D<T>(filename)
{
    Component2D::K = 50;
    _isGrid = isGrid;

}
//
template < typename T >
inline std::vector<T*> Segment2D<T>::get_neighbors(const T &t) const{
    return _isGrid ? _grid_neighbors(t) :  _feature_neighbors(t);
}
template < typename T >
inline bool Segment2D<T>::is_inside(int x, int y) const{
    // when the neighbor is out of image, return force with strength zero
    if (x >= 0 && x < this->_width && y >= 0 && y < this->_height) 
        return true;
    return false;
}
template < typename T >
inline std::vector<T*> Segment2D<T>::_grid_neighbors(const T &t) const{
    // push 8 nearest neighbors
    std::vector<T*> neighbors;
    if(is_inside(t._x - 1, t._y - 1))
        neighbors.push_back(&this->_pixels[t._y - 1][t._x - 1]);
    if(is_inside(t._x, t._y - 1))
        neighbors.push_back(&this->_pixels[t._y - 1][t._x]);
    if(is_inside(t._x + 1, t._y - 1))
        neighbors.push_back(&this->_pixels[t._y - 1][t._x + 1]);

    if(is_inside(t._x - 1, t._y ))
        neighbors.push_back(&this->_pixels[t._y][t._x - 1]);
    if(is_inside(t._x + 1, t._y))
        neighbors.push_back(&this->_pixels[t._y][t._x + 1]);

    if(is_inside(t._x + 1, t._y + 1))
        neighbors.push_back(&this->_pixels[t._y + 1][t._x + 1]);
    if(is_inside(t._x , t._y + 1))
        neighbors.push_back(&this->_pixels[t._y + 1][t._x]);
    if(is_inside(t._x - 1, t._y + 1))
        neighbors.push_back(&this->_pixels[t._y + 1][t._x - 1]);
    return neighbors;

}




template < typename T >
void Segment2D<T>::segment(){
    // map pixel to it's component 
    std::multiset<WeightEdge2D> graph;
    // construt the graph, sort edges in nondecreasing order
    for (int row = 0; row < this->_height; row++) 
        for (int col = 0; col < this->_width; col++){
            T *t = &this->_pixels[row][col];
            // get neighbor pixels, it could be at most 8 nearest grid pixel,
            // or nearest neighbors in the feature space
            std::vector<T*> neighbors = get_neighbors(*t);
            //                std::cout<<neighbors.size()<<std::endl;
            for(T *nb : neighbors){
                /// @todo reduce symmetric edge
                // create weight edges
                graph.insert(WeightEdge2D(_edge_weight(*t, *nb), t, nb));

            }
        }
    // statics
    float com_com = 0;
    float pixel_pixel = 0;
    float com_pixel = 0;
    float a_com_com = 0;
    float a_pixel_pixel = 0;
    float a_com_pixel = 0;

    // create and merge components from edge list
    // as the number of components is always small, means the convertion from
    // Pixel to Component won't cost that much, the total number of covertions
    // will be num(pixels) + max length of components;
    for(const WeightEdge2D &edge : graph){
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
            int rst = Component2D::merge(edge._weight, *i_b, *i_e);
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
            Component2D tempB(edge._b);
            Component2D tempE(edge._e);
            int rst = Component2D::merge(edge._weight, tempB, tempE);
            // the only place insert elements to #_components
            if(rst == 0){
                _components.push_back(tempB);
                pixel_pixel++;
            }
            else if(rst == 1){
                _components.push_back(tempE);
                pixel_pixel++;
            }
            a_pixel_pixel++;
        }
        else if( i_b == _components.end()){
            // merge pixel edge._b to component i_e
            Component2D temp(edge._b);
            if(Component2D::merge(edge._weight, temp, *i_e) != -1)
                com_pixel++;
            a_com_pixel++;
        }
        else{
            // merge pixel edge._e to component i_b
            Component2D temp(edge._e);
            if(Component2D::merge(edge._weight, *i_b, temp))
                com_pixel++;
            a_com_pixel++;
        }

    }

    std::cout<<"************size of graph"<<graph.size()<<std::endl;
    std::cout<<"**************size of _components: "<<_components.size()<<std::endl;
    for(Component2D &comp: _components){
        std::cout<<"[ "<< comp.get_members().size()<<" ]";
    }
    std::cout<<std::endl;
}

template < typename T >
std::vector<T*> Segment2D<T>::_feature_neighbors(const T &t) const{
    return std::vector<T*>();
}
template < typename T >
void Segment2D<T>::save(std::string filename ){
    // create an image, and set the image color white
    CvSize size;
    size.width = this->_width;
    size.height = this->_height;
    IplImage* temp  = cvCreateImage(size, IPL_DEPTH_8U, T::CHANELS);
    typename T::ImageType img(temp);
    typename T::ColorType white = T::white_color();
    img.set_color(white);
    //
    typename T::ColorType segmentColor;
    segmentColor.v = 0;
    /*segmentColor.r = 0;*/
    /*segmentColor.g = 0;*/
    /*segmentColor.b = 0;*/
    for(auto &comp : _components){
        /*if (comp.get_members().size() < 70) {*/
        /*continue;*/
        /*}*/
        // assign segmentation color
        std::cout<<"**************************"<<std::endl;
        segmentColor.v += 10;

        /*segmentColor.r += 10;*/
        /*segmentColor.g += 10;*/
        /*segmentColor.b += 10;*/
        for(GrayPixel2D *pixel : comp.get_members()){
            img[pixel->_y][pixel->_x] = segmentColor;
            std::cout<<(int)segmentColor.v<<std::endl;
        }
    }
    /*// restore some component composed of one single pixel*/
    /*for (int row = 0; row < _height; row++) */
    /*for (int col = 0; col < _width; col++){*/
    /*if(img[row][col] == white)*/
    /*img[row][col] = _pixels[row][col].get_color();*/
    /*}*/
    img.show();
    img.save(filename + ".jpg");
}

#endif /* end of include guard: SEGMENT2D_H */
