/**
 * @file pixelworld.h
 * @brief compute attraction force
 * @author Dignjie.Wang (dingjie.wang@gmail.com)
 * @version 0.1
 * @date 2012-10-10
 */
#ifndef PIXElWORLD2D_H
#define PIXELWORLD2D_H
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

using std::string;


/**
 * @brief 
 */
template < typename T >
class PixelWorld2D {

    /*---------------------------  lifecycle  ------------------------------------------------ */
    public:
        typedef Component<T*> Component2D;
        typedef WeightEdge<T*> WeightEdge2D;
        explicit PixelWorld2D(string filename, bool isRgb = false, bool isGrid = true);
        ~PixelWorld2D();

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

        bool is_inside(int x, int y) const;
        std::vector<T*> _grid_neighbors(const T &t) const;
        std::vector<T*> _feature_neighbors(const T &t) const;

        /*--------------------  accessor methods  -------------------------------------------- */
    public:
        int             get_width( ) const                     { return _width;              }
        int             get_height( ) const                    { return _height;             }


        /*-----------------------  attributes  ------------------------------------------------ */

    private:
        bool            _isGrid;
        T        **_pixels;    
        int             _height;
        int             _width;
        std::list<Component2D> _components;

}; 

//
template < typename T >
inline std::vector<T*> PixelWorld2D<T>::get_neighbors(const T &t) const{
    return _isGrid ? _grid_neighbors(t) :  _feature_neighbors(t);
}
template < typename T >
inline bool PixelWorld2D<T>::is_inside(int x, int y) const{
    // when the neighbor is out of image, return force with strength zero
    if (x >= 0 && x < _width && y >= 0 && y < _height) 
        return true;
    return false;
}
template < typename T >
inline std::vector<T*> PixelWorld2D<T>::_grid_neighbors(const T &t) const{
    // push 8 nearest neighbors
    std::vector<T*> neighbors;
    if(is_inside(t._x - 1, t._y - 1))
        neighbors.push_back(&_pixels[t._y - 1][t._x - 1]);
    if(is_inside(t._x, t._y - 1))
        neighbors.push_back(&_pixels[t._y - 1][t._x]);
    if(is_inside(t._x + 1, t._y - 1))
        neighbors.push_back(&_pixels[t._y - 1][t._x + 1]);

    if(is_inside(t._x - 1, t._y ))
        neighbors.push_back(&_pixels[t._y][t._x - 1]);
    if(is_inside(t._x + 1, t._y))
        neighbors.push_back(&_pixels[t._y][t._x + 1]);

    if(is_inside(t._x + 1, t._y + 1))
        neighbors.push_back(&_pixels[t._y + 1][t._x + 1]);
    if(is_inside(t._x , t._y + 1))
        neighbors.push_back(&_pixels[t._y + 1][t._x]);
    if(is_inside(t._x - 1, t._y + 1))
        neighbors.push_back(&_pixels[t._y + 1][t._x - 1]);
    return neighbors;

}


    template < typename T >
PixelWorld2D<T>::PixelWorld2D(string filename, bool isRgb, bool isGrid)
{
    IplImage* temp =  cvLoadImage(filename.data(), isRgb);
    Component2D::K = 50;
    _isGrid = isGrid;
    assert(temp);
    _height = temp->height;
    _width = temp->width;
    typename T::Image image(temp);
    //allocate memory for pixels
    _pixels = new T*[_height];
    for (int i = 0; i < _height; i++) {
        _pixels[i] = new T[ _width ];
    }
    //fetch gray/rgb value of pixels
    for (int row = 0; row < _height; row++) 
        for (int col = 0; col < _width; col++){
            _pixels[row][col].set_color(image[row][col]);

            _pixels[row][col].set_location(col, row);
        }
    image.output_img_info();

}

    template < typename T >
PixelWorld2D<T>::~PixelWorld2D()
{
    for (int i = 0; i < _height; i++) {
        delete []_pixels[i];
    }
    delete []_pixels;
}

template < typename T >
void PixelWorld2D<T>::construct_graph(){
    // map pixel to it's component 
    std::multiset<WeightEdge2D> graph;
    // construt the graph, sort edges in nondecreasing order
    for (int row = 0; row < _height; row++) 
        for (int col = 0; col < _width; col++){
            T *t = &_pixels[row][col];
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
std::vector<T*> PixelWorld2D<T>::_feature_neighbors(const T &t) const{
    return std::vector<T*>();
}
template < typename T >
void PixelWorld2D<T>::save_segmentation( ){
    /*save_segmentation_helper(_pixels, _width, _height, _components);*/
    CvSize size;
    size.width = _width;
    size.height = _height;
    /// @todo rgb and gray image have different channels
    IplImage* temp  = cvCreateImage(size, IPL_DEPTH_8U,1);
    std::cout<<"**************************"<<std::endl;
    typename T::Image img(temp);
    typename T::ColorType white = T::get_white();
    img.set_color(white);
    std::cout<<"**************************"<<std::endl;
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
    img.save("segmentation.jpg");
}

#endif /* end of include guard: PIXELWORLD2D.H */
