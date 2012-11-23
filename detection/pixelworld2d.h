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
#include "pixel.h" 
#include "m_opencv.h" 
#include <cassert>
#include "m_opencv.h"

template < typename T >
class PixelWorld2D {
    public:
        PixelWorld2D (std::string filename = "");
        virtual ~PixelWorld2D ();

        T* operator[](int rowIndx) { return _pixels[rowIndx]; }
        const T* operator[](int rowIndx)const { return _pixels[rowIndx]; }


    protected:
        T        **_pixels;    
        int             _height;
        int             _width;
        /* data */
};

template < typename T >
PixelWorld2D<T>::PixelWorld2D(std::string filename)
{
    IplImage* temp =  cvLoadImage(filename.data(), T::ISRGB);
    assert(temp);
    _height = temp->height;
    _width = temp->width;
    typename T::ImageType image(temp);
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

#endif /* end of include guard: PIXELWORLD2D.H */
