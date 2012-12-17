/**
 * @file pixel.h
 * @brief 
 * @author Dingjie.Wang (dingjie.wang@gmail.com)
 * @version 0.1
 * @date 2012-10-16
 */

#ifndef PIXEL_H

#define PIXEL_H


#include "m_opencv.h" 
#include <cmath>
#include "m_math.h"
using namespace m_opencv;

/**
 * @brief 
 */
/*class PixelGray {*/
/*public:*/
/*PixelGray(){ }*/
/*public:*/
/*void set_density(const GrayColor &gray) {*/
/*_density.gray = gray;*/
/*}*/
/*GrayColor get_gray( ) const {*/
/*return _density.gray;*/
/*}*/
/*RgbColor get_rgb( ) const {*/
/*return _density.rgb;*/
/*}*/
/*double  get_density() const{*/
/*if (is_rgb()) */
/*return 0;   */
/*else*/
/*return _density.gray.v;*/
/*}*/
/*bool is_rgb() const { */
/*return _isRgb; */
/*}*/
/*static double density_distance(const Pixel &a, const Pixel &b){*/
/*return a.is_rgb() ? Pixel::_rgb_distance(a, b) :  Pixel::_gray_distance(a, b);*/
/*}*/

/*protected:*/
/*static double _gray_distance(const Pixel &a , const Pixel &b) {*/
/*return abs(a.get_gray().v - b.get_gray().v);*/
/*}*/
/*static  double _rgb_distance(const Pixel &a, const Pixel &b)  { return 0;}*/

/*union Density{*/
/*RgbColor rgb;*/
/*GrayColor gray;*/
/*};*/

/*public:*/
/*// two dimension force with index [homogeneity][spatial]*/
/*protected:*/
/*bool _isRgb;*/
/*Density _density;*/

/*}; */
/**
 * @brief 
 */
class GrayPixel2D{

    public:
        int _x;
        int _y;
        static const int CHANELS = 1;
        static const bool ISRGB = false;
        GrayColor _color;
    public:
        typedef GrayImage ImageType;
        typedef GrayColor ColorType;
        typedef GrayColor MeasureColorType;
        GrayPixel2D(int x = 0 , int y = 0):_x(x), _y(y) { }
        void set_location(int x, int y){
            _x = x;
            _y = y;
        }

        bool operator < (const GrayPixel2D &other) const{
            if (_x < other._x)
                return true;
            else if (_x > other._x)
                return false;
            else if (_y < other._y)
                return true;
            else 
                return false;
        }

        static GrayColor white_color(){
            GrayColor white;
            white.v = 255;
            return white;
        }
        static double density_distance(const GrayPixel2D &a, const GrayPixel2D &b){
            return GrayColor::color_distance(a._color, b._color);
        }
        static double spatial_distance(const GrayPixel2D &a, const GrayPixel2D &b){
            return (double)sqrt(pow(a._x - b._x, 2) +  pow(a._y - b._y, 2));
        }
        double get_density() const{
            return _color.v;
        }

        void set_color(const GrayColor &gray){
            _color = gray;
        }
        const GrayColor&  get_color() const{
            return _color;
        }
        const GrayColor&  get_measure_color() const{
            return _color;
        }
};
class RgbPixel2D {

    public:
        int _x;
        int _y;
        LuvColor _luvColor;
        static const int CHANELS = 3;
        static const bool ISRGB = true;
        RgbColor _color;

    public:
        RgbPixel2D(int x = 0 , int y = 0):_x(x), _y(y) { }
        static double spatial_distance(const RgbPixel2D &a, const RgbPixel2D &b){
            return (double)sqrt(pow(a._x - b._x, 2) +  pow(a._y - b._y, 2));
        }
        void set_location(int x, int y){
            _x = x;
            _y = y;
        }

        bool operator < (const RgbPixel2D &other) const{
            if (_x < other._x)
                return true;
            else if (_x > other._x)
                return false;
            else if (_y < other._y)
                return true;
            else 
                return false;
        }
    public:

        typedef RgbImage ImageType;
        typedef RgbColor ColorType;
        typedef LuvColor MeasureColorType;
        void set_color(const RgbColor &rgb){
            _color = rgb;
            rgb2luv(_color.r, _color.g, _color.b, _luvColor);
        }
        double get_density( ) const{
            /*return _co*/
            return 0;
        }
        const RgbColor&  get_color() const{
            return _color;
        }
        const LuvColor& get_measure_color()const {
            return _luvColor;
        }
        static double density_distance(const RgbPixel2D &a, const RgbPixel2D &b){
            /// @todo ...
            return m_math::length_edge(a._luvColor.l, a._luvColor.u, a._luvColor.v,
                    b._luvColor.l, b._luvColor.u, b._luvColor.v);
        }


};

class GrayPixel3D{

    public:
        int _x;
        int _y;
        int _z;
        int _size;
    protected:
        GrayColor _color;
    public:
        /*typedef GrayImage Image;*/
        typedef GrayColor ColorType;
        typedef GrayColor MeasureColorType;
        GrayPixel3D(int x = 0, int y = 0, int z = 0):_x(x), _y(y), _z(z){ }

        GrayPixel3D(const GrayColor &color, int x = 0 , int y = 0, int z = 0, int size = 0):_x(x), _y(y), _z(z),
        _size(size), _color(color) { }
        static double spatial_distance(const GrayPixel3D &a, const GrayPixel3D &b){
            return (double)sqrt(pow(a._x - b._x, 2) +  pow(a._y - b._y, 2) + pow(a._z - b._z, 2));
        }
        void set_location(int x, int y, int z){
            _x = x;
            _y = y;
            _x = z;
        }

        const GrayColor& get_measure_color()const {
            return _color;
        }

        bool operator < (const GrayPixel3D &other) const{
            if (_x < other._x)
                return true;
            else if (_x > other._x)
                return false;
            else if (_y < other._y)
                return true;
            else if (_y > other._y)
                return false;
            else if (_z < other._z)
                return true;
            else 
                return false;
        }
        static double density_distance(const GrayPixel3D &a, const GrayPixel3D &b){
            return abs(a.get_density() - b.get_density());
        }

        double get_density() const{
            return _color.v;
        }

        void set_color(const GrayColor &gray){
            _color = gray;
        }
        GrayColor  get_color() const{
            return _color;
        }
};

class RgbPixel3D {

    public:
        int _x;
        int _y;
        int _z;
        int _size;
        RgbColor _color;
        LuvColor _luvColor;

    public:

        RgbPixel3D(int x = 0, int y = 0, int z = 0):_x(x), _y(y), _z(z){ }

        RgbPixel3D(const RgbColor &color, int x = 0 , int y = 0, int z = 0, int size = 0):_x(x), _y(y), _z(z),
        _size(size), _color(color) { }
        static double spatial_distance(const RgbPixel3D &a, const RgbPixel3D &b){
            return (double)sqrt(pow(a._x - b._x, 2) +  pow(a._y - b._y, 2) + pow(a._z - b._z, 2));
        }
        void set_location(int x, int y, int z){
            _x = x;
            _y = y;
            _x = z;
        }
        bool operator < (const RgbPixel3D &other) const{
            if (_x < other._x)
                return true;
            else if (_x > other._x)
                return false;
            else if (_y < other._y)
                return true;
            else if (_y > other._y)
                return false;
            else if (_z < other._z)
                return true;
            else 
                return false;
        }
    public:

        typedef RgbColor ColorType;
        typedef LuvColor MeasureColorType;
        void set_color(const RgbColor &rgb){
            _color = rgb;
        }
        double get_density( ) const{
            /*return _co*/
            return 0;
        }
        RgbColor  get_color() const{
            return _color;
        }

        const LuvColor& get_measure_color()const {
            return _luvColor;
        }
        static double density_distance(const RgbPixel3D &a, const RgbPixel3D &b){
            // @bug temporal use, before find a way to measure the difference between two colors
            if (a.get_color() != b.get_color()) {
                return 1;
            }
            return 0;
        }


};
typedef GrayPixel3D Block;
std::ostream& operator << (std::ostream& out, const GrayPixel2D& t);
std::ostream& operator << (std::ostream& out, const GrayPixel2D* t);
std::ostream& operator << (std::ostream& out, const RgbPixel2D& t);
std::ostream& operator << (std::ostream& out, const RgbPixel2D* t);
std::ostream& operator << (std::ostream& out, const GrayPixel3D& t);
std::ostream& operator << (std::ostream& out, const GrayPixel3D* t);
std::ostream& operator << (std::ostream& out, const RgbPixel3D& t);
std::ostream& operator << (std::ostream& out, const RgbPixel3D* t);
#endif /* end of include guard: PIXEL_H */
