/**
 * @file m_geometry.h
 * @brief some useful algorithms
 * @author Dignjie.Wang (dingjie.wang@gmail.com)
 * @version 0.1
 * @date 2012-10-10
 */
#ifndef M_GEOMETRY_H

#define M_GEOMETRY_H
#include <iostream>
namespace m_geometry {
    
    struct Point {
        int x;
        int y;
        //
        Point():x(0),y(0) { }
        Point(int x0, int y0){ x = x0; y = y0; }

        Point( const Point &other ){ *this = other; }   
        Point& operator = ( const Point &other ) { x = other.x; y = other.y; return *this; }
        Point operator * (const Point &other) const{
            return Point(x * other.x, y * other.y);
        }
        Point& operator *= (int num){
            x *= num;
            y *= num;
            return *this;
        }
        Point operator + (const Point &other) const{
            return Point(x + other.x, y + other.y);
        }
        Point& operator += (const Point &other){
            x += other.x;
            y += other.y;
            return *this;
        }
        //! do support string like weak order comparing
        bool operator < (const Point &other) const{
            if (x < other.x)
                return true;
            else if (x > other.x)
                return false;
            else if (y < other.y)
                return true;
            else 
                return false;
        };
    }; 

    struct PointF {
        double x;
        double y;
        //
        PointF():x(0),y(0) { }
        PointF(double x0, double y0):x(x0), y(y0){ }
        PointF(const Point &other):x(other.x), y(other.y){ }

        PointF( const PointF &other ){ *this = other; }   
        PointF& operator = ( const PointF &other ) { x = other.x; y = other.y; return *this; }

        PointF operator + (const Point &other) const{
            return PointF(x + other.x, y + other.y);
        }
        PointF& operator += (const PointF &other){
            x += other.x;
            y += other.y;
            return *this;
        }

        PointF operator * (const Point &other) const{
            return PointF(x * other.x, y * other.y);
        }
        PointF& operator *= (double num){
            x *= num;
            y *= num;
            return *this;
        }
        //! do support string like weak order comparing
        bool operator < (const PointF &other) const{
            if (x < other.x)
                return true;
            else if (x > other.x)
                return false;
            else if (y < other.y)
                return true;
            else 
                return false;
        };

    }; 

    struct PointF3D {
        double x;
        double y;
        double z;
        //
        PointF3D():x(0),y(0),z(0) { }
        PointF3D(double v):x(v), y(v), z(v){ }
        PointF3D(double x0, double y0, double z0):x(x0), y(y0),z(z0){ }
//        PointF3D(const PointF3D &other):x(other.x), y(other.y), z(other.z){ }

        PointF3D( const PointF3D &other ){ *this = other; }   
        PointF3D& operator = ( const PointF3D &other ) { x = other.x; y = other.y; z = other.z; return *this; }

        PointF3D operator + (const PointF3D &other) const{
            return PointF3D(x + other.x, y + other.y, z + other.z);
        }
        PointF3D& operator += (const PointF3D &other){
            x += other.x;
            y += other.y;
            z += other.z;
            return *this;
        }

        PointF3D operator * (const PointF3D &other) const{
            return PointF3D(x * other.x, y * other.y, z * other.z);
        }
        PointF3D& operator *= (double num){
            x *= num;
            y *= num;
            z *= num;
            return *this;
        }

        PointF3D operator / (const PointF3D &other) const{
            return PointF3D(x / other.x, y / other.y, z / other.z);
        }
        PointF3D& operator /= (const PointF3D &other){
            x /= other.x;
            y /= other.y;
            z /= other.z;
            return *this;
        }
        //! do support string like weak order comparing
        bool operator < (const PointF3D &other) const{
            if (x < other.x)
                return true;
            else if (x > other.x)
                return false;
            else if (y < other.y)
                return true;
            else if (y > other.y)
                return false;
            else if( z < other.z)
                return true;
            else
                return false;
        };

    }; 

    std::ostream& operator<<(std::ostream& out, const Point &t);
    std::ostream& operator<<(std::ostream& out, const PointF &t);
    std::ostream& operator<<(std::ostream& out, const PointF3D &t);

} /* m_geometry */
#endif /* end of include guard: M_GEOMETRY_H */
