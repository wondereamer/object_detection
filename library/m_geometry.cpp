#include <iostream>
#include <iterator>
#include "m_geometry.h"
    std::ostream& operator<<(std::ostream& out, const Point &t){
        out<<"[x:"<<t.x<<" "<<"y:"<<t.y<<"]" ;
        return out;
    }
    std::ostream& operator<<(std::ostream& out, const PointF &t){
        out<<"[x:"<<t.x<<" "<<"y:"<<t.y<<"]" ;
        return out;
    }

    std::ostream& operator<<(std::ostream& out, const PointF3D &t){
        out<<"[x:"<<t.x<<" "<<"y:"<<t.y<<"z:"<<t.z<<"]";
        return out;
    }
