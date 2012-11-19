#include "m_util.h" 
#include "pixel.h" 
std::ostream& operator << (std::ostream& out, const GrayPixel2D& t){
        out<<m_util::string_format("[%i, %i: gray=%i]", t._x, t._y, t.get_color().v);
    return out;
}

std::ostream& operator << (std::ostream& out, const GrayPixel2D* t){
    out<<*t;
    return out;
}

std::ostream& operator << (std::ostream& out, const RgbPixel2D& t){
        out<<m_util::string_format("[%i, %i: r=%i, g=%i, b=%i]", t._x,t._y,t.get_color().r, t.get_color().g, t.get_color().b);
    return out;
}

std::ostream& operator << (std::ostream& out, const RgbPixel2D* t){
    out<<*t;
    return out;
}
// 3D
std::ostream& operator << (std::ostream& out, const GrayPixel3D& t){
    out<<m_util::string_format("[%i, %i, %i: gray=%i]", t._x, t._y, t._z, t.get_color().v);
    return out;
}

std::ostream& operator << (std::ostream& out, const GrayPixel3D* t){
    out<<*t;
    return out;
}

std::ostream& operator << (std::ostream& out, const RgbPixel3D& t){
    out<<m_util::string_format("[%i, %i, %i: r=%i, g=%i, b=%i]", t._x, t._y, t._z, t.get_color().r, t.get_color().g, t.get_color().b);
    return out;
}

std::ostream& operator << (std::ostream& out, const RgbPixel3D* t){
    out<<*t;
    return out;
}
