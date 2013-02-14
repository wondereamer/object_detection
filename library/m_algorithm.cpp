#include "m_algorithm.h"
#include <cstdlib>
namespace m_lib  {
/**
 * @brief midpoint circle algorithm
 *
 * @param x0 coordinate of center
 * @param y0 coordinate of center
 * @param radius
 *
 * @return elements along the circle 
 */
std::set<Point> get_elemnts_along_circle(int x0, int y0, int radius)
{
    std::set<Point> rst; 
    int f = 1 - radius;
    int ddF_x = 1;
    int ddF_y = -2 * radius;
    int x = 0;
    int y = radius;
    rst.insert(Point(x0, y0 + radius)); 
    rst.insert(Point(x0, y0 - radius)); 
    rst.insert(Point(x0 + radius, y0));
    rst.insert(Point(x0 - radius, y0));


    while(x < y) {
        if(f >= 0) 
        {
            y--;
            ddF_y += 2;
            f += ddF_y;
        }
        x++;
        ddF_x += 2;
        f += ddF_x;    
        rst.insert(Point(x0 + x, y0 + y));
        rst.insert(Point(x0 - x, y0 + y));
        rst.insert(Point(x0 + x, y0 - y));
        rst.insert(Point(x0 - x, y0 - y));
        rst.insert(Point(x0 + y, y0 + x));
        rst.insert(Point(x0 - y, y0 + x));
        rst.insert(Point(x0 + y, y0 - x));
        rst.insert(Point(x0 - y, y0 - x));
    }
    return rst;
}
//! return integer between [min, max]
} /* m_lib  */
