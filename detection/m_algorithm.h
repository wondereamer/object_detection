/**
 * @file m_algorithm.h
 * @brief some useful algorithms
 * @author Dignjie.Wang (dingjie.wang@gmail.com)
 * @version 0.1
 * @date 2012-10-10
 */

#ifndef M_ALGORITHM.H
#define M_ALGORITHM.H
#include <stdio.h>
#include "m_util.h"
#include <algorithm>
#include <set>
#include "m_geometry.h"
namespace m_lib{

/**
* @brief midpoint circle algorithm
*
* have checked that return points of circles of different radious won't intersect 
* @param x0 coordinate of center
* @param y0 coordinate of center
* @param radius
*
* @return elements along the circle 
*/
std::set<Point> get_elemnts_along_circle(int x0, int y0, int radius);

template < typename T >    
void discrete_differential(const T &y, T &rst){
    rst[0] =  0;
    for (size_t i = 1; i < y.size(); i++) {
       rst[i] = y[i] - y[i - 1] ;
    }
}

template < typename T >
void previous_sum(T &container){
    auto i = container.begin() + 1;
    for (; i < container.end() ; i++) {
        *i = *i + *(i-1);
    }
}

} /* m_lib */
#endif /* end of include guard: M_ALGORITHM.H */
