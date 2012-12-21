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
//#include <flann/flann.hpp>
//#include <flann/io/hdf5.h>
namespace m_lib{

//using namespace flann;
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

//template < class T >
//void nearest_k_search2(std::vector<T> &points, const T& searchPoint, int nn = 10, int dim = 3){
//
//
//    Matrix<float> dataset;
//    Matrix<float> query;
//    dataset =  flann::Matrix<float>(new float[points.size() * dim], points.size(), dim);
//    //
//    Matrix<int> indices(new int[query.rows*nn], query.rows, nn);
//    Matrix<float> dists(new float[query.rows*nn], query.rows, nn);
//
//    // construct an randomized kd-tree index using 4 kd-trees
//    Index<L2<float> > index(dataset, flann::KDTreeIndexParams(4));
//    index.buildIndex();                                                                                               
////
////    // do a knn search, using 128 checks
////    index.knnSearch(query, indices, dists, nn, flann::SearchParams(128));
////
////    flann::save_to_file(indices,"result.hdf5","result");
////
////    delete[] dataset.ptr();
////    delete[] query.ptr();
////    delete[] indices.ptr();
////    delete[] dists.ptr();
//    
//}

} /* m_lib */
#endif /* end of include guard: M_ALGORITHM.H */
