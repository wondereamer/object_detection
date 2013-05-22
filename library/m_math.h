/**
 * @file m_math.h
 * @brief 
 * @author Dingjie.Wang(dingjie.wang@gmail.com)
 * @version 0.1
 * @date 2012-11-22
 */

#ifndef M_MATH_H

#define M_MATH_H



#endif /* end of include guard: M_MATH_H */
#include "m_util.h" 
namespace m_math {
    const double Pi = 3.1415926;
    template < typename T >
        inline T math_vector(const T &begin, const T &end){
            return T(end.x - begin.x, end.y - begin.y); 
        }
    template < typename T >
        inline double length_vector(const T &v){
            return (double)sqrt(pow(v.x, 2) +pow(v.y, 2));
        }
    template < typename T >
        inline double length_edge(const T &begin, const T &end){
            return (double)sqrt(pow(begin.x - end.x, 2) +  pow(begin.y - end.y, 2));
        }
    template < typename T >
        inline double angle_vector(const T &vec){
            return atan2(vec.y, vec.x) * 180 / Pi;
        }
    inline int rand_int(int min, int max){
        return rand() % (max - min + 1) + min;
    }
    inline void rand_sample(int min, int max, int num, std::vector<int> *rst)
    {
        /// @todo less efficient
        std::set<int> temp;
        for (int i = 0; i < num; i++) {
            int t = rand_int(min, max);
            while(!temp.insert(t).second)
                t = rand_int(min, max);
            rst->push_back(t);
        }
    }
    template < typename T >
    void rand_sample(std::vector<T> &input, int num, std::vector<T> *rst)
    {
        /// @todo less efficient
        std::set<int> temp;
        for (int i = 0; i < num; i++) {
            int t = rand_int(0, input.size() - 1);
            while(!temp.insert(t).second)
                t = rand_int(0, input.size() - 1);
            rst->push_back(input[t]);
        }
    }
    

    template < typename T >
        inline double length_edge(T x0, T y0,T z0, T x1, T y1, T z1){
            return (double)sqrt(pow(x0 - x1, 2) +  pow(y0 - y1, 2) + pow(z0 - z1, 2));
        }

    template < typename T >
        inline double length_edge(T x0, T y0, T x1, T y1){
            return (double)sqrt(pow(x0 - x1, 2) +  pow(y0 - y1, 2));
        }

} /* m_math */
