/**
 * @file m_plot.h
 * @brief plot interface to MatPlotlib
 * @author Dignjie.Wang (dingjie.wang@gmail.com)
 * @version 0.1
 * @date 2012-10-10
 */
#ifndef M_PLOT_H

#define M_PLOT_H

#include "m_message.h"
//! assume y is a two dimension vector that has member #x and #y
namespace m_plot {

    template < typename T >
        xmlrpc_c::paramList translate_format(const std::vector<T> &y){

            xmlrpc_c::paramList params;
            std::vector<xmlrpc_c::value> x_array;
            std::vector<xmlrpc_c::value> y_array;
            for(const T &val : y){
                x_array.push_back(xmlrpc_c::value_double(val.x));
                y_array.push_back(xmlrpc_c::value_double(val.y));
            }
            params.add(xmlrpc_c::value_array(x_array));
            params.add(xmlrpc_c::value_array(y_array));
            //    std::copy(forces.begin(), forces.end(),    
            //            ostream_iterator<T>(std::cout, "\n"));
            return params;
        }

    //! plot graph reflect the direction variation and strenth variation
    template < typename T >
        void plot_vectors(const std::vector<T> &y )
        {
            m_lib::call_py("plot_vectors", translate_format(y));
        }
    //! plot force field
    template < typename T >
        void plot_force_field(const std::vector<T> &y, int width, int height){
            auto params = translate_format(y);
            params.add(xmlrpc_c::value_int(width));
            params.add(xmlrpc_c::value_int(height));
            m_lib::call_py("plot_force_field", params);
        }
} /* m_plot */

#endif /* end of include guard: M_PLOT_H */
