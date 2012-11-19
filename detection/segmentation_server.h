
#include <getopt.h>
#include <langinfo.h>
#include <locale.h>

#include <boost/filesystem/operations.hpp>

#include <boost/algorithm/string.hpp>
#include <boost/foreach.hpp>

#include <utility>
#include <string>

#include <xmlrpc-c/base.hpp>
#include <xmlrpc-c/registry.hpp>
#include <xmlrpc-c/server_abyss.hpp>
#include <vector>
#include <cassert>
#include <stdlib.h>
#include "m_util.h" 
#include "component.h" 
#include <list>
using m_util::sth2string;

/**
 * @brief return block datas from data server
 *
 * @tparam T
 */
template < typename T >
class OriginMethod: public xmlrpc_c::method {
    public:
        OriginMethod(const std::set<T> &pixels):_pixels(pixels){ 

            std::vector<xmlrpc_c::value> position_array;
            std::vector<xmlrpc_c::value> size_array;
            std::vector<xmlrpc_c::value> color_array;
            std::vector<xmlrpc_c::value> material_array;
            for(const T &block : _pixels){
                position_array.push_back(xmlrpc_c::value_string(sth2string<int>(block._x) + "," + sth2string<int>(block._y) + "," + sth2string<int>(block._z)));
                size_array.push_back(xmlrpc_c::value_int(block._size));
                color_array.push_back(xmlrpc_c::value_string(sth2string<int>(block.get_color().r) + "," + sth2string<int>(block.get_color().g) 
                            + "," + sth2string<int>(block.get_color().b)));
                material_array.push_back(xmlrpc_c::value_string(""));
            }

            std::vector<xmlrpc_c::value> ttt;
            ttt.push_back(xmlrpc_c::value_array(position_array));
            ttt.push_back(xmlrpc_c::value_array(size_array));
            ttt.push_back(xmlrpc_c::value_array(material_array));
            ttt.push_back(xmlrpc_c::value_array(color_array));
            _rst = xmlrpc_c::value_array(ttt);

        }
        void execute(xmlrpc_c::paramList const& paramList,
                xmlrpc_c::value *   const  retvalP) {
            *retvalP = _rst;
            std::cout<<"Having sent origin data."<<std::endl;
        }
    private:
        xmlrpc_c::value _rst;
        const std::set<T> &_pixels;
};

/**
 * @brief return segmented result
 *
 * @tparam T
 */
template < typename T >
class SegMethod: public xmlrpc_c::method {
    public:
        SegMethod(std::list<Component<T*>> &components):_components(components){ 

            std::vector<xmlrpc_c::value> position_array;
            std::vector<xmlrpc_c::value> size_array;
            std::vector<xmlrpc_c::value> color_array;
            std::vector<xmlrpc_c::value> material_array;
            int colorValue = 0;
            for(auto &com : _components){
                // for every component
                colorValue += 15;
                auto &members = com.get_members();
                // for every block in the component
                for(T *block: members){
                    position_array.push_back(xmlrpc_c::value_string(sth2string<int>(block->_x) + "," + sth2string<int>(block->_y) + "," + sth2string<int>(block->_z)));
                    size_array.push_back(xmlrpc_c::value_int(block->_size));
                    color_array.push_back(xmlrpc_c::value_string(sth2string<int>(colorValue+10) + "," + sth2string<int>(colorValue) 
                                + "," + sth2string<int>(colorValue-5)));
                    material_array.push_back(xmlrpc_c::value_string(""));

                }
            }
            std::vector<xmlrpc_c::value> ttt;
            ttt.push_back(xmlrpc_c::value_array(position_array));
            ttt.push_back(xmlrpc_c::value_array(size_array));
            ttt.push_back(xmlrpc_c::value_array(material_array));
            ttt.push_back(xmlrpc_c::value_array(color_array));
            _rst = xmlrpc_c::value_array(ttt);

        }
        void execute(xmlrpc_c::paramList const& paramList,
                xmlrpc_c::value *   const  retvalP) {
            *retvalP = _rst;
            std::cout<<"Having sent segmentation result."<<std::endl;
        }
    private:
        xmlrpc_c::value _rst;
        std::list<Component<T*>> &_components;
};
