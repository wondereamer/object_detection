#include "m_util.h"
#include "data.h"
#include <boost/thread/thread.hpp>
void _get_block_attrs(AttrVector &posV, AttrVector &sizeV, AttrVector& materialV, AttrVector &colorV, std::string method, int port)
{

    xmlrpc_c::paramList params;
    // get block attributes from server
    xmlrpc_c::value_array rst = m_util::call_py(method,params, port);
    // parse result
    std::vector<xmlrpc_c::value> t = rst.vectorValueValue();
    // 
    xmlrpc_c::value_array positionList = t[0];
    xmlrpc_c::value_array sizeList = t[1];
    xmlrpc_c::value_array materialList = t[2];
    xmlrpc_c::value_array colorList = t[3];
    //
    posV = positionList.vectorValueValue();
    sizeV = sizeList.vectorValueValue();
    materialV = materialList.vectorValueValue();
    colorV = colorList.vectorValueValue();
}
void get_block_attrs(vector<float> &xList, vector<float> &yList, vector<float> &zList,
                    vector<int> &rList, vector<int> &gList, vector<int> &bList, 
                    vector<std::string> &materialList,vector<int> &sizeList, std::string method, int port)
{
    AttrVector posV;
    AttrVector sizeV;
    AttrVector materialV;
    AttrVector colorV;
    _get_block_attrs(posV, sizeV, materialV, colorV, method, port);

    // get (x, y, z), (r, g, b) and size of blocks
    for (int i = 0; i < posV.size(); i++) {
        std::string pos = xmlrpc_c::value_string(posV[i]);
        std::string material = xmlrpc_c::value_string(materialV[i]);
        std::string color = xmlrpc_c::value_string(colorV[i]);
        std::vector<std::string> xyz = m_util::split(pos,",");
        std::vector<std::string> rgb = m_util::split(color,",");
        assert(xyz.size() == 3 && rgb.size() == 3);
        int x = m_util::string2sth<int>(xyz[0]);
        int y = m_util::string2sth<int>(xyz[1]);
        int z = m_util::string2sth<int>(xyz[2]);
        int r = m_util::string2sth<int>(rgb[0]);
        int g = m_util::string2sth<int>(rgb[1]);
        int b = m_util::string2sth<int>(rgb[2]);
        xList.push_back(x);
        yList.push_back(y);
        zList.push_back(z);
        rList.push_back(r);
        gList.push_back(g);
        bList.push_back(b);
        materialList.push_back(material);
        sizeList.push_back(int(xmlrpc_c::value_int(sizeV[i])));
    }
}


