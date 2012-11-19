#ifndef COLORMAP_H

#define COLORMAP_H



#include <map>
#include <utility>
#include <string>
#include "m_opencv.h"
#include <cassert>
#include "m_util.h" 

using namespace m_opencv;
//void material2color(std::string material, GrayPixel &color);
// temporal solution before Lake give me rgb value of each block
inline std::string make_color(float r, float g, float b)
{

    return m_util::string_format("%d,%d,%d", int(r * 255), int(g *255), int(b * 255));

}
inline std::string material2color(std::string material){

    static bool first = true;
    static std::map<std::string, std::string> str2colorMap;
    if (first){
        first = false;
        str2colorMap.insert(std::make_pair("CoalOre", make_color(0.3,0.3,0.3)));
        str2colorMap.insert(std::make_pair("Dandelion", make_color(0.3,0.9,0.1)));
        str2colorMap.insert(std::make_pair("Dirt", make_color(0.7,0.4,0.2)));
        str2colorMap.insert(std::make_pair("Gravel", make_color(0.5,0.5,0.5)));
        str2colorMap.insert(std::make_pair("IronOre", make_color(0.2,0.2,0.2)));
        str2colorMap.insert(std::make_pair("Leaves", make_color(0.1,0.9,0.1)));
        str2colorMap.insert(std::make_pair("Snow", make_color(0.9,0.9,0.9)));
        str2colorMap.insert(std::make_pair("Stone", make_color(0.4,0.4,0.4)));
        str2colorMap.insert(std::make_pair("TallGrass", make_color(0,0.9,0)));
        str2colorMap.insert(std::make_pair("TopSoil", make_color(0.5,0.7,0.1)));
        str2colorMap.insert(std::make_pair("Wood", make_color(0.6,0.3,0.1)));

    }
    std::string rst = str2colorMap[material];
    if(rst == ""){
        std::cout<<"Unknown material type: "<<material<<std::endl;
        assert(false);
    }
    return rst;
}

//void material2color(std::string material, GrayPixel &color){
//static bool first = true;
//static std::map<std::string, unsigned char> str2colorMap;
//if (first){
//first = false;
//str2colorMap.insert(std::make_pair("CoalOre", 0));
//str2colorMap.insert(std::make_pair("Dandelion", 1));
//str2colorMap.insert(std::make_pair("Dirt", 2));
//str2colorMap.insert(std::make_pair("Gravel", 3));
//str2colorMap.insert(std::make_pair("IronOre", 4));
//str2colorMap.insert(std::make_pair("Leaves", 5));
//str2colorMap.insert(std::make_pair("Snow", 6));
//str2colorMap.insert(std::make_pair("Stone", 7));
//str2colorMap.insert(std::make_pair("TallGrass", 8));
//str2colorMap.insert(std::make_pair("TopSoil", 9));
//str2colorMap.insert(std::make_pair("Wood", 10));
//}
//color.v = str2colorMap[material];
//}

#endif /* end of include guard: COLORMAP_H */
