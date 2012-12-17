#include "util.h"
using namespace m_opencv;
std::vector<RgbColor>& components_color( ){
    static std::vector<RgbColor> marks;
    if(marks.size() == 0){

        RgbColor color;
        color.r = 234;
        color.g = 16;
        color.b = 7;
        marks.push_back(color);

        color.r = 162;
        color.g = 210;
        color.b = 101;
        marks.push_back(color);

        color.r = 28;
        color.g = 166;
        color.b = 205;
        marks.push_back(color);

        color.r = 69;
        color.g = 183;
        color.b = 17;
        marks.push_back(color);

        color.r = 255;
        color.g = 112;
        color.b = 117;
        marks.push_back(color);

        color.r = 184;
        color.g = 255;
        color.b = 92;
        marks.push_back(color);

        color.r = 162;
        color.g = 92;
        color.b = 255;
        marks.push_back(color);

        color.r = 137;
        color.g = 242;
        color.b = 218;
        marks.push_back(color);
    }
    return marks;
}
