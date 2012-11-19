#include <iostream>
#include <algorithm>
#include <iterator>
#include <cv.h>
#include <stdio.h>
#include "m_opencv.h"
#include "m_algorithm.h"
#include <vector>
#include <map>
#include <string>
#include <xmlrpc-c/base.hpp>
#include "pixelworld3d.h" 
#include "pixelworld2d.h" 
#include "m_util.h"

using namespace std;
using namespace m_opencv;
using namespace m_lib;

int main(int argc, char** argv)
{ 
    if (argc != 2) {
        printf("Usage: %s <image>\n", argv[0]);
        return 1;
    }
    //// inquiry block data from server
    //AttrVector a;
    //AttrVector b;
    //AttrVector c;
    //get_block_attrs(a, b, c);
    //for(xmlrpc_c::value_string t: a){
        //std::cout<<std::string(t)<<std::endl;
        
    //}
    // segmentation

    //PixelWorld2D<GrayPixel2D> image(argv[1], true);
    PixelWorld3D<RgbPixel3D> image;
    image.construct_graph();
    image.save_segmentation();
    std::cout<<std::endl;
    

    return 0;
}

