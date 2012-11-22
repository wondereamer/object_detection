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
        PixelWorld3D<RgbPixel3D> image;
        //printf("Usage: %s <image>\n", argv[0]);
        //return 1;
        image.construct_graph();
        image.save_segmentation();

    }else{

        PixelWorld2D<GrayPixel2D> image(argv[1], false);
        image.construct_graph();
        image.save_segmentation();
    }


    return 0;
}
