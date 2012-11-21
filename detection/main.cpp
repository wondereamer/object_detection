//#include <iostream>
//#include <algorithm>
//#include <iterator>
//#include <cv.h>
//#include <stdio.h>
//#include "m_opencv.h"
//#include "m_algorithm.h"
//#include <vector>
//#include <map>
//#include <string>
//#include <xmlrpc-c/base.hpp>
//#include "pixelworld3d.h" 
//#include "pixelworld2d.h" 
//#include "m_util.h"

//using namespace std;
//using namespace m_opencv;
//using namespace m_lib;

//int main(int argc, char** argv)
//{ 
//if (argc != 2) {
//printf("Usage: %s <image>\n", argv[0]);
//return 1;
//}
////// inquiry block data from server
////AttrVector a;
////AttrVector b;
////AttrVector c;
////get_block_attrs(a, b, c);
////for(xmlrpc_c::value_string t: a){
////std::cout<<std::string(t)<<std::endl;

////}
//// segmentation

////PixelWorld2D<GrayPixel2D> image(argv[1], true);
//PixelWorld3D<RgbPixel3D> image;
//image.construct_graph();
//image.save_segmentation();
//std::cout<<std::endl;


//return 0;
//}
#include "../test/test.h" 
int main()
{
    test_VizGraph();
    //Graph G;
    //GraphAttributes GA(G, GraphAttributes::nodeGraphics |	
    //GraphAttributes::edgeGraphics );

    //const int LEN = 11;

    //node left = G.newNode();
    //GA.x(left) = 1;
    //GA.y(left) = 1;
    //GA.width(left) = 3;
    //GA.height(left) = 3;

    //node bottom = G.newNode();
    //GA.x(bottom) = 10;
    //GA.y(bottom) = 10;
    //GA.width(bottom) = 30;
    //GA.height(bottom) = 50;

    //edge e = G.newEdge(left,bottom);
    ////for(int i = 1; i<LEN; ++i) {
    ////DPolyline &p = GA.bends(e);
    ////p.pushBack(DPoint(10,-20*i));
    ////p.pushBack(DPoint(20*(LEN-i),-10));
    ////}

    //GA.writeGML("manual_graph.gml");

    return 0;
}
