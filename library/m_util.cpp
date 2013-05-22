#include "m_util.h"
#include <iostream>
#include <xmlrpc-c/girerr.hpp>
#include <xmlrpc-c/base.hpp>
#include <xmlrpc-c/client_simple.hpp>
#include <boost/filesystem.hpp>
#include <string>
#include <algorithm>
#include <vector>
#include <limits>
using namespace std;
namespace m_util  {
    using namespace std;



    
    xmlrpc_c::value call_py( const string &method_name, xmlrpc_c::paramList params, 
            int port, const string &server_url)
    {
        // set the limit size of xmlrpc
        xmlrpc_limit_set(XMLRPC_XML_SIZE_LIMIT_ID, 5e6);
        std::string url = server_url + ":" + sth2string<int>(port) + "/RPC2";
        std::cout<<"calling remote server on port "<<port<<"..."<<std::endl;
        xmlrpc_c::value result;
        try {
            xmlrpc_c::clientSimple myClient;
            myClient.call(url, method_name, params, &result);

        } catch (exception const& e) {
            cerr << "Client threw error: " << e.what() << endl;
        } catch (...) {
            cerr << "Client threw unexpected error." << endl;
        }
        return result;
    }

    std::string string_format(const std::string &fmt, ...) {
        int size = 100;
        std::string str;
        va_list ap;
        while (1) {

            str.resize(size);
            va_start(ap, fmt);
            int n =  vsnprintf((char *)str.c_str(), size, fmt.c_str(), ap);
            va_end(ap);
            if (n > -1 && n < size) {
                str.resize(n);
                return str;
            }
            if (n > -1)
                size = n+1;
            else
                size*=2;
        }
    }

    bool is_nun(double a){
        return a != a;
    }
    double create_nun()
    {

//        unsigned long nan[2]={0xffffffff, 0x7fffffff};    // code representing a NaN
//        return *( double*)nan;    
        return numeric_limits<double>::quiet_NaN();
    }

    //*********************************************************************
    using namespace boost::filesystem;
    void files_in_path(const std::string &fname){
        path p (fname);   // p reads clearer than argv[1] in the following code
        try {
            if (exists(p))    // does p actually exist?
            {
                if (is_regular_file(p))        // is p a regular file?
                    cout << p << " size is " << file_size(p) << '\n';
                else if (is_directory(p))      // is p a directory?
                {
                    cout << p << " is a directory containing:\n";
                    typedef vector<path> vec;             // store paths,
                    vec v;                                // so we can sort them later
                    copy(directory_iterator(p), directory_iterator(), back_inserter(v));
                    sort(v.begin(), v.end());             // sort, since directory iteration
                    // is not ordered on some file systems
                    for (vec::const_iterator it(v.begin()), it_end(v.end()); it != it_end; ++it)
                    {
                        cout << "   " << *it << '\n';
                    }
                }
                else
                    cout << p << " exists, but is neither a regular file nor a directory\n";
            }
            else
                cout << p << " does not exist\n";
        }catch (const filesystem_error& ex)
        {
            cout << ex.what() << '\n';
        }

    }
    //void make_pixel_force_jpg(){
    //CvSize size;
    //size.width = 400;
    //size.height = 400;
    //IplImage* temp  = cvCreateImage(size, IPL_DEPTH_8U,1);

    ////         GaussianBlur( src, dst, Size( i, i ), 0, 0 );
    //GrayColor white;
    //white.v = 255;
    //GrayImage gray_img(temp);
    //gray_img.set_color(white);
    //draw_rentangle(temp, 90, 90, 310, 310, 0);
    //draw_rentangle(temp, 120, 140, 220, 240, 0);

    //fill_rectangle(gray_img, 0, 0, 399, 399, 0, 80);
    //fill_rectangle(gray_img, 90, 90, 310, 310, 81, 160);
    //fill_rectangle(gray_img, 120, 140, 220, 240, 161, 255);

    //gray_img.show();
    //gray_img.save("test_force.jpg");
    //gray_img.output_img_info();
    //}

} /* m_lib  */    

