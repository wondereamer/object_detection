/**
 * @file m_opencv.h
 * @brief image process related library
 * @author Dignjie.Wang (dingjie.wang@gmail.com)
 * @version 0.1
 * @date 2012-10-10
 */

#ifndef M_OPENCV_H

#define M_OPENCV_H

#include <highgui.h>
#include <string>
#include <iostream>
#include <cmath>
#include <map>
namespace m_opencv {
    using std::string;


    template<class T> class Image
    {
        private:
            IplImage* _pSrc;
        public:
            Image(IplImage* img):_pSrc(img) {}
            ~Image(){};
            void operator = (IplImage* img) {_pSrc = img;}

            T* operator[](const int rowIndx) {
                return ((T *)(_pSrc->imageData + rowIndx*_pSrc->widthStep));}

            const T* operator[](const int rowIndx) const{
                return ((T *)(_pSrc->imageData + rowIndx*_pSrc->widthStep));}

            void set_color(const T &color) {
                for (int row = 0; row < this->get_height(); row++) 
                    for (int col = 0; col < this->get_width(); col++) 
                        (*this)[row][col] = color;
            }
            void save(const string &filename){
                cvSaveImage(filename.data(), _pSrc);
            }
            void show()const {
                cvNamedWindow("image", 1);
                cvShowImage("image", _pSrc);
                cvWaitKey(0);
                cvDestroyAllWindows();
            }
            void output_img_info() const {
                std::cout<<"the width of image:"<<_pSrc->width<<std::endl;
                std::cout<<"the height of image:"<<_pSrc->height<<std::endl;
                std::cout<<"the widthStep of image:"<<_pSrc->widthStep<<std::endl;
                std::cout<<"the depth of image:"<<_pSrc->depth<<std::endl;
                std::cout<<"the number of channels in image:"<<_pSrc->nChannels<<std::endl;
                std::cout<<"the size of image( height * widthStep):"<<_pSrc->imageSize<<std::endl;
            }


            const IplImage* get__pSrc() const{
                return _pSrc;
            }
            int get_width() const{
                return _pSrc->width;
            }
            int get_height() const{
                return _pSrc->height;
            }
            int get_nChannels() const{
                return _pSrc->nChannels;
            }
            int get_depth() const{
                return _pSrc->depth;
            }
    }; 

    struct RgbColor{
        unsigned char b,g,r;
        bool operator==(const RgbColor& other) const {
            return b == other.b && g == other.g && r == other.r;
        }
        RgbColor(){ }
        RgbColor(unsigned char r_, unsigned char g_,
                unsigned char b_):r(r_), g(g_), b(b_){ }

        bool operator!=(const RgbColor& other) const {
            return !(*this ==(other));
        }
        bool operator<(const RgbColor& other) const{
            if( r < other.r)
                return true;
            else if(r > other.r)
                return false;
            else if(g < other.g)
                return true;
            else if(g > other.g)
                return false;
            else if(b < other.b)
                return true;
            else return false;
        }

        static RgbColor white_color(){
            RgbColor white;
            white.r = 255;
            white.g = 255;
            white.b = 255;
            return white;
        }
    } ;

    struct GrayColor{
        unsigned char v;
        bool operator==(const GrayColor& other) const {
            return (v == other.v);
        }
        static unsigned char color_distance(const GrayColor& a, 
                const GrayColor &b){
            return abs(a.v - b.v);
        }
        static void add2colorpool(const GrayColor &c){
            sum += (double)c.v;
        }
        static GrayColor calcu_average_color(int size){
            GrayColor color;
            color.v = sum / size;
            // clear the color pool
            sum = 0;
            return color;
        }

        static GrayColor white_color(){
            GrayColor white;
            white.v = 255;
            return white;
        }

        //! help to calculate average color of a color pool
        static double sum;
    };

    struct HsvColor{
        double h;        /* Hue degree between 0.0 and 360.0 */
        double s;        /* Saturation between 0.0 (gray) and 1.0 */
        double v;        /* Value between 0.0 (black) and 1.0 */
    };


    typedef Image<RgbColor> RgbImage;
    typedef Image<GrayColor> GrayImage;
    //    RgbImage  imgR(pSrc);
    struct LuvColor
    {
        float l;
        float u;
        float v;
        LuvColor(float l_ = 0, float u_ = 0, float v_ = 0){
            l = l_;
            u = u_;
            v = v_;
        }
        bool operator==(const LuvColor& other) const {
            return (v == other.v);
        }
        static double color_distance(const LuvColor& a, 
                const LuvColor &b){
            return sqrt(pow(a.l - b.l, 2) +  pow(a.v - b.v, 2) + pow(a.u - b.u, 2));
        }
        static void add2colorpool(const LuvColor &c){
            sum.l += c.l;
            sum.u += c.u;
            sum.v += c.v;
        }
        static LuvColor calcu_average_color(int size){
            LuvColor color;
            color.l = sum.l / size;
            color.u = sum.u / size;
            color.v = sum.v / size;
            // clear the color pool
            sum.l = 0;
            sum.u = 0;
            sum.v = 0;
            return color;
        }
        //! help to calculate average color of a color pool
        static LuvColor sum;
        /*static void color_pool(RgbColor &c){*/
        /*static double  */
        /*}*/
        /*static RgbColor averageColor;*/
    };

    void     rgb2luv(int R,int G, int B, LuvColor &luvdata);
    LuvColor rgb2luv(const RgbColor &rgb);
    class RandomColor{
        public:

            typedef std::map<std::string, RgbColor> ColorMap;
            RandomColor(double dist){
                _dist = dist;
//               _idxDefColors.push_back(_defColors.insert(std::make_pair("#66BBAE",
//                                                 RgbColor(234, 16, 7))).first);
//               _idxDefColors.push_back(_defColors.insert(std::make_pair("#9DD4FF",
//                                                 RgbColor(28, 166, 205))).first);
//               _idxDefColors.push_back(_defColors.insert(std::make_pair("#D53533",
//                                                 RgbColor(69, 183, 17))).first);
//               _idxDefColors.push_back(_defColors.insert(std::make_pair("#509467",
//                                                 RgbColor(255, 112, 117))).first);
//               _idxDefColors.push_back(_defColors.insert(std::make_pair("#A6CD1B",
//                                                 RgbColor(184, 255,92))).first);
//               _idxDefColors.push_back(_defColors.insert(std::make_pair("#ED9F9F",
//                                                 RgbColor(162, 92, 255))).first);
//               _idxDefColors.push_back(_defColors.insert(std::make_pair("#373A7F",
//                                                   RgbColor(137, 242, 218))).first);
//               _idxDefColors.push_back(_defColors.insert(std::make_pair("#8FBC8F",
//                                                 RgbColor(162, 210, 101))).first);

               _idxDefColors.push_back(_defColors.insert(std::make_pair("#66BBAE",
                                                 RgbColor(0,0,0))).first);
               _idxDefColors.push_back(_defColors.insert(std::make_pair("#66BBAE",
                                                 RgbColor(35,35,35))).first);
               _idxDefColors.push_back(_defColors.insert(std::make_pair("#9DD4FF",
                                                 RgbColor(70,70,70))).first);
               _idxDefColors.push_back(_defColors.insert(std::make_pair("#D53533",
                                                 RgbColor(105,105,105))).first);
               _idxDefColors.push_back(_defColors.insert(std::make_pair("#509467",
                                                 RgbColor(140,140,140))).first);
               _idxDefColors.push_back(_defColors.insert(std::make_pair("#A6CD1B",
                                                 RgbColor(175,175,175))).first);
               _idxDefColors.push_back(_defColors.insert(std::make_pair("#ED9F9F",
                                                 RgbColor(210,210,210))).first);
               _idxDefColors.push_back(_defColors.insert(std::make_pair("#373A7F",
                                                   RgbColor(245,245,245))).first);
//               _idxDefColors.push_back(_defColors.insert(std::make_pair("#8FBC8F",
//                                                 RgbColor(162, 210, 101))).first);
            }
            RgbColor random_color();
            RgbColor diff_random_color();

            void accept_color(const RgbColor &c){
                LuvColor luv(rgb2luv(c));
                _acceptedColors.push_back(luv);
            }
            void pop_color(){
                _acceptedColors.pop_back();
            }
            void set_def_color(const ColorMap &t){
                _idxDefColors.clear();
                _defColors.clear();
                for(auto &p : t){
                   _idxDefColors.push_back(_defColors.insert(p).first);
                }
            }
        public:
            double _dist;
            std::vector<LuvColor> _acceptedColors;
            ColorMap _defColors;
            std::vector<ColorMap::const_iterator> _idxDefColors;

    };

    int blur(IplImage *input);
    //
    IplImage* create_gray_image(const IplImage* psrc);
    LuvColor* create_luv_image(const IplImage *pSrc);
    void    output_img_info(const IplImage *img);
    void    show_image(const IplImage *image);
    void    draw_circle(IplImage *pSrc, int x, int y, int radius, int color_gray);
    void    draw_rentangle(IplImage *pSrc, int x0, int y0, int x1, int y1, int color_gray);
    //    void    fill_rectangle(IplImage *pSrc, int x0, int y0, int x1, int y1, int ctrst0, int ctrst1);
    void    fill_rectangle(GrayImage &pSrc, int x0, int y0, int x1, int y1, int min, int max);
    void rgb2hsv(RgbColor rgb, HsvColor& hsv);

    //    //! fill rectangle, including point (x0,x0), (x1,y1)
    //    template < typename T >
    //    void fill_rectangle(const T& img, int x0, int y0, int x1, int y1, T min, T max){
    //        int height = y1 - y0;
    //        int width = x1 - x0;
    //        for (int row = 0; row <= height; row++) 
    //            for (int col = 0; col <= width; col++)
    //                img[y0 + row][x0 + col] = min;
    //    }


    std::ostream& operator<<(std::ostream& out, const RgbColor& pixel);
    std::ostream& operator<<(std::ostream& out, const GrayColor& pixel);
    std::ostream& operator<<(std::ostream& out, const RgbImage& image);
    std::ostream& operator<<(std::ostream& out, const GrayImage& image);
    std::ostream& operator<<(std::ostream& out, const LuvColor& pixel);
}; /* m_opencv */
#endif /* end of include guard: M_OPENCV.H */

