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

    struct RgbPixel{
        unsigned char b,g,r;
        bool operator==(const RgbPixel& other) const {
            return b == other.b && g == other.g && r == other.r;
        }

        bool operator!=(const RgbPixel& other) const {
            return !(*this ==(other));
        }
    } ;

    struct GrayPixel{
        unsigned char v;
        bool operator==(const GrayPixel& other) const {
            return (v == other.v);
        }
    };

    struct HsvPixel{
        double h;        /* Hue degree between 0.0 and 360.0 */
        double s;        /* Saturation between 0.0 (gray) and 1.0 */
        double v;        /* Value between 0.0 (black) and 1.0 */
    };


    typedef Image<RgbPixel> RgbImage;
    typedef Image<GrayPixel> GrayImage;
    //    RgbImage  imgR(pSrc);
    struct LuvPixel
    {
        float l;
        float u;
        float v;
    };

    int blur(IplImage *input);
    //
    void      rgb2luv(int R,int G, int B, LuvPixel &luvdata);
    IplImage* create_gray_image(const IplImage* psrc);
    LuvPixel* create_luv_image(const IplImage *pSrc);
    void    output_img_info(const IplImage *img);
    void    show_image(const IplImage *image);
    void    draw_circle(IplImage *pSrc, int x, int y, int radius, int color_gray);
    void    draw_rentangle(IplImage *pSrc, int x0, int y0, int x1, int y1, int color_gray);
//    void    fill_rectangle(IplImage *pSrc, int x0, int y0, int x1, int y1, int ctrst0, int ctrst1);
    void    fill_rectangle(GrayImage &pSrc, int x0, int y0, int x1, int y1, int min, int max);
    void rgb2hsv(RgbPixel rgb, HsvPixel& hsv);

//    //! fill rectangle, including point (x0,x0), (x1,y1)
//    template < typename T >
//    void fill_rectangle(const T& img, int x0, int y0, int x1, int y1, T min, T max){
//        int height = y1 - y0;
//        int width = x1 - x0;
//        for (int row = 0; row <= height; row++) 
//            for (int col = 0; col <= width; col++)
//                img[y0 + row][x0 + col] = min;
//    }

    
    std::ostream& operator<<(std::ostream& out, const RgbPixel& pixel);
    std::ostream& operator<<(std::ostream& out, const GrayPixel& pixel);
    std::ostream& operator<<(std::ostream& out, const RgbImage& image);
    std::ostream& operator<<(std::ostream& out, const GrayImage& image);
    std::ostream& operator<<(std::ostream& out, const LuvPixel& pixel);
}; /* m_opencv */
#endif /* end of include guard: M_OPENCV.H */

