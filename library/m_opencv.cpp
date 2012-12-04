#include <iostream>
#include <cv.h>
#include "m_opencv.h"
#include <string>
#include "m_algorithm.h"
#include "m_math.h" 
#define MIN3(x,y,z)  ((y) <= (z) ? \
        ((x) <= (y) ? (x) : (y)) \
        : \
        ((x) <= (z) ? (x) : (z)))

#define MAX3(x,y,z)  ((y) >= (z) ? \
        ((x) >= (y) ? (x) : (y)) \
        : \
        ((x) >= (z) ? (x) : (z)))
namespace m_opencv {

    double GrayColor::sum = 0;
    LuvColor LuvColor::sum = LuvColor();
    using namespace cv;
    void draw_circle(IplImage *pSrc, int x, int y, int radius, int color_gray){
        CvPoint center;
        center.x = x;
        center.y = y;
        CvScalar col;
        col.val[0] = color_gray;
        cvCircle(pSrc, center, radius, col);
    }
    void draw_rentangle(IplImage *pSrc, int x0, int y0, int x1, int y1, int color_gray){
        CvPoint leftmost;
        CvPoint rightmost;
        leftmost.x = x0;
        leftmost.y = y0;
        rightmost.x = x1;
        rightmost.y = y1;
        CvScalar color;
        color.val[0] = color_gray;
        cvRectangle(pSrc, leftmost, rightmost, color);
    }

    void fill_rectangle(GrayImage &img, int x0, int y0, int x1, int y1, int min, int max){
        int height = y1 - y0;
        int width = x1 - x0;
        GrayColor v;
        for (int row = 0; row <= height; row++) 
            for (int col = 0; col <= width; col++){
                //            v.v = m_math::rand_int(min, max);
                v.v = m_math::rand_int(min, max);
                img[y0 + row][x0 + col] = v;
            }
    }
    void rgb2luv(int R,int G, int B, LuvColor& luvdata)
    {
        float r, g, b,  X, Y, Z,  yr;
        float L;
        float eps = 216.f/24389.f;
        float k = 24389.f/27.f;
        float Xr = 0.964221f;  // reference white D50
        float Yr = 1.0f;
        float Zr = 0.825211f;
        // RGB to XYZ
        r = R/255.f; //R 0..1
        g = G/255.f; //G 0..1
        b = B/255.f; //B 0..1   
        // assuming sRGB (D65)
        if (r <= 0.04045)
            r = r/12;
        else
            r = (float) pow((r+0.055)/1.055,2.4);
        if (g <= 0.04045)
            g = g/12;
        else
            g = (float) pow((g+0.055)/1.055,2.4);
        if (b <= 0.04045)
            b = b/12;
        else
            b = (float) pow((b+0.055)/1.055,2.4);
        X =  0.436052025f*r     + 0.385081593f*g + 0.143087414f *b;
        Y =  0.222491598f*r     + 0.71688606f *g + 0.060621486f *b;
        Z =  0.013929122f*r     + 0.097097002f*g + 0.71418547f  *b;
        // XYZ to Luv
        float u, v, u_, v_, ur_, vr_;			
        u_ = 4*X / (X + 15*Y + 3*Z);
        v_ = 9*Y / (X + 15*Y + 3*Z);		 
        ur_ = 4*Xr / (Xr + 15*Yr + 3*Zr);
        vr_ = 9*Yr / (Xr + 15*Yr + 3*Zr);
        yr = Y/Yr;
        if ( yr > eps )
            L =  116*pow(yr,float(1/3.0)) - 16;
        else
            L = k * yr;
        u = 13*L*(u_ -ur_);
        v = 13*L*(v_ -vr_);
        luvdata.l = (int) (2.55*L + .5);
        luvdata.u = (int) (u + .5); 
        luvdata.v = (int) (v + .5);       
    } 

    IplImage* create_gray_image(const IplImage* pSrc)
    {
        IplImage* gray_img  = cvCreateImage(cvGetSize(pSrc),pSrc->depth,1);
        cvCvtColor(pSrc,gray_img,CV_BGR2GRAY); 
        return gray_img;
    }

    void output_img_info(const IplImage *img)
    {

        //    IplImage* img2 = NULL;
        //    img2 = cvCreateImage(cvGetSize(img),img->depth,img->nChannels);
        //    cvCopy(img,img2);
        std::cout<<"the width of image:"<<img->width<<std::endl;
        std::cout<<"the height of image:"<<img->height<<std::endl;
        std::cout<<"the widthStep of image:"<<img->widthStep<<std::endl;
        std::cout<<"the depth of image:"<<img->depth<<std::endl;
        std::cout<<"the number of channels in image:"<<img->nChannels<<std::endl;
        std::cout<<"the size of image( height * widthStep):"<<img->imageSize<<std::endl;

    }


    LuvColor* create_luv_image(const IplImage *pSrc)
    {
        long xstart=0;
        long deltapos=0;
        IplImage *temp = const_cast<IplImage*>(pSrc);
        RgbImage img(temp);
        int width = pSrc->width;
        int height = pSrc->height;
        LuvColor *luvData = new LuvColor[width * height];
        for (int i = 0; i < height; i++)
        {
            xstart = i*width;
            for ( int j = 0; j < width; j++)
            {
                deltapos = xstart + j;
                rgb2luv((int)img[i][j].r,(int)img[i][j].g , (int)img[i][j].b, luvData[deltapos] );
            }
        }
    }


    void show_image(const IplImage *image){

        //    IplImage* img =  cvLoadImage(argv[1], 1);
        cvNamedWindow("image", 1);
        cvShowImage("image",image);
        cvWaitKey(0);
        /* be tidy */
        cvDestroyAllWindows();
        //    cvReleaseImage(&img);
    }
    std::ostream& operator<<(std::ostream& out, const RgbColor& pixel){
        out<<"["<<(int)pixel.r<<", "<<(int)pixel.g<<", "<<(int)pixel.b<<"]";
        return out;
    }

    std::ostream& operator<<(std::ostream& out, const GrayColor& pixel){
        out<<"["<<(int)pixel.v<<"]";
        return out;
    }

    std::ostream& operator<<(std::ostream& out, const RgbImage& image){
        for (int row = 0; row < image.get_height(); row++) {
            for (int col = 0; col < image.get_width(); col++) {
                out<<image[row][col];
            }
            out<<std::endl;
        }
        return out;
    }

    std::ostream& operator<<(std::ostream& out, const GrayImage& image){
        for (int row = 0; row < image.get_height(); row++) {
            for (int col = 0; col < image.get_width(); col++) {
                out<<image[row][col];
            }
            out<<std::endl;
        }
        return out;
    }

    std::ostream& operator<<(std::ostream& out, const LuvColor& pixel){
        out<<"["<<(int)pixel.l<<", "<<(int)pixel.u<<", "<<(int)pixel.v<<"]";
        return out;
    }

    void test_rgbimage(const std::string &filename){
        IplImage* pSrc =  cvLoadImage(filename.data(), 1);
        RgbImage rgbs_s(pSrc);
        IplImage* img  = cvCreateImage(cvGetSize(pSrc),pSrc->depth,pSrc->nChannels);
        RgbImage rgbs_t(img);
        for (int r = 0; r < rgbs_s.get_height(); r++) {
            for (int col = 0; col < rgbs_s.get_width(); col++) {
                rgbs_t[r][col] = rgbs_s[r][col];
            }
        }
        rgbs_t.show();
    }

    int DELAY_CAPTION = 1500;
    int DELAY_BLUR = 100;
    int MAX_KERNEL_LENGTH = 5;

    char window_name[] = "Filter Demo 1";
    cv::Mat src, dst; 
    int display_caption( char* caption )
    {
        dst = Mat::zeros( src.size(), src.type() );
        putText( dst, caption,
                cv::Point( src.cols/4, src.rows/2),
                CV_FONT_HERSHEY_COMPLEX, 1, Scalar(255, 255, 255) );

        imshow( window_name, dst );
        int c = waitKey( DELAY_CAPTION );
        if( c >= 0 ) { return -1; }
        return 0;
    }

    int display_dst( int delay )
    {
        imshow( window_name, dst );
        int c = waitKey ( delay );
        if( c >= 0 ) { return -1; }
        return 0;
    }
    int blur(IplImage *input){
        cv::Mat temp(input);
        src = temp;
        //        IplImage *tt = new IplImage(t);


        namedWindow( window_name, CV_WINDOW_AUTOSIZE );

        /// 载入原图像

        if( display_caption( "Original Image" ) != 0 ) { return 0; }

        dst = src.clone();
        if( display_dst( DELAY_CAPTION ) != 0 ) { return 0; }

        //        /// 使用 均值平滑
        //        if( display_caption( "Homogeneous Blur" ) != 0 ) { return 0; }
        //
        //        for ( int i = 1; i < MAX_KERNEL_LENGTH; i = i + 2 )
        //        { blur( src, dst, Size( i, i ), cv::Point(-1,-1) );
        //            if( display_dst( DELAY_BLUR ) != 0 ) { return 0; } }

        /// 使用高斯平滑
        if( display_caption( "Gaussian Blur" ) != 0 ) { return 0; }

        for ( int i = 1; i < MAX_KERNEL_LENGTH; i = i + 2 )
        { GaussianBlur( src, dst, Size( i, i ), 0, 0 );
            if( display_dst( DELAY_BLUR ) != 0 ) { return 0; } }


            //                /// 使用中值平滑
            //                if( display_caption( "Median Blur" ) != 0 ) { return 0; }
            //
            //                for ( int i = 1; i < MAX_KERNEL_LENGTH; i = i + 2 )
            //                { medianBlur ( src, dst, i );
            //                    if( display_dst( DELAY_BLUR ) != 0 ) { return 0; } }
            //
            //                    /// 使用双边平滑
            //                    if( display_caption( "Bilateral Blur" ) != 0 ) { return 0; }
            //
            //                    for ( int i = 1; i < MAX_KERNEL_LENGTH; i = i + 2 )
            //                    { bilateralFilter ( src, dst, i, i*2, i/2 );
            //                        if( display_dst( DELAY_BLUR ) != 0 ) { return 0; } }


            IplImage *ttt = new IplImage(dst);
            cvSaveImage("blur.jpg", ttt);

            /// 等待用户输入
            display_caption( "End: Press a key!" );
            waitKey(0);
    }


    // convert rgb to hsv
    void rgb2hsv(RgbColor rgb, HsvColor& hsv) {
        double rgb_min, rgb_max;
        rgb_min = MIN3(rgb.r, rgb.g, rgb.b);
        rgb_max = MAX3(rgb.r, rgb.g, rgb.b);
        hsv.v = rgb_max;
        if (hsv.v == 0) {
            hsv.h = hsv.s = 0;
            return;
        }
        /* Normalize value to 1 */
        rgb.r /= hsv.v;
        rgb.g /= hsv.v;
        rgb.b /= hsv.v;
        rgb_min = MIN3(rgb.r, rgb.g, rgb.b);
        rgb_max = MAX3(rgb.r, rgb.g, rgb.b);
        hsv.s = rgb_max - rgb_min;
        if (hsv.s == 0) {
            hsv.h = 0;
            return; 
        }
        /* Normalize saturation to 1 */
        rgb.r = (rgb.r - rgb_min)/(rgb_max - rgb_min);
        rgb.g = (rgb.g - rgb_min)/(rgb_max - rgb_min);
        rgb.b = (rgb.b - rgb_min)/(rgb_max - rgb_min);
        rgb_min = MIN3(rgb.r, rgb.g, rgb.b);
        rgb_max = MAX3(rgb.r, rgb.g, rgb.b);
        /* Compute hue */
        if (rgb_max == rgb.r) {
            hsv.h = 0.0 + 60.0*(rgb.g - rgb.b);
            if (hsv.h < 0.0) {
                hsv.h += 360.0;
            }
        } else if (rgb_max == rgb.g) {
            hsv.h = 120.0 + 60.0*(rgb.b - rgb.r);
        } else /* rgb_max == rgb.b */ {
            hsv.h = 240.0 + 60.0*(rgb.r - rgb.g);
        }
        return;
    }

    //struct hsv_color {
    //unsigned char hue;        /* Hue degree between 0 and 255 */
    //unsigned char sat;        /* Saturation between 0 (gray) and 255 */
    //unsigned char val;        /* Value between 0 (black) and 255 */
    //};

    //struct hsv_color rgb_to_hsv(struct rgb_color rgb) {
    //struct hsv_color hsv;
    //unsigned char rgb_min, rgb_max;
    //rgb_min = MIN3(rgb.r, rgb.g, rgb.b);
    //rgb_max = MAX3(rgb.r, rgb.g, rgb.b);
    //hsv.val = rgb_max;
    //if (hsv.val == 0) {
    //hsv.hue = hsv.sat = 0;
    //return hsv;
    //}
    //hsv.sat = 255*long(rgb_max - rgb_min)/hsv.val;
    //if (hsv.sat == 0) {
    //hsv.hue = 0;
    //return hsv;
    //}
    ///* Compute hue */
    //if (rgb_max == rgb.r) {
    //hsv.hue = 0 + 43*(rgb.g - rgb.b)/(rgb_max - rgb_min);
    //} else if (rgb_max == rgb.g) {
    //hsv.hue = 85 + 43*(rgb.b - rgb.r)/(rgb_max - rgb_min);
    //} else /* rgb_max == rgb.b */ {
    //hsv.hue = 171 + 43*(rgb.r - rgb.g)/(rgb_max - rgb_min);
    //}
    //return hsv;
    //}
} /* m_opencv */
