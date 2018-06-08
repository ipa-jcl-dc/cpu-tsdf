//C/C++
#include <fstream>
#include <iomanip>
#include <iostream>
#include <string>
#include <cstdlib>
#include <unistd.h>
#include <sstream>
#include <stdlib.h>
#include <chrono>
#include <ctime>
#include <boost/filesystem.hpp>
#include <boost/shared_ptr.hpp>
#include <boost/make_shared.hpp>
//#include "segmentation.hpp"
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include <Eigen/Core>
using namespace std;

//Bounding box
int xmin,xmax,ymin,ymax;
//HSV values
int h_lower,h_upper,s_lower,s_upper,v_lower,v_upper;
int debug =0;
cv::Mat img;
cv::Mat hsv,dst;
int l_h = 0;
int u_h = 70;

int l_s =0;
int u_s = 255;

int l_v = 0;
int u_v = 255;

void on_trackbar(int, void* ) {
    l_h = std::min(u_h-1,l_h);
    u_h = std::max(u_h,l_h+1);

    l_s = std::min(u_s-1,l_s);
    u_s = std::max(u_s,l_s+1);

    l_v = std::min(u_v-1,l_v);
    u_v = std::max(u_v,l_v+1);

    cv::setTrackbarPos("L_H","result",l_h);
    cv::setTrackbarPos("L_S","result",l_s);
    cv::setTrackbarPos("L_V","result",l_v);
    cv::setTrackbarPos("U_H","result",u_h);
    cv::setTrackbarPos("U_S","result",u_s);
    cv::setTrackbarPos("U_V","result",u_v);

    cv::inRange(hsv,cv::Scalar(l_h,l_s,l_v),cv::Scalar(u_h,u_s,u_v),dst);
    cv::imshow("result", dst );
}
void readParameters()
{
    std::string file_path = "../data_color/params.yaml";
    cv::FileStorage fs(file_path,cv::FileStorage::READ);
    fs["xmin"]>>xmin;
    fs["xmax"]>>xmax;
    fs["ymin"]>>ymin;
    fs["ymax"]>>ymax;
    fs["h_lower"]>>h_lower;
    fs["h_upper"]>>h_upper;
    fs["s_lower"]>>s_lower;
    fs["s_upper"]>>s_upper;
    fs["v_lower"]>>v_lower;
    fs["v_upper"]>>v_upper;
    fs["debug"]>>debug;
}

int main( int argc, char** argv ) {


    std::string file_name = argv[1];
    img = cv::imread(file_name,1);
    readParameters();
    cv::cvtColor(img,hsv,CV_RGB2HSV);
    cv::imshow("hsv", hsv );
    cv::imshow("original",img);
    /// Create Windows
    cv::namedWindow("result", 1);
    /// Create Trackbars
    cv::createTrackbar("L_H", "result", &l_h, 179, on_trackbar);
    cv::createTrackbar("L_S", "result", &l_s, 255, on_trackbar);
    cv::createTrackbar("L_V", "result", &l_v, 255, on_trackbar);
    cv::createTrackbar("U_H", "result", &u_h, 179, on_trackbar);
    cv::createTrackbar("U_S", "result", &u_s, 255, on_trackbar);
    cv::createTrackbar("U_V", "result", &u_v, 255, on_trackbar);

    /// Show some stuff
    cv::waitKey(0);
    //Create a white image of hsv_result
   cv::Mat hsv_mask = cv::Mat(dst.size(),dst.type(),cv::Scalar(255));
    //crop region of interest of hsv result
   cv::Point top_left = cv::Point(xmin,ymin);
   cv::Point bottom_right = cv::Point(xmax,ymax);
   cv::Rect roi(top_left,bottom_right);
   cv::Mat crop = dst(roi);
   //Copy to region of interest to hsv mask
   crop.copyTo(hsv_mask(cv::Rect(xmin,ymin,crop.cols,crop.rows)));
   //convert 8UC1 to 8UC3
   cv::Mat img_mask;
   hsv_mask.convertTo(img_mask,CV_8UC3);
   //Invert img_mask
   img_mask = 255.0 - img_mask;
   //Do erosion
   // Create a structuring element (SE)
   int morph_size = 1;
   cv::Mat element = cv::getStructuringElement(cv::MORPH_RECT, cv::Size( 2*morph_size + 1,2*morph_size+1),
                                        cv::Point(morph_size,morph_size));
   //result after mophological methods
   //remove black pixels inside white mask
   cv::erode(img_mask, img_mask,element);
   //
   //cv::dilate(img_mask, img_mask,element);

   //Perfor bitwise
   cv::Mat result;
   cv::bitwise_and(img,img,result,img_mask);

    cv::imwrite(file_name,result);
    cv::imwrite("../mask.png",img_mask);

    return 0;
}
