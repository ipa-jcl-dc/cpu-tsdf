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
int debug = 1;
//Image data
std::vector<cv::Mat> images;

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

void readImg (string filename,cv::Mat &img)
{
  string imagefile;
  imagefile = filename;
  //imagefile.append (suffix);
  img = cv::imread(imagefile,1);
}
void loadImages(std::vector<cv::Mat>& images, const string& root_dir)
{


ifstream index;
index.open(string(root_dir+"/input_image.txt").c_str());
while(!index.eof())
{
  cv::Mat temp;
  string file;
  index>>file;
  if(file.empty())
    break;
  file = root_dir+"/"+"color-"+file;
  readImg (file,temp);
  cout << "loading image:  " << file << endl;
  images.push_back(temp);
 if(0)
 {
   cv::namedWindow(file,cv::WINDOW_NORMAL);
   cv::imshow(file,temp);
   cv::waitKey(0);
 }
}
}


int main(int argc,char** argv)
{
    //std::string path = argv[1];
    std::string root_dir = argv[1];
    std::string data_path = "../data_collection";
    std::string segmentation_path = root_dir +"/segmentation";
    int flag_created=  system(("mkdir -p " + segmentation_path).c_str());
    loadImages(images,root_dir);
    readParameters();

    std::ifstream index_mask,index_segmented;
    index_mask.open(string(data_path+"/image_mask.txt").c_str());
    index_segmented.open(string(data_path+"/image_segmented.txt").c_str());

    for(int i=0;i<images.size();i++)
    {
    cout<<"frame "<<i<<endl;
    cv::Mat img;
    img = images[i];
    cv::Mat hsv,hsv_result;
    cv::cvtColor(img,hsv,CV_RGB2HSV);
    cv::Mat hsv_mask;
    cv::Mat img_mask;
    //Create a mask based on HSV values
    cv::inRange(hsv,cv::Scalar(h_lower,s_lower,v_lower),cv::Scalar(h_upper,s_upper,v_upper),hsv_result);
    //Create a white image of hsv_result
    hsv_mask = cv::Mat(hsv_result.size(),hsv_result.type(),cv::Scalar(255));

     //crop region of interest of hsv result
    cv::Point top_left = cv::Point(xmin,ymin);
    cv::Point bottom_right = cv::Point(xmax,ymax);
    cv::Rect roi(top_left,bottom_right);
    cv::Mat crop = hsv_result(roi);
    //Copy to region of interest to hsv mask
    crop.copyTo(hsv_mask(cv::Rect(xmin,ymin,crop.cols,crop.rows)));
    //convert 8UC1 to 8UC3
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
    if(debug)
    {
    cv::imshow("img_mask",img_mask);
    //cv::imshow("result_mask",result_mask);
    cv::imshow("result",result);
    cv::waitKey(0);
    }
    //Save
    std::string file_segmented,file_mask;
    index_mask>>file_mask;
    if(file_mask.empty())
    {
        cout<<"no mask file"<<endl;
        break;
    }
    index_segmented>>file_segmented;
        if(file_segmented.empty())
        {
            cout<<"no segmented file"<<endl;
            break;
        }

    file_segmented = segmentation_path+"/"+file_segmented;
    file_mask = segmentation_path+"/"+file_mask;
    cv::imwrite(file_segmented,result);
    cv::imwrite(file_mask,img_mask);
    }
    return 0;
}
