//C++
#include <curl/curl.h>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <sstream>
#include <istream>
#include <string>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/crop_box.h>
#include "segmentation.hpp"
#include "filters.hpp"
#include "util.hpp"
#include "matrix_utils.hpp"
#include "aruco_marker.hpp"
#include "realsense2.hpp"
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include <Eigen/Core>
#include <opencv2/core/eigen.hpp>
#include <opencv2/opencv.hpp>

using namespace std;
const int w =640;
const int h = 480;
static float min_x = 0.0;
static float max_x = 0.3915;
static float min_y = 0.00;
static float max_y = 0.28575;
static float min_z = 0.007;
static float max_z = 0.4;

int main(int argc,char** argv)
{
     Segmentation segmentation;
    Filters filter;

    Marker aruco_marker;
    RealSense2 realsense2;
    cv::Mat camera_matrix = realsense2.getCameraMatrix();
    cv::Mat charuco_board;
    std::string aruco_board_file;
    cv::Mat converted_image;
    if(argc < 2)
    {
        std::cout <<"Usage : ./test_functions board_xml_path"<<std::endl;
        return EXIT_FAILURE;
    }
    aruco_board_file = argv[1]; //Aruco board parameters
    //Set up charuco marker with intrinsic parameters of each camera
        aruco_marker.setCameraMatrix(camera_matrix);
        aruco_marker.LoadParameters(aruco_board_file);
        aruco_marker.createChaRucoMarker(charuco_board);


    char c =0;
    while((c=(char)cv::waitKey(10)) !=27)
    {
        aruco_pose pose;
        int pose_flag;
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZRGB>);
        cv::Mat depth;
        cv::Mat img;
        realsense2.waitForFrames();

        realsense2.getPointCloud(cloud);
        realsense2.getColorImage(img);
        pose_flag = aruco_marker.estimatePoseCharuco(img,pose);
        if(pose_flag !=1) cv::imshow("object contours",img);
        else{
            Eigen::Vector3f T;
            Eigen::Matrix3f R;
            std::vector<int> indices;
            T.setOnes();
            R.setIdentity();
            Eigen::Matrix4f marker_to_camera;
            Eigen::Matrix4f camera_to_marker;
            cv::cv2eigen(pose.rot,R);
            cv::cv2eigen(pose.trans,T);
            marker_to_camera.setIdentity();   // Set to Identity to make bottom row of Matrix 0,0,0,1
            marker_to_camera.block<3,3>(0,0) = R;
            marker_to_camera.block<3,1>(0,3) = T;
            camera_to_marker = marker_to_camera.inverse();
            pcl::transformPointCloud(*cloud,*cloud,camera_to_marker);
            filter.CropBox(cloud,min_x,min_y,min_z,max_x,max_y,max_z,indices);
            segmentation.cloud2image(converted_image,cloud,indices);
            segmentation.objectContour(img,converted_image);
            cv::imshow("object contours",img);
        }
    }

return EXIT_SUCCESS;
}
