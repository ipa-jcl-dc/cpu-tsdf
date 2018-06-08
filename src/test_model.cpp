//C/C++
#include <fstream>
#include <iomanip>
#include <iostream>
#include <string>
#include <cstdlib>
#include <unistd.h>
#include <sstream>
#include <stdlib.h>
#include <boost/filesystem.hpp>
#include <boost/shared_ptr.hpp>
#include <boost/make_shared.hpp>
//OpenCV
#include <opencv2/opencv.hpp>
#include <Eigen/Core>
#include <opencv2/core/eigen.hpp>

//PCL
#include <pcl/common/common_headers.h>
#include <pcl/io/io.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <boost/thread/thread.hpp>
#include <pcl/console/parse.h>
#include <pcl/common/transforms.h>
#include <pcl/features/board.h>
#include <pcl/console/parse.h>
#include <pcl/io/vtk_io.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/features/normal_3d.h>
#include <pcl/surface/gp3.h>
#include <pcl/surface/texture_mapping.h>
#include <pcl/TextureMesh.h>
#include <pcl/io/obj_io.h>
#include <pcl/io/vtk_lib_io.h>
//Realsense camera
#include <librealsense2/rs.hpp>
#include <librealsense2/rsutil.h>
//Header files
#include "util.hpp"
#include "registrator.hpp"
#include "filters.hpp"
#include "segmentation.hpp"
#include "loader.hpp"
#include "aruco_marker.hpp"
#include "matrix_utils.hpp"
#include "config.hpp"
#include "dynamixel.h"
#include "Dynamixel.h"
#include "realsense.hpp"
#include "util/random_utils.h"
#include "util/depth_utils.h"
#include "kinect_fusion.hpp"
cv::Mat camera_matrix =  (cv::Mat_<double>(3,3)<<   619.69,0,313.013 ,
                                                 0,619.69, 245.14,
                                                    0,0,1);

using namespace std;


int main (int argc, char** argv) try
{
    std::string dir_path;

    std::string aruco_board_file = argv[1]; //Aruco board parameters

    boost::shared_ptr<RealSense> realsense = boost::make_shared<RealSense>();
    boost::shared_ptr<Filters> filters = boost::make_shared<Filters>();
    boost::shared_ptr<Segmentation> segmentation = boost::make_shared<Segmentation>();
    boost::shared_ptr<Registrator> registrator = boost::make_shared<Registrator>();
    std::vector<Eigen::Matrix4f> transforms;
    boost::shared_ptr<Utils> utils = boost::make_shared<Utils>();
    boost::shared_ptr<Marker> aruco_marker = boost::make_shared<Marker>();

    std::vector<cv::Mat> camera_matrices;
    vector<std::string> serial_numbers; //serial number of connected camra
    float axisLength = 0.1 ; //10 cm

    // Get list of cameras
    rs2::device_list devices = realsense->get_device_list();
    //Enable all camera and store serial numbers for further processing
    for (rs2::device device : devices)
      {
          realsense->enable_device(device);
          serial_numbers.push_back(realsense->get_serial_number(device));
      }
    realsense->setConfiguration("../config/realsense_parameters.yaml");
    for(int i=0;i<40;i++) realsense->poll_frames(); //wait some frame for effect to take place
    cv::Mat charuco_board;
    realsense->getCameraInformation(camera_matrices);

    aruco_marker->setCameraMatrix(camera_matrices[0]);
    aruco_marker->LoadParameters(aruco_board_file);
    aruco_marker->createChaRucoMarker(charuco_board);
    char c =0;
    while((c=(char)cv::waitKey(5)) !=27)
    {

        realsense->poll_frames();
       // for(int i=0;i<30;i++) realsense->poll_frames(); //wait some frame for effect to take place


                    cv::Mat img = realsense->displayColorImages(serial_numbers[0]);
                    aruco_pose pose;
                    int pose_flag =0;
                    pose_flag = aruco_marker->estimatePoseCharuco(img,pose);
                    if(pose_flag ==1)
                    {
                        cv::aruco::drawAxis(img, camera_matrices[0], cv::Mat(), pose.rot, pose.trans, axisLength);
                        cv::imshow("pose",img);
                    }
                    else
                        cv::imshow("pose",img);

  }
  return EXIT_SUCCESS;
}
catch (const rs2::error & e)
{
    std::cerr << "RealSense error calling " << e.get_failed_function() << "(" << e.get_failed_args() << "):\n    " << e.what() << std::endl;
    return EXIT_FAILURE;
}
catch (const std::exception & e)
{
    std::cerr << e.what() << std::endl;
    return EXIT_FAILURE;
}



