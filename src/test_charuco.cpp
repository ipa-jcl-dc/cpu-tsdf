//C++
#include <fstream>
#include <iomanip>
#include <iostream>
#include <string>
#include <cstdlib>
//OpenCV
#include <opencv2/opencv.hpp>
#include <Eigen/Core>
#include <opencv2/core/eigen.hpp>
#include <boost/filesystem.hpp>
#include <boost/shared_ptr.hpp>
#include <boost/make_shared.hpp>
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
using namespace std;
#define DEBUG 1
//Parameters
bool MLS = 0;                   /*Denoise - moving least square*/
bool LS = 0;                    /*Apply 3D least square*/

int main (int argc, char** argv)
{
    if(argc < 3)
    {
        std::cout <<"Usage : ./charuco_test board_xml data_dir"<<std::endl;
        return EXIT_FAILURE;
    }
    std::string aruco_board_file = argv[1]; //Aruco board parameters
    std::string dir_path = argv[2]; //data folder

    //Declare classes
    boost::shared_ptr<Marker> aruco_marker = boost::make_shared<Marker>();
    boost::shared_ptr<Filters> filters = boost::make_shared<Filters>();
    boost::shared_ptr<Utils> utils = boost::make_shared<Utils>();
    boost::shared_ptr<Loader> loaders = boost::make_shared<Loader>();
    boost::shared_ptr<Config> configs = boost::make_shared<Config>();
    boost::shared_ptr<Segmentation> segmentation = boost::make_shared<Segmentation>();
    boost::shared_ptr<Registrator> registrator = boost::make_shared<Registrator>();
    //Load first camera matrix from config
    cv::Mat camera_matrix1 = configs->cameraMatrix1();
    //Load data and parameters
    vector<pcl::PointCloud<pcl::PointXYZRGB>::Ptr> cloud_data;
    vector<pcl::PointCloud<pcl::PointXYZRGB>::Ptr> segmented_cloud_data;
    cv::Mat chacuco_board;
    image_data images;
    int data_n;
    std::vector<Eigen::Matrix4f> matrix_camera_to_marker; //vector stores matrix from camera to board
    std::vector<Eigen::Matrix4f> matrix_camera_to_marker_EQ; //vector stores matrix from camera to board after EQ

    std::vector<int> pose_flags; //vector stores successful found pose of aruco board(0:not found, 1: found)
    std::vector<int> pose_flags_EQ; //vector stores successful found pose of aruco board(0:not found, 1: found)

    pcl::PointCloud<pcl::PointXYZRGB>::Ptr output(new pcl::PointCloud<pcl::PointXYZRGB>); // fused cloud
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr final_output (new pcl::PointCloud<pcl::PointXYZRGB>);
    aruco_marker->setCameraMatrix(camera_matrix1);
    aruco_marker->LoadParameters(aruco_board_file);
    aruco_marker->createChaRucoMarker(chacuco_board);
    loaders->loadImages(images,dir_path);
    loaders->loadClouds(cloud_data,dir_path);
    data_n = cloud_data.size();
    segmented_cloud_data.resize(data_n);

    pcl::console::print_highlight("Loading clouds is done \n");

    //Get pose of frames using marker

    for(int i=0;i<images.size();i++)
    {
    pcl::console::print_highlight("frame %d \n",i);
    segmented_cloud_data[i].reset(new pcl::PointCloud<pcl::PointXYZRGB>);
    aruco_pose pose;
    int pose_flag;
    pose_flag = aruco_marker->estimatePoseCharuco(images[i].img,pose);
    Eigen::Vector3f T;
    Eigen::Matrix3f R;
    Eigen::Matrix4f marker_to_camera;
    Eigen::Matrix4f camera_to_marker;
    cv::cv2eigen(pose.rot,R);
    cv::cv2eigen(pose.trans,T);
    marker_to_camera.setIdentity();   // Set to Identity to make bottom row of Matrix 0,0,0,1
    marker_to_camera.block<3,3>(0,0) = R;
    marker_to_camera.block<3,1>(0,3) = T;   
    camera_to_marker = marker_to_camera.inverse();
    pose_flags.push_back(pose_flag);
    matrix_camera_to_marker.push_back(camera_to_marker);
    }
    cv::destroyAllWindows();
    writeTransforms(dir_path+"transform.yaml",matrix_camera_to_marker,pose_flags);
    writeTransformsTxt(dir_path+"transform.txt",matrix_camera_to_marker,pose_flags);
    if(LS)
    {
        /*
        //Do least square 3d plane-equalization, modify transformation matrices
        Eigen::Vector3f plane_coeffs;
        plane_coeffs =leastSquare3d(matrix_camera_to_marker,pose_flags);
        for(int i  = 0; i< matrix_camera_to_marker.size(); i++){
           //  matrix_camera_to_marker[i] = projectTransformToPlane(matrix_camera_to_marker[i],plane_coeffs);
        }
        projectTransformTranslationsToPlane(plane_coeffs,matrix_camera_to_marker,pose_flags); //project the translations onto the plane
        writeTransforms(dir_path+"transform_plane_equalization.yaml",matrix_camera_to_marker,pose_flags);
        */
        readTransformsTxt(dir_path+"transforms.txt",matrix_camera_to_marker_EQ,pose_flags_EQ);
        matrix_camera_to_marker = matrix_camera_to_marker_EQ;
        pose_flags = pose_flags_EQ;
    }

    //Transform points from camera to marker coordinate
    for(int i=0;i<matrix_camera_to_marker.size();i++)
    {
        pcl::transformPointCloud(*cloud_data[i],*segmented_cloud_data[i],matrix_camera_to_marker[i]);
    }

    pcl::console::print_highlight("get pose is done \n");
    //Pass-Through filter and remove outliers
    for(int i=0;i<cloud_data.size();i++)
    {
        filters->threshold(*segmented_cloud_data[i],"z",0.008,0.4);
        filters->threshold(*segmented_cloud_data[i],"x",0.03,0.3); //Size x of board 1480 pixels = 0.39159 meters
        filters->threshold(*segmented_cloud_data[i],"y",0.03,0.25); //size y of board 1080 pixels = 0.28575 meters
        filters->removeOutliers(segmented_cloud_data[i],0.5,100);
    }

    for(int i=0;i<segmented_cloud_data.size();i++)
    {
        std::ostringstream ss;
        ss<<i;
        *output += * segmented_cloud_data[i];
        pcl::io::savePCDFile(dir_path+"segmented_cloud"+ss.str()+".pcd",*segmented_cloud_data[i],true);
    }

    filters->removeOutliers(output,1.0,100);
    std::cout<<"number of points "<<output->size()<<std::endl;
    segmentation->euclideanCluster(output,20,2500000,0.01,output);
    std::cout<<"number of points "<<output->size()<<std::endl;
    if(MLS)
    {
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr mls_points (new pcl::PointCloud<pcl::PointXYZRGB>);
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr downsample (new pcl::PointCloud<pcl::PointXYZRGB>);
    downsample = filters->voxelize(output,0.0001f);
    output = downsample;
    std::cout<<"moving least square begins ..."<<std::endl;
    mls_points = filters->movingLeastSquares(output,0.01);
    output = mls_points;
    std::string cloud_file = dir_path + "fused_MLS_cloud.pcd";
    pcl::io::savePCDFile(cloud_file,*output,true);
    }    
  //  utils->bbox3DPCA(output);
    utils->viewCloud(output);
    utils->isStopped();
    std::string cloud_file = dir_path + "fused_cloud.pcd";
    std::string ply_file = dir_path +"fused_cloud.ply";
    pcl::io::savePCDFile(cloud_file,*output,true);
    utils->save2ply(*output,ply_file);


return EXIT_SUCCESS;
}
