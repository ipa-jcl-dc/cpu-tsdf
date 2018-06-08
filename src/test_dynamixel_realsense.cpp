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
using namespace std;
#define DEBUG 1
//Parameters
bool MLS = 0;                   /*Denoise - moving least square*/
bool LS = 0;                    /*Apply 3D least square*/
bool RS =0;
int main (int argc, char** argv) try
{


    if(argc < 2)
    {
        std::cout <<"Usage : ./charuco_test board_xml"<<std::endl;
        return EXIT_FAILURE;
    }
    if(pcl::console::find_argument (argc, argv, "-r") >= 0)
        {
             RS = 1;
             cout << "Registration is ON" << endl;
        }
    std::string aruco_board_file = argv[1]; //Aruco board parameters  
    ////Declare classes
    boost::shared_ptr<Marker> aruco_marker = boost::make_shared<Marker>();
    boost::shared_ptr<Filters> filters = boost::make_shared<Filters>();
    boost::shared_ptr<Loader> loaders = boost::make_shared<Loader>();
    boost::shared_ptr<Config> configs = boost::make_shared<Config>();
    boost::shared_ptr<RealSense> realsense = boost::make_shared<RealSense>();
    boost::shared_ptr<Dynamixel> dynamixel = boost::make_shared<Dynamixel>();
    boost::shared_ptr<Segmentation> segmentation = boost::make_shared<Segmentation>();
    boost::shared_ptr<Registrator> registrator = boost::make_shared<Registrator>();

    ////Declare vector to store camera matrices
   // cv::Mat camera_matrix1 = configs->cameraMatrix1();
      std::vector<cv::Mat> camera_matrices;
    ////Declare variables to store data
    vector<vector<pcl::PointCloud<pcl::PointXYZRGB>::Ptr> >cloud_data_vec; //vector to store pointclouds from cameras
    vector<vector<pcl::PointCloud<pcl::PointXYZRGB>::Ptr> >segmented_cloud_data_vec; //vector to store segmented pointclouds in marker board
    vector<pcl::PointCloud<pcl::PointXYZRGB>::Ptr> output_vecs; //fused cloud of each camera
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr output(new pcl::PointCloud<pcl::PointXYZRGB>); // fused cloud

    vector<vector<cv::Mat> > image_data_vec;
    cv::Mat chacuco_board;

    std::vector<std::vector<Eigen::Matrix4f> >matrix_camera_to_marker_vec; //vector stores matrix from camera to board
   // std::vector<Eigen::Matrix4f> matrix_camera_to_marker_EQ; //vector stores matrix from camera to board after EQ

    std::vector<std::vector<int> >pose_flags_vec; //vector stores successful found pose of aruco board(0:not found, 1: found)
    //std::vector<int> pose_flags_EQ; //vector stores successful found pose of aruco board(0:not found, 1: found)

    vector<std::string> serial_numbers; //serial number of connected camra
    //// Start camera and dynamixel motor
    // Get list of cameras
    rs2::device_list devices = realsense->get_device_list();

    //Enable all camera and store serial numbers for further processing
    for (rs2::device device : devices)
      {
          realsense->enable_device(device);
          serial_numbers.push_back(realsense->get_serial_number(device));
      }
    //Resize cloud data,image data, poses and pose flags = number of connected cameras
    cloud_data_vec.resize(serial_numbers.size());
    image_data_vec.resize(serial_numbers.size());
    matrix_camera_to_marker_vec.resize(serial_numbers.size());
    pose_flags_vec.resize(serial_numbers.size());
    output_vecs.resize(serial_numbers.size());
    //Turn on all depth streams at the beginning
     for(int i=0;i<serial_numbers.size();i++)
       {
            realsense->turnOnDepthStream(serial_numbers[i]);
       }
     //Init and do homing dynamixel motor
     dynamixel->Init();
     dynamixel->Open();
     dynamixel->DoHoming();
     //Just test to capture 12 images, number of image will be set in yaml configuration file
     double degree =0;
     int delta =30;
     char c =0;
     //Get camera information, parameters are stored in params.yaml file in folder data
     realsense->getCameraInformation(camera_matrices);

     //Set configuration from yaml file
     realsense->setConfiguration("../config/realsense_parameters.yaml");
     for(int i=0;i<40;i++) realsense->poll_frames(); //wait some frame for effect to take place
     while((c=(char)cv::waitKey(12)) !=27)
     {

         realsense->poll_frames();
        // for(int i=0;i<30;i++) realsense->poll_frames(); //wait some frame for effect to take place


         //Visualize streams
         for(int i=0;i<serial_numbers.size();i++)
                {
                     std::ostringstream ss;
                     ss<<i;
                     cv::imshow("color "+ss.str(),realsense->displayColorImages(serial_numbers[i]));
                     //cv::imshow("depth "+ss.str(),realsense->displayDepthImages(serial_numbers[i]));
                }

	/*	
        if(c==112) //p
        {
                realsense->turnOffDepthStream(serial_numbers[0]);
        }
        if(c==111) //o
        {
                realsense->turnOnDepthStream(serial_numbers[0]);
        }
	*/
        //Collect data
         if(c==13) //Enter
         {
            cv::destroyAllWindows();
            for(int n=0;n<(int)360/delta;n++)
              {

               for(int i=0;i<serial_numbers.size();i++)
               {
                   if(n==0) break;
                   std::ostringstream ss;
                   ss<< i;
                     //Turn off other depth streams
                for(int j=0;j<serial_numbers.size();j++)
                       {
                          if(serial_numbers[j] !=serial_numbers[i])
                             {
                             realsense->turnOffDepthStream(serial_numbers[j]);
                             cout<<"turn off serial_numbers "<<serial_numbers[j]<<endl;
                             }
                        }
                  usleep(500000);
                  for(int t=0;t<40; t++)
                    {
                      realsense->poll_frames();// wait several frames for the effect to take place
                    }
                //Collect data
                  cv::Mat color_image;
                  pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZRGB>);
                  realsense->saveData(serial_numbers[i],color_image,cloud);
                  cloud_data_vec[i].push_back(cloud);
                  image_data_vec[i].push_back(color_image.clone());
                //turn on all streams
                for(int k=0;k<serial_numbers.size();k++)
                {
                      realsense->turnOnDepthStream(serial_numbers[k]);
                      cout<<"turn on serial_numbers "<<serial_numbers[k]<<endl;
                }
               }
               usleep(100000);
               //Rotate motor
                    dynamixel->Rotate(degree);
                    degree +=delta;

            }
            break;
         }



   }
    cv::destroyAllWindows();
    dynamixel->DoHoming();
    dynamixel->Close();
    realsense->stopStreaming();

    boost::shared_ptr<Utils> utils = boost::make_shared<Utils>();
/*
    for(int k=0;k<serial_numbers.size();k++)
    {
        for(int i=0;i<image_data_vec[k].size();i++)
        {
            cv::imshow("test",image_data_vec[k][i]);
            cv::waitKey(0);
            cv::destroyAllWindows();
        }
    }

    for(int i=0;i<cloud_data_vec[0].size();i++)
    {
        utils->viewCloud(cloud_data_vec[0][i]);
        utils->isStopped();
    }
    */
    std::string dir_path = "../data/test/";
for(int i=0;i<serial_numbers.size();i++)
{
    std::ostringstream s;
    s<<i;
    for(int k=0;k<image_data_vec[i].size();k++)
    {
       std::ostringstream ss;
       ss<<k;
       cv::imwrite(dir_path+"frame-"+s.str()+"-"+ss.str()+".png",image_data_vec[i][k]);
       pcl::io::savePCDFile(dir_path+"frame"+s.str()+"-"+ss.str()+".pcd",*cloud_data_vec[i][k],true);
    }
}
    ////Perform reconstruction
    /// For each camera, get data, camera matrix, find pose of cloud, and fuse all cloud into "output" cloud

   for(int k=0;k<serial_numbers.size();k++)
    {
    output_vecs[k].reset(new pcl::PointCloud<pcl::PointXYZRGB>);
    //segmented cloud data after segmentation with threshold filter
    vector<pcl::PointCloud<pcl::PointXYZRGB>::Ptr> segmented_cloud_data;

//Set up charuco marker with intrinsic parameters of each camera
    cout<<"Fusing cloud with camera matrix "<<endl<<camera_matrices[k]<<endl;
    aruco_marker->setCameraMatrix(camera_matrices[k]);
    aruco_marker->LoadParameters(aruco_board_file);
    aruco_marker->createChaRucoMarker(chacuco_board);


    //Get pose of frames using marker
    for(int i=0;i<image_data_vec[k].size();i++)
    {
    pcl::console::print_highlight("frame %d \n",i);
    aruco_pose pose;

    int pose_flag;
    pose_flag = aruco_marker->estimatePoseCharuco(image_data_vec[k][i],pose);
    cout<<pose_flag<<endl;
    Eigen::Vector3f T;
    Eigen::Matrix3f R;
    T.setOnes();
    R.setIdentity();
    Eigen::Matrix4f marker_to_camera;
    Eigen::Matrix4f camera_to_marker;  
    //Only convert to eigen matrix with valid pose
    if(pose_flag==1)
    {
    cv::cv2eigen(pose.rot,R);
    cv::cv2eigen(pose.trans,T);
    }
    marker_to_camera.setIdentity();   // Set to Identity to make bottom row of Matrix 0,0,0,1
    marker_to_camera.block<3,3>(0,0) = R;
    marker_to_camera.block<3,1>(0,3) = T;
    camera_to_marker = marker_to_camera.inverse();

    pose_flags_vec[k].push_back(pose_flag);
    matrix_camera_to_marker_vec[k].push_back(camera_to_marker);

    //write transform to file
   // std::ostringstream ss;
   // ss<<k;
   // writeTransforms(dir_path+"transform-"+ss.str()+".yaml",matrix_camera_to_marker_vec[k],pose_flags_vec[k]);
    }

    cv::destroyAllWindows();
    //Transform points from camera to marker coordinate
     for(int i=0;i<cloud_data_vec[k].size();i++)
     {
         //Only tranform cloud with valid pose
        if(pose_flags_vec[k][i]==1)
        {
            pcl::PointCloud<pcl::PointXYZRGB>::Ptr transformed_cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
            pcl::transformPointCloud(*cloud_data_vec[k][i],*transformed_cloud,matrix_camera_to_marker_vec[k][i]);
            segmented_cloud_data.push_back(transformed_cloud);
        }
     }
    cout<<"number of transformed clouds "<<segmented_cloud_data.size()<<endl;
    pcl::console::print_highlight("get pose is done \n");
     //Pass-Through filter and remove outliers
     for(int i=0;i<segmented_cloud_data.size();i++)
     {
         filters->threshold(*segmented_cloud_data[i],"z",0.006,0.4);
         filters->threshold(*segmented_cloud_data[i],"x",0.03,0.3); //Size x of board 1480 pixels = 0.3915 meters
         filters->threshold(*segmented_cloud_data[i],"y",0.03,0.25); //size y of board 1080 pixels = 0.28575 meters
         filters->removeOutliers(segmented_cloud_data[i],0.5,100);
     }
     //Euclean clustering to remove tiny cluster of point
     for(int i=0;i<segmented_cloud_data.size();i++)
     {
        // utils->viewCloud(segmented_cloud_data[i]);
       //  utils->isStopped();
        // cout<<segmented_cloud_data[i]->points.size()<<endl;
         //return maximum cloud in a cluster
      //   segmentation->euclideanCluster(segmented_cloud_data[i],100,2500000,0.01,segmented_cloud_data[i]);
       //  utils->viewCloud(segmented_cloud_data[i]);
       //  cout<<segmented_cloud_data[i]->points.size()<<endl;
        // utils->isStopped();
     }
     pcl::console::print_highlight("Threshold is done \n");
     for(int i=0;i<segmented_cloud_data.size();i++)
        {
         //if(pose_flags[i]==1)
         //Fused cloud of all cameras
                *output += * segmented_cloud_data[i];
         //Fused cloud of each camera
                *output_vecs[k] += *segmented_cloud_data[i];
        }
     segmented_cloud_data_vec.push_back(segmented_cloud_data);
    }
   //// After transforming all frames of connected cameras to marker board. The fused cloud has
   //// slight ambiguities due to slightly incorrect poses with different cameras.
   ///
   /// TODO: Registration
   if(RS)
   {
   //Remove outliers and tiny cluster before registration
    for(int i=0;i<output_vecs.size();i++)
    {
        filters->removeOutliers(output_vecs[i],1.0,100);
        segmentation->euclideanCluster(output_vecs[i],15,2500000,0.005,output_vecs[i]);
    }
    Eigen::Matrix4f matrix_initial = Eigen::Matrix4f::Identity();
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr icp_output(new pcl::PointCloud<pcl::PointXYZRGB>); // icp cloud
    *icp_output = *output_vecs[0];
    for(int i=1;i<output_vecs.size();i++)
    {
            cout<<"Proces frame "<<i-1 <<": "<<i<<endl;
            Eigen::Matrix4f globalMatrix;
            Eigen::Matrix4f matrix;
            pcl::PointCloud<pcl::PointXYZRGB>::Ptr temp (new pcl::PointCloud<pcl::PointXYZRGB>);
            matrix = registrator->icpAlignment(output_vecs[i], output_vecs[i - 1],temp,configs->maxDistance());
            globalMatrix = matrix* matrix_initial; //matrix relatives to first frame
            //Transform frame to first frame coordinate
            pcl::transformPointCloud(*temp,*temp,matrix_initial);
            *icp_output += *temp;
             matrix_initial =  globalMatrix;
    }
    *output = *icp_output;
    }


   //remove outliers
     if(RS==0) filters->removeOutliers(output,1.0,50);
    cout<<output->points.size()<<endl;
   //return maximum cloud in a cluster
     segmentation->euclideanCluster(output,12,2500000,0.005,output);
     cout<<"number of point"<<output->points.size()<<endl;
    // output = filters->movingLeastSquares(output,0.01);
     pcl::console::print_highlight("Fused cloud is done \n");

     utils->bbox3DPCA(output);
     utils->viewCloud(output);
     utils->isStopped();
     pcl::io::savePCDFile(dir_path+"fused_cloud.pcd",*output,true);
     cout<<"finished"<<endl;
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





