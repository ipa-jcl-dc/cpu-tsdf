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
//Header files
#include "curlserver.hpp"
#include "util.hpp"
#include "registrator.hpp"
#include "filters.hpp"
#include "segmentation.hpp"
#include "loader.hpp"
#include "aruco_marker.hpp"
#include "matrix_utils.hpp"
#include "config.hpp"
#include "realsense.hpp"
#include "util/random_utils.h"
static float min_x = 0.03;
static float max_x = 0.3;
static float min_y = 0.03;
static float max_y = 0.25;
static float min_z = 0.007;
static float max_z = 0.4;
using namespace std;
bool RS =0;

const int degree = 30;
//Declare helpers globally
boost::shared_ptr<Webserver2> webserver = boost::make_shared<Webserver2>();
boost::shared_ptr<Marker> aruco_marker = boost::make_shared<Marker>();
boost::shared_ptr<Filters> filters = boost::make_shared<Filters>();
boost::shared_ptr<RealSense> realsense = boost::make_shared<RealSense>();
boost::shared_ptr<Segmentation> segmentation = boost::make_shared<Segmentation>();
boost::shared_ptr<Config> config = boost::make_shared<Config>();

//Declare some variables globally
std::vector<std::string> serial_numbers; //serial number of connected camra
std::vector<std::vector<pcl::PointCloud<pcl::PointXYZRGB>::Ptr> >cloud_data_vec; //vector to store pointclouds from cameras
std::vector<std::vector<cv::Mat> > image_data_vec;
std::string aruco_board_file;
cv::Mat charuco_board;
std::vector<cv::Mat> camera_matrices;
std::vector<std::vector<Eigen::Matrix4f> >matrix_camera_to_marker_vec; //vector stores matrix from camera to board
std::vector<std::vector<int> >pose_flags_vec; //vector stores successful found pose of aruco board(0:not found, 1: found)

void collect360Degrees()
{
    Timer log;
    webserver->rotateDegRel(360);
    webserver->pollStatus();
    for(int n=0;n<config->nCapture();n++)
    {
      for(int i=0;i<serial_numbers.size();i++)
       {
          //Set up charuco marker with intrinsic parameters of each camera
              aruco_marker->setCameraMatrix(camera_matrices[i]);
              aruco_marker->LoadParameters(aruco_board_file);
              aruco_marker->createChaRucoMarker(charuco_board);


             //Turn off other depth streams
        for(int j=0;j<serial_numbers.size();j++)
               {
                  if(serial_numbers[j] !=serial_numbers[i])
                     {
                     realsense->turnOffDepthStream(serial_numbers[j]);
                    // std::cout<<"turn off serial_numbers "<<serial_numbers[j]<<std::endl;
                     }
                }
           for(int t=0;t<40; t++)realsense->poll_frames();// wait several frames for the effect to take place

        //Collect data
          cv::Mat color_image;
          pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZRGB>);
          realsense->saveData2(serial_numbers[i],color_image,cloud);
          std::cout<<"view "<< n <<endl;

          //save data odometry
          cloud_data_vec[i].push_back(cloud);
          image_data_vec[i].push_back(color_image.clone());
          aruco_pose pose;
          int pose_flag;
          pose_flag = aruco_marker->estimatePoseCharuco(color_image,pose);
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
          pose_flags_vec[i].push_back(pose_flag);
          matrix_camera_to_marker_vec[i].push_back(camera_to_marker);
        //turn on all streams
          for(int k=0;k<serial_numbers.size();k++)
          {
             realsense->turnOnDepthStream(serial_numbers[k]);
                     // std::cout<<"turn on serial_numbers "<<serial_numbers[k]<<std::endl;
          }
          //// T = 360/14 = 26s
          ///  t = 26/ 12 views = 2,16s for 1 view with n camera
          /// 2s for 12 views, t = 24/n for n views
          ///
          usleep(1000000*24/(config->nCapture())/(serial_numbers.size())); //
         }
     }
    log.tok_();
}

void collectNDegrees()
{
    webserver_signal status;
    for(int n=0;n<config->nCapture();n++)
    {
        webserver->rotateDegRel(360/config->nCapture());
        status = webserver->pollStatus();
        cout<<"N = " <<n<<endl;
        cout<<"Initial Position "<<status.motor_position<<endl;

      for(int i=0;i<serial_numbers.size();i++)
       {
          //Set up charuco marker with intrinsic parameters of each camera
              aruco_marker->setCameraMatrix(camera_matrices[i]);
              aruco_marker->LoadParameters(aruco_board_file);
              aruco_marker->createChaRucoMarker(charuco_board);


             //Turn off other depth streams
        for(int j=0;j<serial_numbers.size();j++)
               {
                  if(serial_numbers[j] !=serial_numbers[i])
                     {
                     realsense->turnOffDepthStream(serial_numbers[j]);
                     }
                }
          for(int t=0;t<30; t++)realsense->poll_frames();// wait several frames for the effect to take place
          while(1)
          {
              realsense->poll_frames();
              if(webserver->pollStatus().motor_position - status.motor_position >=  (360/config->nCapture() -1))
              {

                  cout<<"Position in while loop "<<webserver->pollStatus().motor_position<<endl;
                  break;
              }

              //cout<<webserver->pollStatus().motor_position<<endl;
          }
          //Collect data
          cv::Mat color_image;
          pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZRGB>);
          realsense->saveData2(serial_numbers[i],color_image,cloud);
        //save data odometry
          cloud_data_vec[i].push_back(cloud);
          image_data_vec[i].push_back(color_image.clone());
          aruco_pose pose;
          int pose_flag;
          pose_flag = aruco_marker->estimatePoseCharuco(color_image,pose);
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
          pose_flags_vec[i].push_back(pose_flag);
          matrix_camera_to_marker_vec[i].push_back(camera_to_marker);

        //turn on all streams
             for(int k=0;k<serial_numbers.size();k++)
                 {
                      realsense->turnOnDepthStream(serial_numbers[k]);
                     // std::cout<<"turn on serial_numbers "<<serial_numbers[k]<<std::endl;
                  }
           usleep(1000000); //1s
         }

     }
}

int main(int argc, char *argv[])
{
    if(argc < 2)
    {
        std::cout <<"Usage : ./multi_reconstruction board_xml_path"<<std::endl;
        return EXIT_FAILURE;
    }
    aruco_board_file = argv[1]; //Aruco board parameters
    //Set angular speep for station, max 14.5, default 12
    webserver->setAngularSpeed(14.0);
   ////Declare variables to store data
    std::vector<std::vector<pcl::PointCloud<pcl::PointXYZRGB>::Ptr> >segmented_cloud_data_vec; //vector to store segmented pointclouds in marker board
    std::vector<pcl::PointCloud<pcl::PointXYZRGB>::Ptr> output_vecs; //fused cloud of each camera
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr output(new pcl::PointCloud<pcl::PointXYZRGB>); // fused cloud



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

     //Get camera information, parameters are stored in params.yaml file in folder data
    realsense->setConfiguration("../config/realsense_parameters.yaml");
    realsense->getConfiguration();
    std::string data_path= currentDateTime();

    realsense->getCameraInformation(data_path,camera_matrices);
    for(int i=0;i<40;i++) realsense->poll_frames(); //wait some frame for effect to take place
    char c =0;
    while((c=(char)cv::waitKey(12)) !=27)
    {

        realsense->poll_frames();
        //Visualize streams
        for(int i=0;i<serial_numbers.size();i++)
               {
                    std::ostringstream ss;
                    ss<<i;
                    cv::imshow("color "+ss.str(),realsense->displayColorImages(serial_numbers[i]));
               }

       //Collect data
        if(c==13) //Enter
        {
           cv::destroyAllWindows();
           //Colect step by step
           if(config->mode()==1)
           {
               collectNDegrees();
           }
           //Continiously collect
           else
           {
                collect360Degrees();
           }

           break;
         }
  }
    cv::destroyAllWindows();

    realsense->stopStreaming();
    cout<<"data collection finished"<<endl;
    if(config->debug())
    {
    for(int k=0;k<serial_numbers.size();k++)
        {
        for(int i=0;i<image_data_vec[k].size();i++)
        {
            cv::imshow("test",image_data_vec[k][i]);
            cv::waitKey(0);
            cv::destroyAllWindows();
        }
        }
    }
    std::string dir_path = "../data/test2/";
    cout<<"test"<<endl;
    boost::shared_ptr<Utils> utils = boost::make_shared<Utils>();
    for(int i=0;i<serial_numbers.size();i++)
    {

        std::ostringstream s,s_serial;
        s<<i;
        s_serial<<serial_numbers[i];
        writeTransforms(dir_path+"transforms_"+s_serial.str()+".yaml",matrix_camera_to_marker_vec[i],pose_flags_vec[i]);
        writeTransformsTxt(dir_path+"transforms_"+s_serial.str()+".txt",matrix_camera_to_marker_vec[i],pose_flags_vec[i]);
        for(int k=0;k<image_data_vec[i].size();k++)
        {
            cout<<"save frame " <<i <<"-"<<k<<endl;
           std::ostringstream ss;
           ss<<k;
           cv::imwrite(dir_path+"frame-"+s.str()+"-"+ss.str()+".png",image_data_vec[i][k]);
           pcl::io::savePCDFile(dir_path+"frame"+s.str()+"-"+ss.str()+".pcd",*cloud_data_vec[i][k],true);
        }
    }
    cout<<"data is save in folder "<<dir_path<<endl;
    if(config->debug())
    {
        for(int k=0;k<serial_numbers.size();k++)
        {
         for(int i=0;i<cloud_data_vec[k].size();i++)
        {
            std::cout<<cloud_data_vec[k][i]->points.size()<<std::endl;
            utils->viewCloud(cloud_data_vec[k][i]);
            utils->isStopped();
        }
        }
    }
   vector<pcl::PointCloud<pcl::PointXYZRGB>::Ptr> segmented_cloud_data;

   //Transform points from camera to marker coordinate
    for(int i=0;i<cloud_data_vec.size();i++)
    {
        for(int k=0;k<cloud_data_vec[i].size();k++)
        {
        //Only tranform cloud with valid pose
            if(pose_flags_vec[i][k]==1)
            {
                pcl::PointCloud<pcl::PointXYZRGB>::Ptr transformed_cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
                pcl::transformPointCloud(*cloud_data_vec[i][k],*transformed_cloud,matrix_camera_to_marker_vec[i][k]);
                segmented_cloud_data.push_back(transformed_cloud);
            }
        }
    }
    cout<<"number of transformed clouds "<<segmented_cloud_data.size()<<endl;
    printf("get pose is done \n");
     //Pass-Through filter and remove outliers
     for(int i=0;i<segmented_cloud_data.size();i++)
     {
         std::ostringstream s;
         s<<i;
        // filters->threshold(*segmented_cloud_data[i],"z",min_z,max_z);
        // filters->threshold(*segmented_cloud_data[i],"x",min_x,max_x); //Size x of board 1480 pixels = 0.3915 meters
        // filters->threshold(*segmented_cloud_data[i],"y",min_y,max_y); //size y of board 1080 pixels = 0.28575 meters
         filters->CropBox(segmented_cloud_data[i],config->minX(),config->minY(),config->minZ(),config->maxX(),config->maxY(),config->maxZ());

         filters->removeOutliers(segmented_cloud_data[i],0.5,100);
         pcl::io::savePCDFile(dir_path+"segmented_frame-"+s.str()+".pcd",*segmented_cloud_data[i],true);
         //segmentation->euclideanCluster(segmented_cloud_data[i],12,2500000,0.005,segmented_cloud_data[i]);

         if(config->debug())
         {
            utils->viewCloud(segmented_cloud_data[i]);
            utils->isStopped();
         }
      }
     for(int i=0;i<segmented_cloud_data.size();i++)
        {
         //if(pose_flags[i]==1)
         //Fused cloud of all cameras
                *output += * segmented_cloud_data[i];
        }

     //remove outliers
       if(RS==0) filters->removeOutliers(output,1.0,100);
      cout<<output->points.size()<<endl;
     //return maximum cloud in a cluster
       segmentation->euclideanCluster(output,12,2500000,0.001,output);
       cout<<"number of point"<<output->points.size()<<endl;
       printf("Fused cloud is done \n");

       utils->bbox3DPCA(output);
       utils->bbox3DInertia(output);
        //Create a mesh of cloud
       if(config->mesh())
       {
           utils->closeCloud(output,0.001);
       }
       utils->viewCloud(output);
       utils->isStopped();
       pcl::io::savePCDFile(dir_path+"fused_cloud.pcd",*output,true);

       cout<<"finished"<<endl;
    return 0;
}
