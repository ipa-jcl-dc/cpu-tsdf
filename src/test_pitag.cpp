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
#include <boost/thread/thread.hpp>
//PCL
#include <pcl/common/common_headers.h>
#include <pcl/io/io.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <boost/thread/thread.hpp>
#include <pcl/console/parse.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/kdtree/impl/kdtree_flann.hpp>
#include <pcl/common/transforms.h>
#include <pcl/features/board.h>
#include <pcl/console/parse.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/io/vtk_io.h>
#include <pcl/filters/passthrough.h>
#include <pcl/registration/correspondence_rejection_sample_consensus.h>
#include <pcl/registration/transformation_estimation_point_to_plane.h>
//Header files
#include "util.hpp"
#include "registrator.hpp"
#include "filters.hpp"
#include "segmentation.hpp"
#include "loader.hpp"
#include "fiducial/FiducialDefines.h"
#include "fiducial/FiducialModelPi.h"
#include "fiducial/AbstractFiducialModel.h"
#include "aruco_marker.hpp"
using namespace ipa_Fiducials;
using namespace std;
#define DEBUG 1
//Parameters
double MIN_DISTANCE = 0.0035f; /*Registration threshold*/

bool MLS = 0;                   /*Denoise - moving least square*/
bool REGISTRATION = 1;   /*use registration*/
bool POSE =0; 					/*Draw pose*/

cv::Mat camera_matrix =  (cv::Mat_<double>(3,3)<<   619.69,0,313.013 ,
                                                 0,619.69, 245.14,
                                                    0,0,1);

int main(int argc, char** argv)
{
    if(argc < 4)
    {
        std::cout <<"Usage : ./pi_tag_test tag_dir data_dir para_dir"<<std::endl;
        return EXIT_FAILURE;
    }
    std::string model_filename = argv[1]; //pi tag file
    std::string dir_path = argv[2]; //path to image folder
    std::string parameter_path = argv[3]; //configuration file
 //Structure of RGB and depth image

    boost::shared_ptr<AbstractFiducialModel> m_pi_tag;
    m_pi_tag = boost::shared_ptr<FiducialModelPi>(new FiducialModelPi());
    boost::shared_ptr<Registrator> registrator = boost::make_shared<Registrator>();
    boost::shared_ptr<Filters> filters = boost::make_shared<Filters>();
    boost::shared_ptr<Utils> utils = boost::make_shared<Utils>();
    boost::shared_ptr<Segmentation> segmentations = boost::make_shared<Segmentation>();
    boost::shared_ptr<Loader> loaders = boost::make_shared<Loader>();


    if (pcl::console::find_argument (argc, argv, "-m") >= 0)
    {
        MLS = 1;
        std::cout << "Moving least square smoothing is ON" << std::endl;
    }
    if(pcl::console::find_argument (argc, argv, "-p") >= 0)
    {
         POSE = 1;
         cout << "Drawing Pose is ON" << endl;
    }

    if(pcl::console::find_argument (argc, argv, "-r") >= 0)
    {
        REGISTRATION = 0;
        std::cout << "Fused Model is on, registration is off " << std::endl;
    }





// Load Parameters
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr output(new pcl::PointCloud<pcl::PointXYZRGB>);
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr final_output (new pcl::PointCloud<pcl::PointXYZRGB>);
    pcl::PointXYZRGB min_point,max_point;
    double max_correspondence_distance;
    image_data images;
    std::vector<pcl::PointCloud<pcl::PointXYZRGB>::Ptr> cloud_data;
    std:vector<pcl::PointCloud<pcl::PointXYZRGB>::Ptr> segmented_cloud_data;


    loaders->ReadParameters(parameter_path,min_point,max_point,max_correspondence_distance);
    loaders->loadClouds(cloud_data,dir_path);
    pcl::console::print_highlight("Loading clouds is done \n");
    loaders->loadImages(images,dir_path);
    pcl::console::print_highlight("Loading images is done \n");
    segmented_cloud_data.resize(cloud_data.size());
    if (m_pi_tag->Init(camera_matrix, model_filename, false) & RET_FAILED)
    {
         puts("Initializing fiducial detector with camera matrix [FAILED]");
         return EXIT_FAILURE;
    }

    std::vector<int> indices;
//remove nan and zero values of Z
    for(int i=0;i<cloud_data.size();i++)
    {
    pcl::PassThrough<pcl::PointXYZRGB> pass;
    pass.setInputCloud (cloud_data[i]);
    pass.setFilterFieldName ("z");
    pass.setFilterLimits (0.0,0.0);
    pass.setFilterLimitsNegative (true);
    pass.filter (*cloud_data[i]);
    pcl::removeNaNFromPointCloud(*cloud_data[i],*cloud_data[i],indices);
    }
    std::vector<Eigen::Matrix4f> matrix_camera_to_marker;
    std::vector<ipa_Fiducials::t_pose>  tags_vec_data;
    tags_vec_data.resize(images.size());

//Get pose of frames using marker
    for(int i=0;i<images.size();i++)
    {

    pcl::console::print_highlight("frame %d \n",i);
    segmented_cloud_data[i].reset(new pcl::PointCloud<pcl::PointXYZRGB>);
    vector<ipa_Fiducials::t_pose> tags_vec;
    m_pi_tag->GetPose(images[i].img,tags_vec);
    tags_vec_data[i] = tags_vec[0];
    Eigen::Vector3f T;
    Eigen::Matrix3f R;
    Eigen::Matrix4f marker_to_camera;
    Eigen::Matrix4f camera_to_marker;
    cv::cv2eigen(tags_vec[0].rot,R);
    cv::cv2eigen(tags_vec[0].trans,T);
    marker_to_camera.setIdentity();   // Set to Identity to make bottom row of Matrix 0,0,0,1
    marker_to_camera.block<3,3>(0,0) = R;
    marker_to_camera.block<3,1>(0,3) = T;
//Transform points from camera to marker coordinate
    camera_to_marker = marker_to_camera.inverse(); //transformation from camera to marker
    matrix_camera_to_marker.push_back(camera_to_marker);
    pcl::transformPointCloud(*cloud_data[i],*segmented_cloud_data[i],camera_to_marker);
    if(POSE)
        utils->drawPose(tags_vec[0].rot,tags_vec[0].trans,images[i].img,min_point,max_point);
    }
    cv::destroyAllWindows();
    pcl::console::print_highlight("get pose is done \n");
//Pass-Through filter and remove outliers
    for(int i=0;i<cloud_data.size();i++)
    {
        filters->threshold(*segmented_cloud_data[i],"z",min_point.z,max_point.z);
        filters->threshold(*segmented_cloud_data[i],"x",min_point.x,max_point.x);
        filters->threshold(*segmented_cloud_data[i],"y",min_point.y,max_point.y);
        filters->removeOutliers(segmented_cloud_data[i],0.5,100);
        segmentations->euclideanCluster(segmented_cloud_data[i],20,2500000,0.01,segmented_cloud_data[i]);
    }
    if(!REGISTRATION) //Switch off registration, just fuse all frames to marker coordinate
    {
    for(int i=0;i<segmented_cloud_data.size();i++)
    {
	//if(tags_vec_data[i].id==0)
		*output += * segmented_cloud_data[i];
    }
    filters->removeOutliers(output,1.0,60);
    if(MLS)
    {
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr mls_points (new pcl::PointCloud<pcl::PointXYZRGB>);
    cout<<"moving least square begins ..."<<endl;
    mls_points = filters->movingLeastSquares(output,0.01);
    output = mls_points;
    }
//std::vector<pcl::PointCloud<pcl::PointXYZRGB>::Ptr > output_data;
//segmentations->euclideanCluster(output,10,250000,k,output_data);
    utils->viewCloud(output);
    utils->isStopped();
    std::string cloud_file = dir_path + "fused_cloud.pcd";
    std::string ply_file = dir_path +"fused_cloud.ply";
    pcl::io::savePCDFile(cloud_file,*output,true);
    utils->save2ply(*output,ply_file);
    }
//Using registration
    else
    {
    pcl::console::print_highlight("pass-through filter is done \n");
//remove outliers
    for(int i=0;i<segmented_cloud_data.size();i++)
    {
	filters->removeOutliers(segmented_cloud_data[i],1.0,50);
    }
//output of icp alignment
    std::vector<pcl::PointCloud<pcl::PointXYZRGB>::Ptr > output_data;
    output_data.push_back(segmented_cloud_data[0]);
//Matrices of all frames relative to first frame
    vector<Eigen::Matrix4f>  matrix_buffer;
    Eigen::Matrix4f matrix_initial = Eigen::Matrix4f::Identity();
    matrix_buffer.push_back(matrix_initial);
    *output = *segmented_cloud_data[0];
    for(int i=1;i<segmented_cloud_data.size();i++)
    {
	cout<<"Proces frame "<<i-1 <<": "<<i<<endl;
	Eigen::Matrix4f globalMatrix;
	Eigen::Matrix4f matrix;
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr temp (new pcl::PointCloud<pcl::PointXYZRGB>);
	matrix = registrator->icpAlignment(segmented_cloud_data[i], segmented_cloud_data[i - 1],temp,max_correspondence_distance);

	globalMatrix = matrix* matrix_initial; //matrix relatives to first frame
    //Transform frame to first frame coordinate
	pcl::transformPointCloud(*temp,*temp,matrix_initial);
	// save matrix to perform global alignment
		         if (matrix_buffer.size () < i + 1)
		           matrix_buffer.push_back (globalMatrix);
		         else
		           matrix_buffer[i] = globalMatrix * matrix_buffer[i];
                 //Fuse frames to output
		         *output +=  *temp;
		         output_data.push_back(temp);
		         matrix_initial =  globalMatrix;
                 utils->visual(matrix_buffer[i]*matrix_camera_to_marker[i],*output);
    }
    utils->visual(matrix_buffer[segmented_cloud_data.size()-1]*matrix_camera_to_marker[segmented_cloud_data.size()-1],*output);
    vector<Eigen::Matrix4f>  matrix_buffer_elch; //Matrix relative to first frame after ELCH
//Run elch multiple times to optimize
    for(int i=0;i<4;i++)
    {
    registrator->elchCorrection(output_data,matrix_buffer_elch,final_output);
    utils->visual(matrix_buffer[segmented_cloud_data.size()-1]*matrix_camera_to_marker[segmented_cloud_data.size()-1],*final_output);
    }
//remove outliers of output
    filters->removeOutliers(final_output,1.0,60);
//Run moving least square to make object smoother, lose some details
    if(MLS)
    {
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr mls_points (new pcl::PointCloud<pcl::PointXYZRGB>);
    pcl::console::print_highlight("moving least square begins\n");
    mls_points = filters->movingLeastSquares(final_output,0.005);
    utils->visual(matrix_buffer[segmented_cloud_data.size()-1]*matrix_camera_to_marker[segmented_cloud_data.size()-1],*final_output);
    }
    else
    utils->visual(matrix_buffer[segmented_cloud_data.size()-1]*matrix_camera_to_marker[segmented_cloud_data.size()-1],*final_output);
//Draw bounding box using PCA
    utils->bbox3DPCA(final_output);
    utils->bbox3DInertia(final_output);
    std::string cloud_file = dir_path + "registration_cloud42.pcd";
    std::string ply_file = dir_path +"registration_cloud42.ply";
    pcl::io::savePCDFile(cloud_file,*final_output,true);
    utils->save2ply(*final_output,ply_file);
    utils->isStopped();
    }

    pcl::console::print_highlight("Finished ! \n");
  return EXIT_SUCCESS;
}
