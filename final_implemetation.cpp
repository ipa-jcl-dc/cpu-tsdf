//C/C++
#include <fstream>
#include <iomanip>
#include <iostream>
#include <string>
#include <cstdlib>
#include <unistd.h>
#include <sstream>
#include <stdlib.h>
#include <vector>
#include <boost/filesystem.hpp>
#include <boost/shared_ptr.hpp>
#include <boost/make_shared.hpp>
//CUDA
#include "cuda_headers.hpp"
#include <cuda_runtime.h>
#include "device_launch_parameters.h"
//OpenCV
#include <opencv2/opencv.hpp>
#include <Eigen/Core>
#include <opencv2/core/eigen.hpp>
#include <opencv2/core/cuda.hpp>
#include <opencv2/cudawarping.hpp>
//PCL
#include <pcl/common/common_headers.h>
#include <pcl/io/io.h>
#include <pcl/io/pcd_io.h>
#include <pcl/console/parse.h>
#include <pcl/io/obj_io.h>
#include <pcl/io/vtk_lib_io.h>
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
#include "realsense2.hpp"
#include "configuration.hpp"
#include "util/random_utils.h"

void readParameters(Configuration& config)
{
    const std::string config_file_kinect = "../config/kinectfusion_config.yaml";

     // ********** try to load the yaml file that is located at the given path **********
             cv::FileStorage config_file(config_file_kinect,cv::FileStorage::READ);
             //Surface Measurement parameters
             config_file["num_level"]>> config.num_level_;
             config_file["kernel_size"]>> config.kernel_size_;
             config_file["sigma_color"]>> config.sigma_color_;
             config_file["sigma_space"]>> config.sigma_space_;
             config_file["distance_threshold"]>> config.distance_threshold_;
             config_file["angle_threshold"]>>config.angle_threshold_;
             config_file["icp_iterations"]>>config.iterations;
             //TSDF Volume parameters
             config_file["volume_resolution"]>> config.volume_resolution_;
             config_file["voxel_size"]>> config.voxel_size_;
             config_file["original_distance_x"]>> config.original_distance_x_;
             config_file["original_distance_y"]>> config.original_distance_y_;
             config_file["original_distance_z"]>> config.original_distance_z_;
             config_file["truncation_distance"]>> config.truncation_distance_;
             config_file["depth_cutoff"]>> config.depth_cutoff_;
             config_file["pointcloud_buffer_size"]>> config.pointcloud_buffer_size_;
             //The learning station parameters
             config_file["maxDistance"]>>config.max_distance_;
             config_file["nCapture"]>>config.n_capture_;
             config_file["debug"]>>config.debug_;
             config_file["mode"]>>config.mode_;
             config_file["mesh"]>>config.mesh_;
             config_file["mls"]>>config.mls_;
             config_file["plane_equalized"]>>config.plane_equalized_;
             cv::FileNode n;
             n = config_file["BoundingBox"];
             n["mixX"]>>config.min_x_;
             n["maxX"]>>config.max_x_;
             n["minY"]>>config.min_y_;
             n["maxY"]>>config.max_y_;
             n["minZ"]>>config.min_z_;
             n["maxZ"]>>config.max_z_;

}
void export_mesh(const std::string& filename, const SurfaceMesh& surface_mesh)
{

            std::ofstream file_out { filename };
            if (!file_out.is_open())
                return;


            file_out << "ply" << std::endl;
            file_out << "format ascii 1.0" << std::endl;
            file_out << "element vertex " << surface_mesh.num_vertices << std::endl;
            file_out << "property float x" << std::endl;
            file_out << "property float y" << std::endl;
            file_out << "property float z" << std::endl;
           // file_out << "property uchar red" << std::endl;
           // file_out << "property uchar green" << std::endl;
           // file_out << "property uchar blue" << std::endl;
            file_out << "element face " << surface_mesh.num_triangles << std::endl;
            file_out << "property list uchar int vertex_indices" << std::endl;
            file_out << "end_header" << std::endl;

            for (int v_idx = 0; v_idx < surface_mesh.num_vertices; ++v_idx) {
                float3 vertex = surface_mesh.triangles.ptr<float3>(0)[v_idx];
               // uchar3 color = surface_mesh.colors.ptr<uchar3>(0)[v_idx];
                file_out << vertex.x << " " << vertex.y << " " << vertex.z << std::endl;;
               // file_out << (int) color.z << " " << (int) color.y << " " << (int) color.x << std::endl;
            }

           for (int t_idx = 0; t_idx < surface_mesh.num_vertices; t_idx += 3) {
                file_out << 3 << " " << t_idx + 1 << " " << t_idx << " " << t_idx + 2 << std::endl;
              // file_out << 3 << " " << t_idx << " " << t_idx+1 << " " << t_idx + 2 << std::endl;
            }

           file_out.close();

}
void export_ply(const std::string& filename, const PointCloud& point_cloud)
{
     // Create header for .ply file
      FILE *fp = fopen(filename.c_str(), "w");
      fprintf(fp, "ply\n");
      fprintf(fp, "format binary_little_endian 1.0	\n");
      fprintf(fp, "element vertex %d\n", point_cloud.num_points);
      fprintf(fp, "property float x\n");
      fprintf(fp, "property float y\n");
      fprintf(fp, "property float z\n");
      fprintf(fp, "property float nx\n");
      fprintf(fp, "property float ny\n");
      fprintf(fp, "property float nz\n");
      fprintf(fp, "property uchar red\n");
      fprintf(fp, "property uchar green\n");
      fprintf(fp, "property uchar blue\n");
      fprintf(fp, "end_header\n");
        for (int i = 0; i < point_cloud.num_points; ++i) {
            float3 vertex = point_cloud.vertices.ptr<float3>(0)[i];
            float3 normal = point_cloud.normals.ptr<float3>(0)[i];
            uchar3 color = point_cloud.color.ptr<uchar3>(0)[i];
            uchar r =  static_cast<uchar>(color.x);
            uchar g =  static_cast<uchar>(color.y);
            uchar b =  static_cast<uchar>(color.z);

            fwrite(&vertex.x, sizeof(float), 1, fp);
            fwrite(&vertex.y, sizeof(float), 1, fp);
            fwrite(&vertex.z, sizeof(float), 1, fp);
            fwrite(&normal.x, sizeof(float), 1, fp);
            fwrite(&normal.y, sizeof(float), 1, fp);
            fwrite(&normal.z, sizeof(float), 1, fp);
            fwrite(&b, sizeof(uchar), 1, fp);
            fwrite(&g, sizeof(uchar), 1, fp);
            fwrite(&r, sizeof(uchar), 1, fp);
        }
        fclose(fp);
}


///XYZRGB+ NORMAL
void saveTSDFCloudPCD(std::string file_pcd,const PointCloud& cloud_data,pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr& cloud_with_normals )
{
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
        pcl::PointCloud<pcl::Normal>::Ptr normals(new pcl::PointCloud<pcl::Normal>);
            cloud->points.resize(cloud_data.num_points);
            normals->points.resize(cloud_data.num_points);

            for(int i=0;i<cloud_data.num_points;i++)
            {
                pcl::PointXYZRGB& pt = cloud->points[i];
                pcl::Normal& pt_normal = normals->points[i];
                float3 vertex = cloud_data.vertices.ptr<float3>(0)[i];
                float3 normal = cloud_data.normals.ptr<float3>(0)[i];
                uchar3 color = cloud_data.color.ptr<uchar3>(0)[i];
                pt.x = vertex.x;
                pt.y = vertex.y;
                pt.z = vertex.z;
                pt.b = static_cast<uchar>(color.x);
                pt.g = static_cast<uchar>(color.y);
                pt.r = static_cast<uchar>(color.z);
                pt_normal.normal_x =normal.x;
                pt_normal.normal_y =normal.y;
                pt_normal.normal_z =normal.z;

            }
        pcl::concatenateFields(*cloud, *normals, *cloud_with_normals);
        pcl::io::savePCDFile(file_pcd,*cloud_with_normals,true);

}
///XYZRGB
void saveTSDFCloudPCD(std::string file_pcd,const PointCloud& cloud_data,pcl::PointCloud<pcl::PointXYZRGB>::Ptr& cloud)
{
            cloud->points.resize(cloud_data.num_points);

            for(int i=0;i<cloud_data.num_points;i++)
            {
                pcl::PointXYZRGB& pt = cloud->points[i];
                float3 vertex = cloud_data.vertices.ptr<float3>(0)[i];
                uchar3 color = cloud_data.color.ptr<uchar3>(0)[i];
                pt.x = vertex.x;
                pt.y = vertex.y;
                pt.z = vertex.z;
                pt.b = static_cast<uchar>(color.x);
                pt.g = static_cast<uchar>(color.y);
                pt.r = static_cast<uchar>(color.z);
            }
        pcl::io::savePCDFile(file_pcd,*cloud,true);
}
//XYZ + Normal
void saveTSDFCloudPCD(std::string file_pcd,const PointCloud& cloud_data,pcl::PointCloud<pcl::PointNormal>::Ptr& cloud_with_normals)
{
            pcl::PointCloud<pcl::Normal>::Ptr normals(new pcl::PointCloud<pcl::Normal>);
            pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
            cloud->points.resize(cloud_data.num_points);
            normals->points.resize(cloud_data.num_points);
            for(int i=0;i<cloud_data.num_points;i++)
            {
                pcl::PointXYZ& pt = cloud->points[i];
                pcl::Normal& pt_normal = normals->points[i];

                float3 vertex = cloud_data.vertices.ptr<float3>(0)[i];
                float3 normal = cloud_data.normals.ptr<float3>(0)[i];
                pt.x = vertex.x;
                pt.y = vertex.y;
                pt.z = vertex.z;
                pt_normal.normal_x =normal.x;
                pt_normal.normal_y =normal.y;
                pt_normal.normal_z =normal.z;
            }
            pcl::concatenateFields(*cloud, *normals, *cloud_with_normals);

        pcl::io::savePCDFile(file_pcd,*cloud,true);
}
void saveBinFile(std::string bin_file,const cv::Mat& tsdf_mat,const Configuration& config )
{
            // Save TSDF voxel grid and its parameters to disk as binary file (float array)
            std::cout << "Saving TSDF voxel grid values to disk (tsdf.bin)..." << std::endl;
            std::string voxel_grid_saveto_path = bin_file;
            std::ofstream outFile(voxel_grid_saveto_path, std::ios::binary | std::ios::out);
            float voxel_grid_dim_xf = (float) config.volume_resolution_;
            float voxel_grid_dim_yf = (float) config.volume_resolution_;
            float voxel_grid_dim_zf = (float) config.volume_resolution_;
            outFile.write((char*)&voxel_grid_dim_xf, sizeof(float));
            outFile.write((char*)&voxel_grid_dim_yf, sizeof(float));
            outFile.write((char*)&voxel_grid_dim_zf, sizeof(float));
            outFile.write((char*)&config.original_distance_x_, sizeof(float));
            outFile.write((char*)&config.original_distance_y_, sizeof(float));
            outFile.write((char*)&config.original_distance_z_, sizeof(float));
            outFile.write((char*)&config.voxel_size_, sizeof(float));
            outFile.write((char*)&config.truncation_distance_, sizeof(float));
            for(int z=0;z<config.volume_resolution_;z++){
                for(int y=0;y<config.volume_resolution_;y++){
                        for(int x=0;x<config.volume_resolution_;x++){
                                outFile.write((char*)&tsdf_mat.at<float>(z*config.volume_resolution_+y,x), sizeof(float));

                        }
                }
              }


            outFile.close();
}


//Declare helpers globally
boost::shared_ptr<Webserver2> webserver = boost::make_shared<Webserver2>();
boost::shared_ptr<Marker> aruco_marker = boost::make_shared<Marker>();
boost::shared_ptr<Filters> filters = boost::make_shared<Filters>();
boost::shared_ptr<RealSense2> realsense = boost::make_shared<RealSense2>();
boost::shared_ptr<Segmentation> segmentation = boost::make_shared<Segmentation>();

//Declare some variables globally
std::vector<cv::Mat>  image_data_vec;
std::vector<cv::Mat> depth_data_vec;
std::vector<pcl::PointCloud<pcl::PointXYZRGB>::Ptr> cloud_data_vec; //vector to store pointclouds
std::string aruco_board_file;
cv::Mat charuco_board;
cv::Mat camera_matrix;
std::vector<Eigen::Matrix4f> matrix_camera_to_marker_vec; //vector stores matrix from camera to board
std::vector<cv::Mat> image_segmented_vec;
std::vector<int> pose_flags_vec; //vector stores successful found pose of aruco board(0:not found, 1: found)
//Colect data 360 degrees, time to rotate full cycle with 14 degree/s is around 29.5s
void collect360Degrees(const int num_captures)
{
    Timer log;
    //Set up charuco marker with intrinsic parameters of each camera
    aruco_marker->setCameraMatrix(camera_matrix);
    aruco_marker->LoadParameters(aruco_board_file);
    aruco_marker->createChaRucoMarker(charuco_board);
    webserver->rotateDegRel(360);
    for(int n=0;n<num_captures;n++)
    {
          std::cout<<"view "<< n <<endl;
        //Collect data
          cv::Mat color_image,depth_image;
          pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZRGB>);
          realsense->waitForFrames();
          realsense->getColorImage(color_image);
          realsense->getDepthImage(depth_image);
          realsense->getPointCloud(cloud);
          image_data_vec.push_back(color_image.clone());
          depth_data_vec.push_back(depth_image.clone());
          cloud_data_vec.push_back(cloud);
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
          matrix_camera_to_marker_vec[n] = camera_to_marker;
          pose_flags_vec.push_back(pose_flag);

         // matrix_camera_to_marker_vec.push_back(camera_to_marker);
          //usleep(2000000); // 2s for 12 views => time = 24/n with n views
            usleep(24000000/(num_captures));

     }
    log.tok_();
}

int main(int argc, char *argv[])
{


    if(argc < 2)
    {
        std::cout <<"Usage : ./reconstruction ../config/aruco_board.xml"<<std::endl;
        return EXIT_FAILURE;
    }

    aruco_board_file = argv[1]; //Aruco board parameters
    Configuration config_tsdf;
    readParameters(config_tsdf);

    //Set angular speep for station, max 14.5, default 12
    webserver->setAngularSpeed(14.0);
   ////Declare variables to store data
    camera_matrix = realsense->getCameraMatrix();
   // vector<pcl::PointCloud<pcl::PointXYZRGB>::Ptr> segmented_cloud_data;
    printf("To collect data, press Enter \n");
    //printf("Press ESC to finish streamming and perform object modelling \n");
    matrix_camera_to_marker_vec.resize(config_tsdf.n_capture_);
    char c =0;
    while((c=(char)cv::waitKey(12)) !=27)
    {

        realsense->waitForFrames();
        //Visualize streams
        cv::Mat img;
        realsense->getColorImage(img);
        cv::imshow("color frame",img);
       //Collect data
        if(c==13) //Enter
        {
           cv::destroyAllWindows();
           //Colect step by step
           if(config_tsdf.mode_==1)
           {

               printf("no step by step mode right now \n");
               return -1;
           }
           //Continiously collect
           else
           {
                collect360Degrees(config_tsdf.n_capture_);
                //printf("press Enter if you want to collect more data \n");

           }
           break;
         }
        if(c==27) //ESC, exit to perform object modelling
        {
            break;
        }
  }
    cv::destroyAllWindows();
    realsense->stopStreaming();
    cout<<"data collection finished, perform reconstruction"<<endl;
    if(config_tsdf.plane_equalized_)
       {

           //Do least square 3d plane-equalization, modify transformation matrices
           Eigen::Vector3f plane_coeffs;
           plane_coeffs =leastSquare3d(matrix_camera_to_marker_vec,pose_flags_vec);
           //Transform rotation
           for(int i  = 0; i< matrix_camera_to_marker_vec.size(); i++){
             //  matrix_camera_to_marker_vec[i] = projectTransformToPlane(matrix_camera_to_marker_vec[i],plane_coeffs);
           }
           //Transform translation
           projectTransformTranslationsToPlane(plane_coeffs,matrix_camera_to_marker_vec,pose_flags_vec); //project the translations onto the plane
       }
    //Save to folder
    std::string data_path;
    if(config_tsdf.debug_)
    {
    // Create a data folder (with a random hash name) to save frames
    std::string hash_name = currentDateTime();
    data_path = "../data/" + hash_name + "/";
    int flag_created=  system(("mkdir -p " + data_path).c_str());
    writeTransforms(data_path+"transform.yaml",matrix_camera_to_marker_vec,pose_flags_vec);
    writeTransformsTxt(data_path+"transform.txt",matrix_camera_to_marker_vec,pose_flags_vec);
    writeTransforms(data_path+"transform_plane_equalization.yaml",matrix_camera_to_marker_vec,pose_flags_vec);
    std::vector<std::string> vector_img_name,vector_cloud_name;
    cv::Mat converted_image;
        for(int k=0;k<image_data_vec.size();k++)
        {
           std::vector<int> indices;
           pcl::PointCloud<pcl::PointXYZRGB>::Ptr segmented_cloud(new pcl::PointCloud<pcl::PointXYZRGB>); // fused cloud
           pcl::transformPointCloud(*cloud_data_vec[k],*segmented_cloud,matrix_camera_to_marker_vec[k]);
           filters->CropBox(segmented_cloud,config_tsdf.min_x_,config_tsdf.min_y_,config_tsdf.min_z_,
                            config_tsdf.max_x_,config_tsdf.max_y_,config_tsdf.max_z_,indices);
           segmentation->cloud2image(converted_image,segmented_cloud,indices);
           cv::cvtColor(converted_image,converted_image,CV_RGB2BGR);
           std::ostringstream ss;
           ss<<k;
           std::ostringstream cloud_name_ss;
           cloud_name_ss << std::setw(5) << std::setfill('0') << k;
           cv::imwrite(data_path+"color-frame-"+ss.str()+".png",image_data_vec[k]);
           cv::imwrite(data_path+"depth-frame-"+ss.str()+".png",depth_data_vec[k]);
           cv::imwrite(data_path+"color-segmented-frame-"+ss.str()+".png",converted_image);
           pcl::io::savePCDFile(data_path+"original_cloud_"+cloud_name_ss.str()+".pcd",*cloud_data_vec[k],true);
           std::string img_name,cloud_name;
           img_name = "frame-"+ss.str()+".png";
           cloud_name = cloud_name_ss.str()+".pcd";
           vector_img_name.push_back(img_name);
           vector_cloud_name.push_back(cloud_name);
        }
    saveNameTxt(data_path+"input_image.txt",vector_img_name);
    saveNameTxt(data_path+"input_cloud.txt",vector_cloud_name);

    //Save camera matrix
    std::string cam_info_file = data_path + "cam.info.txt";
    FILE *fp = fopen(cam_info_file.c_str(), "w");
    fprintf(fp, "%15.8e\t %15.8e\t %15.8e\t\n", realsense->getFocalX(), 0.0f, realsense->getPrintcipalX());
    fprintf(fp, "%15.8e\t %15.8e\t %15.8e\t\n", 0.0f, realsense->getFocalY(), realsense->getPrintcipalY());
    fprintf(fp, "%15.8e\t %15.8e\t %15.8e\t\n", 0.0f, 0.0f, 1.0f);
    fclose(fp);
    }

    CameraParameters cam_params;
    cam_params.focal_x = realsense->getFocalX();
    cam_params.c_x =realsense->getPrintcipalX();
    cam_params.focal_y =  realsense->getFocalY();
    cam_params.c_y = realsense->getPrintcipalY();
    cam_params.image_width = realsense->getWidth();
    cam_params.image_height = realsense->getHeight();
    float base2world[4 * 4];
    float cam2base[4 * 4];
    float cam2world[4 * 4];
   VolumeData volume(config_tsdf.volume_resolution_,config_tsdf.voxel_size_);
   eigenToPointer(matrix_camera_to_marker_vec[0],base2world);
   float base2world_inv[16] = {0};
   invert_matrix(base2world, base2world_inv);
   for(int i=0;i<depth_data_vec.size();i++)
   {
       //Only fuse frame with valid pose
       if(pose_flags_vec[i] ==0) continue;
       cout<<"fusing frame "<<i<<endl;
       cv::Mat color_mat = image_data_vec[i];
       cv::Mat depth_mat;
       depth_data_vec[i].convertTo(depth_mat,CV_32FC1,realsense->depthValue());
       eigenToPointer(matrix_camera_to_marker_vec[i],cam2world);
       multiply_matrix(base2world_inv, cam2world, cam2base);
       cv::Mat cam2base_cv ;
       float2cvMat(cam2base,cam2base_cv);
       //cout<<cam2base_cv<<endl;
       cv::cuda::GpuMat gpu_cam2base,gpu_depth_image,gpu_color_image;
       gpu_cam2base = cv::cuda::createContinuous(4,4,CV_32FC1);
       gpu_cam2base.upload(cam2base_cv);
       gpu_depth_image.upload(depth_mat);
       gpu_color_image.upload(color_mat);
       Cuda::hostTSDFVolume(gpu_depth_image,gpu_color_image,volume,
               cam_params,gpu_cam2base,config_tsdf.truncation_distance_,config_tsdf.depth_cutoff_,
                config_tsdf.original_distance_x_,config_tsdf.original_distance_y_,config_tsdf.original_distance_z_);
   }

   PointCloud cloud_tsdf = Cuda::hostExtractPointCloud(volume,config_tsdf.pointcloud_buffer_size_,
              config_tsdf.original_distance_x_,config_tsdf.original_distance_y_,config_tsdf.original_distance_z_);
   PCL_INFO("Save tsdf.ly \n");
   export_ply(data_path+"tsdf.ply",cloud_tsdf);
   SurfaceMesh marching_cube_mesh = Cuda::hostMarchingCubes(volume,config_tsdf.mesh_buffer_size_,config_tsdf.original_distance_x_,
                                                config_tsdf.original_distance_y_,config_tsdf.original_distance_z_);
   PCL_INFO("Save marching cube mesh \n");
   export_mesh(data_path + "marching_cubes_mesh.ply",marching_cube_mesh);
   Utils util;
   //TSDF cloud
   pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZRGB>);

   if(config_tsdf.debug_){
   saveTSDFCloudPCD(data_path + "/tsdf.pcd",cloud_tsdf,cloud );   
   }
    //Transform points to marker coordinate
   pcl::transformPointCloud(*cloud,*cloud,matrix_camera_to_marker_vec[0]);
   //Crop points that are outside of 3D bounding box
   filters->CropBox(cloud,config_tsdf.min_x_,config_tsdf.min_y_,config_tsdf.min_z_,
                  config_tsdf.max_x_,config_tsdf.max_y_,config_tsdf.max_z_);

   segmentation->euclideanCluster(cloud,1,2500000,config_tsdf.voxel_size_*1.1f,cloud);

   cout<<"MLS"<<endl;
   cloud = filters->movingLeastSquares(cloud,config_tsdf.mls_);

   //Close bottom of cloud
   util.closeCloud(cloud,config_tsdf.voxel_size_/2);
   cout<<"Create PCA"<<endl;
   util.bbox3DPCA(cloud);
   util.bbox3DInertia(cloud);
   util.viewCloud(cloud);
   util.isStopped();

   if(config_tsdf.debug_) pcl::io::savePCDFile(data_path+"closed_cloud.pcd",*cloud,true);

   cout<<"Mesing"<<endl;
   //Mesing
   pcl::PolygonMesh mesh;
   mesh = filters->surfaceReconstruction(cloud,config_tsdf.voxel_size_);
   util.viewMesh(mesh);
   util.isStopped();


   if(config_tsdf.debug_){
       cout<<"saving mesh"<<endl;
    pcl::io::saveOBJFile(data_path+"mesh.obj",mesh);
   }


    return 0;
}
