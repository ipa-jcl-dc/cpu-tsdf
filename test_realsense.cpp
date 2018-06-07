//C++
#include <curl/curl.h>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <sstream>
#include <istream>
#include <string>
//PCL



#include <pcl/common/common_headers.h>
#include <pcl/io/io.h>
#include <pcl/io/obj_io.h>
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
#include <pcl/surface/impl/mls.hpp>
#include <pcl/features/normal_3d_omp.h>
#include <pcl/surface/marching_cubes_rbf.h>
#include <pcl/features/normal_3d.h>
#include <pcl/surface/gp3.h>
#include <pcl/features/normal_3d_omp.h>
#include <pcl/surface/grid_projection.h>
#include <pcl/surface/vtk_smoothing/vtk_mesh_smoothing_laplacian.h>
#include <pcl/surface/marching_cubes_hoppe.h>
#include <pcl/surface/mls.h>
#include "segmentation.hpp"
#include "filters.hpp"
#include "util.hpp"
#include "matrix_utils.hpp"
#include "aruco_marker.hpp"
#include "realsense2.hpp"
#include "realsense.hpp"
#include "curlserver.hpp"
#include "util/random_utils.h"
//CUDA
#include "cuda_headers.hpp"
#include <cuda_runtime.h>
#include "device_launch_parameters.h"
#include "configuration.hpp"

#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include <opencv2/core/eigen.hpp>
#include <opencv2/opencv.hpp>
#define EIGEN_MAX_STATIC_ALIGN_BYTES =16
#include <Eigen/Core>



using namespace std;
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
             cv::FileNode n;
             n = config_file["BoundingBox"];
             n["mixX"]>>config.min_x_;
             n["maxX"]>>config.max_x_;
             n["minY"]>>config.min_y_;
             n["maxY"]>>config.max_y_;
             n["minZ"]>>config.min_z_;
             n["maxZ"]>>config.max_z_;

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
            file_out << "property uchar red" << std::endl;
            file_out << "property uchar green" << std::endl;
            file_out << "property uchar blue" << std::endl;
            file_out << "element face " << surface_mesh.num_triangles << std::endl;
            file_out << "property list uchar int vertex_index" << std::endl;
            file_out << "end_header" << std::endl;

            for (int v_idx = 0; v_idx < surface_mesh.num_vertices; ++v_idx) {
                float3 vertex = surface_mesh.triangles.ptr<float3>(0)[v_idx];
                uchar3 color = surface_mesh.colors.ptr<uchar3>(0)[v_idx];
                file_out << vertex.x << " " << vertex.y << " " << vertex.z << " ";
                file_out << (int) color.z << " " << (int) color.y << " " << (int) color.x << std::endl;
            }

           for (int t_idx = 0; t_idx < surface_mesh.num_vertices; t_idx += 3) {
                //file_out << 3 << " " << t_idx + 1 << " " << t_idx << " " << t_idx + 2 << std::endl;
               file_out << 3 << " " << t_idx << " " << t_idx+1 << " " << t_idx + 2 << std::endl;
            }

           file_out.close();

}
void exportPCLMesh(const SurfaceMesh& surface_mesh,pcl::PolygonMesh& mesh)
{
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
    cloud->points.resize(surface_mesh.num_vertices);
    for(int i=0;i<surface_mesh.num_vertices;++i)
    {
        pcl::PointXYZRGB& pt = cloud->points[i];
        float3 vertex = surface_mesh.triangles.ptr<float3>(0)[i];
        uchar3 color = surface_mesh.colors.ptr<uchar3>(0)[i];
        pt.x = vertex.x;
        pt.y = vertex.y;
        pt.z = vertex.z;
        pt.b = static_cast<uchar>(color.x);
        pt.g = static_cast<uchar>(color.y);
        pt.r = static_cast<uchar>(color.z);
    }
   pcl::toPCLPointCloud2 (*cloud,mesh.cloud);
   mesh.polygons.resize(surface_mesh.num_vertices/3);
   for (size_t i = 0; i < mesh.polygons.size(); ++i)
     {
       pcl::Vertices v;
       v.vertices.push_back (i*3+0);
       v.vertices.push_back (i*3+1);
       v.vertices.push_back (i*3+2);
       mesh.polygons[i] = v;
   }
}

int main(int argc,char** argv)
{

    if(argc < 2)
    {
        std::cout <<"Usage : ./test_realsense ../path/to/data folder/"<<std::endl;
        return EXIT_FAILURE;
    }
    RealSense2 realsense;
    CameraParameters cam_params;
    cam_params.focal_x = realsense.getFocalX();
    cam_params.c_x =realsense.getPrintcipalX();
    cam_params.focal_y =  realsense.getFocalY();
    cam_params.c_y = realsense.getPrintcipalY();
    cam_params.image_width = realsense.getWidth();
    cam_params.image_height = realsense.getHeight();


    string data_path = argv[1];


    Segmentation segmentation;
    Filters filter;
    Loader loader;
    //Utils utils;
    Configuration config_;

    readParameters(config_);
    float depth_scale = realsense.depthValue();
    //cout<<depth_K<<endl;
    vector<Eigen::Matrix4f> transforms;
    vector<int> pose_flags;
    std::vector<cv::Mat> depth_image_vec;
    std::vector<cv::Mat> color_image_vec;
    readTransformsTxt(data_path+"transform.txt",transforms,pose_flags);
    loader.loadImages(color_image_vec,data_path);
    loader.loadDepthImages(depth_image_vec,data_path);

/*
    for(int i=0;i<cloud_data.size();i++)
    {
        pcl::transformPointCloud(*cloud_data[i],*cloud_data[i],transforms[i]);
        vector<int> indices;
        filter.CropBox(cloud_data[i],config_.min_x_,config_.min_y_,config_.min_z_,config_.max_x_,config_.max_y_,config_.max_z_,indices);
        cv::Mat img;
        segmentation.cloud2image(img,cloud_data[i],indices);
        segmentation.cropDepthFromColor(img,depth_image_vec[i]);
    }
*/
    cout<<"asd"<<endl;
    float base2world[4 * 4];
    float cam2base[4 * 4];
    float cam2world[4 * 4];
   VolumeData volume(config_.volume_resolution_,config_.voxel_size_);
   eigenToPointer(transforms[0],base2world);
   float base2world_inv[16] = {0};
   invert_matrix(base2world, base2world_inv);
    cout<<depth_image_vec.size()<<endl;
    cout<<color_image_vec.size()<<endl;

   for(int i=0;i<depth_image_vec.size();i++)
   {
        cout<<"fusing frame "<<i<<endl;
        cv::Mat color_mat = color_image_vec[i];
        cv::Mat depth_mat;
        depth_image_vec[i].convertTo(depth_mat,CV_32FC1,depth_scale);
        eigenToPointer(transforms[i],cam2world);
        multiply_matrix(base2world_inv, cam2world, cam2base);
        cv::Mat cam2base_cv ;
        float2cvMat(cam2base,cam2base_cv);
        //cout<<cam2base_cv<<endl;
        //upload
       cv::cuda::GpuMat gpu_cam2base,gpu_depth_image,gpu_color_image;
       gpu_cam2base = cv::cuda::createContinuous(4,4,CV_32FC1);
       gpu_cam2base.upload(cam2base_cv);
       gpu_depth_image.upload(depth_mat);
       gpu_color_image.upload(color_mat);
       Cuda::hostTSDFVolume(gpu_depth_image,gpu_color_image,volume,
               cam_params,gpu_cam2base,config_.truncation_distance_,config_.depth_cutoff_,
                config_.original_distance_x_,config_.original_distance_y_,config_.original_distance_z_);
   }

   PointCloud cloud_tsdf = Cuda::hostExtractPointCloud(volume,config_.pointcloud_buffer_size_,
              config_.original_distance_x_,config_.original_distance_y_,config_.original_distance_z_);

  SurfaceMesh marching_cube_mesh = Cuda::hostMarchingCubes(volume,config_.mesh_buffer_size_,config_.original_distance_x_,
                                               config_.original_distance_y_,config_.original_distance_z_);
  export_ply(data_path+ "/test_realsense_tsdf.ply",cloud_tsdf);
  export_mesh(data_path + "/test_realsense_mesh_marching.ply",marching_cube_mesh);
  pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr cloud_with_normals(new pcl::PointCloud<pcl::PointXYZRGBNormal>);
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZRGB>);

  saveTSDFCloudPCD(data_path + "/test_realsense_tsdf_with_normal.pcd",cloud_tsdf,cloud_with_normals );
  saveTSDFCloudPCD(data_path + "/test_realsense_tsdf.pcd",cloud_tsdf,cloud);
 // cv::Mat tsdf_mat;
 // volume.tsdf_volume.download(tsdf_mat);
  //saveBinFile(data_path+"/tsdf.bin",tsdf_mat,config_);

  pcl::PolygonMesh mesh,mesh2;
  exportPCLMesh(marching_cube_mesh,mesh);
  Utils util;

  util.viewMesh(mesh);
  util.isStopped();

  pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_mesh(new pcl::PointCloud<pcl::PointXYZRGB>);
  pcl::fromPCLPointCloud2(mesh.cloud,*cloud_mesh);
  pcl::transformPointCloud(*cloud_mesh,*cloud_mesh,transforms[0]);

  filter.CropBox(cloud_mesh,config_.min_x_,config_.min_y_,config_.min_z_,
                 config_.max_x_,config_.max_y_,config_.max_z_);
//Transform back
  Eigen::Matrix4f m_inv;
  m_inv = transforms[0].inverse();
  pcl::transformPointCloud(*cloud_mesh,*cloud_mesh,m_inv);


  pcl::toPCLPointCloud2(*cloud_mesh,mesh2.cloud);

  mesh2.polygons.resize(cloud_mesh->points.size()/3);
  for (size_t i = 0; i < mesh2.polygons.size(); ++i)
    {
      pcl::Vertices v;
      v.vertices.push_back (i*3+0);
      v.vertices.push_back (i*3+1);
      v.vertices.push_back (i*3+2);
      mesh2.polygons[i] = v;
  }


  util.viewMesh(mesh2);
  util.isStopped();

        //Laplacian object
/*
        pcl:: MeshSmoothingLaplacianVTK vtk;
         pcl:: PolygonMesh output;
        //Laplacian Smoothing of mesh
        vtk.setInputMesh(triangles);
        vtk.setNumIter(20000);
        vtk.setConvergence(0.0001);
        vtk.setRelaxationFactor(0.0001);
        vtk.setFeatureEdgeSmoothing(true);
        vtk.setFeatureAngle(M_PI/5);
        vtk.setBoundarySmoothing(true);
        vtk.process(output);
        pcl::io::saveOBJFile(data_path+"/test_realsense_smoothed.obj",output);
        */


    /*

    mesh = filter.surfaceReconstruction(cloud,config_.voxel_size_);
     util.viewMesh(mesh);
     util.isStopped();
    */
 // cout<<"meshing"<<endl;
 // pcl:: PolygonMesh mesh;
 // pcl::transformPointCloud(*cloud_with_normals,*cloud_with_normals,transforms[0]);
 // filter.CropBox(cloud_with_normals,config_.min_x_,config_.min_y_,config_.min_z_,config_.max_x_,config_.max_y_,config_.max_z_);
 // filter.removeOutliers(cloud_with_normals,1,10);
 // pcl::PointCloud<pcl::PointNormal>::Ptr cloud_normals(new pcl::PointCloud<pcl::PointNormal>);
 // pcl::copyPointCloud(*cloud_with_normals,*cloud_normals);
/*  pcl::Poisson<pcl::PointNormal> poisson;
  poisson.setDepth(9);
  poisson.setInputCloud(cloud_normals);
  poisson.reconstruct(mesh);
  //pcl::io::savePCDFile(data_path+"/test_realsense_closed_cloud.pcd",*cloud,true);
  Utils util;

  util.viewMesh(mesh);
  util.isStopped();
   */
  cout<<"results are saved into folder : "<<data_path<<endl;
return 0;
}
