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
#include <pcl/io/pcd_io.h>
#include <pcl/io/ply_io.h>
#include <pcl/io/obj_io.h>
#include <pcl/io/vtk_io.h>
#include <pcl/io/vtk_lib_io.h>
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
#include <pcl/filters/passthrough.h>
#include <pcl/registration/correspondence_rejection_sample_consensus.h>
#include <pcl/registration/transformation_estimation_point_to_plane.h>
#include <pcl/surface/impl/mls.hpp>
#include <pcl/features/normal_3d_omp.h>
#include <pcl/surface/marching_cubes_rbf.h>
#include <pcl/features/normal_3d.h>
#include <pcl/surface/gp3.h>
#include <pcl/surface/vtk_smoothing/vtk_mesh_smoothing_laplacian.h>
#include <pcl/surface/marching_cubes_rbf.h>
#include "segmentation.hpp"
#include "filters.hpp"
#include "util.hpp"
#include "matrix_utils.hpp"
#include "aruco_marker.hpp"
#include "realsense2.hpp"
#include "realsense.hpp"
#include "curlserver.hpp"
#include "util/random_utils.h"
#include "kinect_fusion.hpp"
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
            file_out << "property list uchar int vertex_indices" << std::endl;
            file_out << "end_header" << std::endl;

            for (int v_idx = 0; v_idx < surface_mesh.num_vertices; ++v_idx) {
                float3 vertex = surface_mesh.triangles.ptr<float3>(0)[v_idx];
                uchar3 color = surface_mesh.colors.ptr<uchar3>(0)[v_idx];
                file_out << vertex.x << " " << vertex.y << " " << vertex.z << " ";
                file_out << (int) color.z << " " << (int) color.y << " " << (int) color.x << std::endl;
            }

           for (int t_idx = 0; t_idx < surface_mesh.num_vertices; t_idx += 3) {
                file_out << 3 << " " << t_idx + 1 << " " << t_idx << " " << t_idx + 2 << std::endl;
            }

           file_out.close();

}

int main(int argc,char** argv)
{
    if(argc < 2)
    {
        std::cout <<"Usage : ./test_realsense ../path/to/data folder/"<<std::endl;
        return EXIT_FAILURE;
    }
    Configuration config;
    string data_path = argv[1];
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PolygonMesh triangles ;
    //pcl::io::loadPLYFile(data_path+"/marching_cubes_mesh.ply",triangles);
    pcl::io::loadOBJFile(data_path+"/mesh.obj",triangles);
    std::vector<int> pose_flags;
    std::vector<Eigen::Matrix4f> transforms;
    readTransforms(data_path+"/transform.yaml",transforms,pose_flags,36);
    pcl::texture_mapping::CameraVector cams;
    pcl::texture_mapping::Camera cam,cam1,cam2,cam3;
    //Default camera parameter for the SR300 that currently is using
    cam3.focal_length =  6.14901245e+02	;
    cam3.height = 480;
    cam3.width = 640;
    cam3.center_w = 3.15093414e+02;
    cam3.center_h = 2.38158493e+02;
    std::ostringstream ss3;
    ss3<<17;
    // RGB image file that contain texture information, should be change when different types of file
    cam3.texture_file = data_path + "/color-segmented-frame-"+ss3.str() +".png";
    cam3.pose(0, 3) = transforms[17](0,3);
    cam3.pose(1, 3) = transforms[17](1,3);
    cam3.pose(2, 3) = transforms[17](2,3);

    cam3.pose(0, 0) = transforms[17](0,0);
    cam3.pose(0, 1) = transforms[17](0,1);
    cam3.pose(0, 2) = transforms[17](0,2);

    cam3.pose(1, 0) = transforms[17](1,0);
    cam3.pose(1, 1) = transforms[17](1,1);
    cam3.pose(1, 2) = transforms[17](1,2);

    cam3.pose(2, 0) = transforms[17](2,0);
    cam3.pose(2, 1) = transforms[17](2,1);
    cam3.pose(2, 2) = transforms[17](2,2);

    cam3.pose(3, 0) = 0.0;
    cam3.pose(3, 1) = 0.0;
    cam3.pose(3, 2) = 0.0;
    cam3.pose(3, 3) = 1.0; //scale
    cams.push_back(cam3);




           //Default camera parameter for the SR300 that currently is using
           cam.focal_length =  6.14901245e+02;
           cam.height = 480;
           cam.width = 640;
           cam.center_w = 3.15093414e+02;
           cam.center_h = 2.38158493e+02;
           std::ostringstream ss;
           ss<<35;
           // RGB image file that contain texture information, should be change when different types of file
           cam.texture_file = data_path + "/color-segmented-frame-"+ss.str() +".png";
           cam.pose(0, 3) = transforms[35](0,3);
           cam.pose(1, 3) = transforms[35](1,3);
           cam.pose(2, 3) = transforms[35](2,3);

           cam.pose(0, 0) = transforms[35](0,0);
           cam.pose(0, 1) = transforms[35](0,1);
           cam.pose(0, 2) = transforms[35](0,2);

           cam.pose(1, 0) = transforms[35](1,0);
           cam.pose(1, 1) = transforms[35](1,1);
           cam.pose(1, 2) = transforms[35](1,2);

           cam.pose(2, 0) = transforms[35](2,0);
           cam.pose(2, 1) = transforms[35](2,1);
           cam.pose(2, 2) = transforms[35](2,2);

           cam.pose(3, 0) = 0.0;
           cam.pose(3, 1) = 0.0;
           cam.pose(3, 2) = 0.0;
           cam.pose(3, 3) = 1.0; //scale
           cams.push_back(cam);


           //Default camera parameter for the SR300 that currently is using
           cam1.focal_length =  6.14901245e+02	;
           cam1.height = 480;
           cam1.width = 640;
           cam1.center_w = 3.15093414e+02;
           cam1.center_h = 2.38158493e+02;
           std::ostringstream ss1;
           ss1<<8;
           // RGB image file that contain texture information, should be change when different types of file
           cam1.texture_file = data_path + "/color-segmented-frame-"+ss1.str() +".png";
           cam1.pose(0, 3) = transforms[8](0,3);
           cam1.pose(1, 3) = transforms[8](1,3);
           cam1.pose(2, 3) = transforms[8](2,3);

           cam1.pose(0, 0) = transforms[8](0,0);
           cam1.pose(0, 1) = transforms[8](0,1);
           cam1.pose(0, 2) = transforms[8](0,2);

           cam1.pose(1, 0) = transforms[8](1,0);
           cam1.pose(1, 1) = transforms[8](1,1);
           cam1.pose(1, 2) = transforms[8](1,2);

           cam1.pose(2, 0) = transforms[8](2,0);
           cam1.pose(2, 1) = transforms[8](2,1);
           cam1.pose(2, 2) = transforms[8](2,2);

           cam1.pose(3, 0) = 0.0;
           cam1.pose(3, 1) = 0.0;
           cam1.pose(3, 2) = 0.0;
           cam1.pose(3, 3) = 1.0; //scale
           cams.push_back(cam1);




           //Default camera parameter for the SR300 that currently is using
           cam2.focal_length =  6.14901245e+02	;
           cam2.height = 480;
           cam2.width = 640;
           cam2.center_w = 3.15093414e+02;
           cam2.center_h =  2.38158493e+02;
           std::ostringstream ss2;
           ss2<<23;
           // RGB image file that contain texture information, should be change when different types of file
           cam2.texture_file = data_path + "/color-segmented-frame-"+ss2.str() +".png";
           cam2.pose(0, 3) = transforms[23](0,3);
           cam2.pose(1, 3) = transforms[23](1,3);
           cam2.pose(2, 3) = transforms[23](2,3);

           cam2.pose(0, 0) = transforms[23](0,0);
           cam2.pose(0, 1) = transforms[23](0,1);
           cam2.pose(0, 2) = transforms[23](0,2);

           cam2.pose(1, 0) = transforms[23](1,0);
           cam2.pose(1, 1) = transforms[23](1,1);
           cam2.pose(1, 2) = transforms[23](1,2);

           cam2.pose(2, 0) = transforms[23](2,0);
           cam2.pose(2, 1) = transforms[23](2,1);
           cam2.pose(2, 2) = transforms[23](2,2);

           cam2.pose(3, 0) = 0.0;
           cam2.pose(3, 1) = 0.0;
           cam2.pose(3, 2) = 0.0;
           cam2.pose(3, 3) = 1.0; //scale
           cams.push_back(cam2);







               pcl::fromPCLPointCloud2(triangles.cloud, *cloud);
               //Create a texture mesh object that will contain UV-mapped mesh
               pcl::TextureMesh mesh;
               mesh.cloud = triangles.cloud;
               std::vector<pcl::Vertices> polygon;
                // push faces into the texturemesh object
               polygon.resize(triangles.polygons.size());
               for(size_t i=0;i<triangles.polygons.size();i++)
               {
                    polygon[i] = triangles.polygons[i];
               }
               mesh.tex_polygons.push_back(polygon);
               // Load textures and cameras poses and intrinsics
                PCL_INFO ("\nLoading textures and camera poses...\n");
                // Create materials for each texture (and one extra for occluded faces)
                  mesh.tex_materials.resize (cams.size () + 1);
                  for(int i = 0 ; i <= cams.size() ; ++i)
                  {
                    pcl::TexMaterial mesh_material;
                    mesh_material.tex_Ka.r = 0.2f;
                    mesh_material.tex_Ka.g = 0.2f;
                    mesh_material.tex_Ka.b = 0.2f;

                    mesh_material.tex_Kd.r = 0.8f;
                    mesh_material.tex_Kd.g = 0.8f;
                    mesh_material.tex_Kd.b = 0.8f;

                    mesh_material.tex_Ks.r = 1.0f;
                    mesh_material.tex_Ks.g = 1.0f;
                    mesh_material.tex_Ks.b = 1.0f;

                    mesh_material.tex_d = 1.0f;
                    mesh_material.tex_Ns = 75.0f;
                    mesh_material.tex_illum = 2;

                    std::stringstream tex_name;
                    tex_name << "material_" << i;
                    tex_name >> mesh_material.tex_name;

                    if(i < cams.size ())
                      mesh_material.tex_file = cams[i].texture_file;
                    else
                      mesh_material.tex_file = "occluded.jpg";

                    mesh.tex_materials[i] = mesh_material;
                  }
                  // Sort faces
                  PCL_INFO ("\nSorting faces by cameras...\n");
                pcl::TextureMapping<pcl::PointXYZ> texture_mapping; // TextureMapping object that will perform the sort
                texture_mapping.textureMeshwithMultipleCameras(mesh, cams);
                PCL_INFO ("Sorting faces by cameras done.\n");
                for(int i = 0 ; i <= cams.size() ; ++i)
                {
                  PCL_INFO ("\tSub mesh %d contains %d faces and %d UV coordinates.\n", i, mesh.tex_polygons[i].size (), mesh.tex_coordinates[i].size ());
                }
                // compute normals for the mesh
                        PCL_INFO ("\nEstimating normals...\n");
                        pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> n;
                        pcl::search::KdTree<pcl::PointXYZ>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZ>);
                        pcl::PointCloud<pcl::Normal>::Ptr normals_mesh (new pcl::PointCloud<pcl::Normal>);
                        tree->setInputCloud (cloud);
                        n.setInputCloud (cloud);
                        n.setSearchMethod (tree);
                        n.setKSearch (20);
                        n.compute (*normals_mesh);
                        // Concatenate XYZ and normal fields
                        pcl::PointCloud<pcl::PointNormal>::Ptr cloud_with_normals_mesh (new pcl::PointCloud<pcl::PointNormal>);
                        pcl::concatenateFields (*cloud, *normals_mesh, *cloud_with_normals_mesh);
                        PCL_INFO ("...Done.\n");
                        pcl::toPCLPointCloud2 (*cloud_with_normals_mesh, mesh.cloud);
                        PCL_INFO ("\nSaving mesh to textured_mesh.obj\n");
           pcl::io::saveOBJFile("texture_mesh.obj",mesh,5);


    return 0;
}
