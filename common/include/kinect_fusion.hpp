#pragma once
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
#include <mutex>
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
//header files
#include "util.hpp"
#include "matrix_utils.hpp"
#include "configuration.hpp"
#include "data_types.hpp"
class Kinect{
public:
    ////
    /// \brief showCameras  Display a 3D representation showing the a cloud and a list of camera with their 6DOf poses
    /// \param cams : camera pose information
    /// \param cloud : the point cloud of an object
    ///
    void showCameras (const pcl::texture_mapping::CameraVector& cams,const pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud);
    ////
    /// \brief readCamPose read camera pose from file "transform.yaml" in the folder
    /// \param file_name directory path contains data and camera pose
    /// \param cams : camera pose from camera to world coordinate information
    /// \param transforms: world transforms of frames and their pose flags
    /// \return true if successful
    ///
    bool readCamPose(std::string file_name, pcl::texture_mapping::CameraVector& cams,const std::vector<Eigen::Matrix4f>& transforms,
                     const std::vector<int>& pose_flags);
    ////
    /// \brief createMesh : This function create a polygon mesh using Fast triangulation algorithm with given point cloud
    /// Detail can be found: http://pointclouds.org/documentation/tutorials/greedy_projection.php#greedy-triangulation
    /// \param cloud : input point cloud
    /// \return polygon mesh
    ///
    pcl::PolygonMesh createPolygonMesh(const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud);
    
    void textureMapping(const pcl::PolygonMesh& triangles, const pcl::texture_mapping::CameraVector& cams,std::string mesh_file);
//private:
    //std::mutex mutex_;
    //const CameraParameters cam_params_;
    //const Configuration config_;
};
