#pragma once


#include <opencv2/core/core.hpp>
#include <opencv2/opencv.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <librealsense2/rs.hpp>
#include <librealsense2/rsutil.h>
#include <librealsense2/hpp/rs_internal.hpp>
#include "config.hpp"
#include "filters.hpp"

//PCL
#include <pcl/ModelCoefficients.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/features/normal_3d.h>
#include <pcl/kdtree/kdtree.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/segmentation/extract_clusters.h>
#include <iostream>

const int HEIGHT = 480;
const int WIDTH = 640;

class Segmentation :public Config {

public:
    ////
    /// \brief cloud2image : convert point cloud to cv::Mat without indiece(size should be 640x480)
    /// \param img
    /// \param cloud
    /// \return
    ///
    unsigned long cloud2image(cv::Mat& img,const pcl::PointCloud<pcl::PointXYZRGB>::Ptr& cloud);

    ////
    /// \brief cloud2image Convert organized point cloud to cv::Mat (assume that depth and color frame are registed with 640x480 size)
    /// \param img: output img
    /// \param cloud input point cloud
    /// \param Given indice vector of input point cloud
    /// \return  success or fairlure
    ///
    unsigned long cloud2image(cv::Mat& img,const pcl::PointCloud<pcl::PointXYZRGB>::Ptr& cloud,const std::vector<int>& indices);


    unsigned long cropDepthFromColor(const cv::Mat& color_map,cv::Mat& depth_map);

    ////
    /// \brief objectContour : Draw contour of object in marker board
    /// \param img : img from rgb stream
    /// \param converted_img : img converted from aligned depth stream
    ///
    void objectContour(cv::Mat& img,const cv::Mat& converted_img);
	//void createMask(const cv::Mat& img,const pcl::PointCloud<pcl::PointXYZRGB>::Ptr& cloud,cv::Mat& mask);


    ////
    /// \brief euclideanCluster : perform Euclidean Clustering
    /// \param cloud : pointcloud input
    /// \param min : clusters found must have at least min points
    /// \param max : clusters found must have at least max points
    /// \param tolerance : L2 distance of cluster, if value is small, an actual object can be seen as multiple clusters
    ///  if value is too high, multiple objects are seen as one cluster.
    /// \param output : the cluster with maximum points
    ///
    void euclideanCluster(pcl::PointCloud<pcl::PointXYZRGB>::Ptr& cloud, int min, int max, double tolerance,pcl::PointCloud<pcl::PointXYZRGB>::Ptr & output);

private:
    boost::shared_ptr<Config> config_ = boost::make_shared<Config>();



};

