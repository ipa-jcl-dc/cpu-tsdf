#pragma once

#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/io/pcd_io.h>
#include "opencv2/opencv.hpp"
#include <string.h>
#include "features.hpp"
#include "tinyxml.h"
/**
 * convenient helper class to load the point clouds and images from disk
 */

/*Struct of images */
struct image {
      cv::Mat img;      
      cv::Mat dep;      
  };
typedef std::vector<image> image_data;

class Loader{
public:
  /*Load point cloud */

    boost::shared_ptr<pcl::PointCloud<pcl::PointXYZRGB> >
    loadPointCloud (std::string filename, std::string suffix);

    void loadClouds(std::vector<pcl::PointCloud<pcl::PointXYZRGB>::Ptr>& cloud_data, const std::string& root_dir);

    pcl::PointCloud<pcl::PointXYZRGB>::Ptr
    loadPoints (std::string filename);

    pcl::PointCloud<pcl::Normal>::Ptr
    loadSurfaceNormals(std::string filename);

    pcl::PointCloud<pcl::PointXYZRGB>::Ptr
    loadKeypoints (std::string filename);

    pcl::PointCloud<pcl::FPFHSignature33>::Ptr
    loadLocalDescriptors (std::string filename);

    pcl::PointCloud<pcl::VFHSignature308>::Ptr
    loadGlobalDescriptors (std::string filename);

  /* Load Images */
    cv::Mat readColorImage(std::string filename,std::string suffix);
    void loadDepthImages(std::vector<cv::Mat>& images, const std::string& root_dir);
    void loadImages(std::vector<cv::Mat>& images, const std::string& root_dir);
/*Load Parameters */
    unsigned long ReadParameters(std::string directory_and_filename,
                   pcl::PointXYZRGB& min_point, pcl::PointXYZRGB& max_point,double&  max_correspondence_distance);
};


/**
 * convenient helper class to save the point clouds to disk
 */


class Saver{

public:

    int saveObjectFeatures(std::string filename, boost::shared_ptr<Features::ObjectFeatures> &objFeatures);

    int savePoints(std::string filename, pcl::PointCloud<pcl::PointXYZRGB>::Ptr &points);

    int saveSurfaceNormals(std::string filename, pcl::PointCloud<pcl::Normal>::Ptr &normals);

    int saveKeypoints(std::string filename, pcl::PointCloud<pcl::PointXYZRGB>::Ptr &keypoints);

    int saveLocalDescriptors(std::string filename, pcl::PointCloud<pcl::FPFHSignature33>::Ptr &signature);

    int saveGlobalDescriptors(std::string filename, pcl::PointCloud<pcl::VFHSignature308>::Ptr &signature);





};
