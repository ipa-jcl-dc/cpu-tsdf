#pragma once
#include <iostream>
#include <vector>

#include <opencv2/core/core.hpp>
#include <opencv2/opencv.hpp>

/**
 * @brief The Config class provides the API for the parameter configuration.It loads parameters from yaml file stored in "Config"
 * folder to perform needed functions
 *
 */
const std::string config_file_path = "../config/config.yaml";
class Config
{
protected:
    /*intrinsics matrix of camera*/
     cv::Mat camera_matrix1_, camera_matrix2_, camera_matrix3_, camera_matrix4_ ;
     /*The distance to perform registration */
     float max_distance_;
     /*defaut bouding box to do the segmentation in the marker board for the pi tag, original point in the center
     * of the board
     * */
     float min_x_,max_x_,min_y_,max_y_,min_z_,max_z_;
     /*Number of perspectives to capture*/
     int n_capture_;
     /*show cloud or image for debugging*/
     bool debug_;
     /* parameter indicate moving mode of learning station*/
     int mode_;
     bool mesh_;
     /*Perform Euclidean Clustering, only fuse clusters of points with more than threshold*/
     int threshold_cluster_;
public:


    Config()
    {

        // ********** try to load the yaml file that is located at the given path **********
        cv::FileStorage config_file(config_file_path,cv::FileStorage::READ);
        config_file["cameraMatrix1"]>> camera_matrix1_;
        config_file["cameraMatrix2"]>> camera_matrix2_;
        config_file["cameraMatrix3"]>> camera_matrix3_;
        config_file["cameraMatrix4"]>> camera_matrix4_;
        config_file["maxDistance"]>>max_distance_;
        config_file["nCapture"]>>n_capture_;
        config_file["debug"]>>debug_;
        config_file["mode"]>>mode_;
        config_file["mesh"]>>mesh_;
        config_file["threshold_cluster"]>>threshold_cluster_;
        cv::FileNode n;
        n = config_file["BoundingBox"];
        n["mixX"]>>min_x_;
        n["maxX"]>>max_x_;
        n["minY"]>>min_y_;
        n["maxY"]>>max_y_;
        n["minZ"]>>min_z_;
        n["maxZ"]>>max_z_;

    }
    ~Config()
    {
    }

// Getter funtions that return parameters from configuration file
      cv::Mat cameraMatrix1() { return camera_matrix1_;}
      cv::Mat cameraMatrix2() { return camera_matrix2_;}
      cv::Mat cameraMatrix3() { return camera_matrix3_;}
      cv::Mat cameraMatrix4() { return camera_matrix4_;}

      float maxDistance() {return max_distance_;}
      float minX() {return min_x_;}
      float maxX() {return max_x_;}
      float minY() {return min_y_;}
      float maxY() {return max_y_;}
      float minZ() {return min_z_;}
      float maxZ() {return max_z_;}
      float nCapture() {return n_capture_;}
      bool debug() {return debug_;}
      int mode() {return mode_;}
      bool mesh() {return mesh_;}
      int clusterThreshold() {return threshold_cluster_;}
};
