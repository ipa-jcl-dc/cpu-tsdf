#ifndef KINECFUSION_CONFIGURATION_HPP
#define KINECFUSION_CONFIGURATION_HPP


#include <iostream>
#include <vector>
#include <opencv2/opencv.hpp>
/**
 * @brief The Configuration class provides the API for the parameter configuration.It loads parameters from yaml file stored in "Config"
 * folder to perform needed functions
 *
 */

struct Configuration{
	//Declare default parameters, most of them are from the paper
	/*Bilateral filter parameters*/
	int num_level_ {3};
	int kernel_size_  {5};
	float sigma_color_ {1};
	float sigma_space_ {1};
	//Volume resolution of TSDF volume, it defines how many voxels in 3 dimensions x,y,z
	int volume_resolution_ {512};
	//Voxel size (in meters), the size of each voxel
	float voxel_size_ = 0.002f;
	//The initial distance of the camera from the volume center along the x,y,z-axis (in meters)
	float original_distance_x_ {-1.5f};
	float original_distance_y_ {-1.5f};
	float original_distance_z_ {0.5f};
	//The truncation distance for both updating and raycasting the TSDF volume
	float truncation_distance_ {0.025};
	//Depth distance cut off, if it is too far away from camera
	float depth_cutoff_ {2.0};
    // The maximum buffer size for exporting pointclouds; adjust if you run out of memory when exporting
	int pointcloud_buffer_size_ { 3 * 2000000 };
	// The maximum buffer size for exporting mesh marching cube; adjust if you run out of memory when exporting
	int mesh_buffer_size_ { 3 * 2000000 };
	float max_distance_{0.007};
    //# The distance threshold (as described in the paper) in meters
    float distance_threshold_{0.01};
    //# The angle threshold (as described in the paper) in degrees
    float angle_threshold_{20.0};
    //Number of iterations
    std::vector<int> iterations{10, 5, 4};

				/***Learning Station parameters***/
	//Perform Euclidean Clustering, only fuse clusters of points with more than threshold
	int cluster_threshold_ {3000};
	//Perform Marching Cubes if true
	int mesh_{1};
	//#Set mode for the learning station : Step by step :1 Continiously move 0
	int mode_{0};
	//Set up the speed of the learning station, the maximum speed = 14
	float speed_{14};
	//Bounding box to remove points outside(in meters)
	float min_x_{0.03};
	float max_x_{0.3};
	float min_y_{0.03};
	float max_y_{0.25};
	float min_z_{0.007};
	float max_z_{0.4};
    //Perform plane equalization
    int plane_equalized_{1};
	//Number of captures
	int n_capture_{12};
    //moving least square search radius
    float mls_{0.01};

    // Show images, clouds, etc for debugging
    int debug_{1};
};







#endif //KINECFUSION_CONFIGURATION_HPP
