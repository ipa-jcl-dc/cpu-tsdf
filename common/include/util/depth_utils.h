
#include <opencv2/opencv.hpp>
#include <librealsense2/rs.hpp>
#include <librealsense2/rsutil.h> // Include RealSense Cross Platform API
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/io/pcd_io.h>
#include <iostream>
#include <string.h>


void WriteDepth(float * depth_values, int frame_height, int frame_width,cv::Mat& depth_mat) {
	depth_mat = cv::Mat(frame_height, frame_width, CV_16UC1);
	for (size_t y = 0; y < frame_height; y++)
		for (size_t x = 0; x < frame_width; x++) {
			unsigned short depth_short = (unsigned short)(depth_values[y * frame_width + x]);
			//depth_short = (depth_short >> 13 | depth_short << 3);
			depth_mat.at<unsigned short>(y, x) = depth_short;
		}	
}

