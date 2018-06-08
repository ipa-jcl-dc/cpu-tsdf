#pragma once

#include <string.h>

#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/registration/correspondence_rejection_sample_consensus.h>
#include <pcl/registration/transformation_estimation.h>
#include <pcl/registration/transformation_estimation_point_to_plane.h>
#include <pcl/registration/elch.h>
#include <pcl/registration/icp.h>


#include <Eigen/Geometry>
#include <pcl/keypoints/sift_keypoint.h>
#include <pcl/search/impl/search.hpp>


/**
 * this class present registration methods that are used for the implementationi
 */
class Registrator {

public:

/*
 * TransformationEstimationSVD implements SVD-based estimation of
 * the transformation aligning the given correspondences.
 *
 * \note The class is templated on the source and target point types as well as on the output scalar of the transformation matrix (i.e., float or double). Default: float.
 *
 *
 */
	Eigen::Matrix4f TransformationEstimationSVD (const pcl::PointCloud<pcl::PointXYZRGB>::Ptr& source_points,
													const pcl::PointCloud<pcl::PointXYZRGB>::Ptr& target_points);


    /** Use SampleConsensusInitialAlignment to find a rough alignment from the source cloud to the target cloud by finding
 * correspondences between two sets of local features
 * Inputs:
 *   source_points
 *     The "source" points, i.e., the points that must be transformed to align with the target point cloud
 *   source_descriptors
 *
 *     The "target" points, i.e., the points to which the source point cloud will be aligned
 *   target_descriptors
 *
 *     The minimum distance between any two random samples
 *   max_correspondence_distance
 *     The
 *   nr_interations
 *     The number of RANSAC iterations to perform
 * Return: A transformation matrix that will roughly align the points in source to the points in target
 */
    Eigen::Matrix4f
    computeInitialAlignmentRANSAC (const pcl::PointCloud<pcl::PointXYZRGB>::Ptr& source_points,
                             const pcl::PointCloud<pcl::PointXYZRGB>::Ptr& target_points,
                             const pcl::CorrespondencesPtr& correspondences,double threshold);


/** Use IterativeClosestPoint to find a precise alignment from the source cloud to the target cloud,
 * starting with an intial guess
 * Inputs:
 *   source_points
 *     The "source" points, i.e., the points that must be transformed to align with the target point cloud
 *   target_points
 *     The "target" points, i.e., the points to which the source point cloud will be aligned
 *   intial_alignment
 *     An initial estimate of the transformation matrix that aligns the source points to the target points
 *   max_correspondence_distance
 *
 *     The maximum number of ICP iterations to perform is 100
 * Return: A transformation matrix that will precisely align the points in source to the points in target
 */
    Eigen::Matrix4f icpAlignment(const pcl::PointCloud<pcl::PointXYZRGB>::Ptr& source_points,
                             const pcl::PointCloud<pcl::PointXYZRGB>::Ptr& target_points,
                             pcl::PointCloud<pcl::PointXYZRGB>::Ptr& output,double max_correspondence_distance);


       Eigen::Matrix4f nicpAlignment(const pcl::PointCloud<pcl::PointXYZRGB>::Ptr& source_points,
                                const pcl::PointCloud<pcl::PointXYZRGB>::Ptr& target_points,
                                pcl::PointCloud<pcl::PointXYZRGB>::Ptr& output,double max_correspondence_distance);
  /*
   * Perform two step Alignments with given point clouds, key point clouds, correspondences
   *
   *
   */
    Eigen::Matrix4f twostepAlign(const pcl::PointCloud<pcl::PointXYZRGB>::Ptr & source_points,
    						  const pcl::PointCloud<pcl::PointXYZRGB>::Ptr & target_points,
    						  const pcl::PointCloud<pcl::PointXYZRGB>::Ptr& keypoints_sources,
    						  const pcl::PointCloud<pcl::PointXYZRGB>::Ptr& keypoints_target,
    						  pcl::CorrespondencesPtr correspondences, const Eigen::Matrix4f initial_matrix,
    						  pcl::PointCloud<pcl::PointXYZRGB>::Ptr& output, bool initial,double threshold);
    /*
    	* Loop Detection detects the loop based on eucliean distance between 2 centroids of clouds or computing number of correspondences of 2 clouds
    	*/
    	bool loopDetection(int end, std::vector<pcl::PointCloud<pcl::PointXYZRGB>::Ptr> &clouds, double dist,
    			int &first, int &last, double MIN_DISTANCE);
    	/*
    	* Global Alignment ELCH based on Loop Dectection
    	* cloud_data: a bunch of aligned clouds after performing fine alignment ICP
    	* output: The fused model after ELCH correction
    	* matrix_buffer : transformation of cloud relative to first frame
    	* cloud_out: a bunch of clouds after performing ELCH
    	* first, last : The parameters to detect loop
    	*/
    	void globalAlignmentELCH( std::vector<pcl::PointCloud<pcl::PointXYZRGB>::Ptr>& cloud_data,pcl::PointCloud<pcl::PointXYZRGB>::Ptr& output,
    			std::vector<Eigen::Matrix4f>& matrix_buffer,std::vector<pcl::PointCloud<pcl::PointXYZRGB>::Ptr> cloud_out,int& first, int& last);
    	//Compare 2 point clouds in 2 poses
    	Eigen::Affine3f compare(pcl::PointCloud<pcl::PointXYZRGB> p1, pcl::PointCloud<pcl::PointXYZRGB> p2);
    	/*
    	 * This function assumes that the loop detection is between first frame and last frame. No need to perform loop detection
    	 */
    	void elchCorrection(std::vector<pcl::PointCloud<pcl::PointXYZRGB>::Ptr>& cloud_data,std::vector<Eigen::Matrix4f>& matrix_buffer, pcl::PointCloud<pcl::PointXYZRGB>::Ptr& output);
};
