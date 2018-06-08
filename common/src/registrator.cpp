#include "registrator.hpp"
#include "util.hpp"
#include <pcl/search/impl/search.hpp> //this header is needed for succesful linking on *unix machines
#include <pcl/registration/icp.h>
#include <pcl/features/normal_3d.h>
#include <pcl/filters/radius_outlier_removal.h>
#include <pcl/search/kdtree.h>
#include <pcl/impl/point_types.hpp>
#include <pcl/registration/sample_consensus_prerejective.h>
#include <pcl/registration/icp_nl.h>

#include <pcl/keypoints/harris_3d.h>
#include <pcl/registration/transformation_estimation.h>
#include <pcl/filters/approximate_voxel_grid.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/io/pcd_io.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/registration/ia_ransac.h>
#include <pcl/registration/ndt.h>
#include <pcl/registration/icp_nl.h>
#include <pcl/keypoints/sift_keypoint.h>
#include <pcl/registration/correspondence_rejection_features.h>
#include <pcl/common/transformation_from_correspondences.h>
#include <pcl/registration/correspondence_rejection_sample_consensus.h>
#include <pcl/registration/transformation_estimation_point_to_plane.h>
#include <boost/shared_ptr.hpp>
#include <boost/make_shared.hpp>

#include <string.h>

Eigen::Matrix4f Registrator::TransformationEstimationSVD (const pcl::PointCloud<pcl::PointXYZRGB>::Ptr& source_points,
													const pcl::PointCloud<pcl::PointXYZRGB>::Ptr& target_points)
{
    pcl::console::print_highlight ("starting initial alignment using TransformationEstimationSVD...\n");

	Eigen::Matrix4f transformation_matrix_SVD;
	pcl::registration::TransformationEstimationSVD <pcl::PointXYZRGB, pcl::PointXYZRGB> SVD;
	SVD.estimateRigidTransformation(*source_points, *source_points, transformation_matrix_SVD);
	return transformation_matrix_SVD;
}

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
 *
 * Return: A transformation matrix that will roughly align the points in source to the points in target
 */
Eigen::Matrix4f
Registrator::computeInitialAlignmentRANSAC(const pcl::PointCloud<pcl::PointXYZRGB>::Ptr& source_points,
                                     const pcl::PointCloud<pcl::PointXYZRGB>::Ptr& target_points,
                                     const pcl::CorrespondencesPtr& correspondences,double threshold
                                    )
{


    pcl::console::print_highlight ("starting initial alignment using RANSAC...\n");
    pcl::Correspondences inliers;


    pcl::registration::CorrespondenceRejectorSampleConsensus<pcl::PointXYZRGB> rejector;
    rejector.setInputSource(source_points);
    rejector.setInputTarget(target_points);
    rejector.setInputCorrespondences (correspondences);

    rejector.setMaximumIterations (100);
    rejector.setInlierThreshold (threshold);
    rejector.getCorrespondences (inliers);
    cout << "RANSAC_rejection--correspondences: " << correspondences->size ()<< " --> " << inliers.size () << endl;
    return (rejector.getBestTransformation());
}


/** Use IterativeClosestPoint to find a precise alignment from the source cloud to the target cloud,
 * starting with an initial guess
 * Inputs:
 *   source_points
 *     The "source" points, i.e., the points that must be transformed to align with the target point cloud
 *   target_points
 *     The "target" points, i.e., the points to which the source point cloud will be aligned
 *
 *     A threshold on the distance between any two corresponding points.  Any corresponding points that are further
 *     apart than this threshold will be ignored when computing the source-to-target transformation
 *   outlier_rejection_thresholh
 *     The maximum number of ICP iterations to perform
 *  This function will take longer time because it needs to compute normal surface of point clouds, so
 * it need to be downsampled point cloud before computing the surface normal
 * Return: A transformation matrix that will precisely align the points in source to the points in target
 */
//Iterative closet point with normal surface
Eigen::Matrix4f
Registrator::nicpAlignment(const pcl::PointCloud<pcl::PointXYZRGB>::Ptr& source_points,
                         const pcl::PointCloud<pcl::PointXYZRGB>::Ptr& target_points,
                         pcl::PointCloud<pcl::PointXYZRGB>::Ptr& output,double max_correspondence_distance)
{

	pcl::PointCloud<pcl::PointXYZRGB>::Ptr src (new pcl::PointCloud<pcl::PointXYZRGB>);
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr tgt (new pcl::PointCloud<pcl::PointXYZRGB>);
	pcl::VoxelGrid<pcl::PointXYZRGB> sor;
	sor.setInputCloud(source_points);
	float leafSize = 0.002f;
	sor.setLeafSize(leafSize, leafSize, leafSize);
	sor.filter(*src);
	sor.setInputCloud(target_points);
	sor.setLeafSize(leafSize, leafSize, leafSize);
	sor.filter(*tgt);
	pcl::PointCloud<pcl::PointNormal>::Ptr points_with_normals_src(new pcl::PointCloud<pcl::PointNormal>);
	pcl::PointCloud<pcl::PointNormal>::Ptr points_with_normals_tgt(new pcl::PointCloud<pcl::PointNormal>);
	pcl::NormalEstimation<pcl::PointXYZRGB, pcl::PointNormal> norm_est;
	pcl::search::KdTree<pcl::PointXYZRGB>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZRGB>());
	norm_est.setSearchMethod (tree);
	norm_est.setKSearch (30);
	norm_est.setInputCloud (src);
	norm_est.compute (*points_with_normals_src);
	pcl::copyPointCloud (*src, *points_with_normals_src);
	norm_est.setInputCloud (tgt);
	norm_est.compute (*points_with_normals_tgt);
	pcl::copyPointCloud (*tgt, *points_with_normals_tgt);

	MyPointRepresentation point_representation;
	float alpha[4] = {1.0, 1.0, 1.0, 1.0};
	point_representation.setRescaleValues (alpha);
	pcl::IterativeClosestPointNonLinear<pcl::PointNormal, pcl::PointNormal> icp;
	icp.setTransformationEpsilon (1e-6);
	  // Set the maximum distance between two correspondence
    icp.setMaxCorrespondenceDistance (max_correspondence_distance);
     // Set the point representation
    icp.setPointRepresentation ( boost::make_shared<const MyPointRepresentation> (point_representation));
    icp.setInputSource (points_with_normals_src);
    icp.setInputTarget (points_with_normals_tgt);
    // Run the same optimization in a loop
    Eigen::Matrix4f Ti = Eigen::Matrix4f::Identity (), prev;
    pcl::PointCloud<pcl::PointNormal>::Ptr reg_result = points_with_normals_src;
    icp.setMaximumIterations (2);
    for (int i = 0; i < 30; ++i)
        {
      	    points_with_normals_src = reg_result;
	        // Estimate
	  	    icp.setInputSource (points_with_normals_src);
	        icp.align (*reg_result);
	   	 //accumulate transformation between each Iteration
	      	Ti = icp.getFinalTransformation () * Ti;
	    	 //if the difference between this transformation and the previous one
	        //is smaller than the threshold, refine the process by reducing
	       	//the maximal correspondence distance
	      	if (fabs ((icp.getLastIncrementalTransformation () - prev).sum ()) < icp.getTransformationEpsilon ())
	       		icp.setMaxCorrespondenceDistance (icp.getMaxCorrespondenceDistance () - 0.001);
	        	prev = icp.getLastIncrementalTransformation ();
	          }
	        pcl::console::print_highlight("ICP Score: %f \n", icp.getFitnessScore ());
	         //targetToSource = Ti.inverse();
		    pcl::transformPointCloud (*source_points, *output,Ti);

	return Ti;
}

/**
 * Iterative closet point : does not need to find normal surface of point cloud
 * source points: that must be transfor to align with target points in advance
 * This function iterates 100 times because it does not know the correct correspondences
 * it is impossible to determine optimal transformation in 1 step
 * output: transformation of 2 point clouds
*/
Eigen::Matrix4f
Registrator::icpAlignment(const pcl::PointCloud<pcl::PointXYZRGB>::Ptr& source_points,
                         const pcl::PointCloud<pcl::PointXYZRGB>::Ptr& target_points,
                         pcl::PointCloud<pcl::PointXYZRGB>::Ptr& output,double max_correspondence_distance)
{
	      MyPointRepresentation1 point_representation;
	      float alpha[6] = { 1.0, 1.0, 1.0, 0.1 / 255, 0.1 / 255, 0.1 / 255 };
	    point_representation.setRescaleValues (alpha);
	    pcl::IterativeClosestPoint<pcl::PointXYZRGB, pcl::PointXYZRGB> icp;
	       icp.setTransformationEpsilon (1e-8);
	       icp.setMaximumIterations(100);
	       icp.setMaxCorrespondenceDistance (max_correspondence_distance);
	       // Set the point representation
	        icp.setPointRepresentation (
	        boost::make_shared<const MyPointRepresentation1> (point_representation));

	        icp.setInputSource (source_points);
	        icp.setInputTarget (target_points);
	        icp.align (*output);

            pcl::console::print_highlight("ICP Score: %f \n", icp.getFitnessScore ());
	        pcl::transformPointCloud (*source_points, *output, icp.getFinalTransformation ());
	        return icp.getFinalTransformation ();
}

/*
*  compare two point cloulds (same point cloud in two poses)  after ECLH
*/
Eigen::Affine3f Registrator::compare(pcl::PointCloud<pcl::PointXYZRGB> p1, pcl::PointCloud<pcl::PointXYZRGB> p2)
{
	if (p1.size() != p2.size())
		std::cout << "cannot compare two point clouds" << std::endl;

	else
	{
		pcl::TransformationFromCorrespondences trans;
		double diff = 0;
		for (int i = 0; i < p1.size(); i++)
		{
			Eigen::Vector3f tgt, src;
			tgt(0) = p1[i].x;
			tgt(1) = p1[i].y;
			tgt(2) = p1[i].z;

			src(0) = p2[i].x;
			src(1) = p2[i].y;
			src(2) = p2[i].z;
			trans.add(tgt, src, 1.0);
		}

		return trans.getTransformation();
	}

}

bool Registrator::loopDetection(int end, std::vector<pcl::PointCloud<pcl::PointXYZRGB>::Ptr> &cloud_data, double dist,
	int &first, int &last, double MIN_DISTANCE)
{
	bool euclidean = false	;
	static double min_dist = -1;
	int state = 0;

	for (int i = end - 1; i >= 0; i--)
	{
		double norm;
		//Loop Detection based on eclidean distance of 2 centroids
		if (euclidean)
		{
			Eigen::Vector4f cstart, cend;
			pcl::compute3DCentroid(*(cloud_data[i]), cstart);
			pcl::compute3DCentroid(*(cloud_data[end]), cend);
			Eigen::Vector4f diff = cend - cstart;
			 norm = diff.norm();

			std::cout << "distance between " << i + 1 << " and " << end + 1 << " is " << norm << " state is " << state << std::endl;
		}
		//Loop Detection based on correspondences
		else {

			pcl::registration::CorrespondenceEstimation<pcl::PointXYZRGB, pcl::PointXYZRGB> ce;
			ce.setInputTarget(cloud_data[i]);  //match
			ce.setInputSource(cloud_data[end]);  //query
			pcl::CorrespondencesPtr corr(new pcl::Correspondences);
			ce.determineCorrespondences(*corr, MIN_DISTANCE);
			 norm = 1.0 - double(corr->size()) / double(cloud_data[end]->size());
			 	 std::cout<<"i "<<i<<std::endl;
			 	 std::cout<<"end "<<end<<std::endl;

			 	 std::cout<<"corres "<<double(corr->size())<<std::endl<<"size end"<< double(cloud_data[end]->size())<<" size i"<< double(cloud_data[i]->size())<<std::endl;
				std::cout << "overlapping percentage " << norm << " state is " << state << std::endl;
			}
		if (state == 0 && norm > dist)
		{
			state = 1;
			//std::cout << "state 1" << std::endl;
		}
		if (state > 0 && norm < dist)
		{
			state = 2;
			std::cout << "loop detected between scan " << i << " and scan " << end << std::endl;
			if (min_dist < 0 || norm < min_dist)
			{
				min_dist = norm;
				first = i;
				last = end;
				break;
			}
		}
	}
	//std::cout << "min_dist: " << min_dist << " state: " << state << " first: " << first << " end: " << end << std::endl;
	if (min_dist > 0 && (state < 2 || end == int(cloud_data.size()) - 1)) //TODO
	{
		min_dist = -1;
		return true;
	}
	return false;
}

void Registrator::globalAlignmentELCH( std::vector<pcl::PointCloud<pcl::PointXYZRGB>::Ptr>& cloud_data, pcl::PointCloud<pcl::PointXYZRGB>::Ptr& output,
	std::vector<Eigen::Matrix4f>& matrix_buffer, std::vector<pcl::PointCloud<pcl::PointXYZRGB>::Ptr> cloud_out, int& first, int& last)
{
	bool KEY_FRAME = false;
	int keyframe = 0;
	std::vector<pcl::PointCloud<pcl::PointXYZRGB>::Ptr> cloud_data_afterELCH;

	for (int i = 0; i < cloud_data.size(); i++)
	{
		pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
		*cloud = *cloud_data[i];
		cloud_data_afterELCH.push_back(cloud);
		cout<<cloud->size()<<endl;
	}
	pcl::registration::ELCH<pcl::PointXYZRGB> elch;
	pcl::IterativeClosestPoint<pcl::PointXYZRGB, pcl::PointXYZRGB>::Ptr icp(new pcl::IterativeClosestPoint<pcl::PointXYZRGB, pcl::PointXYZRGB>);

	icp->setMaximumIterations(100);
	icp->setMaxCorrespondenceDistance(0.0025);
	icp->setRANSACOutlierRejectionThreshold(0.2);
	elch.setReg(icp);
	for (int i = 0; i < cloud_data_afterELCH.size(); i++)
		{
		elch.addPointCloud(cloud_data_afterELCH[i]);
		}

	if (first == 0 && last == 0)
	{
		size_t i = cloud_data_afterELCH.size() - 1;
		if (loopDetection(int(i), cloud_data_afterELCH, 0.8, first, last, 0.0025))
		{
			std::cout << "Loop between #" << first + 1 << " and #" << last + 1 << std::endl;
			elch.setLoopStart(first);
			elch.setLoopEnd(last);
			elch.compute();

		}
	}
	else
	{
		std::cout << "Loop between #" << first + 1 << " and #" << last + 1 << std::endl;
		elch.setLoopStart(first);
		elch.setLoopEnd(last);
		elch.compute();
	}
	for (size_t i = 0; i < cloud_data_afterELCH.size(); i++)
	{
		Eigen::Affine3f adjust = compare(*cloud_data[i], *cloud_data_afterELCH[i]);
		matrix_buffer[i] = adjust * matrix_buffer[i];
		cloud_out.push_back(cloud_data_afterELCH[i]);

		// only fuse the key frames with less overlaps
		if (KEY_FRAME)
		{
			if (i == 0)
				*output = *(cloud_data_afterELCH[0]);
			else if(i>0)
			{
				pcl::registration::CorrespondenceEstimation<pcl::PointXYZRGB, pcl::PointXYZRGB> ce;
				ce.setInputTarget(cloud_data_afterELCH[i]);  //match
				ce.setInputSource(cloud_data_afterELCH[keyframe]);  //query
				pcl::CorrespondencesPtr corr(new pcl::Correspondences);
				ce.determineCorrespondences(*corr, 0.003f);
				double nonoverlap = 1 - double(corr->size()) / double(cloud_data_afterELCH[keyframe]->size());
				if (nonoverlap > 0.3)
				{
					std::cout << "Keyframe is " << i << std::endl;
					keyframe = i;
					*output = *output + *(cloud_data_afterELCH[i]);
				}
			}
		}
		else
		{
			*output = *output + *(cloud_data_afterELCH[i]);
		}
	   }
	cloud_data = cloud_out;
}

/**
 * @brief Registrator::elchCorrection
 * @param cloud_data : a set of point clouds that are roughly aligned after ICP, and have loop
 * @param matrix_buffer : store matrix of point cloud relative to first frame
 * @param output : a fused point cloud after ELCH
 */
void Registrator::elchCorrection(std::vector<pcl::PointCloud<pcl::PointXYZRGB>::Ptr>& cloud_data,std::vector<Eigen::Matrix4f>& matrix_buffer, pcl::PointCloud<pcl::PointXYZRGB>::Ptr& output)
		{
    pcl::console::print_highlight ("starting ELCH...\n");
    bool KEY_FRAME = false;
	std::vector<pcl::PointCloud<pcl::PointXYZRGB>::Ptr> cloud_data_afterELCH;
	for (int i = 0; i < cloud_data.size(); i++)
	{
		pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
		*cloud = *cloud_data[i];
		cloud_data_afterELCH.push_back(cloud);

	}
			pcl::registration::ELCH<pcl::PointXYZRGB> elch;
			for (int i = 0; i < cloud_data_afterELCH.size(); i++) {
			        elch.addPointCloud(cloud_data_afterELCH[i]);
			    }
			pcl::IterativeClosestPoint<pcl::PointXYZRGB, pcl::PointXYZRGB>::Ptr icp(new pcl::IterativeClosestPoint<pcl::PointXYZRGB, pcl::PointXYZRGB>);
			icp->setMaximumIterations(100);
            icp->setMaxCorrespondenceDistance(0.008);
			icp->setRANSACOutlierRejectionThreshold(0.2);
			elch.setReg(icp);
		    elch.setLoopStart(0);
		    elch.setLoopEnd(cloud_data_afterELCH.size() - 1);
		    elch.compute();
		     pcl::registration::ELCH<pcl::PointXYZRGB>::LoopGraphPtr loopGraphPtr;
		         loopGraphPtr = elch.getLoopGraph();
		         for (int i = 0; i < cloud_data.size(); i++) {
		        	 matrix_buffer.push_back((*loopGraphPtr)[i].transform.matrix());

		         }
		    output.reset(new pcl::PointCloud<pcl::PointXYZRGB>);
		    	//Only fuse clouds with less overlap
		    if(KEY_FRAME)
		    {
		    int keyframe = 0;
		    for (int i = 0; i < cloud_data_afterELCH.size(); i++) {
		     if(i==0)
		       *output = *(cloud_data_afterELCH[0]);
		     else if (i > 0)
		     	 	 {
		          	  pcl::registration::CorrespondenceEstimation<pcl::PointXYZRGB, pcl::PointXYZRGB> ce;
		              ce.setInputTarget (cloud_data_afterELCH[i]);  //match
		              ce.setInputSource (cloud_data_afterELCH[keyframe]);  //query
		              pcl::CorrespondencesPtr corr (new pcl::Correspondences);
		              ce.determineCorrespondences (*corr, 0.003f);
		              double nonoverlap = 1- double (corr->size ())/ double (cloud_data_afterELCH[keyframe]->size ());
		              if (nonoverlap > 0.2)
		                      {
		            	  	  	cout <<"non-overlap of 2 frames" << nonoverlap<<endl;
		                        cout << "Keyframe is " << i << endl;
		                        keyframe = i;
		                        *output = *output + * (cloud_data_afterELCH[i]);
		                      }
		        	 }
		         }
		    }
		    else
		    	for(int i = 0; i < cloud_data.size(); i++)
		    	{
		    		if(i==0)
		    			*output = *(cloud_data_afterELCH[0]);
		    		else
		    			*output = *output + * (cloud_data_afterELCH[i]);
		    	}

		         for (int i = 0; i < cloud_data.size(); i++)
		         	{
		        	 cloud_data[i] = cloud_data_afterELCH[i];
		         	}
		}
