#ifndef __READ_DATA__
#define __READ_DATA__
#include "../include/types.hpp"
#include "opencv2/opencv.hpp"
#include "opencv2/core/core.hpp"
#include "opencv2/features2d/features2d.hpp"
#include "opencv2/nonfree/nonfree.hpp"
#include <iostream>
#include <fstream>
#include <dirent.h>
#include <string.h>
 #include <pcl/visualization/cloud_viewer.h>
#include <boost/make_shared.hpp>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/io/pcd_io.h>
#include <pcl/features/normal_3d.h>
#include <pcl/filters/filter.h>
#include <pcl/point_representation.h>
#include <pcl/surface/gp3.h>
#include <pcl/common/transformation_from_correspondences.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <sstream>
#include "tinyxml.h"
#include <boost/thread/mutex.hpp>
#define DRAW_MATCHING 1
#define FILTER 0

extern double FOCAL;
extern double MM_PER_M;

//extern double DEPTH_THRESHOLD;
extern double c_x;
extern double c_y;
extern cv::Mat camera_matrix;

 using namespace boost;
 using namespace filesystem;
 using namespace std;

 void readImg (string filename,string suffix, cv::Mat &img)
 {
   string imagefile;
   imagefile = filename;
   imagefile.append (suffix);
   img = cv::imread(imagefile,1);
 }
 void readImage (string filename,string suffix, cv::Mat &img)
 {
   string imagefile;
   imagefile = filename;
   imagefile.append (suffix);
   img = cv::imread(imagefile,-1);
 }
 void readDepth (string filename,string suffix, cv::Mat &img)
 {
   string imagefile;
   imagefile = filename;
   imagefile.append (suffix);
   img = cv::imread(imagefile,CV_LOAD_IMAGE_ANYDEPTH);
 }
void loadImages(images& images, const string& root_dir)
{
  bool debug = false;

ifstream index;
index.open(string(root_dir+"/input.txt").c_str());
while(!index.eof())
{
  image temp;
  string file;
  index>>file;
  if(file.empty())
    break;
  file = root_dir+"/"+file;
  readImg (file,".color.png",temp.img);
  readImage(file,".color.png",temp.gray);
  cv::blur(temp.gray,temp.gray,cv::Size(5,5)); //blur image to remove noise
  readDepth (file,".depth.png",temp.dep);
  //temp.dep.convertTo(temp.dep,CV_32F); //convert to float
  cout << "loading:  " << file << endl;
  images.push_back(temp);
 if(debug)
 {
   cv::imshow(file,temp.img);
  cv::waitKey(0);
 }
}
}


//Convert depth into point cloud
void depthToCloud(const images& images,	depthFrames& depthFrames)
{
	depthFrames.resize(images.size());
	for(int k=0;k<images.size();k++)
	{
		PointCloudRGB::Ptr cloud_ptr( new PointCloudRGB );
		  cloud_ptr->width  = images[k].img.cols;
		  cloud_ptr->height = images[k].img.rows;
		  cloud_ptr->is_dense = false;

		  for ( int y = 0; y < images[k].img.rows; ++ y ) {
		    for ( int x = 0; x < images[k].img.cols; ++ x ) {
		      pcl::PointXYZRGB pt;
		      if ( images[k].dep.at<unsigned short>(y, x) != 0 /*&& mask.at<bool>(y,x) !=0*/)
		      {
		          pt.z = images[k].dep.at<unsigned short>(y, x)/10000.0;
		          pt.x = ((x-camera_matrix.at<double>(0,2))*pt.z/camera_matrix.at<double>(0,0));
		          pt.y = ((y-camera_matrix.at<double>(1,2))*pt.z/camera_matrix.at<double>(1,1));
		          pt.r = images[k].img.at<cv::Vec3b>(y, x)[2];
		          pt.g = images[k].img.at<cv::Vec3b>(y, x)[1];
		          pt.b = images[k].img.at<cv::Vec3b>(y, x)[0];

		          cloud_ptr->points.push_back( pt );
		      }
		      else
		      {
		    	  pt.z = std::numeric_limits<float>::quiet_NaN();
		    	 		          pt.x =std::numeric_limits<float>::quiet_NaN();
		    	 		          pt.y = std::numeric_limits<float>::quiet_NaN();
		    	 		          pt.r = 0;
		    	 		          pt.g =0;
		    	 		          pt.b = 0;
		          cloud_ptr->points.push_back( pt);
		      }
		    }
		  }
	     depthFrames[k].cloud = cloud_ptr;
	}
}
//Convert depth into point cloud
void depth2cloud(const cv::Mat& img,const cv::Mat& depth,PointCloudRGB::Ptr& cloud)
{

	cloud.reset( new PointCloudRGB );
		  cloud->width  = depth.cols;
		  cloud->height = depth.rows;
		  cloud->is_dense = false;

		  for ( int y = 0; y < img.rows; ++ y ) {
		    for ( int x = 0; x < img.cols; ++ x ) {
		      pcl::PointXYZRGB pt;
		      if ( depth.at<unsigned short>(y, x) != 0 /*&& mask.at<bool>(y,x) !=0*/)
		      {
		          pt.z = depth.at<unsigned short>(y, x)/10000.0;
		          pt.x = ((x-camera_matrix.at<double>(0,2))*pt.z/camera_matrix.at<double>(0,0));
		          pt.y = ((y-camera_matrix.at<double>(1,2))*pt.z/camera_matrix.at<double>(1,1));
		          pt.r = img.at<cv::Vec3b>(y, x)[2];
		          pt.g = img.at<cv::Vec3b>(y, x)[1];
		          pt.b = img.at<cv::Vec3b>(y, x)[0];

		          cloud->points.push_back( pt );
		      }
		      else
		      {
		    	  pt.z = std::numeric_limits<float>::quiet_NaN();
		    	  pt.x =std::numeric_limits<float>::quiet_NaN();
		    	 pt.y = std::numeric_limits<float>::quiet_NaN();

		    	 		    	 		          pt.r = 0;
		    	 		    	 		          pt.g = 0;
		    	 		    	 		          pt.b = 0;
		          cloud->points.push_back( pt);
		      }
		    }
		  }
}
void depth2cloudMask(const cv::Mat& img,const cv::Mat& depth,const cv::Mat& mask, PointCloudRGB::Ptr& cloud)
{

	cloud.reset( new PointCloudRGB );
		  cloud->width  = depth.cols;
		  cloud->height = depth.rows;
		  cloud->is_dense = false;

		  for ( int y = 0; y < img.rows; ++ y ) {
		    for ( int x = 0; x < img.cols; ++ x ) {
		      pcl::PointXYZRGB pt;
		      if ( depth.at<unsigned short>(y, x) != 0 && mask.at<float>(y,x) !=0.)
		      {
		          pt.z = depth.at<unsigned short>(y, x)/10000.0;
		          pt.x = ((x-camera_matrix.at<double>(0,2))*pt.z/camera_matrix.at<double>(0,0));
		          pt.y = ((y-camera_matrix.at<double>(1,2))*pt.z/camera_matrix.at<double>(1,1));
		          pt.r = img.at<cv::Vec3b>(y, x)[2];
		          pt.g = img.at<cv::Vec3b>(y, x)[1];
		          pt.b = img.at<cv::Vec3b>(y, x)[0];

		          cloud->points.push_back( pt );
		      }
		      else
		      {
		    	  pt.z = std::numeric_limits<float>::quiet_NaN();
		    	  pt.x =std::numeric_limits<float>::quiet_NaN();
		    	 pt.y = std::numeric_limits<float>::quiet_NaN();

		    	 		    	 		          pt.r = 0;
		    	 		    	 		          pt.g = 0;
		    	 		    	 		          pt.b = 0;
		          cloud->points.push_back( pt);
		      }
		    }
		  }
}

void depthToCloudwithMask(const images& images,depthFrames& maskClouds)
{
  //convert depth image to point cloud in mask
	maskClouds.resize(images.size());
	for(int k=0;k<images.size();k++)
	{
		PointCloudRGB::Ptr cloud_ptr( new PointCloudRGB );
		  cloud_ptr->width  = images[k].img.cols;
		  cloud_ptr->height = images[k].img.rows;
		  cloud_ptr->is_dense = false;

		  for ( int y = 0; y < images[k].img.rows; ++ y ) {
		    for ( int x = 0; x < images[k].img.cols; ++ x ) {
		      pcl::PointXYZRGB pt;
		      if ( images[k].dep.at<unsigned short>(y, x) != 0 && images[k].mask.at<float>(y,x) !=0)
		      {
		          pt.z = images[k].dep.at<unsigned short>(y, x)/10000.0;
		          pt.x = ((x-camera_matrix.at<double>(0,2))*pt.z/camera_matrix.at<double>(0,0));
		          pt.y = ((y-camera_matrix.at<double>(1,2))*pt.z/camera_matrix.at<double>(1,1));
		          pt.r = images[k].img.at<cv::Vec3b>(y, x)[2];
		          pt.g = images[k].img.at<cv::Vec3b>(y, x)[1];
		          pt.b = images[k].img.at<cv::Vec3b>(y, x)[0];

		          cloud_ptr->points.push_back( pt );
		      }
		      else
		      {
		    	  pt.z = std::numeric_limits<float>::quiet_NaN();
		    			          pt.x =std::numeric_limits<float>::quiet_NaN();
		    			          pt.y = std::numeric_limits<float>::quiet_NaN();
		    			          pt.r = 0;
		    			          pt.g = 0;
		    			          pt.b = 0;
		          cloud_ptr->points.push_back( pt);
		      }
		    }
		  }
		  maskClouds[k].cloud = cloud_ptr;

	}
}
void saveCloud(const string& root_dir,const depthFrames& depthFrames)
{

  for(int i=0;i<depthFrames.size();i++)
  {
    PointCloudRGB::Ptr cloud(new PointCloudRGB);
    cloud = depthFrames[i].cloud;
    std::ostringstream frame_name;
    frame_name << std::setw(6) << std::setfill('0') << i;
    string  cloud_file = root_dir + "/frame-" + frame_name.str() + ".original_cloud.pcd";
    pcl::io::savePCDFile(cloud_file,*cloud,true);
  }
}
void keyPointClouds(const images& images,const tagKeyPoints& tagKeyPoints,
		cloudKeypoints& cloudKeypoints2D)
{
	  cloudKeypoints2D.resize(images.size());
	 for(int k=0;k<tagKeyPoints.size();k++)
	 {
		 PointCloudRGB::Ptr cloud (new PointCloudRGB);
		 for(int idx =0;idx<tagKeyPoints[k].vec_keyPoints.size();idx++)
		 {
			 int i = (int)tagKeyPoints[k].list_points_2d[idx].x;
			 int j = (int)tagKeyPoints[k].list_points_2d[idx].y;
			 unsigned short z = images[k].dep.at<unsigned short>(j,i);
			 //cout<<k<<" :" <<z<<endl;
			 if(z!=0)
			 {
				 PointRGB point;
				 unsigned short    z_metric = z/MM_PER_M;
				 	 	 	 	 	 point.z = z_metric;
				 	 	 	 	 	 point.x = point.z *(i-c_x) / FOCAL;
				 	 	 	 	 	 point.y = point.z * (j-c_y ) / FOCAL;
				 const cv::Vec3b& c = images[k].img.at<cv::Vec3b>(j,i);

				 	point.r = c[2];
				 	point.g = c[1];
				 	point.b = c[0];
				 	cloud->points.push_back(point);

			 }

		 }
		cloudKeypoints2D[k].cloud = cloud;
		cloudKeypoints2D[k].cloud->width = images[k].img.cols;
		cloudKeypoints2D[k].cloud->height = images[k].img.rows;
	 }
}



void filter(const depthFrames& originalFrames,float downsample, depthFrames& newFrames)
{
  newFrames.resize(originalFrames.size());
  for(int k = 0;k<originalFrames.size();k++)
  {
    PointCloudRGB::Ptr temp (new PointCloudRGB);
    pcl::VoxelGrid<PointRGB> grid;
    grid.setLeafSize(downsample,downsample,downsample);
    grid.setFilterFieldName ("z");
    grid.setFilterLimits (-1, 1);
    grid.setInputCloud(originalFrames[k].cloud);
    grid.filter (*temp);
    //Outliers removed
    pcl::StatisticalOutlierRemoval<PointRGB> removed;
    removed.setInputCloud(temp);
    removed.setMeanK(20);
    removed.setStddevMulThresh(1.3);
    removed.filter (*newFrames[k].cloud);
  }
}

void symmetryTest(const vector<vector<cv::DMatch> >& matches1, const vector<vector<cv::DMatch> >& matches2,
   vector<cv::DMatch>& symMatches)
 {
   //For all matches from img1 -> img2
               for (vector<vector<cv::DMatch> >::const_iterator i1 = matches1.begin(); i1 != matches1.end(); i1++)
   {
     //ignore deleted matches and does not have 2 neighbors
     if (i1->empty() || i1->size() < 2) continue;
     //For all matches from img2->img1
                       for (vector<vector<cv::DMatch> >::const_iterator i2 = matches2.begin(); i2 != matches2.end(); i2++)
     {
       //ignore deleted matches and does not have 2 neighbors
       if (i2->empty() || i2->size() < 2) continue;
       //Match symmetry test
       if ((*i1)[0].queryIdx == (*i2)[0].trainIdx && (*i2)[0].queryIdx == (*i1)[0].trainIdx)
       {
         symMatches.push_back(cv::DMatch((*i1)[0].queryIdx, (*i1)[0].trainIdx, (*i1)[0].distance));
         break;
       }
     }
   }
 }
int ratioTest(vector<vector<cv::DMatch> >& matches)
  {
    int removed = 0;
                for (vector<vector<cv::DMatch> >::iterator i = matches.begin(); i != matches.end(); i++)
    {
      if (i->size() > 1)
      {
        //Check KNN
        if ((*i)[0].distance > 0.8*(*i)[1].distance) //ratio = 0.8
        {
          i->clear();
          removed++;
        }
      }
      else //does not have 2 neighbors
      {
        i->clear();
        removed++;
      }
    }
    return removed;
  }
void oldMatch(const cv::Mat& img1,const cv::Mat& img2,vector<cv::KeyPoint>& keypoints1,vector<cv::KeyPoint>& keypoints2,
              vector<cv::DMatch>& good_matches, pcl::CorrespondencesPtr& correspondences)
  {
  correspondences.reset(new pcl::Correspondences);

  //cv::DescriptorExtractor* extractor(new cv::xfeatures2d::SIFT::create());
 // cv::Ptr<cv::Feature2D> extractor = cv::xfeatures2d::SIFT::create();
  cv::DescriptorExtractor* extractor(new cv::SiftDescriptorExtractor());
  cv::Mat descriptors1,descriptors2;
  vector<cv::DMatch> matches;
      extractor->compute(img1,keypoints1,descriptors1);
      extractor->compute(img2,keypoints2,descriptors2);

    //2. Match 2 image descriptors
      cv::BFMatcher bfmatcher(cv::NORM_L2,false);//true : cross-check
                vector<vector<cv::DMatch> > matches1, matches2;
    //K-neighbors from img1-> img2
     bfmatcher.knnMatch(descriptors1, descriptors2, matches1, 2); //return 2 neighbors
    //K-neighbors from img2->img1
     bfmatcher.knnMatch(descriptors2, descriptors1, matches2, 2);  //return 2 neighbors
    //3.Remove matches for which KNN> given threshold k=0.8
    //Clean img1->img2
    ratioTest(matches1);
    //clean img2->img1
    ratioTest(matches2);
    //4. Symmetry Test
    symmetryTest(matches1, matches2, matches);
    //Perform RANSAC
        vector<cv::Point2f> matched_points1,matched_points2;
        for(vector<cv::DMatch>::const_iterator idx=matches.begin();idx!= matches.end();idx++)
        {
          matched_points1.push_back(keypoints1[idx->queryIdx].pt);
          matched_points2.push_back(keypoints2[idx->trainIdx].pt);
        }
        vector<uchar> inliers(matched_points1.size(),0);
        cv::Mat fundamental = cv::findFundamentalMat(cv::Mat(matched_points1),cv::Mat(matched_points2),inliers,CV_FM_RANSAC,3.0,0.99);
        vector<uchar>::const_iterator itIn = inliers.begin();
        vector<cv::DMatch>::const_iterator itM =matches.begin();
        for(;itIn != inliers.end();++itIn,++itM)
        {
          if(*itIn)
          {
            good_matches.push_back(*itM);
          }
        }
        correspondences->resize(good_matches.size());
        for(int i=0; i <good_matches.size();i++)
        {
          (*correspondences)[i].index_query = good_matches[i].queryIdx;
          (*correspondences)[i].index_match = good_matches[i].trainIdx;
          if(0) //debug
          {
           cout<<good_matches[i].queryIdx<<"  "<<good_matches[i].trainIdx<<"  "<<good_matches[i].distance<<endl;
          }
        }
        if(DRAW_MATCHING)
            {
              cv::Mat img_matches;
              cv::drawMatches(img1,keypoints1,img2,keypoints2,good_matches,img_matches,cv::Scalar::all(-1),cv::Scalar::all(-1),
                                vector<char>(), cv::DrawMatchesFlags::NOT_DRAW_SINGLE_POINTS);
              cv::namedWindow("Good Matches",cv::WINDOW_NORMAL);
              cv::imshow("Good Matches",img_matches);
              cv::waitKey(0);
            }
  }
#endif //__READ_DATA__








