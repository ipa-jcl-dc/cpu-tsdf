 #ifndef __TYPES_
 #define __TYPES_
#include "opencv2/opencv.hpp"
#include<iostream>
#include <fstream>
#include <boost/make_shared.hpp>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/io/pcd_io.h>
#include <pcl/features/normal_3d.h>
#include <pcl/filters/filter.h>
#include <pcl/point_representation.h>
  using namespace std;






typedef pcl::PointXYZRGB PointRGB;
  typedef pcl::PointCloud<PointRGB> PointCloudRGB;
  typedef pcl::PointXYZ PointXYZ;
  typedef pcl::PointCloud<PointXYZ> PointCloudXYZ;
  typedef pcl::PointNormal PointNormal;
  typedef pcl::PointCloud<PointNormal> PointCLoudNormal;
  typedef pcl::Normal NormalT;
  typedef pcl::PointCloud<NormalT> NormalCloudT;
  typedef pcl::FPFHSignature33 DescriptorType;

struct image {
      cv::Mat img;
      cv::Mat gray;
      cv::Mat dep;
      cv::Mat mask;
      int index;
  };
typedef vector<image> images;

  struct depthFrame
  {
    PointCloudRGB::Ptr cloud;
    depthFrame() : cloud (new PointCloudRGB) {};
  };
typedef vector<depthFrame> depthFrames;

struct tagKeyPoint{
    int id;
    vector<int>  indices;
    vector<cv::KeyPoint>  vec_keyPoints;
    vector<cv::Point2f> list_points_2d;
    vector<cv::Point3f> list_points_3d;
};
typedef vector<tagKeyPoint> tagKeyPoints;
struct cloudNormal
  {
    NormalCloudT::Ptr cloud_normals;
    cloudNormal():cloud_normals (new NormalCloudT) {};
  };
typedef vector<cloudNormal> cloudNormals;
struct cameraPose
{
    int id;
    Eigen::Matrix4f Transformation;
    Eigen::Matrix3f R;
    Eigen::Vector3f t;
};
typedef vector<cameraPose> cameraPoses;
struct cloudKeypoint{
    PointCloudRGB::Ptr cloud;
    cloudKeypoint() : cloud (new PointCloudRGB) {};
};
typedef vector<cloudKeypoint> cloudKeypoints;
struct cloudDescriptor{
    pcl::PointCloud<DescriptorType>::Ptr cloud;
    cloudDescriptor() : cloud (new pcl::PointCloud<DescriptorType>) {};
};
typedef vector<cloudDescriptor> cloudDescriptors;

typedef vector<PointCloudRGB::Ptr,Eigen::aligned_allocator<PointCloudRGB::Ptr> > outData;
//Pair of pointcloud which have matchings
struct correspondencesPointer
 {

   pcl::CorrespondencesPtr correspondences;
   correspondencesPointer() : correspondences (new pcl::Correspondences) {};
 };
typedef vector<correspondencesPointer> correspondencesData;


class myPoint: public pcl::PointRepresentation<PointRGB>
{
      using pcl::PointRepresentation<PointRGB>::nr_dimensions_;
  public:
      myPoint()
  {
        nr_dimensions_=6;
  }
      virtual void copyToFloatArray(const PointRGB& p, float* out) const
      {
        out[0] = p.x;
        out[1]= p.y;
        out[2]= p.z;
        // RGB -> YUV
        out[3] = float (p.r) * 0.299 + float (p.g) * 0.587 + float (p.b) * 0.114;
        out[4] = float (p.r) * 0.595716 - float (p.g) * 0.274453 - float (p.b) * 0.321263;
        out[5] = float (p.r) * 0.211456 - float (p.g) * 0.522591 + float (p.b) * 0.311135;
      }
  };
class myPointNormal: public pcl::PointRepresentation<PointNormal>
{
      using pcl::PointRepresentation<PointNormal>::nr_dimensions_;
  public:
      myPointNormal()
  {
        nr_dimensions_=4;
  }
      virtual void copyToFloatArray(const PointNormal& p, float* out) const
      {
        out[0] = p.x;
        out[1]= p.y;
        out[2]= p.z;
        out[3]= p.curvature;
      }
  };

#endif //__TYPES_
