#include <fstream>
#include <math.h>
#include <iostream>
#include <vector>
#include <sstream>
#include <fstream>
#include <Eigen/Dense>
#include <opencv2/core/core.hpp>
#include <opencv2/opencv.hpp>
#include <Eigen/Core>
#include <opencv2/core/eigen.hpp>
#include <boost/filesystem.hpp>
#include <boost/shared_ptr.hpp>
#include <boost/make_shared.hpp>


void writeTransforms(std::string file_name, std::vector<Eigen::Matrix4f> transforms,std::vector<int> pose_flags);
void readTransforms(std::string file_name,std::vector<Eigen::Matrix4f>& transforms,std::vector<int>& pose_flags,int data_n);
void readTransformsTxt(std::string filename, std::vector<Eigen::Matrix4f>& transforms, std::vector<int>& success_mask);
void writeTransformsTxt(std::string filename, std::vector<Eigen::Matrix4f>& transforms, std::vector<int>& success_mask);
void saveNameTxt(std::string filename,const std::vector<std::string>& vector_img_name);
// Constructs a plane from a collection of points
// so that the summed squared distance to all points is minimzized
Eigen::Vector3f leastSquare3d(const std::vector<Eigen::Matrix4f>& transforms,const std::vector<int>& mask);
Eigen::Vector3f projectPointToPlane(Eigen::Vector3f p,Eigen::Vector3f x);
Eigen::Matrix4f projectTransformToPlane(Eigen::Matrix4f t,Eigen::Vector3f x);
void projectTransformTranslationsToPlane(Eigen::Vector3f x, std::vector<Eigen::Matrix4f>& transforms, std::vector<int> mask);
/*Transform points from world coordinate to camera coordinatep or between different coordinates*/
//  [X',Y',Z',1] = [R t   * [X,Y,Z,1]
//					 0 1]
//
void FromWorld2Camera(cv::Point3f in_pt, cv::Point3f &out_pt, cv::Mat &transform);
/* Given a point in 3D space, compute corresponding pixel coordinates in an image with no distortion coefficients*/
void FromCamera2Image(cv::Point2f pixel, const cv::Mat& camera_matrix, const cv::Point3f& point);
/*Calculate cross product of 2 vectors in 3D dimensions*/
//Detail : http://tutorial.math.lamar.edu/Classes/CalcII/CrossProduct.aspx
void CrossProduct(cv::Point3f in_vector1, cv::Point3f in_vector2, cv::Point3f &out_vector);
void CrossProduct(const float in_vector1[3], const float in_vector2[3], float out_vector[3]);

/*Calculate dot product of 2 vectors in 3D dimensions*/
// c = x1*x2 + y1*y2 + z1*z2
float DotProduct(cv::Point3f in_vector1, cv::Point3f in_vector2);
float DotProduct(const float src1[3],const float src2[3]);
/*Calculate length of vector*/
float length(const float vec[3]);
/*Calculate normalized vector*/
float* nomalized(const float vec[3]);
/*Calculate angle between 2 vectos u,v*/
float angle(cv::Point3f in_vector1, cv::Point3f in_vector2);
/* Given camera matrix and image size, calculate Field of View*/
void fov(float to_fov[2], const cv::Mat& camera_matrix, const int width, const int height);
/*Convert Eigen Matrix to float pointer */
void eigenToPointer(const Eigen::Matrix4f& matrix, float dst[16]);
void eigenToPointer(const Eigen::Matrix3f& matrix, float dst[9]);
/*Convert float pointer to eigen*/
void pointerToEigen(const float m[16],Eigen::Matrix4f& matrix);
/*Convert float to OpenCV*/
void float2cvMat(const float m[16],cv::Mat& matrix);
void cv2Eigen(const cv::Mat& matrix, Eigen::Matrix4f& eigen_matrix);
bool invert_matrix(const float m[16], float invOut[16]);
void multiply_matrix(const float m1[16], const float m2[16], float mOut[16]);
