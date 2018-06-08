#include "matrix_utils.hpp"
//read a file full of transformations into memory
void readTransformsTxt(std::string filename, std::vector<Eigen::Matrix4f>& transforms, std::vector<int>& success_mask){
	std::string line;
	std::ifstream file (filename.c_str());
    if (file.is_open())
    {
        while ( file.good())
        {

        	std::getline (file,line);
             if(line.compare("TRANSFORM") == 0) {
            	 std::getline (file,line);
                 success_mask.push_back(atoi(line.c_str()));
                 Eigen::Matrix4f transform(4,4);
                 for(int i=0;i<4;i++)
                  {
                	 std::getline(file,line);
                    std::istringstream iss(line);
                    iss >> transform(i,0) >> transform(i,1) >> transform(i,2) >> transform(i,3) ;
                   }

                 transforms.push_back(transform);
             }
        }
        file.close();
    }

}

void readTransforms(std::string file_name,std::vector<Eigen::Matrix4f>& transforms,std::vector<int>& pose_flags,int data_n)
{
    cv::FileStorage fs(file_name,cv::FileStorage::READ);
    for(int i=0;i<data_n;i++)
    {

        std::ostringstream ss;
        ss<<i;
        cv::Mat matrix;
        //matrix = cv::Mat(cv::Size(4,4),CV_32FC1);
        Eigen::Matrix4f matrix_eigen;

        int pose_flag;
        std::string transform_flag = "TRANSFORM " + ss.str();
        std::string flag = "Flag " +ss.str();
        //Read transform
        fs[transform_flag]>>matrix;
        cv2Eigen(matrix,matrix_eigen);

        transforms.push_back(matrix_eigen);
        //Read pose flag
        fs[flag]>> pose_flag;
        pose_flags.push_back(pose_flag);
    }
}
void writeTransforms(std::string file_name, std::vector<Eigen::Matrix4f> transforms,std::vector<int> pose_flags)
{
    cv::FileStorage fs(file_name,cv::FileStorage::WRITE);
    std::vector<cv::Mat> transforms_cv;

    for(int i=0;i<transforms.size();i++)
    {
        cv::Mat transform;
        cv::eigen2cv(transforms[i],transform);
        transforms_cv.push_back(transform);
    }

    for(int i=0;i<transforms.size();i++)
    {
        std::ostringstream ss;
        ss<<i;
        std::string flag = "Flag " +ss.str();
        std::string transform_flag = "TRANSFORM " + ss.str();
        fs<<flag<<pose_flags[i];
        fs <<transform_flag<<transforms_cv[i];
    }
}
void writeTransformsTxt(std::string filename, std::vector<Eigen::Matrix4f>& transforms, std::vector<int>& success_mask)
{
    std::ofstream file;
    file.open(filename.c_str());
    for(int i = 0; i<transforms.size(); i++){
        file << "TRANSFORM" << std::endl;
        file << (success_mask)[i] << std::endl;
        Eigen::Matrix4f transform = (transforms)[i];
        for(int r = 0; r<transform.rows(); r++){
            for(int c = 0; c<transform.cols(); c++)
                file << transform(r,c) << " ";
            file << std::endl;
        }
        file << std::endl;
    }
    file.close();
}
void saveNameTxt(std::string filename,const std::vector<std::string>& vector_img_name)
{
    std::ofstream file;
       file.open(filename.c_str());
       for(int i=0;i<vector_img_name.size();i++)
       {
           file<<vector_img_name[i]<<std::endl;
       }
   file.close();
}

Eigen::Vector3f leastSquare3d(const std::vector<Eigen::Matrix4f>& transforms,const std::vector<int>& mask)
{
    //project all transforms to a plane - the transforms should lie on a circular path, project them to a plane to remove vertical noise
    // Plane fitting algorithm found here: http://stackoverflow.com/questions/1400213/3d-least-squares-plane
    //http://www.ilikebigbits.com/blog/2015/3/2/plane-from-points
    Eigen::Matrix3f A;
    Eigen::Vector3f b;
       A <<    0,0,0,
               0,0,0,
               0,0,0;
       b <<    0,0,0;
       for(int i = 0; i<transforms.size(); i++)
       {
           if(mask[i]!=0){
               Eigen::Vector3f p(transforms[i](0,3),transforms[i](1,3),transforms[i](2,3));
               b += Eigen::Vector3f(p(0)*p(2),p(1)*p(2),p(2));
               Eigen::Matrix3f a_tmp;
               a_tmp <<    p(0)*p(0),p(0)*p(1),p(0),
                           p(0)*p(1),p(1)*p(1),p(1),
                           p(0),p(1),1;
               A += a_tmp;
           }
       }
       Eigen::Vector3f x = A.colPivHouseholderQr().solve(b); //plane => x(0)*x + x(1)*y + x(2) = z
   return x;
}
Eigen::Vector3f projectPointToPlane(Eigen::Vector3f p,Eigen::Vector3f x){
    /*plane => x(0)*x + x(1)*y + x(2) = z => x(0)*x + x(1)*y - z  = -x(2)
    / => normal vector:(x(0),x(1),-1), original point(0,0,x(2))
    / https://math.stackexchange.com/questions/444968/project-a-point-in-3d-on-a-given-plane
    */
    Eigen::Vector3f origin(0,0,x(2));
    Eigen::Vector3f to_p = p-origin;
    Eigen::Vector3f normal(x(0),x(1),-1);
    Eigen::Vector3f p3_tmp = p-(normal*normal.dot(to_p));
    return p3_tmp;
}

Eigen::Matrix4f projectTransformToPlane(Eigen::Matrix4f t,Eigen::Vector3f x){
    Eigen::Vector3f origin(0,0,x(2));
    Eigen::Vector3f p(t(0,3),t(1,3),t(2,3));
    Eigen::Vector3f to_p = p-origin;
    Eigen::Vector3f normal(x(0),x(1),-1);

    float theta = atan(-normal(2)/normal(1));
    float alpha = atan(-(cos(theta)*normal(1) + sin(theta)*normal(2))/normal(0));
    Eigen::Matrix4f transform;

    transform <<    cos(alpha),-sin(alpha)*cos(theta),sin(alpha)*sin(theta),    -origin(0),
                    sin(alpha),cos(alpha)*cos(theta),-cos(alpha)*sin(theta),    -origin(1),
                    0,sin(theta),cos(theta),                                    -origin(2),
                    0,0,0,1;

   /* Eigen::Matrix3f roration;
    roration <<    cos(alpha),-sin(alpha)*cos(theta),sin(alpha)*sin(theta),
                    sin(alpha),cos(alpha)*cos(theta),-cos(alpha)*sin(theta),
                    0,sin(theta),cos(theta);
    */
    Eigen::Vector3f p3_tmp = p-(normal*normal.dot(to_p));
    Eigen::Matrix4f out(t);
    out(0,3) = p3_tmp(0);
    out(1,3) = p3_tmp(1);
    out(2,3) = p3_tmp(2);
    return transform * out;
}

void projectTransformTranslationsToPlane(Eigen::Vector3f x, std::vector<Eigen::Matrix4f>& transforms, std::vector<int> mask){
    //project the translation component of the transforms onto a plane defined by the parameters stored in 'x'
    float angle = M_PI*2./transforms.size();
   // std::cout << "STEP: " << angle << std::endl;
    Eigen::Vector3f previous;
    Eigen::Matrix4f previous_t;
    float previous_i;
    for(int i = 0; i<transforms.size(); i++)
    {
        Eigen::Matrix4f t = (transforms)[i];
        Eigen::Vector3f p(t(0,3),t(1,3),t(2,3));
        p = projectPointToPlane(p,x);

        if(mask[i]!=0){
            previous = p;
            previous_t = t;
            previous_i = i;
        }
        else{
            t = previous_t;
            float o = 0;
            float time = 0;
            Eigen::Vector3f p2;
            Eigen::Matrix4f t2;
            for(int j = i+1; i<transforms.size(); j++){
                if(mask[j]!=0)
                {
                    o = (j-previous_i)*angle;
                    time = ((float)(i-previous_i))/(j-previous_i);
                    t2 = (transforms)[j];
                    p2 << t2(0,3),t2(1,3),t2(2,3);
                    p2 = projectPointToPlane(p2,x);
                    break;
                }
            }
     //   std::cout << "ANGLE: " << o << std::endl;
//      p = previous*(sin((1-time)*o)/sin(o)) + p2*(sin(time*o)/sin(o));

            t = previous_t*(1-time) + t2*time;
            p << t(0,3),t(1,3),t(2,3);
        }
       // std::cout << "Transform: " << p(0) << " " << p(1) << " " << p(2) << " " << std::endl;
        (transforms)[i] <<    t(0,0),t(0,1),t(0,2),p(0),
                            t(1,0),t(1,1),t(1,2),p(1),
                            t(2,0),t(2,1),t(2,2),p(2),
                            t(3,0),t(3,1),t(3,2),t(3,3);
    }
}
/* Given a point in 3D space, compute corresponding pixel coordinates in an image with no distortion coefficients*/
 void FromCamera2Image(cv::Point2f pixel, const cv::Mat& camera_matrix, const cv::Point3f& point)
{
    float x = point.x / point.z; float y = point.y / point.z;
    pixel.x = camera_matrix.at<float>(0, 0)* x + camera_matrix.at<float>(0, 2);
    pixel.y = camera_matrix.at<float>(1, 1)* y + camera_matrix.at<float>(1, 2);
}
 /*Transform points from world coordinate to camera coordinatep or between different coordinates*/
 //  [X',Y',Z',1] = [R t   * [X,Y,Z,1]
//					 0 1]
 //
 void FromWorld2Camera(cv::Point3f in_pt, cv::Point3f &out_pt, cv::Mat &transform)
 {
     out_pt.x = in_pt.x * transform.at<float>(0, 0) + in_pt.y * transform.at<float>(0, 1) + in_pt.z * transform.at<float>(0, 2) + transform.at<float>(0, 3);
     out_pt.y = in_pt.x * transform.at<float>(1, 0) + in_pt.y * transform.at<float>(1, 1) + in_pt.z * transform.at<float>(1, 2) + transform.at<float>(1, 3);
     out_pt.z = in_pt.x * transform.at<float>(2, 0) + in_pt.y * transform.at<float>(2, 1) + in_pt.z * transform.at<float>(2, 2) + transform.at<float>(2, 3);

 }

 /*Calculate cross product of 2 vectors in 3D dimensions*/
 //Detail : http://tutorial.math.lamar.edu/Classes/CalcII/CrossProduct.aspx
 void CrossProduct(cv::Point3f in_vector1, cv::Point3f in_vector2, cv::Point3f &out_vector)
 {
     float y1z2 = in_vector1.y * in_vector2.z;
     float z1y2 = in_vector1.z * in_vector2.y;
     float z1x2 = in_vector1.z * in_vector2.x;
     float x1z2 = in_vector1.x * in_vector2.z;
     float x1y2 = in_vector1.x * in_vector2.y;
     float y1x2 = in_vector1.y * in_vector2.x;

     out_vector.x = y1z2 - z1y2;
     out_vector.y = z1x2 - x1z2;
     out_vector.z = x1y2 - y1x2;
 }
 void CrossProduct(const float in_vector1[3], const float in_vector2[3], float out_vector[3])
 {
	 	 float y1z2 = in_vector1[1] * in_vector2[2];
	     float z1y2 = in_vector1[2] * in_vector2[1];
	     float z1x2 = in_vector1[2] * in_vector2[0];
	     float x1z2 = in_vector1[0] * in_vector2[2];
	     float x1y2 = in_vector1[0] * in_vector2[1];
	     float y1x2 = in_vector1[1] * in_vector2[0];

	     out_vector[0] = y1z2 - z1y2;
	     out_vector[1] = z1x2 - x1z2;
	     out_vector[2] = x1y2 - y1x2;
 }
 /*Calculate dot product of 2 vectors in 3D dimensions*/
 // c = x1*x2 + y1*y2 + z1*z2
 float DotProduct(cv::Point3f in_vector1, cv::Point3f in_vector2)
 {
     float x1x2 = in_vector1.x * in_vector2.x;
     float y1y2 = in_vector1.y * in_vector2.y;
     float z1z2 = in_vector1.z * in_vector2.z;

     return (x1x2 + y1y2 + z1z2);
 }
 /*Calculate angle between 2 vectos u,v*/
 // cos(alpha) = DotProduct(u,v)/(|u||v|)
 // Some special case: https://stackoverflow.com/questions/14066933/direct-way-of-computing-clockwise-angle-between-2-vectors
 float angle(cv::Point3f in_vector1, cv::Point3f in_vector2)
 {
     float dot_product = DotProduct(in_vector1, in_vector2);
     float len_vector1 = in_vector1.x*in_vector1.x + in_vector1.y*in_vector1.y + in_vector1.z*in_vector1.z;
     float len_vector2 = in_vector2.x*in_vector2.x + in_vector2.y*in_vector2.y + in_vector2.z*in_vector2.z;
     return std::acos(dot_product / sqrt(len_vector1 * len_vector2));
 }
/* Given camera matrix and image size, calculate Field of View*/
 void fov(float to_fov[2], const cv::Mat& camera_matrix, const int width, const int height)
{
    to_fov[0] = atan2f(camera_matrix.at<float>(0, 2), camera_matrix.at<float>(0, 0)) +
                atan2f(width - camera_matrix.at<float>(0, 2), camera_matrix.at<float>(0, 0));
    to_fov[1] = atan2f(camera_matrix.at<float>(1, 2), camera_matrix.at<float>(1, 1)) +
        atan2f(height - camera_matrix.at<float>(1, 2), camera_matrix.at<float>(1, 1));
}


 void eigenToPointer(const Eigen::Matrix4f& matrix, float dst[16])
 {
	 dst[0] = matrix(0,0);
	 dst[1] = matrix(0,1);
	 dst[2] = matrix(0,2);
	 dst[3] = matrix(0,3);
	 dst[4] = matrix(1,0);
	 dst[5] = matrix(1,1);
	 dst[6] = matrix(1,2);
	 dst[7] = matrix(1,3);
	 dst[8] = matrix(2,0);
	 dst[9] = matrix(2,1);
	 dst[10] = matrix(2,2);
	 dst[11] = matrix(2,3);
	 dst[12] = matrix(3,0);
	 dst[13] = matrix(3,1);
	 dst[14] = matrix(3,2);
	 dst[15] = matrix(3,3);
 }

 void eigenToPointer(const Eigen::Matrix3f& matrix, float dst[9])
 {
	 dst[0] = matrix(0,0);
	 dst[1] = matrix(0,1);
	 dst[2] = matrix(0,2);
	 dst[3] = matrix(1,0);
	 dst[4] = matrix(1,1);
	 dst[5] = matrix(1,2);
	 dst[6] = matrix(2,0);
	 dst[7] = matrix(2,1);
	 dst[8] = matrix(2,2);
 }
 void pointerToEigen(const float m[16],Eigen::Matrix4f& matrix)
{
     matrix(0,0) =m[0] ;
     matrix(0,1) =m[1];
     matrix(0,2) =m[2];
     matrix(0,3) =m[3];
     matrix(1,0) =m[4];
     matrix(1,1) =m[5];
     matrix(1,2) =m[6];
     matrix(1,3) =m[7];
     matrix(2,0) =m[8];
     matrix(2,1) =m[9];
     matrix(2,2) =m[10];
     matrix(2,3) =m[11];
     matrix(3,0) =m[12];
     matrix(3,1) =m[13];
     matrix(3,2) =m[14];
     matrix(3,3) =m[15];
}
 void float2cvMat(const float m[16],cv::Mat& matrix)
 {
 	matrix = cv::Mat(cv::Size(4,4),CV_32FC1);
 	matrix.at<float>(0,0) = m[0];
 	matrix.at<float>(0,1) = m[1];
 	matrix.at<float>(0,2) = m[2];
 	matrix.at<float>(0,3) = m[3];
 	matrix.at<float>(1,0) = m[4];
 	matrix.at<float>(1,1) = m[5];
 	matrix.at<float>(1,2) = m[6];
 	matrix.at<float>(1,3) = m[7];
 	matrix.at<float>(2,0) = m[8];
 	matrix.at<float>(2,1) = m[9];
 	matrix.at<float>(2,2) = m[10];
 	matrix.at<float>(2,3) = m[11];
 	matrix.at<float>(3,0) = m[12];
 	matrix.at<float>(3,1) = m[13];
 	matrix.at<float>(3,2) = m[14];
 	matrix.at<float>(3,3) = m[15];
 }
 void cv2Eigen(const cv::Mat& matrix, Eigen::Matrix4f& eigen_matrix)
 {
    eigen_matrix(0,0) =  matrix.at<float>(0,0) ;
    eigen_matrix(0,1) =  matrix.at<float>(0,1) ;
    eigen_matrix(0,2) =  matrix.at<float>(0,2) ;
    eigen_matrix(0,3) =  matrix.at<float>(0,3) ;
    eigen_matrix(1,0) =  matrix.at<float>(1,0) ;
    eigen_matrix(1,1) =  matrix.at<float>(1,1) ;
    eigen_matrix(1,2) =  matrix.at<float>(1,2) ;
    eigen_matrix(1,3) =  matrix.at<float>(1,3) ;
    eigen_matrix(2,0) =  matrix.at<float>(2,0) ;
    eigen_matrix(2,1) =  matrix.at<float>(2,1) ;
    eigen_matrix(2,2) =  matrix.at<float>(2,2) ;
    eigen_matrix(2,3) =  matrix.at<float>(2,3) ;
    eigen_matrix(3,0) =  matrix.at<float>(3,0) ;
    eigen_matrix(3,1) =  matrix.at<float>(3,1) ;
    eigen_matrix(3,2) =  matrix.at<float>(3,2) ;
    eigen_matrix(3,3) =  matrix.at<float>(3,3) ;
 }

 float DotProduct(const float src1[3],const float src2[3])
 {
	return src1[0]*src2[0]+src1[1]*src2[1]+src1[2]*src2[2];
 }
 float length(const float vec[3])
 {
	 return std::sqrt(DotProduct(vec,vec));
 }
 float* nomalized(const float vec[3])
 {
	 float dst[3] = {0};
	 dst[0] = vec[0]/(length(vec));
	 dst[1] = vec[1]/(length(vec));
	 dst[2] = vec[2]/(length(vec));
	 return dst;
 }

 bool invert_matrix(const float m[16], float invOut[16]) {
   float inv[16], det;
   int i;
   inv[0] = m[5]  * m[10] * m[15] -
            m[5]  * m[11] * m[14] -
            m[9]  * m[6]  * m[15] +
            m[9]  * m[7]  * m[14] +
            m[13] * m[6]  * m[11] -
            m[13] * m[7]  * m[10];

   inv[4] = -m[4]  * m[10] * m[15] +
            m[4]  * m[11] * m[14] +
            m[8]  * m[6]  * m[15] -
            m[8]  * m[7]  * m[14] -
            m[12] * m[6]  * m[11] +
            m[12] * m[7]  * m[10];

   inv[8] = m[4]  * m[9] * m[15] -
            m[4]  * m[11] * m[13] -
            m[8]  * m[5] * m[15] +
            m[8]  * m[7] * m[13] +
            m[12] * m[5] * m[11] -
            m[12] * m[7] * m[9];

   inv[12] = -m[4]  * m[9] * m[14] +
             m[4]  * m[10] * m[13] +
             m[8]  * m[5] * m[14] -
             m[8]  * m[6] * m[13] -
             m[12] * m[5] * m[10] +
             m[12] * m[6] * m[9];

   inv[1] = -m[1]  * m[10] * m[15] +
            m[1]  * m[11] * m[14] +
            m[9]  * m[2] * m[15] -
            m[9]  * m[3] * m[14] -
            m[13] * m[2] * m[11] +
            m[13] * m[3] * m[10];

   inv[5] = m[0]  * m[10] * m[15] -
            m[0]  * m[11] * m[14] -
            m[8]  * m[2] * m[15] +
            m[8]  * m[3] * m[14] +
            m[12] * m[2] * m[11] -
            m[12] * m[3] * m[10];

   inv[9] = -m[0]  * m[9] * m[15] +
            m[0]  * m[11] * m[13] +
            m[8]  * m[1] * m[15] -
            m[8]  * m[3] * m[13] -
            m[12] * m[1] * m[11] +
            m[12] * m[3] * m[9];

   inv[13] = m[0]  * m[9] * m[14] -
             m[0]  * m[10] * m[13] -
             m[8]  * m[1] * m[14] +
             m[8]  * m[2] * m[13] +
             m[12] * m[1] * m[10] -
             m[12] * m[2] * m[9];

   inv[2] = m[1]  * m[6] * m[15] -
            m[1]  * m[7] * m[14] -
            m[5]  * m[2] * m[15] +
            m[5]  * m[3] * m[14] +
            m[13] * m[2] * m[7] -
            m[13] * m[3] * m[6];

   inv[6] = -m[0]  * m[6] * m[15] +
            m[0]  * m[7] * m[14] +
            m[4]  * m[2] * m[15] -
            m[4]  * m[3] * m[14] -
            m[12] * m[2] * m[7] +
            m[12] * m[3] * m[6];

   inv[10] = m[0]  * m[5] * m[15] -
             m[0]  * m[7] * m[13] -
             m[4]  * m[1] * m[15] +
             m[4]  * m[3] * m[13] +
             m[12] * m[1] * m[7] -
             m[12] * m[3] * m[5];

   inv[14] = -m[0]  * m[5] * m[14] +
             m[0]  * m[6] * m[13] +
             m[4]  * m[1] * m[14] -
             m[4]  * m[2] * m[13] -
             m[12] * m[1] * m[6] +
             m[12] * m[2] * m[5];

   inv[3] = -m[1] * m[6] * m[11] +
            m[1] * m[7] * m[10] +
            m[5] * m[2] * m[11] -
            m[5] * m[3] * m[10] -
            m[9] * m[2] * m[7] +
            m[9] * m[3] * m[6];

   inv[7] = m[0] * m[6] * m[11] -
            m[0] * m[7] * m[10] -
            m[4] * m[2] * m[11] +
            m[4] * m[3] * m[10] +
            m[8] * m[2] * m[7] -
            m[8] * m[3] * m[6];

   inv[11] = -m[0] * m[5] * m[11] +
             m[0] * m[7] * m[9] +
             m[4] * m[1] * m[11] -
             m[4] * m[3] * m[9] -
             m[8] * m[1] * m[7] +
             m[8] * m[3] * m[5];

   inv[15] = m[0] * m[5] * m[10] -
             m[0] * m[6] * m[9] -
             m[4] * m[1] * m[10] +
             m[4] * m[2] * m[9] +
             m[8] * m[1] * m[6] -
             m[8] * m[2] * m[5];

   det = m[0] * inv[0] + m[1] * inv[4] + m[2] * inv[8] + m[3] * inv[12];

   if (det == 0)
     return false;

   det = 1.0 / det;

   for (i = 0; i < 16; i++)
     invOut[i] = inv[i] * det;

   return true;
 }
 void multiply_matrix(const float m1[16], const float m2[16], float mOut[16]) {
   mOut[0]  = m1[0] * m2[0]  + m1[1] * m2[4]  + m1[2] * m2[8]   + m1[3] * m2[12];
   mOut[1]  = m1[0] * m2[1]  + m1[1] * m2[5]  + m1[2] * m2[9]   + m1[3] * m2[13];
   mOut[2]  = m1[0] * m2[2]  + m1[1] * m2[6]  + m1[2] * m2[10]  + m1[3] * m2[14];
   mOut[3]  = m1[0] * m2[3]  + m1[1] * m2[7]  + m1[2] * m2[11]  + m1[3] * m2[15];

   mOut[4]  = m1[4] * m2[0]  + m1[5] * m2[4]  + m1[6] * m2[8]   + m1[7] * m2[12];
   mOut[5]  = m1[4] * m2[1]  + m1[5] * m2[5]  + m1[6] * m2[9]   + m1[7] * m2[13];
   mOut[6]  = m1[4] * m2[2]  + m1[5] * m2[6]  + m1[6] * m2[10]  + m1[7] * m2[14];
   mOut[7]  = m1[4] * m2[3]  + m1[5] * m2[7]  + m1[6] * m2[11]  + m1[7] * m2[15];

   mOut[8]  = m1[8] * m2[0]  + m1[9] * m2[4]  + m1[10] * m2[8]  + m1[11] * m2[12];
   mOut[9]  = m1[8] * m2[1]  + m1[9] * m2[5]  + m1[10] * m2[9]  + m1[11] * m2[13];
   mOut[10] = m1[8] * m2[2]  + m1[9] * m2[6]  + m1[10] * m2[10] + m1[11] * m2[14];
   mOut[11] = m1[8] * m2[3]  + m1[9] * m2[7]  + m1[10] * m2[11] + m1[11] * m2[15];

   mOut[12] = m1[12] * m2[0] + m1[13] * m2[4] + m1[14] * m2[8]  + m1[15] * m2[12];
   mOut[13] = m1[12] * m2[1] + m1[13] * m2[5] + m1[14] * m2[9]  + m1[15] * m2[13];
   mOut[14] = m1[12] * m2[2] + m1[13] * m2[6] + m1[14] * m2[10] + m1[15] * m2[14];
   mOut[15] = m1[12] * m2[3] + m1[13] * m2[7] + m1[14] * m2[11] + m1[15] * m2[15];
 }
