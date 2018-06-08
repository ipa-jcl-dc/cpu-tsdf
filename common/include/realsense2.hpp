#ifndef REALSENSE_H
#define REALSENSE_H
#include <librealsense2/rs.hpp>
#include <librealsense2/rsutil.h>
#include <iostream>
#include <unistd.h>
#include <sstream>
#include <stdio.h>
#include <memory>
#include <vector>
#include <opencv2/opencv.hpp>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
////Class realsense to munipulate a single realsense camera
struct filter_options {
    rs2::options* filter;
    std::map<rs2_option, int> int_params;
    std::map<rs2_option, float> float_params;
    std::map<rs2_option, std::vector<char*> > string;
    bool do_filter;
};
class RealSense2
{
public:
    RealSense2();
    ~RealSense2();
    void stopStreaming();
    void printInformation();
    void getData();
    /*Getter*/
    inline bool isConnected() const {return ctx_.query_devices().size()>0 ;}
    // Get color intrinsic
    inline rs2_intrinsics color_intrin() const {return color_K;}
    //Get camera matrix
    inline cv::Mat getCameraMatrix() const {
                                        return (cv::Mat_<double>(3,3)<<   color_K.fx,0.0f,color_K.ppx,
                                                                       0.0f,color_K.fy, color_K.ppy,
                                                                        0.0f,0.0f,1.0f);
                                        }
    //Get depth intrinsic
    inline rs2_intrinsics depth_intrin() const {return depth_K;}
    //Get depth scale
    inline float depthValue() const {return depth_scale_;}
    //Get height
    inline int getHeight() const {return height_;}
    //Get width
    inline int getWidth() const {return width_;}
    //Get size
    inline int getSize() const {return size_;}
    inline float getFocalX() const {return color_K.fx;}
    inline float getFocalY() const {return color_K.fy;}
    inline float getPrintcipalX() const {return color_K.ppx;}
    inline float getPrintcipalY() const {return color_K.ppy;}

    //void waitForFrames from camera
    void waitForFrames();
    //Get point cloud,depth, color image
    void getPointCloud( pcl::PointCloud<pcl::PointXYZRGB>::Ptr& cloud);
    void getColorImage(cv::Mat& color_image);
    void getDepthImage(cv::Mat& depth_image);
    //Get center distance
    float getCenterDistance();
    //Post-processing
    ////
    /// \brief readValues : read values of post-processing filters from yaml file
    /// \param dec_struct: values of decimal filter
    /// \param spat_struct : values of spatial filter
    /// \param temp_struct: value of temporal filter
    ///
    void readValues(std::vector<filter_options>& filters, const std::vector<rs2_option>& option_names);

private:

    rs2::pipeline pipe_;
    rs2::config config_;
    rs2::context ctx_;
    rs2::pipeline_profile selection_;
    rs2_intrinsics color_K;
    rs2_intrinsics depth_K;
    rs2_extrinsics depth2color_ext;
    float depth_scale_;
    rs2::frameset proccessed_;
    int height_,width_,size_;
};

#endif //REALSENSE_H
