#ifndef REALSENSE_h
#define REALSENSE_h
#include <librealsense2/rs.hpp>
#include <librealsense2/rsutil.h>
#include <librealsense2/hpp/rs_internal.hpp>
#include <opencv2/opencv.hpp>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/io/io.h>
#include <pcl/io/pcd_io.h>
#include "config.hpp"
#include "loader.hpp"
#include <map>
#include <sstream>
#include <iomanip>
class RealSense : public Config {
private:
    rs2::device_list devices_list_;

    // Helper struct per pipeline
    struct view_port
    {
       std::map<int, rs2::frame> frames_per_stream;
       rs2::pipeline pipe;
       rs2::pipeline_profile profile;
        };
    std::map<std::string, view_port> devices_;


public:
    ////Get list of connected cameras
    rs2::device_list get_device_list();
    ////
    /// \brief get_serial_number : Get seria number of camera
    /// \param dev : object of realsense camera
    /// \return serial number of connected realsen camera
    ///
    std::string get_serial_number(const rs2::device& dev);
    ////
    /// \brief enable_device : Enable connected camera
    /// \param dev : object of realsense camera
    ///
    void enable_device(rs2::device dev);
    ////
    /// \brief poll_frames : Poll frames with given connected cameras
    ///
    void poll_frames();
    ////
    /// \brief turnOffDepthStream : Turn off depth stream with given serial number of camera
    /// \param serial_number : serial number of connected camera
    ///
    void turnOffDepthStream(const std::string& serial_number);

    ////
    /// \brief turnOnDepthStream : Turn on depth stream with given serial number of camera
    /// \param serial_number : serial number of connected camera
    ///
    void turnOnDepthStream(const std::string& serial_number);
    ////
    ///
    /// \param frame : kind of frame of connected camera, can be depth, rgb or infrared frame
    /// \return cv::Mat
    ///
    cv::Mat frameToMat(const rs2::frame& frame);
    ////
    /// \brief depthFrameToMeters : Converts depth frame to a matrix of doubles with distances in meters
    /// \param pipe : data struct that holds treams of camera
    /// \param f : depth frame of camera
    /// \return  depth cv::Mat
    ///
    cv::Mat depthFrameToMeters(const rs2::pipeline& pipe, const rs2::depth_frame& f);

    ////
    /// \brief convertToImage : Return image with given frame
    /// \param serial_number : serial number of connected camera
    /// \param frame : kind of frame of connected camera, can be depth, rgb or infrared frame
    /// \return
    ///
    cv::Mat convertToImage(const std::string& serial_number,const rs2::frame& frame);
    ////
    /// \brief displayColorImages : Visualize color  stream in opencv cv::Mat
    /// \param serial_number : serial number of connected camera
    /// \return : image with given frame
    ///
    cv::Mat displayColorImages(const std::string& serial_number);
    ////
    /// \brief displayDepthImages : Visualize depth stream in opencv cv::Mat
    /// \param serial_number : serial number of connected camera
    /// \return : depth image with given frame
    ///
    cv::Mat displayDepthImages(const std::string& serial_number);
    ////
    /// \brief saveData : Save point cloud and images into disk
    /// \param serial_number : serial number of connected camera
    /// \param color_image : saved RGB image
    /// \param cloud: save pointcloud
    ///
    void saveData(const std::string& serial_number,cv::Mat& color_image,cv::Mat& depth_image);

    void saveData2(const std::string& serial_number,cv::Mat& color_image,pcl::PointCloud<pcl::PointXYZRGB>::Ptr& cloud);
    void saveData3(const std::string& serial_number,cv::Mat& color_image,cv::Mat& depth,pcl::PointCloud<pcl::PointXYZRGB>::Ptr& cloud);

    ////
    /// \brief getCameraInformation : Get information of all connected cameras and save it to yaml file into disk
    /// \param color_K : vector to store camera matrices of connected cameras
    ///
    void getCameraInformation(const std::string data_path,std::vector<cv::Mat>& color_K);
    ////
    /// \brief getConfiguration : Get configuration of all connected cameras
    ///
    void getConfiguration();
    ////
    /// \brief setConfiguration : Set configuration of all connected camera from yaml file
    /// \param configuration_file : yaml file
    ///
    void setConfiguration(const std::string& configuration_file);

    //void postProcessingFilters();
    ////
    /// \brief stopStreaming : shut down all cameras
    ///
    void stopStreaming();
    ////
    /// \brief getDataPath get data path that store data
    /// \return path string
    ///
    const std::string getDataPath();
    void WriteDepth(float * depth_values, int frame_height, int frame_width,cv::Mat& depth_mat);

};



#endif //REALSENSE_h
