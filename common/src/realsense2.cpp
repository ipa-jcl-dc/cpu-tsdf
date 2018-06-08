#include "realsense2.hpp"
#include <librealsense2/rs.hpp>
RealSense2::RealSense2() {


    //Set heigh,width,size
    height_ = 480; width_ = 640;size_ = height_*width_;
    //Enable color and depth streams with standard configuration
    config_.enable_stream(RS2_STREAM_DEPTH);
    config_.enable_stream(RS2_STREAM_COLOR,640,480, RS2_FORMAT_RGB8, 30);
    //Start device, get neccessary information
    selection_ = pipe_.start(config_);
    //Depth scale
    depth_scale_ = selection_.get_device().first<rs2::depth_sensor>().get_depth_scale();
    //Intrinsic parameters
    color_K = selection_.get_stream(RS2_STREAM_COLOR).as<rs2::video_stream_profile>().get_intrinsics();
    depth_K = selection_.get_stream(RS2_STREAM_DEPTH).as<rs2::video_stream_profile>().get_intrinsics();
    //Extrinsic depth to color
    depth2color_ext = selection_.get_stream(RS2_STREAM_DEPTH).as<rs2::video_stream_profile>().get_extrinsics_to(
                selection_.get_stream(RS2_STREAM_COLOR).as<rs2::video_stream_profile>());
}



RealSense2::~RealSense2()
{

}
void RealSense2::stopStreaming()
{
    pipe_.stop();
}

//Get streams of camera
void RealSense2::waitForFrames()
{
    //Check device opened

   if(!isConnected())
                {
                    throw std::runtime_error("Cannot get data. No device connected.");
                }
    //Get data

     rs2::align align(RS2_STREAM_COLOR);
     rs2::frameset frameset;
    while (!frameset.first_or_default(RS2_STREAM_DEPTH) || !frameset.first_or_default(RS2_STREAM_COLOR))
     {
                frameset = pipe_.wait_for_frames();
    }
    //Align depth to color stream
     proccessed_ = align.process(frameset);

}

void RealSense2::getPointCloud( pcl::PointCloud<pcl::PointXYZRGB>::Ptr& cloud)
 {

        cloud->width = depth_K.width;
        cloud->height = depth_K.height;
        cloud->is_dense = false;
        const uint16_t * depth_data = reinterpret_cast<const uint16_t *> (proccessed_.get_depth_frame().get_data());
        const uint8_t * color_data = reinterpret_cast<const uint8_t *> (proccessed_.get_color_frame().get_data());
        float color_point[3], scaled_depth;
      for(int y=0;y<depth_K.height;y++)
        {
          for(int x=0;x<depth_K.width;x++)
          {
              pcl::PointXYZRGB pt;
              uint16_t depth_value = depth_data[640*y + x];
              scaled_depth = depth_value *depth_scale_;
              float color_pixel[2] = {static_cast<float>(x), static_cast<float>(y)};
              rs2_deproject_pixel_to_point(color_point, &color_K, color_pixel, scaled_depth);
              if (color_point[2] <= 0.1f || color_point[2] > 5.f)
              {
                  //continue; //remove points too close or too far
                    pt.x =0;
                    pt.y =0;
                    pt.z =0;

              }
              else{
                  pt.x = color_point[0];
                  pt.y = color_point[1];
                  pt.z = color_point[2];
              }


              //if color pixel of 3d point is outside of coordinate, mark as 0,0,0 rgb value
             /*
              if (color_pixel[1] < 0.f || color_pixel[1] > color_K.height
                 || color_pixel[0] < 0.f || color_pixel[0] > color_K.width)
               {

                    pt.r = static_cast<uint8_t>(0);
                    pt.g = static_cast<uint8_t>(0);
                    pt.b = static_cast<uint8_t>(0);
              }

             else
             {*/
                auto i = static_cast<int>(color_pixel[0]);
                auto j = static_cast<int>(color_pixel[1]);
                auto offset = i * 3 + j * color_K.width * 3;
                pt.r = static_cast<uint8_t>(color_data[offset]);
                pt.g = static_cast<uint8_t>(color_data[offset + 1]);
                pt.b = static_cast<uint8_t>(color_data[offset + 2]);
               // }
              cloud->points.push_back(pt);
          }
        }

 }
void RealSense2::getColorImage(cv::Mat& color_image)
{
    color_image= cv::Mat(cv::Size(640,480),CV_8UC3,(void*)proccessed_.get_color_frame().get_data(),cv::Mat::AUTO_STEP);
    cv::cvtColor(color_image,color_image,cv::COLOR_BGR2RGB);
   // color_image = img.clone();
}

void RealSense2::getDepthImage(cv::Mat& depth_image)
{
    depth_image = cv::Mat(cv::Size(640,480),CV_16UC1,(void*)proccessed_.get_depth_frame().get_data(),cv::Mat::AUTO_STEP);
}

float RealSense2::getCenterDistance()
{
    int center_x = getWidth()/2;
    int center_y = getHeight()/2;
    int centerIndex = (getWidth()*center_y) + center_x;
    const uint16_t * depth_data = reinterpret_cast<const uint16_t *> (proccessed_.get_depth_frame().get_data());
    return depth_data[centerIndex]*depth_scale_;

}

//Print some information of camera
void RealSense2::printInformation()
{

     ///// Color Information ////
    auto principal_point = std::make_pair(color_K.ppx, color_K.ppy);
    auto focal_length = std::make_pair(color_K.fx, color_K.fy);
    rs2_distortion model = color_K.model;
    std::cout <<"Color sensor infomation "<<std::endl;
    std::cout << "Principal Point         : " << principal_point.first << ", " << principal_point.second << std::endl;
    std::cout << "Focal Length            : " << focal_length.first << ", " << focal_length.second << std::endl;
    std::cout << "Distortion Model        : " << model << std::endl;
    std::cout << "Distortion Coefficients : [" << color_K.coeffs[0] << "," << color_K.coeffs[1] << ","
    << color_K.coeffs[2] << "," << color_K.coeffs[3] << "," << color_K.coeffs[4] << "]" << std::endl;
    ///////// Depth Information ////////////
    auto principal_point_depth = std::make_pair(depth_K.ppx, depth_K.ppy);
    auto focal_length_depth = std::make_pair(depth_K.fx, depth_K.fy);
    rs2_distortion model_depth = depth_K.model;
    std::cout <<"Depth sensor infomation "<<std::endl;
    std::cout << "Principal Point         : " << principal_point_depth.first << ", " << principal_point_depth.second << std::endl;
    std::cout << "Focal Length            : " << focal_length_depth.first << ", " << focal_length_depth.second << std::endl;
    std::cout << "Distortion Model        : " << model_depth << std::endl;
    std::cout << "Distortion Coefficients : [" << depth_K.coeffs[0] << "," << depth_K.coeffs[1] << ","
    << depth_K.coeffs[2] << "," << depth_K.coeffs[3] << "," << depth_K.coeffs[4] << "]" << std::endl;
    //  camera-to-camera extrinsics from depth sensor to color sensor
    std::cout <<" extrinsics from depth sensor to color sensor "<<std::endl;
    std::cout<<"["<<depth2color_ext.rotation[0] <<" "<<depth2color_ext.rotation[1]<< " "<<depth2color_ext.rotation[2]<<std::endl;
    std::cout<<depth2color_ext.rotation[3] <<" "<<depth2color_ext.rotation[4]<< " "<<depth2color_ext.rotation[5]<<std::endl;
    std::cout<<depth2color_ext.rotation[6] <<" "<<depth2color_ext.rotation[7]<< " "<<depth2color_ext.rotation[8]<<"]"<<std::endl;
    //Depth Scale
    std::cout<<"Depth Scale :"<<std::endl<< depth_scale_<<std::endl;
  }

void RealSense2::readValues(std::vector<filter_options>& filters, const std::vector<rs2_option>& option_names)
{
    //cv::FileStorage fs("../config/realsense_parameters.yaml",cv::FileStorage::READ);
    //dec_struct.string[RS2_OPTION_FILTER_MAGNITUDE]
    for (int i = 0; i < filters.size(); i++) {
            for (int j = 0; j < option_names.size(); j++) {
                if (filters[i].filter->supports(option_names[j])) {
                    rs2::option_range range = filters[i].filter->get_option_range(option_names[j]);
                        filters[i].float_params[option_names[j]] = range.def;
                }
            }
            filters[i].do_filter = true;
    }
}


