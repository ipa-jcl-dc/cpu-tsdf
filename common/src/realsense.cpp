#include "realsense.hpp"
#include <exception>
rs2::device_list RealSense::get_device_list()
{
    // First, create a rs2::context.
    // The context represents the current platform with respect to connected devices
    rs2::context ctx;
    // Using the context we can get all connected devices in a device list
    devices_list_ = ctx.query_devices();
    rs2::device selected_device;
    if(devices_list_.size() ==0)
       {
         std::cerr << "No device connected, please connect a RealSense device" << std::endl;

         //To help with the boilerplate code of waiting for a device to connect
          //The SDK provides the rs2::device_hub class
           rs2::device_hub device_hub(ctx);

           //Using the device_hub we can block the program until a device connects
           selected_device = device_hub.wait_for_device();
       }
     else
         {
          std::cout << "Found the following devices:\n" << std::endl;
          //The first way is using an iterator (in this case hidden in the Range-based for loop)
          int index = 0;
          for (rs2::device device : devices_list_)
           {
              std::cout << "  " << index++ << " : " << get_serial_number(device) << std::endl;
           }
           }
        return devices_list_;
}
std::string RealSense::get_serial_number(const rs2::device& dev)
{

    if (dev.supports(RS2_CAMERA_INFO_SERIAL_NUMBER))
          return dev.get_info(RS2_CAMERA_INFO_SERIAL_NUMBER);

}
void RealSense::enable_device(rs2::device dev)
{
          std::string serial_number(dev.get_info(RS2_CAMERA_INFO_SERIAL_NUMBER));
          // Create a pipeline from the given device and enable depth and color stream with desired configurations
          rs2::config c;
          c.enable_device(serial_number);
          c.enable_stream(RS2_STREAM_DEPTH,640,480,RS2_FORMAT_Z16,30);
          c.enable_stream(RS2_STREAM_COLOR,640,480, RS2_FORMAT_RGB8,30);
          // Start the pipeline with the configuration
          rs2::pipeline pipe;
          rs2::pipeline_profile profile = pipe.start(c);
           //hold it in struct internally,
          devices_.emplace(serial_number,view_port{{},pipe,profile});
}

void RealSense::poll_frames()
{
    // Go over all device
    for (auto&& view : devices_)
    {

    // Ask each pipeline if there are new frames available, and align depth stream to color stream
        rs2::frameset frameset, frameset_depth_aligned_to_color;
        rs2::align align_to(RS2_STREAM_COLOR);
        if (view.second.pipe.poll_for_frames(&(frameset)))
       {
            frameset_depth_aligned_to_color = align_to.process(frameset);
            for (int i = 0; i < frameset.size(); i++)
            {


                rs2::frame new_frame = frameset_depth_aligned_to_color[i];
                int stream_id = new_frame.get_profile().unique_id();
                //update view port with the new stream

                view.second.frames_per_stream[stream_id] = new_frame; //update view port with the new stream
            }
       }
    }
}
void RealSense::turnOffDepthStream(const std::string& serial_number)
{
    for(auto&& view: devices_)
           {
               if(view.first == serial_number)
               {

                   auto sensors = view.second.profile.get_device().query_sensors();
                   for(rs2::sensor sensor: sensors)
                   {
                     if(sensor.supports(RS2_OPTION_LASER_POWER))
                       {
                            sensor.set_option(RS2_OPTION_LASER_POWER, 0.f); // Disable laser

                       }
                    }
               }
           }
}
void RealSense::turnOnDepthStream(const std::string& serial_number)
{
    for(auto&& view: devices_)
          {
               if(view.first == serial_number)
               {

                   auto sensors = view.second.profile.get_device().query_sensors();
                   for(rs2::sensor sensor: sensors)
                   {

                       if(sensor.supports(RS2_OPTION_LASER_POWER))
                       {
                           auto range = sensor.get_option_range(RS2_OPTION_LASER_POWER);
                           sensor.set_option(RS2_OPTION_LASER_POWER, range.max); // Set max power
                       }
                   }
               }
           }
}
cv::Mat RealSense::frameToMat(const rs2::frame& frame)
  {

     auto vf = frame.as<rs2::video_frame>();
     const int w = vf.get_width();
     const int h = vf.get_height();
     if (frame.get_profile().format() == RS2_FORMAT_BGR8)
     {
         return cv::Mat(cv::Size(w, h), CV_8UC3, (void*)frame.get_data(), cv::Mat::AUTO_STEP);
     }
     else if (frame.get_profile().format() == RS2_FORMAT_RGB8)
     {
         auto r = cv::Mat(cv::Size(w, h), CV_8UC3, (void*)frame.get_data(), cv::Mat::AUTO_STEP);
         cv::cvtColor(r, r, CV_BGR2RGB);
         return r;
     }
     else if (frame.get_profile().format() == RS2_FORMAT_Z16)
     {
         return cv::Mat(cv::Size(w, h), CV_16UC1, (void*)frame.get_data(), cv::Mat::AUTO_STEP);
     }
     else if (frame.get_profile().format() == RS2_FORMAT_Y8)
     {
         return cv::Mat(cv::Size(w, h), CV_8UC1, (void*)frame.get_data(), cv::Mat::AUTO_STEP);;
     }

 throw std::runtime_error("Frame format is not supported yet!");


  }
cv::Mat RealSense::depthFrameToMeters(const rs2::pipeline& pipe, const rs2::depth_frame& f)
{
    cv::Mat depth_image = frameToMat(f);
    depth_image.convertTo(depth_image,CV_64F);
    auto depth_scale = pipe.get_active_profile().get_device().first<rs2::depth_sensor>().get_depth_scale();
    depth_image = depth_image * depth_scale;
    return depth_image;

}

cv::Mat RealSense::convertToImage(const std::string& serial_number,const rs2::frame& frame)
{

    auto format = frame.get_profile().format();

    for(auto&& view: devices_)
    {
               if(view.first == serial_number)
               {
                   if(format == RS2_FORMAT_RGB8)
                   {
                        cv::Mat color_image = cv::Mat(cv::Size(640,480),CV_8UC3,(void*)frame.get_data(),cv::Mat::AUTO_STEP);
                        cv::cvtColor(color_image,color_image,cv::COLOR_BGR2RGB);

                        return color_image;
                   }
                   else if(format == RS2_FORMAT_Z16)
                   {
                       return cv::Mat(cv::Size(640,480),CV_16UC1,(void*)frame.get_data(),cv::Mat::AUTO_STEP);
                   }
                   else
                       throw std::runtime_error("Realsense::convertToImage, no format supported\n");
               }
    }

}
cv::Mat RealSense::displayColorImages(const std::string& serial_number)
{
    cv::Mat color_image = cv::Mat::ones(cv::Size(640,480),CV_8UC3);
    cv::Mat color_clone;
            //For all devices, get one device
              for(auto&& view: devices_)
              {
                  //Check serial number
                  if(view.first == serial_number)
                  {

                    //Check all frames of camera
                      for(auto&& id_to_frame: view.second.frames_per_stream)
                      {
                          //Get color information
                          if(id_to_frame.second.get_profile().format() == RS2_FORMAT_RGB8)
                             {
                                // convert to cv Mat
                                color_image =   cv::Mat(cv::Size(640,480),CV_8UC3,(void*)id_to_frame.second.get_data(),cv::Mat::AUTO_STEP);
                                color_clone = color_image.clone();
                                cv::cvtColor(color_clone,color_clone,cv::COLOR_BGR2RGB);

                             }
                      }
                   }
               }
            return color_clone;
}
cv::Mat RealSense::displayDepthImages(const std::string& serial_number)
{
    cv::Mat depth_image = cv::Mat::ones(cv::Size(640,480),CV_8UC3);

    //For all devices, get one device
      for(auto&& view: devices_)
      {
          //Check serial number
          if(view.first == serial_number)
          {

            //Check all frames of camera
              for(auto&& id_to_frame: view.second.frames_per_stream)
              {
                  //Get depth information
                  if(id_to_frame.second.get_profile().format() == RS2_FORMAT_Z16)
                     {

                        rs2::colorizer colorize_frame;
                        rs2::frame frame= colorize_frame(id_to_frame.second);
                        // convert to cv Mat
                        //depth_image =   cv::Mat(cv::Size(640,480),CV_16UC1,(void*)id_to_frame.second.get_data(),cv::Mat::AUTO_STEP);
                        depth_image =   cv::Mat(cv::Size(640,480),CV_8UC3,(void*)frame.get_data(),cv::Mat::AUTO_STEP);
                     }
             }
         }
      }
    return depth_image.clone();
}
void RealSense::saveData2(const std::string& serial_number,cv::Mat& color_image,pcl::PointCloud<pcl::PointXYZRGB>::Ptr& cloud)
{
//for all devices get one device
    for(auto& view: devices_)
    {
        if(view.first == serial_number)
        {
            //Get streams
             rs2::frame depth_frame;
            // rs2::video_frame color_video_frame;
             rs2::frame color_frame;
             float scale;
             //Go over the device's sensor, get depth scale
             // Go over the device's sensors, get depth scale
            for (rs2::sensor& sensor : view.second.profile.get_device().query_sensors())
                 {
                      // Check if the sensor if a depth sensor
                      if (rs2::depth_sensor dpt = sensor.as<rs2::depth_sensor>())
                      {
                          //Get depth scale of each depth sensor
                          scale = dpt.get_depth_scale();
                      }
                  }
            //Get color intrinsic
              rs2::video_stream_profile color_stream =
                            view.second.profile.get_stream(RS2_STREAM_COLOR).as<rs2::video_stream_profile>();
              rs2_intrinsics color_intrinsics = color_stream.get_intrinsics();
              //Get depth intrinsic
              rs2::video_stream_profile depth_stream =
                              view.second.profile.get_stream(RS2_STREAM_DEPTH).as<rs2::video_stream_profile>();
              rs2_intrinsics depth_intrinsics = depth_stream.get_intrinsics();
              //Go over streams
            for(auto&& id_to_frame : view.second.frames_per_stream)
            {

               //Get depth stream
             if(id_to_frame.second.get_profile().format() == RS2_FORMAT_Z16)
               {
                 depth_frame = id_to_frame.second;
               }
             //Get color stream
             if(id_to_frame.second.get_profile().format() == RS2_FORMAT_RGB8)
                {
                 color_frame = id_to_frame.second;
                 cv::Mat color_image_temp = cv::Mat(cv::Size(640,480),CV_8UC3,(void*)color_frame.get_data(),cv::Mat::AUTO_STEP);
                 color_image = color_image_temp.clone();
                 cv::cvtColor(color_image,color_image,cv::COLOR_BGR2RGB);
                }
            }
            cloud->width  =depth_intrinsics.width;
            cloud->height = depth_intrinsics.height;
            cloud->is_dense = false;
            const uint16_t * depth_data = reinterpret_cast<const uint16_t *> (depth_frame.get_data());
            float color_point[3], scaled_depth;
            for(int y=0;y<depth_intrinsics.height;++y) //480
            {
                for(int x=0;x<depth_intrinsics.width;++x) // 640
                {
                    pcl::PointXYZRGB pt;
                    uint16_t depth_value = depth_data[y*depth_intrinsics.width+x];
                    scaled_depth = depth_value * scale;
                    float color_pixel[2] = {static_cast<float>(x), static_cast<float>(y)};
                    rs2_deproject_pixel_to_point(color_point, &color_intrinsics, color_pixel, scaled_depth);
                    if (color_point[2] <= 0.1f || color_point[2] > 5.f) continue; //remove points too close or too far
                    pt.x = color_point[0];
                    pt.y = color_point[1];
                    pt.z = color_point[2];
                    //if color pixel of 3d point is outside of coordinate, mark as 0,0,0 rgb value
                    if (color_pixel[1] < 0.f || color_pixel[1] > color_intrinsics.height
                        || color_pixel[0] < 0.f || color_pixel[0] > color_intrinsics.width)
                    {

                      pt.r = static_cast<uint8_t>(0);
                      pt.g = static_cast<uint8_t>(0);
                      pt.b = static_cast<uint8_t>(0);
                    }
                    else
                    {
                        auto i = static_cast<int>(color_pixel[0]);
                        auto j = static_cast<int>(color_pixel[1]);
                        cv::Vec3b* cols = color_image.ptr<cv::Vec3b>(j); //Point to r,g,b value on columns
                        cv::Vec3b color = cols[i];
                        uint8_t r = (uint8_t)color.val[2];
                        uint8_t g = (uint8_t)color.val[1];
                        uint8_t b = (uint8_t)color.val[0];
                        uint32_t rgb = ((uint32_t)r << 16 | (uint32_t)g << 8 | (uint32_t)b);
                        pt.rgb = *reinterpret_cast<float*>(&rgb);
                    }
                     cloud->points.push_back(pt);
                }
            }

        }
    }
}
void RealSense::saveData3(const std::string& serial_number,cv::Mat& color_image,cv::Mat& depth,pcl::PointCloud<pcl::PointXYZRGB>::Ptr& cloud)
{
//for all devices get one device

    for(auto& view: devices_)
    {
        if(view.first == serial_number)
        {
            //Get streams
             rs2::frame depth_frame;
            // rs2::video_frame color_video_frame;
             rs2::frame color_frame;
             float scale;
             //Go over the device's sensor, get depth scale
             // Go over the device's sensors, get depth scale

            for (rs2::sensor& sensor : view.second.profile.get_device().query_sensors())
                 {
                      // Check if the sensor if a depth sensor
                      if (rs2::depth_sensor dpt = sensor.as<rs2::depth_sensor>())
                      {
                          //Get depth scale of each depth sensor
                          scale = dpt.get_depth_scale();
                      }
                  }
            //Get color intrinsic
              rs2::video_stream_profile color_stream =
                            view.second.profile.get_stream(RS2_STREAM_COLOR).as<rs2::video_stream_profile>();
              rs2_intrinsics color_intrinsics = color_stream.get_intrinsics();
              //Get depth intrinsic
              rs2::video_stream_profile depth_stream =
                              view.second.profile.get_stream(RS2_STREAM_DEPTH).as<rs2::video_stream_profile>();
              rs2_intrinsics depth_intrinsics = depth_stream.get_intrinsics();
              //Go over streams
            for(auto&& id_to_frame : view.second.frames_per_stream)
            {

               //Get depth stream
             if(id_to_frame.second.get_profile().format() == RS2_FORMAT_Z16)
               {
                 depth_frame = id_to_frame.second;
                    cv::Mat temp_depth;
                    temp_depth = cv::Mat(cv::Size(640,480),CV_16UC1,(void*)depth_frame.get_data(),cv::Mat::AUTO_STEP);
                    depth = temp_depth.clone();
             }
             //Get color stream
             if(id_to_frame.second.get_profile().format() == RS2_FORMAT_RGB8)
                {
                 color_frame = id_to_frame.second;
                 cv::Mat color_image_temp = cv::Mat(cv::Size(640,480),CV_8UC3,(void*)color_frame.get_data(),cv::Mat::AUTO_STEP);
                 color_image = color_image_temp.clone();
                 cv::cvtColor(color_image,color_image,cv::COLOR_BGR2RGB);
                }
            }
            float * depth_buffer_aligned = new float[depth_intrinsics.width * depth_intrinsics.height];
            cloud->width  =depth_intrinsics.width;
            cloud->height = depth_intrinsics.height;
            cloud->is_dense = false;
            const uint16_t * depth_data = reinterpret_cast<const uint16_t *> (depth_frame.get_data());
            float color_point[3], scaled_depth;
           // #pragma omp parallel for schedule(dynamic) //Using OpenMP to try to parallelise the loop
            for(int y=0;y<depth_intrinsics.height;++y) //480
            {

                for(int x=0;x<depth_intrinsics.width;++x) // 640
                {
                    pcl::PointXYZRGB pt;
                    uint16_t depth_value = depth_data[y*depth_intrinsics.width+x];
                    scaled_depth = depth_value * scale;
                    depth_buffer_aligned[y * depth_intrinsics.width + x] =scaled_depth;

                    float color_pixel[2] = {static_cast<float>(x), static_cast<float>(y)};
                    rs2_deproject_pixel_to_point(color_point, &color_intrinsics, color_pixel, scaled_depth);
                    if (color_point[2] <= 0.1f || color_point[2] > 5.f)
                    {
                       // continue; //remove points too close or too far
                       pt.x =0;
                       pt.y =0;
                       pt.z =0;

                    }
                    pt.x = color_point[0];
                    pt.y = color_point[1];
                    pt.z = color_point[2];

                    //if color pixel of 3d point is outside of coordinate, mark as 0,0,0 rgb value
                    if (color_pixel[1] < 0.f || color_pixel[1] > color_intrinsics.height
                        || color_pixel[0] < 0.f || color_pixel[0] > color_intrinsics.width)
                    {

                      pt.r = static_cast<uint8_t>(0);
                      pt.g = static_cast<uint8_t>(0);
                      pt.b = static_cast<uint8_t>(0);
                    }
                    else
                    {
                        auto i = static_cast<int>(color_pixel[0]);
                        auto j = static_cast<int>(color_pixel[1]);
                        cv::Vec3b* cols = color_image.ptr<cv::Vec3b>(j); //Point to r,g,b value on columns
                        cv::Vec3b color = cols[i];
                        uint8_t r = (uint8_t)color.val[2];
                        uint8_t g = (uint8_t)color.val[1];
                        uint8_t b = (uint8_t)color.val[0];
                        uint32_t rgb = ((uint32_t)r << 16 | (uint32_t)g << 8 | (uint32_t)b);
                        pt.rgb = *reinterpret_cast<float*>(&rgb);
                    }
                     cloud->points.push_back(pt);
                }
            }

        }
    }
}


void RealSense::saveData(const std::string& serial_number,cv::Mat& color_image,cv::Mat& depth_image)
{

           //For all devices, get one device
             for(auto&& view: devices_)
             {
                 if(view.first == serial_number)
                 {
                 //Get streams
                  rs2::frame depth_frame;
                 // rs2::video_frame color_video_frame;
                  rs2::frame color_frame;             
                    //Go over streams
                  for(auto&& id_to_frame : view.second.frames_per_stream)
                  {

                     //Get depth stream
                   if(id_to_frame.second.get_profile().format() == RS2_FORMAT_Z16)
                     {                     
                       depth_frame = id_to_frame.second;
                       cv::Mat depth_temp = cv::Mat(cv::Size(640,480),CV_16UC1,(void*)depth_frame.get_data(),cv::Mat::AUTO_STEP);
                       depth_image = depth_temp.clone();
                     }
                   //Get color stream
                   if(id_to_frame.second.get_profile().format() == RS2_FORMAT_RGB8)
                      {             
                       color_frame = id_to_frame.second;
                       cv::Mat color_image_temp = cv::Mat(cv::Size(640,480),CV_8UC3,(void*)color_frame.get_data(),cv::Mat::AUTO_STEP);
                       color_image = color_image_temp.clone();
                       cv::cvtColor(color_image,color_image,cv::COLOR_BGR2RGB);
                      }
                  }
                  }
                 }


}


void RealSense::getCameraInformation(const std::string data_path, std::vector<cv::Mat>& color_K)
{

    std::vector<float> scales;
    std::vector<cv::Mat> depth_K,depth2color_ext, depth_coeffs,color_coeffs;
    std::vector<std::string> serial_numbers_vec;
    for(auto&& view: devices_)
    {
    // Go over the device's sensors
    for (rs2::sensor& sensor : view.second.profile.get_device().query_sensors())
    {
        // Check if the sensor if a depth sensor
        if (rs2::depth_sensor dpt = sensor.as<rs2::depth_sensor>())
        {
            //Get depth scale of each depth sensor
            scales.push_back(dpt.get_depth_scale());
        }
    }
    rs2::video_stream_profile depth_stream =
            view.second.profile.get_stream(RS2_STREAM_DEPTH).as<rs2::video_stream_profile>();
    rs2::video_stream_profile color_stream =
            view.second.profile.get_stream(RS2_STREAM_COLOR).as<rs2::video_stream_profile>();
    cv::Mat color_camera_matrix, depth_camera_matrix, extrinsic_depth_to_color_matrix;
    cv::Mat depth_distortions, color_distortions;
    rs2_intrinsics intrinsic_depth = depth_stream.get_intrinsics();
            //depth_K.push_back(intrinsic_depth);
    rs2_intrinsics intrinsic_color = color_stream.get_intrinsics();
            //color_K.push_back(intrinsic_color);
    rs2_extrinsics extrinsic_depth_to_color = depth_stream.get_extrinsics_to(color_stream);
            //depth2color_ext.push_back(extrinsic_depth_to_color);
    //Convert to cv::Mat
    color_camera_matrix =  (cv::Mat_<double>(3,3)<<   intrinsic_color.fx,0.0f,intrinsic_color.ppx,
                                                       0.0f,intrinsic_color.fy, intrinsic_color.ppy,
                                                            0.0f,0.0f,1.0f);
    std::cout<<"serial number: " <<view.first <<std::endl;
    serial_numbers_vec.push_back(view.first);
    std::cout<<"camera matrix: "<< color_camera_matrix<<std::endl;
    depth_camera_matrix =  (cv::Mat_<double>(3,3)<<   intrinsic_depth.fx,0.0f,intrinsic_depth.ppx,
                                                       0.0f,intrinsic_depth.fy, intrinsic_depth.ppy,
                                                            0.0f,0.0f,1.0f);
    extrinsic_depth_to_color_matrix =
    (cv::Mat_<double>(4,4)<<  extrinsic_depth_to_color.rotation[0],extrinsic_depth_to_color.rotation[3],extrinsic_depth_to_color.rotation[6],extrinsic_depth_to_color.translation[0],
                              extrinsic_depth_to_color.rotation[1],extrinsic_depth_to_color.rotation[4],extrinsic_depth_to_color.rotation[7],extrinsic_depth_to_color.translation[1],
                              extrinsic_depth_to_color.rotation[2],extrinsic_depth_to_color.rotation[5],extrinsic_depth_to_color.rotation[8],extrinsic_depth_to_color.translation[2]);
    color_distortions = (cv::Mat_<double>(1,5)<<intrinsic_color.coeffs[0],intrinsic_color.coeffs[1],intrinsic_color.coeffs[2],
                                                intrinsic_color.coeffs[3],intrinsic_color.coeffs[4]);
    depth_distortions = (cv::Mat_<double>(1,5)<<intrinsic_depth.coeffs[0],intrinsic_depth.coeffs[1],intrinsic_depth.coeffs[2],
                                                intrinsic_depth.coeffs[3],intrinsic_depth.coeffs[4]);
    color_K.push_back(color_camera_matrix);
    depth_K.push_back(depth_camera_matrix);
    depth_coeffs.push_back(depth_distortions);
    color_coeffs.push_back(color_distortions);
    depth2color_ext.push_back(extrinsic_depth_to_color_matrix);
    }

    //write camera information to yaml file
    cv::FileStorage fs(data_path+"params.yaml",cv::FileStorage::WRITE);
    for(int i=0;i<devices_.size();i++)
    {
        std::ostringstream ss;
        ss<<(i+1);
        std::string serial_number_flag = "serialNumber"+ss.str();
        std::string camera_matrix_flag = "cameraMatrix"+ss.str();
        std::string depth_camera_matrix_flag= "depthCameraMatrix"+ss.str();
        std::string depth2color_ext_flags = "extrinsicsDepthToColor"+ss.str();
        std::string depth_coeffs_flags = "depthCoeffs"+ss.str();
        std::string color_coeffs_flags = "colorCoeffs"+ss.str();
        std::string depth_scale_flags = "depthScale"+ss.str();
        fs<<serial_number_flag<<serial_numbers_vec[i];
        fs<<camera_matrix_flag<<color_K[i];
        fs<<depth_camera_matrix_flag<<depth_K[i];
        fs<<depth2color_ext_flags<<depth2color_ext[i];
        fs<<color_coeffs_flags<<color_coeffs[i];
        fs<<depth_coeffs_flags<<depth_coeffs[i];
        fs<<depth_scale_flags<<scales[i];
    }
    fs.release();
}

void RealSense::getConfiguration()
{
    // Go over all device
    for (auto&& view : devices_)
    {
        std::cout<<"Camera with serial number "<<view.first<<std::endl;
        //Get all sensors in a device
         std::vector<rs2::sensor> sensors = view.second.profile.get_device().query_sensors();
         //For each sensor
         for(rs2::sensor sensor :sensors)
         {
            if(sensor.supports(RS2_CAMERA_INFO_NAME))
            {
                std::cout<<sensor.get_info(RS2_CAMERA_INFO_NAME)<<std::endl;
            }
            else
                std::cout<<"Unknown Sensor"<<std::endl;

            //Get configuration of each sensor in each camera
            for(int i=0;i<static_cast<int>(RS2_OPTION_COUNT); i++)
            {
                 rs2_option option_type = static_cast<rs2_option>(i);
                  // First, verify that the sensor actually supports this option
                 if(sensor.supports(option_type))
                 {
                     rs2::option_range range = sensor.get_option_range(option_type);

                     std::cout << "  " << i << ": " << option_type;
                     std::cout << std::endl;

                    // Get a human readable description of the option
                     const char* description = sensor.get_option_description(option_type);
                     std::cout << "       Description   : " << description << std::endl;

                     // Get the current value of the option
                     float current_value = sensor.get_option(option_type);
                    std::cout << "       Current Value : " << current_value << std::endl;

                    //Get range of this option
                    std::cout <<"Max "<<range.max << " Min "<<range.min <<" step "<<range.step<<std::endl;
                 }
            }
         }

    }
}

void RealSense::setConfiguration(const std::string& configuration_file)
{
    cv::FileStorage fs(configuration_file,cv::FileStorage::READ);
    for(auto&& view : devices_)
    {

        // Go over the device's sensors
        for (rs2::sensor& sensor : view.second.profile.get_device().query_sensors())
        {
            // Check if the sensor if a depth sensor
            if (rs2::depth_sensor dpt = sensor.as<rs2::depth_sensor>())
            {

                dpt.set_option(RS2_OPTION_VISUAL_PRESET,fs["Visual Preset"]);
                dpt.set_option(RS2_OPTION_ACCURACY,fs["Accuracy"]);
                dpt.set_option(RS2_OPTION_CONFIDENCE_THRESHOLD,fs["Confidence Threshold"]);
                dpt.set_option(RS2_OPTION_MOTION_RANGE,fs["Motion Range"]);
                dpt.set_option(RS2_OPTION_FILTER_OPTION,fs["Filter Option"]);

            }
           else
            {       //Color

               //sensor.set_option(RS2_OPTION_ENABLE_AUTO_EXPOSURE,fs["Enable Auto Exposure"]);
               sensor.set_option(RS2_OPTION_WHITE_BALANCE,fs["White Balance"]);
               sensor.set_option(RS2_OPTION_EXPOSURE,fs["Exposure"]);
               sensor.set_option(RS2_OPTION_GAIN,fs["Gain"]);
               sensor.set_option(RS2_OPTION_SHARPNESS,fs["Sharpness"]);
               sensor.set_option(RS2_OPTION_GAMMA,fs["Gamma"]);
               sensor.set_option(RS2_OPTION_BRIGHTNESS,fs["Brightness"]);
               sensor.set_option(RS2_OPTION_SATURATION,fs["Saturation"]);
               sensor.set_option(RS2_OPTION_HUE,fs["Hue"]);

            }

        }


    }
std::cout<<"finish setting configuration"<<std::endl;
fs.release();
}
void RealSense::stopStreaming()
{
    for(auto&& view : devices_)
    {
        view.second.pipe.stop();
    }
}
