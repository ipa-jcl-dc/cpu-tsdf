#include <opencv2/dnn.hpp>
#include <librealsense2/rs.hpp>
#include "realsense.hpp"

const size_t inWidth = 300;
const size_t inHeight = 300;
const float WHRatio = inWidth / (float)inHeight;
const float inScaleFactor = 0.007843f;
const float meanVal       = 127.5;
const char* classNames[]  = {"background",
                             "aeroplane", "bicycle", "bird", "boat",
                             "bottle", "bus", "car", "cat", "chair",
                             "cow", "diningtable", "dog", "horse",
                             "motorbike", "person", "pottedplant",
                            "sheep", "sofa", "train", "tvmonitor", "cell phone", "keyboard",
                            "mouse","book","cup"};
using namespace std;
int main(int argc, char** argv) try
{
   RealSense realsense;
    cv::dnn::Net net = cv::dnn::readNetFromCaffe("../config/MobileNetSSD_deploy.prototxt","../config/MobileNetSSD_deploy.caffemodel");
    // Start streaming from Intel RealSense Camera
    rs2::pipeline pipe;
    auto config = pipe.start();
    auto profile = config.get_stream(RS2_STREAM_COLOR)
                         .as<rs2::video_stream_profile>();
    rs2::align align_to(RS2_STREAM_COLOR); //align all streams to color stream
    cv::Size cropSize;
    if (profile.width() / (float)profile.height() > WHRatio)
       {
           cropSize = cv::Size(static_cast<int>(profile.height() * WHRatio),
                           profile.height());
       }
       else
       {
           cropSize = cv::Size(profile.width(),
                           static_cast<int>(profile.width() / WHRatio));
       }
    cv::Rect crop(cv::Point((profile.width() - cropSize.width) / 2,
                       (profile.height() - cropSize.height) / 2),
    cropSize);


    const std::string window_name = "Display Image";
    cv::namedWindow(window_name, cv::WINDOW_AUTOSIZE);
    char c =0;

    while(1)
    {
        // Wait for the next set of frames
        rs2::frameset data = pipe.wait_for_frames();
        // Make sure the frames are spatially aligned
        data = align_to.process(data);
        auto color_frame = data.get_color_frame();
        auto depth_frame = data.get_depth_frame();
        // If we only received new depth frame,
        // but the color did not update, continue
        static int last_frame_number = 0;

        if (color_frame.get_frame_number() == last_frame_number) continue;
        last_frame_number = color_frame.get_frame_number();
        // Convert RealSense frame to OpenCV matrix:
        auto color_mat = realsense.frameToMat(color_frame);
        auto depth_mat = realsense.depthFrameToMeters(pipe,depth_frame);
        cv::Mat inputBlob = cv::dnn::blobFromImage(color_mat, inScaleFactor,
        cv::Size(inWidth, inHeight), meanVal, false); //Convert Mat to batch of images
        net.setInput(inputBlob, "data"); //set the network input
        cv::Mat detection = net.forward("detection_out"); //compute output
        cv::Mat detectionMat(detection.size[2], detection.size[3], CV_32F, detection.ptr<float>());
        // Crop both color and depth frames
        color_mat = color_mat(crop);
        depth_mat = depth_mat(crop);
        float confidenceThreshold = 0.8f;

        for(int i = 0; i < detectionMat.rows; i++)
              {
                  float confidence = detectionMat.at<float>(i, 2);

                  if(confidence > confidenceThreshold)
                  {
                      size_t objectClass = (size_t)(detectionMat.at<float>(i, 1));

                      int xLeftBottom = static_cast<int>(detectionMat.at<float>(i, 3) * color_mat.cols);
                      int yLeftBottom = static_cast<int>(detectionMat.at<float>(i, 4) * color_mat.rows);
                      int xRightTop = static_cast<int>(detectionMat.at<float>(i, 5) * color_mat.cols);
                      int yRightTop = static_cast<int>(detectionMat.at<float>(i, 6) * color_mat.rows);

                      cv::Rect object((int)xLeftBottom, (int)yLeftBottom,
                                  (int)(xRightTop - xLeftBottom),
                                  (int)(yRightTop - yLeftBottom));

                      object = object  & cv::Rect(0, 0, depth_mat.cols, depth_mat.rows);

                      // Calculate mean depth inside the detection region
                      // This is a very naive way to estimate objects depth
                      // but it is intended to demonstrate how one might
                      // use depht data in general
                      cv::Scalar m = cv::mean(depth_mat(object));

                      std::ostringstream ss;
                      ss << classNames[objectClass] << " ";
                      ss << std::setprecision(2) << m[0] << " meters away";
                      std::string conf(ss.str());

                      cv::rectangle(color_mat, object, cv::Scalar(0, 255, 0));
                      int baseLine = 0;
                      cv::Size labelSize = cv::getTextSize(ss.str(), cv::FONT_HERSHEY_SIMPLEX, 0.5, 1, &baseLine);

                      auto center = (object.br() + object.tl())*0.5;
                      center.x = center.x - labelSize.width / 2;

                      cv::rectangle(color_mat, cv::Rect(cv::Point(center.x, center.y - labelSize.height),
                          cv::Size(labelSize.width, labelSize.height + baseLine)),
                          cv::Scalar(255, 255, 255), CV_FILLED);
                      cv::putText(color_mat, ss.str(), center,
                              cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(0,0,0));
                  }
            }

        cv::imshow("window_name", color_mat);
        if (cv::waitKey(1) >= 0) break;
    }

 return EXIT_SUCCESS;
}
catch (const rs2::error & e)
{
    std::cerr << "RealSense error calling " << e.get_failed_function() << "(" << e.get_failed_args() << "):\n    " << e.what() << std::endl;
    return EXIT_FAILURE;
}
catch (const std::exception& e)
{
    std::cerr << e.what() << std::endl;
    return EXIT_FAILURE;
}
