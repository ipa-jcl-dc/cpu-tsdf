#include "aruco_marker.hpp"

Marker::Marker()
{
}
Marker::~Marker()
{
}
unsigned long Marker::LoadParameters(std::string directory_and_filename)
{
    boost::shared_ptr<TiXmlDocument> p_configXmlDocument (new TiXmlDocument(directory_and_filename));
    if (!p_configXmlDocument->LoadFile())
           {
                   std::cerr << "ERROR - Marker::LoadParameters:" << std::endl;
                   std::cerr << "\t ... Error while loading xml configuration file (Check filename and syntax of the file):" << std::endl;
                   std::cerr << "\t ... '" << directory_and_filename << std::endl;
                   return EXIT_FAILURE;
           }
    std::cerr << "INFO - Marker::LoadParameters:" << std::endl;
    std::cerr << "\t ... Parsing xml configuration file:" << std::endl;
    std::cerr << "\t ... " << directory_and_filename << std::endl;
    if ( p_configXmlDocument )
    {
        //************************************************************************************
        //        BEGIN parameters
        //************************************************************************************
         TiXmlElement *p_xmlElement_root = NULL;
         p_xmlElement_root = p_configXmlDocument->FirstChildElement("parameters");

         if(p_xmlElement_root)
         {

             TiXmlElement *p_xmlElement_charuco = NULL;
             p_xmlElement_charuco = p_xmlElement_root->FirstChildElement("charuco");
       //************************************************************************************
       //        BEGIN parameters -> charuco
       //************************************************************************************
             if(p_xmlElement_charuco)
             {

                 TiXmlElement *p_charuco_params = NULL;
                 p_charuco_params = p_xmlElement_charuco->FirstChildElement("params");
      //************************************************************************************
      //        BEGIN parameters -> charuco -> params
      //************************************************************************************
                 if(p_charuco_params)
                 {
                     //Read and save value of attributes
                     if(p_charuco_params->QueryIntAttribute("squareX",&charuco_params_.square_x) !=TIXML_SUCCESS)
                     {
                         std::cerr << "ERROR - Marker::LoadParameters:" << std::endl;
                         std::cerr << "\t ... Can't find attribute 'squareX'" << std::endl;
                         return EXIT_FAILURE;
                     }
                     if(p_charuco_params->QueryIntAttribute("squareY",&charuco_params_.square_y) !=TIXML_SUCCESS)
                     {
                         std::cerr << "ERROR - Marker::LoadParameters:" << std::endl;
                         std::cerr << "\t ... Can't find attribute 'squareY'" << std::endl;
                         return EXIT_FAILURE;
                     }
                     if(p_charuco_params->QueryFloatAttribute("squareLength",&charuco_params_.square_length) !=TIXML_SUCCESS)
                     {
                         std::cerr << "ERROR - Marker::LoadParameters:" << std::endl;
                         std::cerr << "\t ... Can't find attribute 'squareLength'" << std::endl;
                         return EXIT_FAILURE;
                     }
                     if(p_charuco_params->QueryFloatAttribute("markerLength",&charuco_params_.marker_length) !=TIXML_SUCCESS)
                     {
                         std::cerr << "ERROR - Marker::LoadParameters:" << std::endl;
                         std::cerr << "\t ... Can't find attribute 'markerLength'" << std::endl;
                         return EXIT_FAILURE;
                     }
                     if(p_charuco_params->QueryIntAttribute("dictionaryID",&charuco_params_.dictionary_id) !=TIXML_SUCCESS)
                     {
                         std::cerr << "ERROR - Marker::LoadParameters:" << std::endl;
                         std::cerr << "\t ... Can't find attribute 'dictionaryID'" << std::endl;
                         return EXIT_FAILURE;
                     }
                     if(p_charuco_params->QueryIntAttribute("borderBits",&charuco_params_.border_bits) !=TIXML_SUCCESS)
                     {
                         std::cerr << "ERROR - Marker::LoadParameters:" << std::endl;
                         std::cerr << "\t ... Can't find attribute 'borderBits'" << std::endl;
                         return EXIT_FAILURE;
                     }
                 }
      //************************************************************************************
      //        END parameters -> charuco -> params
      //************************************************************************************
                else{
                     std::cerr<<"Couldn't find params"<<std::endl;
                     return EXIT_FAILURE;
                 }
             }
      //************************************************************************************
      //        END parameters -> charuco
      //************************************************************************************
             else{
                    std::cerr<<"Couldn't find charuco"<<std::endl;
                    return EXIT_FAILURE;
                 }
             TiXmlElement *p_xmlElement_aruco = NULL;
             p_xmlElement_aruco = p_xmlElement_root->FirstChildElement("aruco");
             //************************************************************************************
             //        BEGIN parameters -> aruco
             //************************************************************************************
                   if(p_xmlElement_aruco)
                   {

                       TiXmlElement *p_aruco_params = NULL;
                       p_aruco_params = p_xmlElement_aruco->FirstChildElement("params");
            //************************************************************************************
            //        BEGIN parameters -> aruco -> params
            //************************************************************************************
                       if(p_aruco_params)
                       {
                           //Read and save value of attributes
                           if(p_aruco_params->QueryIntAttribute("markersX",&aruco_params_.markers_x) !=TIXML_SUCCESS)
                           {
                               std::cerr << "ERROR - Marker::LoadParameters::aruco" << std::endl;
                               std::cerr << "\t ... Can't find attribute 'markersX'" << std::endl;
                               return EXIT_FAILURE;
                           }
                           if(p_aruco_params->QueryIntAttribute("markersY",&aruco_params_.markers_y) !=TIXML_SUCCESS)
                           {
                               std::cerr << "ERROR - Marker::LoadParameters::aruco" << std::endl;
                               std::cerr << "\t ... Can't find attribute 'markersY'" << std::endl;
                               return EXIT_FAILURE;
                           }
                           if(p_aruco_params->QueryIntAttribute("markerSeparation",&aruco_params_.marker_separation) !=TIXML_SUCCESS)
                           {
                               std::cerr << "ERROR - Marker::LoadParameters:" << std::endl;
                               std::cerr << "\t ... Can't find attribute 'squareLength'" << std::endl;
                               return EXIT_FAILURE;
                           }
                           if(p_aruco_params->QueryIntAttribute("markerLength",&aruco_params_.marker_length) !=TIXML_SUCCESS)
                           {
                               std::cerr << "ERROR - Marker::LoadParameters:" << std::endl;
                               std::cerr << "\t ... Can't find attribute 'markerLength'" << std::endl;
                               return EXIT_FAILURE;
                           }
                           if(p_aruco_params->QueryIntAttribute("dictionaryID",&aruco_params_.dictionary_id) !=TIXML_SUCCESS)
                           {
                               std::cerr << "ERROR - Marker::LoadParameters:" << std::endl;
                               std::cerr << "\t ... Can't find attribute 'dictionaryID'" << std::endl;
                               return EXIT_FAILURE;
                           }
                           if(p_aruco_params->QueryIntAttribute("borderBits",&aruco_params_.border_bits) !=TIXML_SUCCESS)
                           {
                               std::cerr << "ERROR - Marker::LoadParameters:" << std::endl;
                               std::cerr << "\t ... Can't find attribute 'borderBits'" << std::endl;
                               return EXIT_FAILURE;
                           }
                       }
            //************************************************************************************
            //        END parameters -> aruco -> params
            //************************************************************************************
                      else{
                           std::cerr<<"Couldn't find params"<<std::endl;
                           return EXIT_FAILURE;
                       }
                   }
            //************************************************************************************
            //        END parameters -> aruco
            //************************************************************************************
                   else{
                          std::cerr<<"Couldn't find aruco"<<std::endl;
                          return EXIT_FAILURE;
                       }

         }
     //************************************************************************************
     //        END parameters
     //************************************************************************************
         else{
                 std::cerr<<"Couldn't find parameters"<<std::endl;
                 return EXIT_FAILURE;
              }
    }
   // charuco_params_.margins = charuco_params_.square_length - charuco_params_.marker_length;
    aruco_params_.margins = aruco_params_.marker_separation;
    return EXIT_SUCCESS;
}
unsigned long Marker::createChaRucoMarker(cv::Mat& board_image)
{
    //Create a dictionay of board
    dictionary_ =
    cv::aruco::getPredefinedDictionary(cv::aruco::PREDEFINED_DICTIONARY_NAME(charuco_params_.dictionary_id));
    cv::Size image_size;
    //Define the size of board(in pixels)
    image_size.width = charuco_params_.square_x * 100 + 40*2; //1480 pixels
    image_size.height = charuco_params_.square_y * 100 + 40*2; // 1080 pixels
    charuco_params_.margins = 40; //margins 40 pixels
    //Create Charuco board
    charuco_board_ = cv::aruco::CharucoBoard ::create(charuco_params_.square_x,charuco_params_.square_y,
                                        (float)charuco_params_.square_length,(float)charuco_params_.marker_length,dictionary_);

    //Draw it on image
    charuco_board_->draw(image_size,board_image,charuco_params_.margins,charuco_params_.border_bits);

    //Convert back to aruco structure
    board_ = charuco_board_.staticCast<cv::aruco::Board>();
    return EXIT_SUCCESS;
}
unsigned long Marker::createArucoMarker(cv::Mat& board_images)
{
    cv::Ptr<cv::aruco::GridBoard> board;
      //Define the size of board(in pixels)
    cv::Size image_size;
    image_size.width = aruco_params_.markers_x * (aruco_params_.marker_length + aruco_params_.marker_separation) -
              aruco_params_.marker_separation + 2 * aruco_params_.margins;
    image_size.height =
              aruco_params_.markers_y * (aruco_params_.marker_length + aruco_params_.marker_separation) -
                            aruco_params_.marker_separation + 2 * aruco_params_.margins;

    //Create a dictionay of board
    cv::Ptr<cv::aruco::Dictionary> dictionary =
    cv::aruco::getPredefinedDictionary(cv::aruco::PREDEFINED_DICTIONARY_NAME(aruco_params_.dictionary_id));
    //Create aruco board
    board = cv::aruco::GridBoard::create(aruco_params_.markers_x, aruco_params_.markers_y,
    float(aruco_params_.marker_length),float(aruco_params_.marker_separation), dictionary);
    //Draw it on image
    board->draw(image_size, board_images, aruco_params_.margins, aruco_params_.border_bits);
    return EXIT_SUCCESS;
}

bool Marker::estimatePoseCharuco(cv::Mat &frame,aruco_pose& pose)
{
    bool debug = false;
    float axisLength = 0.1 ; //10 cm
    std::vector< int > markerIds, charucoIds;
    //Pixel coordinates of makers before and after refinement
    std::vector< std::vector< cv::Point2f > > markerCorners, rejectedMarkers;
    std::vector< cv::Point2f > charucoCorners;

    pose.rot = cv::Mat::eye(3,3,CV_32F);
    pose.trans = cv::Mat::zeros(0,0,CV_32F);
    //Defaut parameters to detect marker, it can be set manually
    cv::Ptr<cv::aruco::DetectorParameters> detectorParams = cv::aruco::DetectorParameters::create();
    // detect markers
    cv::aruco::detectMarkers(frame,dictionary_, markerCorners, markerIds,detectorParams,rejectedMarkers);
    //refined strategy to detect mores markers
    cv::aruco::refineDetectedMarkers(frame, board_, markerCorners, markerIds, rejectedMarkers,
    camera_matrix_, cv::Mat());
    // interpolate charuco corners
    int interpolatedCorners = 0;
    if(markerIds.size() > 0)
        interpolatedCorners =cv::aruco::interpolateCornersCharuco(markerCorners, markerIds, frame, charuco_board_,
                        charucoCorners, charucoIds, camera_matrix_, cv::Mat());
    // estimate charuco board pose
    bool validPose = false;
       if(camera_matrix_.total() != 0)
              validPose = cv::aruco::estimatePoseCharucoBoard(charucoCorners, charucoIds, charuco_board_,
                                                          camera_matrix_, cv::Mat(), pose.rot, pose.trans);
    //Convert Rodrigues rotation matrix from euclid angles
    cv::Mat R;
    cv::Rodrigues(pose.rot,R);
    R.copyTo(pose.rot);

    if(markerIds.size() > 0) {
       cv::aruco::drawDetectedMarkers(frame, markerCorners);
       }
    if(rejectedMarkers.size() > 0)
       // cv::aruco::drawDetectedMarkers(frame, rejectedMarkers, cv::noArray(), cv::Scalar(100, 0, 255));
    if(interpolatedCorners > 0) {
                cv::Scalar color;
                color = cv::Scalar(255, 0, 0);
             //   cv::aruco::drawDetectedCornersCharuco(frame, charucoCorners, charucoIds, color);
            }

    if(validPose&&debug)
        {
        cv::aruco::drawAxis(frame, camera_matrix_, cv::Mat(), pose.rot, pose.trans, axisLength);
        addCube(frame,pose);
        cv::imshow("image",frame);
        cv::waitKey(0);
         }
    return validPose;
}
void Marker::addCube(cv::Mat& img, const aruco_pose& pose)
{
    std::vector<cv::Point3f> objectPoints; //coordinates of tag system
    objectPoints.push_back(cv::Point3f(0.370417,0.264583,0)); //C -0
    objectPoints.push_back(cv::Point3f(0.370417,0,0)); //B-1
    objectPoints.push_back(cv::Point3f(0,0.264583,0)); //D-2
    objectPoints.push_back(cv::Point3f(0,0,0));//A-3

    objectPoints.push_back(cv::Point3f(0.370417,0.264583,0.25)); //C1 -0
    objectPoints.push_back(cv::Point3f(0.370417,0,0.25)); //B1 - 1
    objectPoints.push_back(cv::Point3f(0,0.264583,0.25)); //D1-2
    objectPoints.push_back(cv::Point3f(0,0,0.25)); //A1 -3
    std::vector<cv::Point2f> cube_imagePoints;

 //Project objectPoints into image coordinates
        cv::projectPoints( objectPoints,pose.rot,pose.trans, camera_matrix_,cv::Mat(),cube_imagePoints);
        cv::line(img,cube_imagePoints[0],cube_imagePoints[1],cv::Scalar(0,0,255,255),1,CV_AA);//0-1
        cv::line(img,cube_imagePoints[0],cube_imagePoints[2],cv::Scalar(0,0,255,255),1,CV_AA);//0-2
        cv::line(img,cube_imagePoints[3],cube_imagePoints[1],cv::Scalar(0,0,255,255),1,CV_AA);
        cv::line(img,cube_imagePoints[3],cube_imagePoints[2],cv::Scalar(0,0,255,255),1,CV_AA);

        cv::line(img,cube_imagePoints[4],cube_imagePoints[5],cv::Scalar(0,0,255,255),1,CV_AA);
        cv::line(img,cube_imagePoints[4],cube_imagePoints[6],cv::Scalar(0,0,255,255),1,CV_AA);
        cv::line(img,cube_imagePoints[7],cube_imagePoints[5],cv::Scalar(0,0,255,255),1,CV_AA);
        cv::line(img,cube_imagePoints[7],cube_imagePoints[6],cv::Scalar(0,0,255,255),1,CV_AA);
        for (int i=0;i<4;i++)
            {
                cv::line(img,cube_imagePoints[i],cube_imagePoints[i+4],cv::Scalar(0,0,255,255),1,CV_AA);
            }
}
