#include "segmentation.hpp"

unsigned long Segmentation::cloud2image(cv::Mat& img,const pcl::PointCloud<pcl::PointXYZRGB>::Ptr& cloud)
{
    img =  cv::Mat::zeros(HEIGHT,WIDTH,CV_8UC3);
    #pragma omp parallel for
    for( int y = 0; y < img.rows; y++ ) {
        for( int x = 0; x < img.cols; x++ ) {
            int index = y*img.cols + x;
            img.at<cv::Vec3b>( y, x )[0] = (int)cloud->points[index].b;
            img.at<cv::Vec3b>( y, x )[1] = (int)cloud->points[index].g;
            img.at<cv::Vec3b>( y, x )[2] = (int)cloud->points[index].r;
        }
    }
    return EXIT_SUCCESS;
}

unsigned long Segmentation::cloud2image(cv::Mat& img,const pcl::PointCloud<pcl::PointXYZRGB>::Ptr& cloud,
                                        const std::vector<int>& indices)
{	
    //Create an organized point cloud size 640x480
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_clone(new pcl::PointCloud<pcl::PointXYZRGB>);
    cloud_clone->points.resize(HEIGHT*WIDTH);
    //Fill all points for cloud clone
    #pragma omp parallel for
    for(int x =0; x<WIDTH;x++)
                  for(int y =0;y<HEIGHT;y++)
                  {
                    pcl::PointXYZRGB& pt = cloud_clone->points[x*HEIGHT+y];
                    pt.x = pt.y = pt.z = 0;
                    pt.r = pt.g = pt.b = 0;
                  }
    //change value of points with given indices
    for (size_t i = 0; i < indices.size (); ++i)
    {

        cloud_clone->points[indices[i]] =  cloud->points[indices[i]];
    }
    cloud_clone->height = HEIGHT;
    cloud_clone->width = WIDTH;
    cloud_clone->is_dense = false;

    img =  cv::Mat::zeros(HEIGHT,WIDTH,CV_8UC3);
    #pragma omp parallel for
    for( int y = 0; y < img.rows; y++ ) {
            for( int x = 0; x < img.cols; x++ ) {
                int index = y*img.cols + x;
                img.at<cv::Vec3b>( y, x )[0] = (int)cloud_clone->points[index].b;
                img.at<cv::Vec3b>( y, x )[1] = (int)cloud_clone->points[index].g;
                img.at<cv::Vec3b>( y, x )[2] = (int)cloud_clone->points[index].r;
            }
    }

        return EXIT_SUCCESS;
}

unsigned long Segmentation::cropDepthFromColor(const cv::Mat& color_map,cv::Mat& depth_map)
{
        cv::Mat gray;
        cv::cvtColor(color_map,gray,CV_RGB2GRAY);
        cv::Mat mask;
        mask = cv::Mat::ones(gray.size(),CV_32F)*255.;

        for(int w=0;w<gray.cols;w++)
            for(int h=0;h<gray.rows;h++)
            {
                if(gray.at<uchar>(h,w) ==0)
                {
                    mask.at<float>(h,w) = 0.;
                }
            }
        int dilation_size = 2;
        int dilation_type = cv::MORPH_RECT;//MORPH_RECT MORPH_CROSS  MORPH_ELLIPSE;
        cv::Mat element = cv::getStructuringElement( dilation_type, cv::Size( 2*dilation_size + 1, 2*dilation_size+1 ),
        cv::Point( dilation_size, dilation_size ) );
        cv::morphologyEx(mask,mask,cv::MORPH_OPEN,element); //remove outliers
             //  cv::imwrite(mask_file,mask);
        for(int w=0;w<mask.cols;w++)
            for(int h=0;h<mask.rows;h++)
            {
                if(mask.at<float>(h,w) ==0)
                {
                    depth_map.at<unsigned short>(h,w) = 0.;
                }
            }
        return EXIT_SUCCESS;
}

void Segmentation::objectContour(cv::Mat& img,const cv::Mat& converted_img)
{
    cv::Mat gray;
    cv::cvtColor(converted_img,gray,CV_RGB2GRAY);
    int thresh = 50;
      std::vector<std::vector<cv::Point> > contours;
      std::vector<cv::Vec4i> hierarchy;
    /// Detect edges using Threshold
    cv::threshold( gray, gray, thresh, 255, cv::THRESH_BINARY );
    /// Find contours
    cv::findContours( gray, contours, hierarchy, CV_RETR_TREE, CV_CHAIN_APPROX_SIMPLE, cv::Point(0, 0) );
     std::vector<std::vector<cv::Point> > contours_poly( contours.size() );


    ////Find polygon contours
    #pragma omp parallel for
     for( int i = 0; i < contours.size(); i++ )
        {
         cv::approxPolyDP( cv::Mat(contours[i]), contours_poly[i], 3, true );
        }
       int largest_area=0;
       int largest_contour_index=0;
       cv::Rect bounding_rect;
     for( size_t i = 0; i< contours_poly.size(); i++ ) // iterate through each contour.
     {
         double area = cv::contourArea( contours_poly[i] );  //  Find the area of contour

         if( area > largest_area )
         {
             largest_area = area;
             largest_contour_index = i;               //Store the index of largest contour
             bounding_rect = cv::boundingRect( contours_poly[i] ); // Find the bounding rectangle for biggest contour
         }
     }

     cv::drawContours( img, contours_poly,largest_contour_index, cv::Scalar( 0, 255, 0 ), 2 ); // Draw the largest contour using previously stored index.
}










/*
void Segmentation::createMask(const cv::Mat& img,const pcl::PointCloud<pcl::PointXYZRGB>::Ptr& cloud,cv::Mat& mask)
{
	bool debug = false;//show segmented image

	pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_clone (new pcl::PointCloud<pcl::PointXYZRGB>);
	*cloud_clone = *cloud;
		cloud_clone->is_dense = true;
	cv::Mat temp_img;
	temp_img = cv::Mat::zeros(img.size(),CV_8UC3);
	if(cloud_clone->isOrganized()){
	  for (int h=0; h<img.rows; h++) {
	                for (int w=0; w<img.cols; w++) {
	                    pcl::PointXYZRGB& point = cloud_clone->at(w, h);
	                    Eigen::Vector3i rgb = point.getRGBVector3i();
	                    temp_img.at<cv::Vec3b>(h,w)[0] = rgb[2];
	                    temp_img.at<cv::Vec3b>(h,w)[1] = rgb[1];
	                    temp_img.at<cv::Vec3b>(h,w)[2] = rgb[0];
	                      }
	                  }
	              }

	  cv::Mat black,segment_img;
	  black= cv::Mat::zeros(img.size(),CV_8UC3);
	  //create black image with 2D white bounding box inside that should contain objects
	  cv::rectangle(black,cv::Rect(x_box,y_box,width_box,height_box),cv::Scalar(255,255,255),-1,8,0);
	  //perform bitwise_and to remove outside area
	  cv::bitwise_and(temp_img,black,segment_img);
	  mask = cv::Mat::ones(img.size(),CV_32F)*255.;
	  for (int h=0; h<img.rows; h++)
	  {
		  for (int w=0; w<img.cols; w++)
	                 {
	                	 if((segment_img.at<cv::Vec3b>(h,w)[0] ==0) && (segment_img.at<cv::Vec3b>(h,w)[1] ==0) && (segment_img.at<cv::Vec3b>(h,w)[2] ==0))
	                	 {
	                		 mask.at<float>(h,w) = 0.;
	                	 }
	                 }
	  }

	   int dilation_size = 2;
	          int dilation_type = cv::MORPH_RECT;//MORPH_RECT MORPH_CROSS  MORPH_ELLIPSE;
	          cv::Mat element = cv::getStructuringElement( dilation_type, cv::Size( 2*dilation_size + 1, 2*dilation_size+1 ),
	           cv::Point( dilation_size, dilation_size ) );
	         // cv::dilate( mask, mask, element );
	          cv::morphologyEx(mask,mask,cv::MORPH_OPEN,element); //remove outliers
	          cv::morphologyEx(mask,mask,cv::MORPH_CLOSE,element); //fill holes inside objects
	        //  cv::imwrite(mask_file,mask);
	if(debug)
	{
			cv::imshow("mask",mask);
		//	cv::imshow("segment_img",segment_img);
			//cv::imshow("temp_img",temp_img);
			cv::waitKey(0);
	}
}
*/

void Segmentation::euclideanCluster(pcl::PointCloud<pcl::PointXYZRGB>::Ptr& cloud, int min, int max, double tolerance,pcl::PointCloud<pcl::PointXYZRGB>::Ptr & output)
{	
    std::vector<pcl::PointCloud<pcl::PointXYZRGB>::Ptr > output_clouds;
// Creating the KdTree object for the search method of the extraction
  	    pcl::search::KdTree<pcl::PointXYZRGB>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZRGB>);
	    tree->setInputCloud(cloud);
 std::vector<pcl::PointIndices> cluster_indices;
 pcl::EuclideanClusterExtraction<pcl::PointXYZRGB> ec;
 ec.setClusterTolerance (tolerance);
 ec.setMinClusterSize (min);
 ec.setMaxClusterSize (max);
 ec.setSearchMethod (tree);
 ec.setInputCloud (cloud);
 ec.extract (cluster_indices);
 pcl::console::print_value ("%d", cluster_indices.size ()); pcl::console::print_info (" clusters]\n");

 for (std::vector<pcl::PointIndices>::const_iterator it = cluster_indices.begin (); it != cluster_indices.end (); ++it)
   {
     pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_cluster (new pcl::PointCloud<pcl::PointXYZRGB>);
     for (std::vector<int>::const_iterator pit = it->indices.begin (); pit != it->indices.end (); ++pit)
       cloud_cluster->points.push_back (cloud->points[*pit]); //*
     cloud_cluster->width = cloud_cluster->points.size ();
     cloud_cluster->height = 1;
     cloud_cluster->is_dense = false;
     pcl::console::print_value ("%d \n",cloud_cluster->size());
     output_clouds.push_back(cloud_cluster);
   }
 int temp =0;
 std::cout<<"threshold "<<config_->clusterThreshold()<<std::endl;

    if(output_clouds.size()==1)
        *output = *output_clouds[0];
    /*
    else
    {
        for(int i=0;i<output_clouds.size();i++)
        {
            if(output_clouds[i]->points.size()>temp)
            {
                temp = output_clouds[i]->points.size();
                *output = *output_clouds[i];
            }
        }
    }
    */

    else
    {
        output.reset(new pcl::PointCloud<pcl::PointXYZRGB>);
        std::cout<<" output size: "<<output->points.size()<<std::endl;
        for(int i=0;i<output_clouds.size();i++)
        {
            if(output_clouds[i]->points.size()>config_->clusterThreshold())
            {
                std::cout<<"cloud size: "<<output_clouds[i]->points.size()<<std::endl;
                *output += *output_clouds[i];
                std::cout<<" output size: "<<output->points.size()<<std::endl;
            }
            else continue;
        }
    }

}

















