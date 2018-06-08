#include "util.hpp"
#include <pcl/features/moment_of_inertia_estimation.h>
void Utils::save2ply (pcl::PointCloud<pcl::PointXYZRGB> cloud,const std::string name)
{
	int num = cloud.points.size ();
	  std::ofstream myfile;
	  myfile.open(name.c_str());
	  myfile << "ply" << std::endl;
	  myfile << "format ascii 1.0" << std::endl;
	  myfile << "comment manh.ha.hoang@ipa.fraunhofer.de" << std::endl;
	  myfile << "element vertex " << num << std::endl;
	  myfile << "property float x" << std::endl;
	  myfile << "property float y" << std::endl;
	  myfile << "property float z" << std::endl;
	  myfile << "property uchar red" << std::endl;
	  myfile << "property uchar green" << std::endl;
	  myfile << "property uchar blue" << std::endl;
	  myfile << "end_header" << std::endl;

	  for (int i = 0; i < num; i++)
	  {
	    myfile << float (cloud.points[i].x) << " " << float (cloud.points[i].y)
	        << " " << float (cloud.points[i].z) << " ";
	    myfile << int (cloud.points[i].r) << " " << int (cloud.points[i].g)
	        << " " << int (cloud.points[i].b) << " " << std::endl;
	  }
	myfile.close ();
}
void Utils::drawPose(const cv::Mat& rot, const cv::Mat& trans, const cv::Mat& image,const pcl::PointXYZRGB& min_point,
                    const pcl::PointXYZRGB& max_point)
{

	std::vector<cv::Point2f> cube_imagePoints;
		cv::Mat img = image.clone();

		float length = 0.1;
		vector<cv::Point3f> axisPoints;
		axisPoints.push_back(cv::Point3f(0,0,0));
		axisPoints.push_back(cv::Point3f(length,0,0));
		axisPoints.push_back(cv::Point3f(0,length,0));
		axisPoints.push_back(cv::Point3f(0,0,length));
		vector<cv::Point2f> imagePoints;

        cv::projectPoints(axisPoints,rot,trans,camera_matrix1_,cv::Mat(),imagePoints);
		// draw axis lines
		cv::line(img, imagePoints[0], imagePoints[1], cv::Scalar(0, 0, 255), 3);
		cv::line(img, imagePoints[0], imagePoints[2], cv::Scalar(0, 255, 0), 3);
		cv::line(img, imagePoints[0], imagePoints[3], cv::Scalar(255, 0, 0), 3);


		std::vector<cv::Point3f> objectPoints; //coordinates of tag system
        objectPoints.push_back(cv::Point3f(max_point.x,max_point.y,0)); //C -0
        objectPoints.push_back(cv::Point3f(max_point.x,min_point.y,0)); //B-1
        objectPoints.push_back(cv::Point3f(min_point.x,max_point.y,0)); //D-2
        objectPoints.push_back(cv::Point3f(min_point.x,min_point.y,0));//A-3

        objectPoints.push_back(cv::Point3f(max_point.x,max_point.y,max_point.z)); //C1 -0
        objectPoints.push_back(cv::Point3f(max_point.x,min_point.y,max_point.z)); //B1 - 1
        objectPoints.push_back(cv::Point3f(min_point.x,max_point.y,max_point.z)); //D1-2
        objectPoints.push_back(cv::Point3f(min_point.x,min_point.y,max_point.z)); //A1 -3


	 //Project objectPoints into image coordinates
            cv::projectPoints( objectPoints,rot,trans, camera_matrix1_,cv::Mat(),cube_imagePoints);
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
		cv::imshow("pose",img);
		cv::waitKey(0);
}

Utils::Utils()
{
    viewer_ = (new pcl::visualization::PCLVisualizer("3D viewer_"));
    viewer_->setSize (480, 640);
    viewer_->setPosition (480, 200);
    viewer_->setBackgroundColor(255,255,255);

}
Utils::~Utils()
{

}
void Utils::viewCloud(const pcl::PointCloud<pcl::PointXYZRGB>::Ptr& cloud)
{

    viewer_->removeAllPointClouds();
    pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGB> rgb(cloud);
    viewer_->addPointCloud(cloud,rgb, "output");
    viewer_->spinOnce();
}
void Utils::viewCloud(const pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr& cloud)

{
    viewer_->removeAllPointClouds();
    pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGBNormal> rgb(cloud);
    viewer_->addPointCloud(cloud,rgb,"output");
    viewer_->spinOnce();
}
void Utils::viewCloud(const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud)

{
    viewer_->removeAllPointClouds();
    pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> rgb(cloud, 0, 255, 0);
    viewer_->addPointCloud(cloud,rgb,"output");
    viewer_->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, "sample cloud");
    viewer_->spinOnce();
}

void Utils::viewMesh(const pcl::PolygonMesh& mesh)
{
    viewer_->removeAllPointClouds();
    viewer_->addPolygonMesh(mesh,"meshes",0);
    viewer_->spinOnce();

}
void Utils::viewMesh(const pcl::TextureMesh& mesh)
{
    viewer_->removeAllPointClouds();
    viewer_->addTextureMesh(mesh,"texture mesh",0);
    viewer_->spinOnce();

}

void Utils::visual(Eigen::Matrix4f m,pcl::PointCloud<pcl::PointXYZRGB> cloud)
{
    viewer_->removeAllPointClouds();
    viewer_->addPointCloud(cloud.makeShared());
	Eigen::Matrix3f r;
	r<< m(0, 0), m(0, 1), m(0,2), m(1,0), m(1,1), m(1,2), m(2,0), m(2,1), m(2,2);
	Eigen::Vector3f t(m(0,3),m(1,3),m(2,3));
    Eigen::Affine3f pose = viewer_->getViewerPose();
	pose.linear()=r;
	pose.translation()=t;
	Eigen::Vector3f pos_vector = pose * Eigen::Vector3f (0, 0, 0);
	Eigen::Vector3f look_at_vector = pose.rotation () * Eigen::Vector3f (0, 0, 1) + pos_vector;
	Eigen::Vector3f up_vector = pose.rotation () * Eigen::Vector3f (0, 1, 0);
    viewer_->setCameraPosition(pos_vector[0],pos_vector[1],pos_vector[2],look_at_vector[0],look_at_vector[1],look_at_vector[2],
	up_vector[0],up_vector[1],up_vector[2]);
    viewer_->spinOnce();
}
void Utils::bbox3DPCA(const pcl::PointCloud<pcl::PointXYZRGB>::Ptr& cloud)
{

   //Compute the 3x3 covariance matrix of a given set of points.
   // The result is returned as a Eigen::Matrix3f.
   // Note: the covariance matrix is not normalized with the number of points
   //
   pcl::PointXYZRGB min_point,max_point;
   Eigen::Vector4f pcaCentroid;
   pcl::compute3DCentroid(*cloud,pcaCentroid);
   Eigen::Matrix3f covariance;
   pcl::computeCovarianceMatrixNormalized(*cloud,pcaCentroid,covariance);
   //Compute eigenvectors and eigenvalues of covariance matrix using Eigen
   Eigen::SelfAdjointEigenSolver<Eigen::Matrix3f> eigen_solver(covariance, Eigen::ComputeEigenvectors);
   //Eigen vectors
   Eigen::Matrix3f eigenVectorsPCA = eigen_solver.eigenvectors();
   // Extract the eigenvalues and eigenvectors using PCL
   Eigen::Vector3f eigen_values;
   Eigen::Matrix3f eigen_vectors;
   pcl::eigen33 (covariance, eigen_vectors, eigen_values);

   /// This line is necessary for proper orientation in some cases. The numbers come out the same without it, but
/// the signs are different and the box doesn't get correctly oriented in some cases.
   eigenVectorsPCA.col(2) = eigenVectorsPCA.col(0).cross(eigenVectorsPCA.col(1));

   // Transform the original cloud to the origin point where the principal components correspond to the axes.
   Eigen::Matrix4f projectionTransform(Eigen::Matrix4f::Identity());
   projectionTransform.block<3,3>(0,0) = eigenVectorsPCA.transpose();
   projectionTransform.block<3,1>(0,3) = -1.f * (projectionTransform.block<3,3>(0,0) * pcaCentroid.head<3>());

   pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloudPointsProjected (new pcl::PointCloud<pcl::PointXYZRGB>);
   pcl::transformPointCloud(*cloud,*cloudPointsProjected,projectionTransform);
   // Get the minimum and maximum points of the transformed cloud.
   pcl::getMinMax3D(*cloudPointsProjected, min_point, max_point);
   const Eigen::Vector3f meanDiagonal = 0.5f*(max_point.getVector3fMap() + min_point.getVector3fMap());
//Finally, the quaternion is calculated using the eigenvectors (which determines how the final box gets rotated),
//and the transform to put the box in correct location is calculated.
//The minimum and maximum points are used to determine the box width, height, and depth.
   const Eigen::Quaternionf bboxQuaternion(eigenVectorsPCA); //Quaternions are a way to do rotations
   //translation
   const Eigen::Vector3f bboxTransform = eigenVectorsPCA * meanDiagonal + pcaCentroid.head<3>();
   //viewer_->addCube(bboxTransform, bboxQuaternion, max_point.x - min_point.x, max_point.y - min_point.y, max_point.z - min_point.z);
  //  viewer_->addCube(min_point.x, max_point.x, min_point.y,max_point.y, min_point.z ,max_point.z,1.0,0.0,0.0);


    //Add pose
    pcl::PointXYZ center (bboxTransform(0), bboxTransform(1), bboxTransform(2));
    pcl::PointXYZ x_axis= pcl::PointXYZ(eigenVectorsPCA.col(0)(0) + bboxTransform(0), eigenVectorsPCA.col(0)(1) + bboxTransform(1), eigenVectorsPCA.col(0)(2) + bboxTransform(2));
    pcl::PointXYZ y_axis= pcl::PointXYZ(eigenVectorsPCA.col(1)(0) + bboxTransform(0), eigenVectorsPCA.col(1)(1) + bboxTransform(1), eigenVectorsPCA.col(1)(2) + bboxTransform(2));
    pcl::PointXYZ z_axis= pcl::PointXYZ(eigenVectorsPCA.col(2)(0) + bboxTransform(0), eigenVectorsPCA.col(2)(1) + bboxTransform(1), eigenVectorsPCA.col(2)(2) + bboxTransform(2));

    viewer_->addLine (center, x_axis, 1.0f, 0.0f, 0.0f, "major eigen vector"); //data varies a lot in this direction
    viewer_->addLine (center, y_axis, 0.0f, 1.0f, 0.0f, "middle eigen vector");
    viewer_->addLine (center, z_axis, 0.0f, 0.0f, 1.0f, "minor eigen vector"); //data varies less in this direction
    /*
    //Add Cube
    Eigen::Vector3f p1 (min_point.x, min_point.y, min_point.z);
    Eigen::Vector3f p2 (min_point.x, min_point.y, max_point.z);
    Eigen::Vector3f p3 (max_point.x, min_point.y, max_point.z);
    Eigen::Vector3f p4 (max_point.x, min_point.y, min_point.z);
    Eigen::Vector3f p5 (min_point.x, max_point.y, min_point.z);
    Eigen::Vector3f p6 (min_point.x, max_point.y, max_point.z);
    Eigen::Vector3f p7 (max_point.x, max_point.y, max_point.z);
    Eigen::Vector3f p8 (max_point.x, max_point.y, min_point.z);


    p1 = eigenVectorsPCA * p1 + bboxTransform;
    p2 = eigenVectorsPCA * p2 + bboxTransform;
    p3 = eigenVectorsPCA * p3 + bboxTransform;
    p4 = eigenVectorsPCA * p4 + bboxTransform;
    p5 = eigenVectorsPCA * p5 + bboxTransform;
    p6 = eigenVectorsPCA * p6 + bboxTransform;
    p7 = eigenVectorsPCA * p7 + bboxTransform;
    p8 = eigenVectorsPCA * p8 + bboxTransform;

    pcl::PointXYZ pt1 (p1 (0), p1 (1), p1 (2));
    pcl::PointXYZ pt2 (p2 (0), p2 (1), p2 (2));
    pcl::PointXYZ pt3 (p3 (0), p3 (1), p3 (2));
    pcl::PointXYZ pt4 (p4 (0), p4 (1), p4 (2));
    pcl::PointXYZ pt5 (p5 (0), p5 (1), p5 (2));
    pcl::PointXYZ pt6 (p6 (0), p6 (1), p6 (2));
    pcl::PointXYZ pt7 (p7 (0), p7 (1), p7 (2));
    pcl::PointXYZ pt8 (p8 (0), p8 (1), p8 (2));

    viewer_->addLine (pt1, pt2, 1.0, 0.0, 0.0, "1 edge");
    viewer_->addLine (pt1, pt4, 1.0, 0.0, 0.0, "2 edge");
    viewer_->addLine (pt1, pt5, 1.0, 0.0, 0.0, "3 edge");
    viewer_->addLine (pt5, pt6, 1.0, 0.0, 0.0, "4 edge");
    viewer_->addLine (pt5, pt8, 1.0, 0.0, 0.0, "5 edge");
    viewer_->addLine (pt2, pt6, 1.0, 0.0, 0.0, "6 edge");
    viewer_->addLine (pt6, pt7, 1.0, 0.0, 0.0, "7 edge");
    viewer_->addLine (pt7, pt8, 1.0, 0.0, 0.0, "8 edge");
    viewer_->addLine (pt2, pt3, 1.0, 0.0, 0.0, "9 edge");
    viewer_->addLine (pt4, pt8, 1.0, 0.0, 0.0, "10 edge");
    viewer_->addLine (pt3, pt4, 1.0, 0.0, 0.0, "11 edge");
    viewer_->addLine (pt3, pt7, 1.0, 0.0, 0.0, "12 edge");
    */


}

void Utils::bbox3DInertia(const pcl::PointCloud<pcl::PointXYZRGB>::Ptr& cloud)
{
    pcl::MomentOfInertiaEstimation <pcl::PointXYZRGB> feature_extractor;
    feature_extractor.setInputCloud (cloud);
    feature_extractor.compute ();

    std::vector <float> moment_of_inertia;
    std::vector <float> eccentricity;
    pcl::PointXYZRGB min_point_AABB;
    pcl::PointXYZRGB max_point_AABB;
    pcl::PointXYZRGB min_point_OBB;
    pcl::PointXYZRGB max_point_OBB;
    pcl::PointXYZRGB position_OBB;
    Eigen::Matrix3f rotational_matrix_OBB;
    float major_value, middle_value, minor_value;
    Eigen::Vector3f major_vector, middle_vector, minor_vector;
    Eigen::Vector3f mass_center;

    feature_extractor.getMomentOfInertia (moment_of_inertia);
    feature_extractor.getEccentricity (eccentricity);
    feature_extractor.getAABB (min_point_AABB, max_point_AABB);
    feature_extractor.getOBB (min_point_OBB, max_point_OBB, position_OBB, rotational_matrix_OBB);
    feature_extractor.getEigenValues (major_value, middle_value, minor_value);
    feature_extractor.getEigenVectors (major_vector, middle_vector, minor_vector);
    feature_extractor.getMassCenter (mass_center);

    Eigen::Vector3f position (position_OBB.x, position_OBB.y, position_OBB.z);
    Eigen::Quaternionf quat (rotational_matrix_OBB);
  //  cout<<"min point "<<min_point_OBB<<"max point"<<max_point_OBB<<endl;
   // cout<<" mass center " <<mass_center<<endl;
   // cout<<"eigen vectors"<<major_vector<<endl<<middle_vector<<endl<<minor_vector<<endl;
    /*
    pcl::PointXYZ center (mass_center (0), mass_center (1), mass_center (2));
    pcl::PointXYZ x_axis (major_vector (0) + mass_center (0), major_vector (1) + mass_center (1), major_vector (2) + mass_center (2));
    pcl::PointXYZ y_axis (middle_vector (0) + mass_center (0), middle_vector (1) + mass_center (1), middle_vector (2) + mass_center (2));
    pcl::PointXYZ z_axis (minor_vector (0) + mass_center (0), minor_vector (1) + mass_center (1), minor_vector (2) + mass_center (2));
*/

    //viewer_->addLine (center, x_axis, 1.0f, 0.0f, 0.0f, "major eigen vector"); //data varies a lot in this direction
    //viewer_->addLine (center, y_axis, 0.0f, 1.0f, 0.0f, "middle eigen vector");
    //viewer_->addLine (center, z_axis, 0.0f, 0.0f, 1.0f, "minor eigen vector"); //data varies less in this direction

    Eigen::Vector3f p1 (min_point_OBB.x, min_point_OBB.y, min_point_OBB.z);
    Eigen::Vector3f p2 (min_point_OBB.x, min_point_OBB.y, max_point_OBB.z);
    Eigen::Vector3f p3 (max_point_OBB.x, min_point_OBB.y, max_point_OBB.z);
    Eigen::Vector3f p4 (max_point_OBB.x, min_point_OBB.y, min_point_OBB.z);
    Eigen::Vector3f p5 (min_point_OBB.x, max_point_OBB.y, min_point_OBB.z);
    Eigen::Vector3f p6 (min_point_OBB.x, max_point_OBB.y, max_point_OBB.z);
    Eigen::Vector3f p7 (max_point_OBB.x, max_point_OBB.y, max_point_OBB.z);
    Eigen::Vector3f p8 (max_point_OBB.x, max_point_OBB.y, min_point_OBB.z);


    p1 = rotational_matrix_OBB * p1 + position;
    p2 = rotational_matrix_OBB * p2 + position;
    p3 = rotational_matrix_OBB * p3 + position;
    p4 = rotational_matrix_OBB * p4 + position;
    p5 = rotational_matrix_OBB * p5 + position;
    p6 = rotational_matrix_OBB * p6 + position;
    p7 = rotational_matrix_OBB * p7 + position;
    p8 = rotational_matrix_OBB * p8 + position;

    pcl::PointXYZ pt1 (p1 (0), p1 (1), p1 (2));
    pcl::PointXYZ pt2 (p2 (0), p2 (1), p2 (2));
    pcl::PointXYZ pt3 (p3 (0), p3 (1), p3 (2));
    pcl::PointXYZ pt4 (p4 (0), p4 (1), p4 (2));
    pcl::PointXYZ pt5 (p5 (0), p5 (1), p5 (2));
    pcl::PointXYZ pt6 (p6 (0), p6 (1), p6 (2));
    pcl::PointXYZ pt7 (p7 (0), p7 (1), p7 (2));
    pcl::PointXYZ pt8 (p8 (0), p8 (1), p8 (2));

    viewer_->addLine (pt1, pt2, 1.0, 0.0, 0.0, "1 edge");
    viewer_->addLine (pt1, pt4, 1.0, 0.0, 0.0, "2 edge");
    viewer_->addLine (pt1, pt5, 1.0, 0.0, 0.0, "3 edge");
    viewer_->addLine (pt5, pt6, 1.0, 0.0, 0.0, "4 edge");
    viewer_->addLine (pt5, pt8, 1.0, 0.0, 0.0, "5 edge");
    viewer_->addLine (pt2, pt6, 1.0, 0.0, 0.0, "6 edge");
    viewer_->addLine (pt6, pt7, 1.0, 0.0, 0.0, "7 edge");
    viewer_->addLine (pt7, pt8, 1.0, 0.0, 0.0, "8 edge");
    viewer_->addLine (pt2, pt3, 1.0, 0.0, 0.0, "9 edge");
    viewer_->addLine (pt4, pt8, 1.0, 0.0, 0.0, "10 edge");
    viewer_->addLine (pt3, pt4, 1.0, 0.0, 0.0, "11 edge");
    viewer_->addLine (pt3, pt7, 1.0, 0.0, 0.0, "12 edge");

}


void Utils::closeCloud(pcl::PointCloud<pcl::PointXYZRGB>::Ptr& cloud, const float slide_size)
{
     //closes the cloud by adding a plate at the bottom
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr plate(new pcl::PointCloud<pcl::PointXYZRGB>);
    //get the ring of points at the bottom of the cloud
    pcl::PassThrough<pcl::PointXYZRGB> pass;
    pass.setInputCloud (cloud);
    pass.setFilterFieldName ("z");
    pass.setFilterLimits (config_->minZ(),config_->minZ()+0.01);
    pass.filter (*plate);
    //cout<<"plate size: "<< plate->points.size()<<endl;
    //take slices of the ring and add points to fill in the cloud
    for(float x=config_->minX();x <= config_->maxX();x+=0.001)
    {
        pcl::PointCloud<pcl::PointXYZRGB> strip;
        pass.setInputCloud(plate);
        pass.setFilterFieldName ("x");
        pass.setFilterLimits (x, x+slide_size); //slide size
        pass.filter(strip);
        //std::cout<<strip.points.size()<<std::endl;

        if(strip.size()==0) {continue;} //no points, dont do anything

        //find the min and max y values of this strip
        float min = 9;
        float max = -9;
        for(int i = 0; i<strip.size(); i++)
        {
            float y = strip.points[i].y;
            if(y<min)
                  min = y;
             if(y>max)
                  max = y;
        }
        //add points from ymin to ymax for this strip
        for(float y = min; y<=max; y+=slide_size){
                  pcl::PointXYZRGB p;
                  p.x = x;
                  p.y = y;
                  p.z = config_->minZ();
                  //point black color
                  p.r =255;
                  p.g = 255;
                  p.b = 255;
                  cloud->push_back(p);
         }
    }
}

void Utils::closeTopCloud(pcl::PointCloud<pcl::PointXYZRGB>::Ptr& cloud)
{
    //Find min and max point of cloud
    pcl::PointXYZRGB min_point,max_point;
    pcl::getMinMax3D(*cloud, min_point, max_point);
    //Close cloud by adding top of the cloud if the camera is placed horizontially to object
    //So there is no point on the top of cloud
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr top (new pcl::PointCloud<pcl::PointXYZRGB>);
    //Pass through filter in Z direction
    pcl::PassThrough<pcl::PointXYZRGB> pass;
    pass.setInputCloud(cloud);
    pass.setFilterFieldName("z");
    pass.setFilterLimits(max_point.z-0.05,max_point.z);
    pass.filter (*top);
    //take slices of the ring and add points to fill in the cloud
    for(float x=config_->minX();x <= config_->maxX();x+=0.001)
    {
        pcl::PointCloud<pcl::PointXYZRGB> strip;
        pass.setInputCloud(top);
        pass.setFilterFieldName ("x");
        pass.setFilterLimits (x, x+.001); //slide size
        pass.filter(strip);

        if(strip.size()==0) {continue;} //no points, dont do anything

        //find the min and max y values of this strip
        float min = 9;
        float max = -9;
        for(int i = 0; i<strip.size(); i++)
        {
            float y = strip.points[i].y;
            if(y<min)
                  min = y;
             if(y>max)
                  max = y;
        }
        //add points from ymin to ymax for this strip
        for(float y = min; y<=max; y+=.001){
                  pcl::PointXYZRGB p;
                  p.x = x;
                  p.y = y;
                  p.z = max_point.z;
                  //point black color
                  p.r = 255;
                  p.g = 255;
                  p.b = 255;
                  cloud->push_back(p);
         }
    }
}


void Utils::isStopped()
{
    while (!viewer_->wasStopped ())
		   {

            viewer_->spin ();
		     boost::this_thread::sleep (boost::posix_time::microseconds (100000));
		  }

}
void Utils::visualMatching(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_src, pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_tgt,
		pcl::PointCloud<pcl::PointXYZRGB>::Ptr keyPoint_src,pcl::PointCloud<pcl::PointXYZRGB>::Ptr
		keyPoint_tgt,pcl::CorrespondencesPtr correspondences)
{
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr previousCloudTrans(new pcl::PointCloud<pcl::PointXYZRGB>);
		pcl::PointCloud<pcl::PointXYZRGB>::Ptr previousKeypointsTrans(new pcl::PointCloud<pcl::PointXYZRGB>);
           boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer;
           viewer.reset(new pcl::visualization::PCLVisualizer);

		   pcl::transformPointCloud(*cloud_tgt, *previousCloudTrans, Eigen::Vector3f(-1, 0, 0), Eigen::Quaternionf(1, 0, 0, 0));
		   pcl::transformPointCloud(*keyPoint_tgt, *previousKeypointsTrans, Eigen::Vector3f(-1, 0, 0), Eigen::Quaternionf(1, 0, 0, 0));
		   cout << "previous keypoints: " << previousKeypointsTrans->size() << endl;
		   cout << "current keypoints: " << keyPoint_src->size() <<endl;
		   cout << "corres: " << correspondences->size() << endl;

           viewer->addPointCloud(cloud_src, "current cloud");
           viewer->addPointCloud(previousCloudTrans, "previous cloud");

		  pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZRGB> keypoints_handler1(keyPoint_src, 0, 0, 255);
		  pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZRGB> keypoints_handler2(keyPoint_tgt, 255, 0, 0);

          viewer->addPointCloud(keyPoint_src, keypoints_handler1, "currentkeypoints");
          viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 5, "currentkeypoints");

          viewer->addPointCloud(previousKeypointsTrans, keypoints_handler2, "previouskeypoints");
            viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 5, "previouskeypoints");


		    for (size_t i = 0; i < correspondences->size(); i++) {
		        stringstream line_ID;
		        line_ID << "correspondence_line_" << i;


               viewer->addLine<pcl::PointXYZRGB, pcl::PointXYZRGB>(keyPoint_src->at(correspondences->at(i).index_query),
		                                           previousKeypointsTrans->at(correspondences->at(i).index_match), 0, 255, 0, line_ID.str());
		      }

           while (!viewer->wasStopped ())
		   	   	          {
                      viewer->spinOnce ();
		   	   	       }
}




















