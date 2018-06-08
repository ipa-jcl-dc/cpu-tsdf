#include "loader.hpp"

#include <pcl/console/print.h>
#include <pcl/io/pcd_io.h>
#include <boost/make_shared.hpp>


boost::shared_ptr<pcl::PointCloud<pcl::PointXYZRGB> >
Loader::loadPointCloud(std::string filename, std::string suffix) {

    pcl::PointCloud<pcl::PointXYZRGB>::Ptr output (new pcl::PointCloud<pcl::PointXYZRGB>);
    filename.append (suffix);

    pcl::io::loadPCDFile (filename, *output);
    pcl::console::print_info ("loaded %s \n", filename.c_str (), output->size ());

    return (output);
}

void Loader::loadClouds(std::vector<pcl::PointCloud<pcl::PointXYZRGB>::Ptr>& cloud_data, const std::string& root_dir)
{
    std::cout<<"reading clouds"<<std::endl;

    std::ifstream index;
    index.open(std::string(root_dir+"input_cloud.txt").c_str());
    while(!index.eof())
    {
      pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
      std::string file;
      index>>file;
      if(file.empty())
        break;
      file = root_dir+file;
      if(pcl::io::loadPCDFile (file, *cloud)==-1){
          PCL_ERROR ("Couldn't read file %s \n",file.c_str());
             throw std::runtime_error(std::string("Failed: ") + file.c_str());
      }
      //else  pcl::console::print_info ("loaded %s \n", file.c_str (), cloud->size ());

      cloud_data.push_back(cloud);
     }
    std::cout<<"done !"<<std::endl;

}
pcl::PointCloud<pcl::PointXYZRGB>::Ptr Loader::loadPoints(std::string filename) {

    //create empty point cloud
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr
            output = boost::make_shared<pcl::PointCloud<pcl::PointXYZRGB> >();

    filename.append ("_points.pcd");

    //create new *_points.pcd file if not already on filesystem
    //pcl::io::savePCDFile(filename, *output);

    pcl::io::loadPCDFile (filename, *output);
    pcl::console::print_info ("loaded %s \n", filename.c_str (), output->size ());

    return (output);
}


pcl::PointCloud<pcl::FPFHSignature33>::Ptr Loader::loadLocalDescriptors(std::string filename) {

    pcl::PointCloud<pcl::FPFHSignature33>::Ptr
            output = boost::make_shared<pcl::PointCloud<pcl::FPFHSignature33> >();

    filename.append ("_localdesc.pcd");

    pcl::io::loadPCDFile (filename, *output);
    pcl::console::print_info ("loaded %s and generated (%zu points)\n", filename.c_str (), output->size ());

    return (output);
}


pcl::PointCloud<pcl::VFHSignature308>::Ptr Loader::loadGlobalDescriptors(std::string filename) {

    pcl::PointCloud<pcl::VFHSignature308>::Ptr
            output = boost::make_shared<pcl::PointCloud<pcl::VFHSignature308> >();

    filename.append ("_globaldesc.pcd");

    pcl::io::loadPCDFile (filename, *output);
    pcl::console::print_info ("loaded %s and generated (%zu points)\n", filename.c_str (), output->size ());

    return (output);
}


pcl::PointCloud<pcl::Normal>::Ptr Loader::loadSurfaceNormals(std::string filename) {

    pcl::PointCloud<pcl::Normal>::Ptr
            output = boost::make_shared<pcl::PointCloud<pcl::Normal> >();

    filename.append ("_normals.pcd");

    pcl::io::loadPCDFile (filename, *output);
    pcl::console::print_info ("loaded %s and generated (%zu points)\n", filename.c_str (), output->size ());

    return (output);
}


pcl::PointCloud<pcl::PointXYZRGB>::Ptr Loader::loadKeypoints(std::string filename) {

    pcl::PointCloud<pcl::PointXYZRGB>::Ptr
            output = boost::make_shared<pcl::PointCloud<pcl::PointXYZRGB> >();

    filename.append ("_keypoints.pcd");

    pcl::io::loadPCDFile (filename, *output);
    pcl::console::print_info ("loaded %s and generated (%zu points)\n", filename.c_str (), output->size ());

    return (output);
}
/// Load images
///
cv::Mat Loader::readColorImage(std::string filename,std::string suffix)
{
  cv::Mat img;
  std::string imagefile;
  imagefile = filename;
  imagefile.append (suffix);
  img = cv::imread(imagefile,-1);
  return img;
}

void Loader::loadDepthImages(std::vector<cv::Mat>& images, const std::string& root_dir)
{
    std::cout<<"reading depth images"<<std::endl;
std::ifstream index;
index.open(std::string(root_dir+"input_image.txt").c_str());
while(!index.eof())
{
cv::Mat img;
std::string file;
index>>file;
if(file.empty())
   break;
file = root_dir+"/depth-"+file;
img = cv::imread(file,2);
images.push_back(img);
}
std::cout<<"Read "<<images.size()<<"depth images"<<std::endl;

}
void Loader::loadImages(std::vector<cv::Mat>& images, const std::string& root_dir)
{
    std::cout<<"reading color images"<<std::endl;

std::ifstream index;
index.open(std::string(root_dir+"input_image.txt").c_str());
while(!index.eof())
{
cv::Mat img;
std::string file;
index>>file;
if(file.empty())
   break;
file = root_dir+"/color-"+file;
std::cout<<"reading "<<file<<std::endl;
img = cv::imread(file,1);
images.push_back(img);
}
std::cout<<"Read "<<images.size()<<"color images"<<std::endl;

}
//Read Parameters from XML

unsigned long Loader::ReadParameters(std::string directory_and_filename,
             pcl::PointXYZRGB& min_point, pcl::PointXYZRGB& max_point,double&  max_correspondence_distance)
{
    TiXmlDocument doc(directory_and_filename);
        doc.LoadFile();
        TiXmlElement *l_pRootElement = NULL;
            l_pRootElement = doc.RootElement();

        if (l_pRootElement)
        {
            //Bounding box for pass-through segmentation
            TiXmlElement *BoundingBox = NULL;
            BoundingBox = l_pRootElement->FirstChildElement("BoundingBox");
            if (BoundingBox)
            {

                TiXmlElement *Size_label = BoundingBox->FirstChildElement("threshold");

                Size_label->QueryFloatAttribute("mixX", &min_point.x);
                Size_label->QueryFloatAttribute("maxX", &max_point.x);
                Size_label->QueryFloatAttribute("minY", &min_point.y);
                Size_label->QueryFloatAttribute("maxY", &max_point.y);
                Size_label->QueryFloatAttribute("minZ", &min_point.z);
                Size_label->QueryFloatAttribute("maxZ", &max_point.z);
            }
            else
            {
                std::cerr << "\t ... Can't find BoundingBox for segmetation, use defaut size'" << std::endl;
            }
            //Parameters for registration
            TiXmlElement *Registration = NULL;
            Registration = l_pRootElement->FirstChildElement("Registration");
            if(Registration)
            {
                TiXmlElement *parameters =  Registration->FirstChildElement("distance");
                parameters->QueryDoubleAttribute("max_distance",&max_correspondence_distance);
            }

        }
        std::cout<<"read parameters from XML file is done"<<std::endl;
        return EXIT_SUCCESS;
}














/***************************************************************************************************************/
/************************************************SEPERATOR******************************************************/
/***************************************************************************************************************/


int Saver::saveGlobalDescriptors(std::string filename, pcl::PointCloud<pcl::VFHSignature308>::Ptr &signature) {
    int err = 0;

    filename.append ("_globaldesc.pcd");

    err = pcl::io::savePCDFile(filename, *signature);

    return (err);
}


int Saver::saveLocalDescriptors(std::string filename, pcl::PointCloud<pcl::FPFHSignature33>::Ptr &signature) {
    int err = 0;

    filename.append ("_localdesc.pcd");

    err = pcl::io::savePCDFile(filename, *signature);

    return (err);
}


int Saver::saveKeypoints(std::string filename, pcl::PointCloud<pcl::PointXYZRGB>::Ptr &keypoints) {
    int err = 0;

    filename.append ("_keypoints.pcd");

    err = pcl::io::savePCDFile(filename, *keypoints);

    return (err);
}


int Saver::saveSurfaceNormals(std::string filename, pcl::PointCloud<pcl::Normal>::Ptr &normals) {
    int err = 0;

    filename.append ("_normals.pcd");

    err = pcl::io::savePCDFile(filename, *normals);

    return (err);
}


int Saver::savePoints(std::string filename, pcl::PointCloud<pcl::PointXYZRGB>::Ptr &points) {
    int err = 0;

    filename.append ("_points.pcd");

    err = pcl::io::savePCDFile(filename, *points);

    return (err);
}

int Saver::saveObjectFeatures(std::string filename, boost::shared_ptr<Features::ObjectFeatures> &objFeatures) {
    int err = 0;

    return (err);
}


