#include "kinect_fusion.hpp"
void Kinect::showCameras (const pcl::texture_mapping::CameraVector& cams,const pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud)
{
    // visualization object
     pcl::visualization::PCLVisualizer visu ("cameras");

     // add a visual for each camera at the correct pose
     for(int i = 0 ; i < cams.size () ; ++i)
     {
       // read current camera
       pcl::TextureMapping<pcl::PointXYZ>::Camera cam = cams[i];
       double focal = cam.focal_length;
       double height = cam.height;
       double width = cam.width;

       // create a 5-point visual for each camera
       pcl::PointXYZ p1, p2, p3, p4, p5;
       p1.x=0; p1.y=0; p1.z=0; //origin point
       //double angleX = RAD2DEG (2.0 * atan (width / (2.0*focal)));
       //double angleY = RAD2DEG (2.0 * atan (height / (2.0*focal)));
       double dist = 0.75;
       double minX, minY, maxX, maxY;
       maxX = dist*tan (atan (width / (2.0*focal)));
       minX = -maxX;
       maxY = dist*tan (atan (height / (2.0*focal)));
       minY = -maxY;
       p2.x=minX; p2.y=minY; p2.z=dist;
       p3.x=maxX; p3.y=minY; p3.z=dist;
       p4.x=maxX; p4.y=maxY; p4.z=dist;
       p5.x=minX; p5.y=maxY; p5.z=dist;
       //Transform points from camera coordinate to world coordinate
       p1=pcl::transformPoint(p1, cam.pose);
       p2=pcl::transformPoint(p2, cam.pose);
       p3=pcl::transformPoint(p3, cam.pose);
       p4=pcl::transformPoint(p4, cam.pose);
       p5=pcl::transformPoint(p5, cam.pose);
       std::stringstream ss;
       ss << "Cam #" << i+1;
       visu.addText3D(ss.str (), p1, 0.1, 1.0, 1.0, 1.0, ss.str ());

       ss.str ("");
       ss << "camera_" << i << "line1";
       visu.addLine (p1, p2,ss.str ());
       ss.str ("");
       ss << "camera_" << i << "line2";
       visu.addLine (p1, p3,ss.str ());
       ss.str ("");
       ss << "camera_" << i << "line3";
       visu.addLine (p1, p4,ss.str ());
       ss.str ("");
       ss << "camera_" << i << "line4";
       visu.addLine (p1, p5,ss.str ());
       ss.str ("");
       ss << "camera_" << i << "line5";
       visu.addLine (p2, p5,ss.str ());
       ss.str ("");
       ss << "camera_" << i << "line6";
       visu.addLine (p5, p4,ss.str ());
       ss.str ("");
       ss << "camera_" << i << "line7";
       visu.addLine (p4, p3,ss.str ());
       ss.str ("");
       ss << "camera_" << i << "line8";
       visu.addLine (p3, p2,ss.str ());
     }

     // add a coordinate system
     visu.addCoordinateSystem (1.0, "global");

     // add the mesh's cloud (colored on Z axis)
     pcl::visualization::PointCloudColorHandlerGenericField<pcl::PointXYZ> color_handler (cloud, "z");
     visu.addPointCloud (cloud, color_handler, "cloud");

     // reset camera
     visu.resetCamera ();

     // wait for user input
     visu.spin ();
}
bool Kinect::readCamPose(std::string file_name, pcl::texture_mapping::CameraVector& cams,const std::vector<Eigen::Matrix4f>& transforms,
                         const std::vector<int>& pose_flags)
{
       pcl::texture_mapping::Camera cam;
        for(int i=0;i<transforms.size();i++)
       {
        if(pose_flags[i]==0) continue; //indicate that no pose of this frame

        //Default camera parameter for the SR300 that currently is using
        cam.focal_length = 619.69;
        cam.height = 480;
        cam.width = 640;
        cam.center_w = 313.013;
        cam.center_h = 245.14;
        std::ostringstream ss;
        if(i<10){ ss<<0<<i;}
        else ss<<i;
        // RGB image file that contain texture information, should be change when different types of file
        cam.texture_file = file_name + "frame-0000"+ss.str() +".color.png";
        cam.pose(0, 3) = transforms[i](0,3);
        cam.pose(1, 3) = transforms[i](1,3);
        cam.pose(2, 3) = transforms[i](2,3);

        cam.pose(0, 0) = transforms[i](0,0);
        cam.pose(0, 1) = transforms[i](0,1);
        cam.pose(0, 2) = transforms[i](0,2);

        cam.pose(1, 0) = transforms[i](1,0);
        cam.pose(1, 1) = transforms[i](1,1);
        cam.pose(1, 2) = transforms[i](1,2);

        cam.pose(2, 0) = transforms[i](2,0);
        cam.pose(2, 1) = transforms[i](2,1);
        cam.pose(2, 2) = transforms[i](2,2);

        cam.pose(3, 0) = 0.0;
        cam.pose(3, 1) = 0.0;
        cam.pose(3, 2) = 0.0;
        cam.pose(3, 3) = 1.0; //scale
        cams.push_back(cam);
        }
    return true;
}
pcl::PolygonMesh Kinect::createPolygonMesh(const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud)
{
    // Normal estimation of given point cloud
      pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> n;
      pcl::PointCloud<pcl::Normal>::Ptr normals (new pcl::PointCloud<pcl::Normal>);
      pcl::search::KdTree<pcl::PointXYZ>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZ>);
      tree->setInputCloud (cloud);
      n.setInputCloud (cloud);
      n.setSearchMethod (tree);
      n.setKSearch (20);
      n.compute (*normals);
      // Concatenate the XYZ and normal fields*
      pcl::PointCloud<pcl::PointNormal>::Ptr cloud_with_normals (new pcl::PointCloud<pcl::PointNormal>);
      pcl::concatenateFields (*cloud, *normals, *cloud_with_normals);

      // Create search tree*
      pcl::search::KdTree<pcl::PointNormal>::Ptr tree2 (new pcl::search::KdTree<pcl::PointNormal>);
      tree2->setInputCloud (cloud_with_normals);

      // Initialize objects
      pcl::GreedyProjectionTriangulation<pcl::PointNormal> gp3;
      pcl::PolygonMesh triangles;

      // Set the maximum distance between connected points (maximum edge length)
      gp3.setSearchRadius (0.025);
      // Set typical values for the parameters
      gp3.setMu (2.5);
      gp3.setMaximumNearestNeighbors (100);
      gp3.setMaximumSurfaceAngle(M_PI/4); // 45 degrees
      gp3.setMinimumAngle(M_PI/18); // 10 degrees
      gp3.setMaximumAngle(2*M_PI/3); // 120 degrees
      gp3.setNormalConsistency(false);
      gp3.setConsistentVertexOrdering(true);
        // Get result
      gp3.setInputCloud(cloud_with_normals);
      gp3.setSearchMethod(tree2);
      gp3.reconstruct(triangles);

    return triangles;
}
void Kinect::textureMapping(const pcl::PolygonMesh& triangles, const pcl::texture_mapping::CameraVector& cams,std::string mesh_file)
{
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);
    pcl::fromPCLPointCloud2(triangles.cloud, *cloud);
    //Create a texture mesh object that will contain UV-mapped mesh
    pcl::TextureMesh mesh;
    mesh.cloud = triangles.cloud;
    std::vector<pcl::Vertices> polygon;
     // push faces into the texturemesh object
    polygon.resize(triangles.polygons.size());
    for(size_t i=0;i<triangles.polygons.size();i++)
    {
         polygon[i] = triangles.polygons[i];
    }
    mesh.tex_polygons.push_back(polygon);
    // Load textures and cameras poses and intrinsics
     PCL_INFO ("\nLoading textures and camera poses...\n");
     // Create materials for each texture (and one extra for occluded faces)
       mesh.tex_materials.resize (cams.size () + 1);
       for(int i = 0 ; i <= cams.size() ; ++i)
       {
         pcl::TexMaterial mesh_material;
         mesh_material.tex_Ka.r = 0.2f;
         mesh_material.tex_Ka.g = 0.2f;
         mesh_material.tex_Ka.b = 0.2f;

         mesh_material.tex_Kd.r = 0.8f;
         mesh_material.tex_Kd.g = 0.8f;
         mesh_material.tex_Kd.b = 0.8f;

         mesh_material.tex_Ks.r = 1.0f;
         mesh_material.tex_Ks.g = 1.0f;
         mesh_material.tex_Ks.b = 1.0f;

         mesh_material.tex_d = 1.0f;
         mesh_material.tex_Ns = 75.0f;
         mesh_material.tex_illum = 2;

         std::stringstream tex_name;
         tex_name << "material_" << i;
         tex_name >> mesh_material.tex_name;

         if(i < cams.size ())
           mesh_material.tex_file = cams[i].texture_file;
         else
           mesh_material.tex_file = "occluded.jpg";

         mesh.tex_materials[i] = mesh_material;
       }
       // Sort faces
       PCL_INFO ("\nSorting faces by cameras...\n");
     pcl::TextureMapping<pcl::PointXYZ> texture_mapping; // TextureMapping object that will perform the sort
     texture_mapping.textureMeshwithMultipleCameras(mesh, cams);
     PCL_INFO ("Sorting faces by cameras done.\n");
     for(int i = 0 ; i <= cams.size() ; ++i)
     {
       PCL_INFO ("\tSub mesh %d contains %d faces and %d UV coordinates.\n", i, mesh.tex_polygons[i].size (), mesh.tex_coordinates[i].size ());
     }
     // compute normals for the mesh
             PCL_INFO ("\nEstimating normals...\n");
             pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> n;
             pcl::search::KdTree<pcl::PointXYZ>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZ>);
             pcl::PointCloud<pcl::Normal>::Ptr normals_mesh (new pcl::PointCloud<pcl::Normal>);
             tree->setInputCloud (cloud);
             n.setInputCloud (cloud);
             n.setSearchMethod (tree);
             n.setKSearch (20);
             n.compute (*normals_mesh);
             // Concatenate XYZ and normal fields
             pcl::PointCloud<pcl::PointNormal>::Ptr cloud_with_normals_mesh (new pcl::PointCloud<pcl::PointNormal>);
             pcl::concatenateFields (*cloud, *normals_mesh, *cloud_with_normals_mesh);
             PCL_INFO ("...Done.\n");
             pcl::toPCLPointCloud2 (*cloud_with_normals_mesh, mesh.cloud);
             PCL_INFO ("\nSaving mesh to textured_mesh.obj\n");
             pcl::io::saveOBJFile(mesh_file,mesh,5);
}
