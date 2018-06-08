#pragma once

#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/surface/poisson.h>
#include <pcl/filters/crop_box.h>
#include <pcl/filters/radius_outlier_removal.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/filters/filter.h>
#include <pcl/surface/mls.h>
#include <pcl/surface/impl/mls.hpp>
#include <pcl/features/normal_3d_omp.h>
#include <pcl/search/kdtree.h>
#include <pcl/common/common.h>
#include <iostream>
#include <limits>

/**
 * This class includes various methods for filtering the point cloud
 *
 */
class Filters{

public:

    /** Use a PassThrough filter to remove points with depth values that are too large or too small in "axis" direction */
    void
    threshold(pcl::PointCloud<pcl::PointXYZRGB>& input,const char* axis, float min_value, float max_value);
    void
    threshold(const pcl::PointCloud<pcl::PointXYZRGB> &input, const char *axis,
                            float min_value, float max_value, std::vector<int> &indices,
                            pcl::PointCloud<pcl::PointXYZRGB>& output);
    ////
    /// \brief CropBox : crop point cloud inside a cube
    /// \param input point cloud
    /// \param min_point x,y,z directions
    /// \param max_point x,y,z directions
    ///
    void
    CropBox(const pcl::PointCloud<pcl::PointXYZRGB>::Ptr& input,float min_x,float min_y,float min_z,
            float max_x,float max_y,float max_z,std::vector<int>& indices);
    ////
    /// \brief CropBox crop cloud inside a cube
    /// \param input cloud
    /// \param min_point x,y,z directions
    /// \param max_point x,y,z directions
    ///

    void
    CropBox(pcl::PointCloud<pcl::PointXYZRGB>::Ptr& input,float min_x,float min_y,float min_z,
            float max_x,float max_y,float max_z);
    void
    CropBox(pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr& input,float min_x,float min_y,float min_z,
            float max_x,float max_y,float max_z);
/** Use a VoxelGrid filter to reduce the number of points */
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr
    voxelize(const pcl::PointCloud<pcl::PointXYZRGB>::Ptr &input, float leaf_size);
/** Use a StatisticalOutlierRemoval filter to remove all points with too few local neighbors */
    void
    removeOutliers (pcl::PointCloud<pcl::PointXYZRGB>::Ptr &input, float radius, int min_neighbors);
    void
    removeOutliers(pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr& input, float radius, int meanK);
    void
    removeOutliers(pcl::PointCloud<pcl::PointNormal>::Ptr& input, float radius, int meanK);

/** remove all NaN values from Normals */
    pcl::PointCloud<pcl::PointNormal>::Ptr
    removeNaNNormals (const pcl::PointCloud<pcl::PointNormal>::Ptr &inputNormal, const std::string filename);
/*  MovingLeastSquares represent an implementation of the MLS (Moving Least Squares) algorithm
    *for data smoothing and improved normal estimation. It also contains methods for upsampling the
    * resulting cloud based on the parametric fit.
    * Output has the PointNormal type in order to store the normals calculated by MLS
    *
*/
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr
    movingLeastSquares(const pcl::PointCloud<pcl::PointXYZRGB>::Ptr& cloud,float radius);
/*
 * Create mesh of point clouds using poisson
 */
    pcl::PolygonMesh surfaceReconstruction(const pcl::PointCloud<pcl::PointXYZRGB>::Ptr& cloud, const float radius);
private:

};
