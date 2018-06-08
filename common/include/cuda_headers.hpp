#include "data_types.hpp"

namespace Cuda{
    void hostTSDFVolume(const cv::cuda::GpuMat& depth_map,
            const cv::cuda::GpuMat& color_map,
            VolumeData& volume,
            const CameraParameters& cam_params,
            const cv::cuda::GpuMat& cam2base,
            const float truncation_distance,
                        const float depth_cutoff,
                        const float original_distance_x,
                        const float original_distance_y,
                        const float original_distance_z
            );

    void hostPrintTSDF(const VolumeData& volume,float* tsdf_float);

    PointCloud hostExtractPointCloud(const VolumeData& volume,
                                            const int buffer_size,
                                            const float original_distance_x,
                                            const float original_distance_y,
                                            const float original_distance_z);
    SurfaceMesh hostMarchingCubes(const VolumeData& volume,
                            const int triangles_buffer_size,
                            const float original_distance_x,
                            const float original_distance_y,
                           const float original_distance_z);
}
