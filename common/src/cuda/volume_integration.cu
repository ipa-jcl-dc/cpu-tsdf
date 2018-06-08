
#include "data_types.hpp"

    namespace Cuda{
/**
 *  Kernel Volume: Compute TSDF Volume in device (GPU)
 *  depth_map: current depth image input
 *  color_map : current color image input
 *  tsdf_volume : output of tsdf volume
 *  color_volume : output of color volume
 *  rvect, tvec: transformation from current camera to base camera
 *  Detail can be found in Volumetric Representation chapter (listing 2)
 *  http://people.inf.ethz.ch/otmarh/download/Papers/p559-izadi(KinectFusion).pdf
 *
 */

__global__ void kernelTSDFVolume(const cv::cuda::PtrStepSz<float> depth_map,
								 const cv::cuda::PtrStepSz<uchar3> color_map,
                                 cv::cuda::PtrStepSz<float> tsdf_volume,
                                 cv::cuda::PtrStepSz<uchar3> color_volume,
                                 cv::cuda::PtrStepSz<float> weight_volume,
                                 const int volume_res,
                                 const float voxel_size,
                                 CameraParameters cam_params,
                                 const cv::cuda::PtrStepSz<float> cam2base,
                                 const float truncation_distance,
                                 const float depth_cutoff,
                                 const float original_distance_x,
                                 const float original_distance_y,
                                 const float original_distance_z
                                )
{
	//Get id of each thread in 2 dimensions
	const int pt_grid_x = blockIdx.x * blockDim.x + threadIdx.x;
	const int pt_grid_y = blockIdx.y * blockDim.y + threadIdx.y;


	//Out of bounding box
	if(pt_grid_x>=volume_res || pt_grid_y>= volume_res)
		return;

	//Search along z dimension
	for(int pt_grid_z=0;pt_grid_z<volume_res;++pt_grid_z)
	{
		// Convert voxel center from grid coordinates to base frame camera coordinates
        float pt_base_x =  original_distance_x + (pt_grid_x+0.5) * voxel_size;
        float pt_base_y =  original_distance_y + (pt_grid_y+0.5) * voxel_size;
        float pt_base_z =  original_distance_z + (pt_grid_z+0.5) * voxel_size;

		 // Convert from base frame camera coordinates to current frame camera coordinates
		float tmp_pt[3] = {0};
		tmp_pt[0] = pt_base_x - cam2base.ptr(0)[3];
		tmp_pt[1] = pt_base_y - cam2base.ptr(1)[3];
		tmp_pt[2] = pt_base_z - cam2base.ptr(2)[3];
		float pt_cam_x = cam2base.ptr(0)[0] * tmp_pt[0] + cam2base.ptr(1)[0] * tmp_pt[1] + cam2base.ptr(2)[0] * tmp_pt[2];
		float pt_cam_y = cam2base.ptr(0)[1] * tmp_pt[0] + cam2base.ptr(1)[1] * tmp_pt[1] + cam2base.ptr(2)[1] * tmp_pt[2];
		float pt_cam_z = cam2base.ptr(0)[2] * tmp_pt[0] + cam2base.ptr(1)[2] * tmp_pt[1] + cam2base.ptr(2)[2] * tmp_pt[2];

		// Possible alternatives ?
        //Eigen::Matrix<float, 3, 1, Eigen::DontAlign> camera_pos(pt_cam_x,pt_cam_y,pt_cam_z);


		if (pt_cam_z <= 0)
		    continue;

        //printf("%f \n",pt_cam_z);
        int pt_pix_x = __float2int_rn(cam_params.focal_x * (pt_cam_x / pt_cam_z) +  cam_params.c_x);
		int pt_pix_y = __float2int_rn(cam_params.focal_y * (pt_cam_y / pt_cam_z) +  cam_params.c_y);
		if (pt_pix_x < 0 || pt_pix_x >= cam_params.image_width || pt_pix_y < 0 || pt_pix_y >= cam_params.image_height)
		    continue;
        const float depth_value = depth_map.ptr(pt_pix_y)[pt_pix_x];

		if(depth_value<=0 || depth_value >=depth_cutoff)
			continue;
        //printf("%f \n",depth_value);
		//Possible alternatives ?
		//Back project pixel to camera coordinate with z = 1.0f meters
        //const float position_x = (pt_pix_x - cam_params.c_x)/cam_params.focal_x;
        //const float position_y = (pt_pix_y - cam_params.c_y)/cam_params.focal_y;
        //const Eigen::Matrix<float, 3, 1, Eigen::DontAlign> xylambda(position_x,position_y,1.0f);
		//compute euclidean distance of vector
        //const float lambda = xylambda.norm();
        //Compute sdf value,sdf =0 indicates that it is iso surface
        //const float sdf = (-1.f) * ((1.f / lambda) * camera_pos.norm() - depth_value);

		//Can be done like this, no difference, can be tested in printf
        const float sdf= depth_value - pt_cam_z;
		if (sdf <= -truncation_distance)
			continue;
        //printf("%f \n",sdf);
		 // Integrate
		//int volume_idx = pt_grid_z * volume_res * volume_res + pt_grid_y * volume_res + pt_grid_x;
	    float dist = fmin(1.0f, sdf / truncation_distance);
        //printf("%f \n",dist);
	    //Get current TSDF value and weight value of each index
	    const float current_tsdf = tsdf_volume.ptr(pt_grid_z*volume_res+pt_grid_y)[pt_grid_x];
	    float weight_old = weight_volume.ptr(pt_grid_z*volume_res+pt_grid_y)[pt_grid_x];
	    //update weight
	    float weight_new = weight_old+ 1.0f;
	    //Update weight and TSDF values
	    weight_volume.ptr(pt_grid_z*volume_res+pt_grid_y)[pt_grid_x] = weight_new;
	    tsdf_volume.ptr(pt_grid_z*volume_res+pt_grid_y)[pt_grid_x] = (current_tsdf * weight_old + dist)/weight_new;
        //printf("%f \n",tsdf_volume.ptr(pt_grid_z*volume_res+pt_grid_y)[pt_grid_x]);
	    //Update color volume	    	    
        uchar3& color = color_volume.ptr(pt_grid_z*volume_res+pt_grid_y)[pt_grid_x];
        const uchar3 color_value =  color_map.ptr(pt_pix_y)[pt_pix_x];
        color.x = static_cast<uchar>((color.x * weight_old +color_value.x )/weight_new);
        color.y = static_cast<uchar>((color.y * weight_old +color_value.y )/weight_new);
        color.z = static_cast<uchar>((color.z * weight_old +color_value.z)/weight_new);

	}
    
}

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
		)
{
        dim3 threads(WARP_SIZE,WARP_SIZE);
        dim3 blocks((volume.volume_res_ + threads.x -1)/threads.x,
					(volume.volume_res_ + threads.y -1)/threads.y);

        kernelTSDFVolume <<< blocks, threads >>> (depth_map,
				color_map,
				volume.tsdf_volume,
				volume.color_volume,
				volume.weight_volume,
				volume.volume_res_,
				volume.voxel_size_,
				cam_params,
				cam2base,
				truncation_distance,				
                depth_cutoff,
                original_distance_x,
                original_distance_y,
                original_distance_z
				);
     /*
        printf("%f \n",cam_params.c_x);
        printf("%f \n",cam_params.c_y);
        printf("%f \n",cam_params.focal_x);
        printf("%f \n",cam_params.focal_y);
        printf("%f \n",truncation_distance);
        printf("%f \n",depth_cutoff);
        printf("%f \n",original_distance_x);
        printf("%f \n",original_distance_y);
        printf("%f \n",original_distance_z);

       // for(int i=0;i<16;i++)
           // printf("%f \n",cam2base[i]);
           */
        CudaSafeCall ( cudaGetLastError () );
        CudaSafeCall (cudaDeviceSynchronize ());
}
}


