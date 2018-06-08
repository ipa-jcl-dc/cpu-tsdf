
// Thrust, for prefix scanning
#include <thrust/device_ptr.h>
#include <thrust/scan.h>
#include "data_types.hpp"
#include "marching_cubes_table.hpp"
/*-------------------------------------------------------------------------
   Given a grid cell and an isolevel, calculate the triangular
   facets required to represent the isosurface through the cell.
   Return the number of triangular facets, the array "triangles"
   will be loaded up with the vertices at most 5 triangular facets.
   0 will be returned if the grid cell is either totally above
   of totally below the isolevel.
*/
	namespace Cuda{

	__device__ int global_count =0;
	__device__ int output_count;
	__device__ unsigned int blocks_done =0;

	/*Helper functions for debugging*/

	/*
	 * laneID: return which warp lane the (thread) code belongs to. (from 1 to 32)
	 */
	static __device__ __forceinline__
	uint32_t getLaneID()
	{
		unsigned int laneid;
		asm("mov.u32 %0, %laneid;" : "=r"(laneid));
		return laneid;
	}
	/*
	 * laneMask: return decimal value of power of 2 of the thread id belongs to. e.g 3 = 2^3 = 8
	 */
	static __device__ __forceinline__
	int laneMask()
	{
		unsigned int lane_mask;
		asm("mov.u32 %0, %lanemask_lt;" : "=r"(lane_mask));
		return lane_mask;
	}
	/*
	 *
	 */
	static __device__ __forceinline__
	int binaryExclScan(int ballot_mask)
	{
		return __popc(laneMask() & ballot_mask);
	}

	/*
	 * Device function readTSDF gets tsdf value and weight
	 * at the coordinate(x,y,z) of the tsdf volume
	 */
	__device__ __forceinline__ float
	readTSDF(const cv::cuda::PtrStep<float> tsdf_volume,
			 const cv::cuda::PtrStep<float> weight_volume,
			const int volume_res,
			const int x,
			const int y,
			const int z,
			float& weight)
	{
			float tsdf = tsdf_volume.ptr(z*volume_res+y)[x];
			weight = weight_volume.ptr(z*volume_res+y)[x];
			//if(tsdf !=1)
				//printf("%f %f\n",tsdf,weight);
			return tsdf;
	}

	__device__ __forceinline__
	int computeCubeIndex(const cv::cuda::PtrStep<float> tsdf_volume,
			const cv::cuda::PtrStep<float> weight_volume,
			const int volume_res,
			const int x, const int y, const int z,
			float tsdf_values[8])
	{
		float weight;



		int cube_index = 0; //flag if vertex is inside or outside iso surface

		cube_index += static_cast<int>(tsdf_values[0] =
			readTSDF(tsdf_volume,weight_volume, volume_res, x, y, z, weight) < 0.f);
		if (weight == 0) return 0;
		cube_index += static_cast<int>(tsdf_values[1] =
			readTSDF(tsdf_volume,weight_volume, volume_res, x + 1, y, z, weight) < 0.f) << 1;
		if (weight == 0) return 0;
		cube_index += static_cast<int>(tsdf_values[2] =
			readTSDF(tsdf_volume,weight_volume, volume_res, x + 1, y + 1, z, weight) < 0.f) << 2;
		if (weight == 0) return 0;
		cube_index += static_cast<int>(tsdf_values[3] =
			readTSDF(tsdf_volume,weight_volume, volume_res, x, y + 1, z, weight) < 0.f) << 3;
		if (weight == 0) return 0;
		cube_index += static_cast<int>(tsdf_values[4] =
			readTSDF(tsdf_volume,weight_volume, volume_res, x, y, z + 1, weight) < 0.f) << 4;
		if (weight == 0) return 0;
		cube_index += static_cast<int>(tsdf_values[5] =
			readTSDF(tsdf_volume,weight_volume, volume_res, x + 1, y, z + 1, weight) < 0.f) << 5;
		if (weight == 0) return 0;
		cube_index += static_cast<int>(tsdf_values[6] =
			readTSDF(tsdf_volume,weight_volume, volume_res, x + 1, y + 1, z + 1, weight) < 0.f) << 6;
		if (weight == 0) return 0;
		cube_index += static_cast<int>(tsdf_values[7] =
			readTSDF(tsdf_volume,weight_volume, volume_res, x, y + 1, z + 1, weight) < 0.f) << 7;
		if (weight == 0) return 0;
			//if(cube_index !=0)
			//printf("%d \n",cube_index);

		return cube_index;
	}
	/*
	 * Get coordinate of a center of voxel relative to base camera
	 */
	__device__ __forceinline__
	float3 getVoxelCoordinates(const float original_distance_x,
								const float original_distance_y,
								const float original_distance_z,
								const int x, const int y, const int z,
								const float voxel_size)
	{
		float3 position;
		position.x = original_distance_x + (x+0.5)* voxel_size;
		position.y = original_distance_y + (y+0.5)* voxel_size;
		position.z = original_distance_z + (z+0.5)* voxel_size;
		return position;
	}
	/*
	 * get coordinate of the intersection on the edge
	 * isolevel =0.f is hard coded indicating the surface
	 */
	__device__ __forceinline__
	float3 getInterpolate(const float3 p0,const float3 p1,const float f0, const float f1)
	{

		//If intersection lays on the voxel nodes return that node
		if (std::abs(0.f-f0) < 0.00001){
		      return(p0);
		}
		if (std::abs(0.f-f1) < 0.00001){
		      return(p1);
		}
		if (std::abs(f0-f1) < 0.00001){
		      return(p0);
		}

		//else, compute the distance relative to voxel p0
		float mu = (0.f - f0) / (f1 - f0 + 1e-15f);
		return make_float3(p0.x + mu * (p1.x - p0.x),
                			p0.y + mu * (p1.y - p0.y),
                			p0.z + mu * (p1.z - p0.z));
	}

		/*Kernel functions*/
	__global__ void getOccupiedVoxelsKernel(const cv::cuda::PtrStep<float> tsdf_volume,
			const int volume_res,
			const cv::cuda::PtrStep<float> weight_volume,
			cv::cuda::PtrStepSz<int> occupied_voxel_indices,
			cv::cuda::PtrStepSz<int> number_vertices,
			const cv::cuda::PtrStepSz<int> number_vertices_table)
	{
		const auto x = threadIdx.x + blockIdx.x * blockDim.x;
		const auto y = threadIdx.y + blockIdx.y * blockDim.y;
		//Return if thread id is out of warp size e.g (x/warp_size +1) threads are not returned
		if (__all(x >= volume_res) || __all(y >= volume_res))
			return;
		//return thread id of each processor of GPU core (thread id with n(processors) times)
		const auto flattened_tid =   threadIdx.z * blockDim.x *
				blockDim.y + threadIdx.y * blockDim.x + threadIdx.x;
		const auto warp_id = flattened_tid >> 5; // warp size = 32 = 2^5, shift 5 bits or flattened_tid%32
		//Get index of thread in warp
		const auto lane_id = getLaneID();
		// keyword "volatile"  instructs the compiler to always generate a read or write for
		//that access, and never "optimize" it into a register or some other optimization.
		//only for shared memory
		volatile __shared__ int warps_buffer[WARP_SIZE]; // Number of threads / Warp size
		//Search along z direction
		for(auto z=0;z<volume_res-1;++z)
		{
			int num_vertices =0;
			//compute number of vertices
			if(x+1<volume_res && y+1<volume_res)
			{
				 float tsdf_values[8];
				 const int cube_index = computeCubeIndex(tsdf_volume, weight_volume,volume_res,
						 	 	 	 	 	 	 	 	 x, y, z, tsdf_values);
				 num_vertices = (cube_index == 0 || cube_index == 255) ? 0 :
						 number_vertices_table.ptr(0)[cube_index];
			}
		// __ballot: returns a value with the Nth bit set, where N is the thread index.
		 const int total = __popc(__ballot(num_vertices > 0));

         if (total == 0)
             continue;
         //only increment in thread 0
         if (lane_id == 0) {
             const int old = atomicAdd(&global_count, total);
             warps_buffer[warp_id] = old;
         }
         const int old_global_voxels_count = warps_buffer[warp_id];
         const int offs = binaryExclScan(__ballot(num_vertices > 0));
         const int max_size = occupied_voxel_indices.cols;

         if(old_global_voxels_count + offs < max_size && num_vertices>0)
         {
        	 const int current_voxel_index = z*volume_res*volume_res + y*volume_res + x;
        	 occupied_voxel_indices.ptr(0)[old_global_voxels_count+offs] = current_voxel_index;
        	 number_vertices.ptr(0)[old_global_voxels_count+offs] = num_vertices;
         }
         bool full = old_global_voxels_count + total>= max_size;
         if(full) break;
		}
         if(flattened_tid==0)
         {
        	unsigned int total_blocks = gridDim.x * gridDim.y * gridDim.z;
        	unsigned int value = atomicInc(&blocks_done, total_blocks);

         if(value ==total_blocks-1){
        	output_count = min(occupied_voxel_indices.cols, global_count);
        	blocks_done = 0;
        	global_count = 0;
         }
         }

	}
	__global__ void generateTriangleKernel(const cv::cuda::PtrStep<float> tsdf_volume,
										   const cv::cuda::PtrStep<float> weight_volume,
											const int volume_res,
											const float voxel_size,
											const cv::cuda::PtrStepSz<int> occupied_voxels,
											const cv::cuda::PtrStepSz<int> vertex_offsets,
											const cv::cuda::PtrStep<int> number_vertices_table,
											const cv::cuda::PtrStep<int> triangle_table,
											cv::cuda::PtrStep<float3> triangle_value,
											const float original_distance_x,
											const float original_distance_y,
											const float original_distance_z)
	{
		const auto idx = (blockIdx.y * 65536 + blockIdx.x) * 256 + threadIdx.x;
		//OOB
		if(idx>=occupied_voxels.cols)
			return;
		const int voxel = occupied_voxels.ptr(0)[idx];
		//index of voxel in x,y,z directions
		const int z = voxel / (volume_res * volume_res);
		const int y = (voxel - z * volume_res * volume_res) / volume_res;
		const int x = (voxel - z * volume_res * volume_res) - y * volume_res;

		float tsdf_values[8];
		const int cube_index = computeCubeIndex(tsdf_volume,weight_volume,
				volume_res, x, y, z, tsdf_values);
		float3 v[8];
		v[0] = getVoxelCoordinates(original_distance_x,original_distance_y,original_distance_z,x,y,z,voxel_size);
		v[1] = getVoxelCoordinates(original_distance_x,original_distance_y,original_distance_z,x+1,y,z,voxel_size);
		v[2] = getVoxelCoordinates(original_distance_x,original_distance_y,original_distance_z,x+1,y+1,z,voxel_size);
		v[3] = getVoxelCoordinates(original_distance_x,original_distance_y,original_distance_z,x,y+1,z,voxel_size);
		v[4] = getVoxelCoordinates(original_distance_x,original_distance_y,original_distance_z,x,y,z+1,voxel_size);
		v[5] = getVoxelCoordinates(original_distance_x,original_distance_y,original_distance_z,x+1,y,z+1,voxel_size);
		v[6] = getVoxelCoordinates(original_distance_x,original_distance_y,original_distance_z,x+1,y+1,z+1,voxel_size);
		v[7] = getVoxelCoordinates(original_distance_x,original_distance_y,original_distance_z,x,y+1,z+1,voxel_size);
		__shared__ float3 vertex_list[12][256];
		vertex_list[0][threadIdx.x] = getInterpolate(v[0], v[1], tsdf_values[0], tsdf_values[1]);
		vertex_list[1][threadIdx.x] = getInterpolate(v[1], v[2], tsdf_values[1], tsdf_values[2]);
		vertex_list[2][threadIdx.x] = getInterpolate(v[2], v[3], tsdf_values[2], tsdf_values[3]);
		vertex_list[3][threadIdx.x] = getInterpolate(v[3], v[0], tsdf_values[3], tsdf_values[0]);
		vertex_list[4][threadIdx.x] = getInterpolate(v[4], v[5], tsdf_values[4], tsdf_values[5]);
		vertex_list[5][threadIdx.x] = getInterpolate(v[5], v[6], tsdf_values[5], tsdf_values[6]);
		vertex_list[6][threadIdx.x] = getInterpolate(v[6], v[7], tsdf_values[6], tsdf_values[7]);
		vertex_list[7][threadIdx.x] = getInterpolate(v[7], v[4], tsdf_values[7], tsdf_values[4]);
		vertex_list[8][threadIdx.x] = getInterpolate(v[0], v[4], tsdf_values[0], tsdf_values[4]);
		vertex_list[9][threadIdx.x] = getInterpolate(v[1], v[5], tsdf_values[1], tsdf_values[5]);
		vertex_list[10][threadIdx.x] = getInterpolate(v[2], v[6], tsdf_values[2], tsdf_values[6]);
		vertex_list[11][threadIdx.x] = getInterpolate(v[3], v[7], tsdf_values[3], tsdf_values[7]);
		__syncthreads();
		const int num_vertices = number_vertices_table.ptr(0)[cube_index];
		for (int i = 0; i < num_vertices; i += 3) {
		const int index = vertex_offsets.ptr(0)[idx] + i;

		const int v1 = triangle_table.ptr(0)[(cube_index * 16) + i + 0];
		const int v2 = triangle_table.ptr(0)[(cube_index * 16) + i + 1];
		const int v3 = triangle_table.ptr(0)[(cube_index * 16) + i + 2];

		triangle_value.ptr(0)[index + 0] = make_float3(vertex_list[v1][threadIdx.x].x,
		                                           		vertex_list[v1][threadIdx.x].y,
		                                           		vertex_list[v1][threadIdx.x].z);
		triangle_value.ptr(0)[index + 1] = make_float3(vertex_list[v2][threadIdx.x].x,
		                                             	vertex_list[v2][threadIdx.x].y,
		                                             	vertex_list[v2][threadIdx.x].z);
		triangle_value.ptr(0)[index + 2] = make_float3(vertex_list[v3][threadIdx.x].x,
														vertex_list[v3][threadIdx.x].y,
														vertex_list[v3][threadIdx.x].z);
	}
	}
	__global__ void getColorKernel(const cv::cuda::PtrStep<uchar3> color_volume,
								   const cv::cuda::PtrStep<float3> vertices,
								   cv::cuda::PtrStepSz<uchar3> vertex_colors,
                                    const int volume_res,const float voxel_size,
                                   const float original_distance_x,
                                   const float original_distance_y,
                                   const float original_distance_z)

	{
			const auto i = blockDim.x * blockIdx.x + threadIdx.x;
			if(i>vertex_colors.cols)
				return;
			const float3 vertex = vertices.ptr(0)[i];
            const int3 grid_pt{static_cast<int>((vertex.x-original_distance_x) / voxel_size),
                               static_cast<int>((vertex.y-original_distance_y) / voxel_size),
                               static_cast<int>((vertex.z-original_distance_z) / voxel_size)};
			uchar3 color_value = color_volume.ptr(grid_pt.z*volume_res+grid_pt.y)[grid_pt.x];

			vertex_colors.ptr(0)[i] = color_value;

	}
	/***Host Side***/
	SurfaceMesh hostMarchingCubes(const VolumeData& volume,
							const int triangles_buffer_size,
							const float original_distance_x,
							const float original_distance_y,
							const float original_distance_z)
	{
		 MeshData mesh_data(triangles_buffer_size / 3);
		 //Lookup table
		 cv::cuda::GpuMat number_vertices_table, triangle_table;
		 number_vertices_table = cv::cuda::createContinuous(256,1,CV_32SC1);
		 number_vertices_table.upload(cv::Mat(256, 1, CV_32SC1,
				 number_vertices_table_host, cv::Mat::AUTO_STEP));

		 triangle_table = cv::cuda::createContinuous(256, 16, CV_32SC1);
		 triangle_table.upload(cv::Mat(256, 16, CV_32SC1, tri_table_host, cv::Mat::AUTO_STEP));
		 //### KERNEL ONE : Get occupied voxels ###
		 dim3 threads(WARP_SIZE, WARP_SIZE);
		 dim3 blocks(static_cast<unsigned>(std::ceil(volume.volume_res_ / threads.x)),
				     static_cast<unsigned>(std::ceil(volume.volume_res_ / threads.y)));
		 getOccupiedVoxelsKernel<<<blocks, threads>>>(volume.tsdf_volume,
				 volume.volume_res_,
				 volume.weight_volume,
				 mesh_data.occupied_voxel_ids_buffer,
				 mesh_data.number_vertices_buffer,
				 number_vertices_table);
		 cudaDeviceSynchronize();

		 int active_voxels = 0;
		 cudaMemcpyFromSymbol(&active_voxels, output_count, sizeof(active_voxels));

		 //### THRUST PART : Do an exclusive scan on the GPU ###
		 mesh_data.create_view(active_voxels);

		 thrust::device_ptr<int> beg = thrust::device_pointer_cast(mesh_data.number_vertices.ptr<int>(0));
		 thrust::device_ptr<int> end = beg + active_voxels;

		 thrust::device_ptr<int> out = thrust::device_pointer_cast(mesh_data.vertex_offsets.ptr<int>(0));
		 thrust::exclusive_scan(beg, end, out);

		 int last_element, last_scan_element;

		 cudaMemcpy(&last_element, mesh_data.number_vertices.ptr<int>(0) + active_voxels - 1,
				 sizeof(int),cudaMemcpyDeviceToHost);
		 cudaMemcpy(&last_scan_element, mesh_data.vertex_offsets.ptr<int>(0) + active_voxels - 1,
				 sizeof(int),cudaMemcpyDeviceToHost);
		 const int total_vertices = last_element + last_scan_element;
		 printf("total_vertices %d \n",total_vertices);
		 // ### ###

		 //### KERNEL TWO ###
		 const int n_threads = 256;
		 dim3 block(n_threads);
		 unsigned blocks_num = static_cast<unsigned>(std::ceil(active_voxels / n_threads));
		 dim3 grid(min(blocks_num, 65536), static_cast<unsigned>(std::ceil(blocks_num / 65536)));
			grid.y = 1;


		generateTriangleKernel<<<grid, block>>> (volume.tsdf_volume,volume.weight_volume,
		                         volume.volume_res_, volume.voxel_size_,
		                         mesh_data.occupied_voxel_ids, mesh_data.vertex_offsets,
		                         number_vertices_table, triangle_table,
		                         mesh_data.triangle_buffer,
		                         original_distance_x,
		                         original_distance_y,
		                         original_distance_z);

		 cudaDeviceSynchronize();

		 // Get triangle vertex colors
		 cv::cuda::GpuMat triangles_output(mesh_data.triangle_buffer,
				 cv::Range::all(), cv::Range(0, total_vertices));
		 cv::cuda::GpuMat vertex_colors = cv::cuda::createContinuous(1, total_vertices, CV_8UC3);


		                int n_blocks = static_cast<int>(std::ceil(total_vertices / 1024));
		                getColorKernel<<<n_blocks, 1024>>> (volume.color_volume,
		                		triangles_output,vertex_colors,
                                volume.volume_res_, volume.voxel_size_,
                                original_distance_x,
                                original_distance_y,
                                original_distance_z
		                      	  );
		 cudaDeviceSynchronize();

		  // Download triangles
		cv::Mat vertex_output {};
		triangles_output.download(vertex_output);

		cv::Mat color_output {};
		vertex_colors.download(color_output);

		return SurfaceMesh { vertex_output, color_output, total_vertices, total_vertices / 3 };

	}



	}



