#ifndef KINECFUSION_DATA_TYPE_HPP
#define KINECFUSION_DATA_TYPE_HPP


#if __GNUC__ > 4 || (__GNUC__ == 4 && __GNUC_MINOR__ >= 6)
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wall"
#pragma GCC diagnostic ignored "-Wextra"
#pragma GCC diagnostic ignored "-Weffc++"
#include <cuda_runtime.h>
#include <opencv2/core/cuda.hpp>
#include <iostream>
#pragma GCC diagnostic pop
#else
#include <cuda_runtime.h>
#include <opencv2/core/cuda.hpp>
#include <eigen3/Eigen/Eigen>
#include <iostream>

#endif

//define warp size for NVIDA GPU
#define WARP_SIZE 32

// Define this to turn on error checking
#define CUDA_ERROR_CHECK

#define CudaSafeCall( err ) __cudaSafeCall( err, __FILE__, __LINE__ )
#define CudaCheckError()    __cudaCheckError( __FILE__, __LINE__ )

inline void __cudaSafeCall( cudaError err, const char *file, const int line )
{
#ifdef CUDA_ERROR_CHECK
    if ( cudaSuccess != err )
    {
        fprintf( stderr, "cudaSafeCall() failed at %s:%i : %s\n",
                 file, line, cudaGetErrorString( err ) );
        exit( -1 );
    }
#endif

    return;
}

template<typename T>
struct CudaBuffer{
private:
	size_t buffer_size_;
	//Buffer on host
	T* h_buffer_;
	//Buffer on device
	T* d_buffer_;

	//make non-copyable
	CudaBuffer(const CudaBuffer&);
	CudaBuffer& operator=(const CudaBuffer&);
public:
	 //decleare Buffer on device
	CudaBuffer(size_t size):buffer_size_(size),h_buffer_(new T[size])
	{
		cudaMalloc(&d_buffer_,sizeof(T) * size);
	}
	~CudaBuffer()
	{
	               delete[] h_buffer_;

	}
	size_t size()
	{
	              return buffer_size_;
	}
	 //initilize value for host
	void memsetHost(int32_t value)
	{
		   memset(h_buffer_,value, size()*sizeof(T));
	}
	void upload()
	{
		  cudaMemcpy(d_buffer_, h_buffer_,size() * sizeof(float), cudaMemcpyHostToDevice);
	}
	void download()
	{
		  cudaMemcpy(h_buffer_, d_buffer_,size() * sizeof(float), cudaMemcpyDeviceToHost);
	}
	T* deviceBuffer()
	{
		return 	d_buffer_;
	}
	T* hostBuffer()
	{
		return h_buffer_;
	}
};


inline void __cudaCheckError( const char *file, const int line )
{
#ifdef CUDA_ERROR_CHECK
    cudaError err = cudaGetLastError();
    if ( cudaSuccess != err )
    {
        fprintf( stderr, "cudaCheckError() failed at %s:%i : %s\n",
                 file, line, cudaGetErrorString( err ) );
        exit( -1 );
    }

    // More careful checking. However, this will affect performance.
    // Comment away if needed.
    err = cudaDeviceSynchronize();
    if( cudaSuccess != err )
    {
        fprintf( stderr, "cudaCheckError() with sync failed at %s:%i : %s\n",
                 file, line, cudaGetErrorString( err ) );
        exit( -1 );
    }
#endif

    return;
}
/**
 * CameraParameters contains intrinsic parameters of depth camera, width and height of input image as well as
 * the level of pyramid level of input image with 0 being original image, higher level correspond smaller size
 */
struct CameraParameters{
	//Size of image from camera
	int image_width, image_height;
	//camera matrix
	float focal_x, focal_y;
	float c_x,c_y;
	/**
	 * Build a paramid of input image, KinectFusion paper num_levels = 3
	 * camera parameters contain scaled value
	 */
	CameraParameters level(const size_t level) const
	{
		if(level==0) return *this;
		const float scale = powf(0.5f, static_cast<float> (level));
		return CameraParameters{
			image_width>>level,image_height>>level,
			focal_x*scale, focal_y* scale,
			(c_x+0.5f)*scale -0.5f, (c_y+0.5f)*scale -0.5f
		};
	}
};
/**
 * FrameData structure holds data uploaded on device(GPU), must be assigned to the function at instantiation
 */
struct FrameData{
	std::vector<cv::cuda::GpuMat> depth_pyramid;
        std::vector<cv::cuda::GpuMat> color_pyramid;
	std::vector<cv::cuda::GpuMat> smoothed_depth_pyramid;
	std::vector<cv::cuda::GpuMat> normal_pyramid;
	std::vector<cv::cuda::GpuMat> vertex_pyramid;
        //Assign at initialization
        explicit FrameData(const size_t pyramid_level):
							depth_pyramid(pyramid_level),
                                                        color_pyramid(pyramid_level),
							smoothed_depth_pyramid(pyramid_level),
							normal_pyramid(pyramid_level),
							vertex_pyramid(pyramid_level)
	{}
	//make non-copyable
	FrameData(const FrameData&) = delete;
	FrameData& operator = (const FrameData& other) = delete;
	//requires a function to not throw any exceptions
	FrameData(FrameData&& data) noexcept :
			////move, not copy
			depth_pyramid(std::move(data.depth_pyramid)),
                        color_pyramid(std::move(data.color_pyramid)),
			smoothed_depth_pyramid(std::move(data.smoothed_depth_pyramid)),
			normal_pyramid(std::move(data.normal_pyramid)),
			vertex_pyramid(std::move(data.vertex_pyramid))
			{}
	FrameData& operator= (FrameData&& data) noexcept
			{
			depth_pyramid = std::move(data.depth_pyramid);
                        color_pyramid = std::move(data.color_pyramid);
			smoothed_depth_pyramid= std::move(data.smoothed_depth_pyramid);
			normal_pyramid = std::move(data.normal_pyramid);
			vertex_pyramid = std::move(data.vertex_pyramid);
			return *this;
			}

};
        /*
         * Contains the internal data representation of one single frame as raycast by surface prediction
         * Consists of depth, smoothed depth and color pyramids as well as vertex and normal pyramids
         */
    struct ModelData{
        std::vector<cv::cuda::GpuMat> vertex_pyramid;
        std::vector<cv::cuda::GpuMat> normal_pyramid;
        std::vector<cv::cuda::GpuMat> color_pyramid;
        ModelData(const size_t pyramid_height, const CameraParameters camera_parameters) :
                          vertex_pyramid(pyramid_height), normal_pyramid(pyramid_height),
                          color_pyramid(pyramid_height)
    {
          for (size_t level = 0; level < pyramid_height; ++level) {
                      vertex_pyramid[level] =
                      cv::cuda::createContinuous(camera_parameters.level(level).image_height,
                                                          camera_parameters.level(level).image_width,
                                                          CV_32FC3);
                       normal_pyramid[level] =
                               cv::cuda::createContinuous(camera_parameters.level(level).image_height,
                                                          camera_parameters.level(level).image_width,
                                                          CV_32FC3);
                       color_pyramid[level] =
                               cv::cuda::createContinuous(camera_parameters.level(level).image_height,
                                                          camera_parameters.level(level).image_width,
                                                          CV_8UC3);
                       vertex_pyramid[level].setTo(0);
                       normal_pyramid[level].setTo(0);
                   }
      }

        // No copying
        ModelData(const ModelData&) = delete;
        ModelData& operator=(const ModelData& data) = delete;

        ModelData(ModelData&& data) noexcept :
                vertex_pyramid(std::move(data.vertex_pyramid)),
                normal_pyramid(std::move(data.normal_pyramid)),
                color_pyramid(std::move(data.color_pyramid))
        { }

        ModelData& operator=(ModelData&& data) noexcept
        {
            vertex_pyramid = std::move(data.vertex_pyramid);
            normal_pyramid = std::move(data.normal_pyramid);
            color_pyramid = std::move(data.color_pyramid);
            return *this;
        }
    };

   struct VolumeData{
	cv::cuda::GpuMat tsdf_volume; //float
	cv::cuda::GpuMat weight_volume; //int
	cv::cuda::GpuMat color_volume; //uchar

	int volume_res_;
	float voxel_size_;
	VolumeData(const int volume_res, const float voxel_size):
		tsdf_volume(cv::cuda::createContinuous(volume_res*volume_res,volume_res,CV_32FC1)),
		weight_volume(cv::cuda::createContinuous(volume_res*volume_res,volume_res,CV_32FC1)),
		color_volume(cv::cuda::createContinuous(volume_res*volume_res,volume_res,CV_8UC3)),
		volume_res_(volume_res),
		voxel_size_(voxel_size)
	{


        tsdf_volume.setTo(0);
		weight_volume.setTo(0);
		color_volume.setTo(0);

    }

};
   /*
            *
            * \brief Contains the internal pointcloud representation
            * This is only used for exporting the data kept in the internal volumes
            * It holds GPU containers for vertices, normals and vertex colors
            * It also contains host containers for this data and defines the total number of points
            */
           struct CloudData {
               cv::cuda::GpuMat vertices;
               cv::cuda::GpuMat normals;
               cv::cuda::GpuMat color;

               cv::Mat host_vertices;
               cv::Mat host_normals;
               cv::Mat host_color;

               int* point_num;
               int host_point_num;

               explicit CloudData(const int max_number) :
                       vertices{cv::cuda::createContinuous(1, max_number, CV_32FC3)},
                       normals{cv::cuda::createContinuous(1, max_number, CV_32FC3)},
                       color{cv::cuda::createContinuous(1, max_number, CV_8UC3)},
                       host_vertices{}, host_normals{}, host_color{}, point_num{nullptr}, host_point_num{}
               {
                   vertices.setTo(0.f);
                   normals.setTo(0.f);
                   color.setTo(0.f);

                   cudaMalloc(&point_num, sizeof(int));
                   cudaMemset(point_num, 0, sizeof(int));
               }

               // No copying
               CloudData(const CloudData&) = delete;
               CloudData& operator=(const CloudData& data) = delete;

               void download()
               {
                   vertices.download(host_vertices);
                   normals.download(host_normals);
                   color.download(host_color);
                   cudaMemcpy(&host_point_num, point_num, sizeof(int), cudaMemcpyDeviceToHost);
               }
           };
           /*
                   *
                   * \brief Contains the internal surface mesh representation
                   *
                   * This is only used for exporting the data kept in the internal volumes
                   *
                   * It holds several GPU containers needed for the MarchingCubes algorithm
                   *
                   */
                  struct MeshData {
        	   	   	   cv::cuda::GpuMat occupied_voxel_ids_buffer;
        	   	   	   cv::cuda::GpuMat number_vertices_buffer;
        	   	   	   cv::cuda::GpuMat vertex_offsets_buffer;
        	   	   	   cv::cuda::GpuMat triangle_buffer;

        	   	   	   cv::cuda::GpuMat occupied_voxel_ids;
        	   	   	   cv::cuda::GpuMat number_vertices;
        	   	   	   cv::cuda::GpuMat vertex_offsets;

                      explicit MeshData(const int buffer_size):
                              occupied_voxel_ids_buffer{cv::cuda::createContinuous(1, buffer_size, CV_32SC1)},
                              number_vertices_buffer{cv::cuda::createContinuous(1, buffer_size, CV_32SC1)},
                              vertex_offsets_buffer{cv::cuda::createContinuous(1, buffer_size, CV_32SC1)},
                              triangle_buffer{cv::cuda::createContinuous(1, buffer_size * 3, CV_32FC3)},
                              occupied_voxel_ids{}, number_vertices{}, vertex_offsets{}
                      { }

                      void create_view(const int length)
                      {
                          occupied_voxel_ids = cv::cuda::GpuMat(1, length, CV_32SC1, occupied_voxel_ids_buffer.ptr<int>(0),
                                                      occupied_voxel_ids_buffer.step);
                          number_vertices = cv::cuda::GpuMat(1, length, CV_32SC1, number_vertices_buffer.ptr<int>(0),
                                                   number_vertices_buffer.step);
                          vertex_offsets = cv::cuda::GpuMat(1, length, CV_32SC1, vertex_offsets_buffer.ptr<int>(0),
                                                  vertex_offsets_buffer.step);
                      }
                  };

           struct PointCloud {
        	   // World coordinates of all vertices
        	   cv::Mat vertices;
        	   // Normal directions
        	   cv::Mat normals;
        	   // RGB color values
        	   cv::Mat color;
        	   // Total number of valid points
        	   int num_points;
           };
           struct SurfaceMesh {
          	   //Triangular faces
          	   cv::Mat triangles;
          	   //Colors of vertices
          	   cv::Mat colors;
          	   //Total number of vertices
          	   int num_vertices;
          	   //number of triangles
          	   int num_triangles;
             };


#endif //KINECTFUSION_DATA_TYPES_HPP
